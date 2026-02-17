#!/usr/bin/env python3
import sys
import time
import cv2
import ctypes
import numpy as np
import pyrealsense2 as rs

# --- Helpers for T265 Calibration ---
def camera_matrix(intrinsics):
    return np.array([[intrinsics.fx, 0, intrinsics.ppx],
                     [0, intrinsics.fy, intrinsics.ppy],
                     [0, 0, 1]])

def fisheye_distortion(intrinsics):
    return np.array(intrinsics.coeffs[:4])

def get_extrinsics(src, dst):
    extrinsics = src.get_extrinsics_to(dst)
    R = np.reshape(extrinsics.rotation, [3, 3]).T
    T = np.array(extrinsics.translation)
    return R, T

class OptimizedStereoPipeline:
    def __init__(self, width, height, max_disp=128):
        self.input_w = width
        self.input_h = height
        self.output_w = width // 2
        self.output_h = height // 2
        self.max_disp = max_disp 
        self.use_vpi = False
        
        try:
            self.lib = ctypes.CDLL('./libstereo.so')
            self.lib.init_pipeline.restype = ctypes.c_void_p
            self.lib.compute_frame.argtypes = [
                ctypes.c_void_p, 
                np.ctypeslib.ndpointer(dtype=np.uint8, flags='C_CONTIGUOUS'),
                np.ctypeslib.ndpointer(dtype=np.uint8, flags='C_CONTIGUOUS'),
                np.ctypeslib.ndpointer(dtype=np.uint16, flags='C_CONTIGUOUS')
            ]
            
            self.ctx = self.lib.init_pipeline(width, height, max_disp)
            self.use_vpi = True
            print(f"[VPI] Pipeline Loaded. Input: {width}x{height} -> Output: {self.output_w}x{self.output_h}")
        except Exception as e:
            print(f"[Error] Failed to load VPI library: {e}")
            sys.exit(1)

        self.d_buffer = np.zeros((self.output_h, self.output_w), dtype=np.uint16)

    def compute(self, left, right):
        self.lib.compute_frame(self.ctx, left, right, self.d_buffer)
        return self.d_buffer

def reset_camera():
    try:
        ctx = rs.context()
        devices = ctx.query_devices()
        for dev in devices:
            dev.hardware_reset()
        print("Resetting camera... (Waiting 5s)")
        time.sleep(5)
    except:
        pass

def main():
    reset_camera()
    
    # --- PHASE 1: Get Calibration Data ---
    print("Reading Calibration Data...")
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.fisheye, 1)
    config.enable_stream(rs.stream.fisheye, 2)
    
    # Start briefly just to read the EEPROM
    profile = pipeline.start(config)
    
    stream_left = profile.get_stream(rs.stream.fisheye, 1).as_video_stream_profile()
    stream_right = profile.get_stream(rs.stream.fisheye, 2).as_video_stream_profile()
    
    intrinsics_left = stream_left.get_intrinsics()
    intrinsics_right = stream_right.get_intrinsics()
    
    w, h = intrinsics_left.width, intrinsics_left.height
    
    K_left = camera_matrix(intrinsics_left)
    D_left = fisheye_distortion(intrinsics_left)
    K_right = camera_matrix(intrinsics_right)
    D_right = fisheye_distortion(intrinsics_right)
    R, T = get_extrinsics(stream_left, stream_right)
    
    # STOP the camera so it doesn't crash while we do math
    pipeline.stop()
    print("Calibration read. Camera stopped.")

    # --- PHASE 2: Heavy Lifting (Math & Allocation) ---
    print("Computing Rectification Maps & Init VPI...")
    
    # 1. Calculate Rectification Transforms
    R1, R2, P1, P2, Q = cv2.fisheye.stereoRectify(
        K_left, D_left, K_right, D_right, (w, h), R, T, 
        flags=cv2.fisheye.CALIB_ZERO_DISPARITY, 
        balance=0.0, fov_scale=1.0
    )
    
    # 2. Custom Zoom to fix distortion (Zoom 1.5x)
    zoom_factor = 1.5
    new_fx = w / 2.0 * zoom_factor
    new_fy = h / 2.0 * zoom_factor
    P_custom = np.array([
        [new_fx, 0,      w/2.0, 0],
        [0,      new_fy, h/2.0, 0],
        [0,      0,      1,     0]
    ])

    # 3. Build Maps
    l_map1, l_map2 = cv2.fisheye.initUndistortRectifyMap(K_left, D_left, R1, P_custom, (w, h), cv2.CV_16SC2)
    r_map1, r_map2 = cv2.fisheye.initUndistortRectifyMap(K_right, D_right, R2, P_custom, (w, h), cv2.CV_16SC2)

    # 4. Init VPI (Allocates GPU Memory)
    stereo_pipe = OptimizedStereoPipeline(w, h, max_disp=256)

    # --- PHASE 3: Streaming Loop ---
    print("Restarting Camera for Streaming...")
    pipeline.start(config)
    
    fps_timer = time.time()
    frame_count = 0
    current_fps = 0.0

    try:
        # Warmup
        for _ in range(5): 
            pipeline.wait_for_frames()

        print(f"Streaming Active {w}x{h}. Press 'q' to quit.")
        
        while True:
            frames = pipeline.wait_for_frames()
            f_left = frames.get_fisheye_frame(1)
            f_right = frames.get_fisheye_frame(2)
            if not f_left or not f_right: continue

            # Zero-Copy to Numpy
            left_data = np.asanyarray(f_left.get_data())
            right_data = np.asanyarray(f_right.get_data())

            # 1. Rectify (CPU)
            left_rect = cv2.remap(left_data, l_map1, l_map2, cv2.INTER_LINEAR)
            right_rect = cv2.remap(right_data, r_map1, r_map2, cv2.INTER_LINEAR)

            # 2. Contiguous Check
            left_rect = np.ascontiguousarray(left_rect)
            right_rect = np.ascontiguousarray(right_rect)

            # 3. VPI Stereo (GPU)
            d_raw = stereo_pipe.compute(left_rect, right_rect)
            
            # 4. Median Filter (CPU Polish)
            d_filtered = cv2.medianBlur(d_raw, 3)

            # 5. Viz
            d_float = d_filtered.astype(np.float32) / 32.0
            d_vis = (d_float / 64.0 * 255.0).astype(np.uint8)
            d_color = cv2.applyColorMap(d_vis, cv2.COLORMAP_TURBO)
            
            left_small = cv2.resize(left_rect, (stereo_pipe.output_w, stereo_pipe.output_h))
            left_bgr = cv2.cvtColor(left_small, cv2.COLOR_GRAY2BGR)
            combined = np.hstack((left_bgr, d_color))

            # Stats
            frame_count += 1
            if frame_count % 10 == 0:
                now = time.time()
                current_fps = 10.0 / (now - fps_timer)
                fps_timer = now

            cv2.putText(combined, f"FPS: {current_fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow("Jetson Nano VPI Stereo", combined)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()