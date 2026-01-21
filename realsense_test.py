import pyrealsense2 as rs
import numpy as np
import cv2

# --- Start T265 pipeline ---
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.fisheye, 1)  # left
config.enable_stream(rs.stream.fisheye, 2)  # right
profile = pipeline.start(config)

# --- Stereo SGBM parameters ---
window_size = 5
min_disp = 0
num_disp = 64  # must be divisible by 16
stereo = cv2.StereoSGBM_create(
    minDisparity=min_disp,
    numDisparities=num_disp,
    blockSize=window_size,
    P1=8 * window_size**2,
    P2=32 * window_size**2,
    disp12MaxDiff=1,
    uniquenessRatio=10,
    speckleWindowSize=100,
    speckleRange=32,
    preFilterCap=63,
    mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
)

try:
    while True:
        frames = pipeline.wait_for_frames()
        left = np.asanyarray(frames.get_fisheye_frame(1).get_data())
        right = np.asanyarray(frames.get_fisheye_frame(2).get_data())

        # Stereo SGBM expects 8-bit single-channel images
        left_gray = left
        right_gray = right

        # --- Compute disparity ---
        disparity = stereo.compute(left_gray, right_gray).astype(np.float32) / 16.0

        # --- Mask invalid disparity ---
        disparity[disparity < min_disp] = 0

        # --- Apply bilateral filter to smooth disparity but keep edges ---
        disp_filtered = cv2.bilateralFilter(disparity, d=9, sigmaColor=75, sigmaSpace=75)

        # --- Normalize for visualization ---
        disp_vis = cv2.normalize(disp_filtered, None, 0, 255, cv2.NORM_MINMAX)
        disp_vis = np.uint8(disp_vis)
        disp_color = cv2.applyColorMap(disp_vis, cv2.COLORMAP_JET)

        cv2.imshow("SGM Disparity (Filtered)", disp_color)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    pipeline.stop()
    cv2.destroyAllWindows()
