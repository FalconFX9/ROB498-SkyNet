#!/usr/bin/env python3
"""Print T265 calibration data in a format ready to paste into ROS nodes."""
import numpy as np
import pyrealsense2 as rs
import time
import json

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

# Reset camera
ctx = rs.context()
for dev in ctx.query_devices():
    dev.hardware_reset()
print("Resetting camera... (5s)")
time.sleep(5)

# Start pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.fisheye, 1)
config.enable_stream(rs.stream.fisheye, 2)
profile = pipeline.start(config)

stream_left = profile.get_stream(rs.stream.fisheye, 1).as_video_stream_profile()
stream_right = profile.get_stream(rs.stream.fisheye, 2).as_video_stream_profile()

intrinsics_left = stream_left.get_intrinsics()
intrinsics_right = stream_right.get_intrinsics()

K_left = camera_matrix(intrinsics_left)
D_left = fisheye_distortion(intrinsics_left)
K_right = camera_matrix(intrinsics_right)
D_right = fisheye_distortion(intrinsics_right)
R, T = get_extrinsics(stream_left, stream_right)

pipeline.stop()

print("\n" + "="*60)
print("T265 CALIBRATION DATA (for ROS node hardcoding)")
print("="*60)

print(f"\nImage size: {intrinsics_left.width} x {intrinsics_left.height}")
print(f"Distortion model: {intrinsics_left.model}")

print(f"\n# Left intrinsics")
print(f"K_left = np.array({K_left.tolist()})")
print(f"D_left = np.array({D_left.tolist()})")

print(f"\n# Right intrinsics")
print(f"K_right = np.array({K_right.tolist()})")
print(f"D_right = np.array({D_right.tolist()})")

print(f"\n# Extrinsics (left -> right, same as realsense_test.py)")
print(f"R = np.array({R.tolist()})")
print(f"T = np.array({T.tolist()})")
print(f"baseline = {np.linalg.norm(T)*1000:.2f} mm")

print(f"\n# Compare with ROS camera_info K matrices:")
print(f"# (these should match K_left and K_right above)")
print(f"# ros2 topic echo /camera/fisheye1/camera_info --no-arr")
