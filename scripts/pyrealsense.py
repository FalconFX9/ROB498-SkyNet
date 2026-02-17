import pyrealsense2 as rs
import cv2 as cv

print(len(rs.context().devices))

print("successfully imported pyrealsense2:", rs)