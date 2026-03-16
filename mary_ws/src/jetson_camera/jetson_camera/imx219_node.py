import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np

class JetsonCameraNode(Node):
    def __init__(self):
        super().__init__('imx219_node')
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        
        # PIPELINE STRATEGY:
        # 1. nvarguscamerasrc: Captures from CSI
        # 2. nvvidconv: Scales to 640x480 AND converts to RGBA on the GPU (VIC engine)
        # 3. appsink: Hands off the hardware-converted frame to Python
        gst_pipeline = (
            "nvarguscamerasrc sensor-id=0 sensor-mode=4 ! "
            "video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=30/1 ! "
            "nvvidconv ! "
            "video/x-raw, width=640, height=480, format=RGBA ! "
            "appsink drop=true"
        )
        
        self.get_logger().info('Initializing GStreamer pipeline...')
        self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
        
        if not self.cap.isOpened():
            self.get_logger().error('CRITICAL: Could not open GStreamer pipeline. Check camera cable or run: sudo systemctl restart nvargus-daemon')
            return

        # 30Hz timer to match the sensor framerate
        self.timer = self.create_timer(1.0/30.0, self.timer_callback)
        self.get_logger().info('IMX219 Node started. Resolution: 640x480 @ 30fps (RGBA Optimized)')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            # If this triggers, the hardware daemon might have hung
            self.get_logger().warn('Frame drop detected from GStreamer')
            return

        # 'frame' is RGBA (4 channels). We slice it to get BGR (3 channels).
        # This is a memory-view operation in Numpy and is significantly faster 
        # than using cv2.cvtColor() on the CPU.
        bgr_frame = frame[:, :, :3]

        # Manually construct the ROS 2 Image message
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_link"
        msg.height = bgr_frame.shape[0]
        msg.width = bgr_frame.shape[1]
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = bgr_frame.shape[1] * 3
        msg.data = bgr_frame.tobytes()
        
        self.publisher_.publish(msg)

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = JetsonCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down IMX219 Node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
