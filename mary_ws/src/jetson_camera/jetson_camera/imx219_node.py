import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2

class JetsonCameraNode(Node):
    def __init__(self):
        super().__init__('imx219_node')
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        
        # PIPELINE EXPLANATION:
        # 1. nvarguscamerasrc: Uses the Jetson ISP hardware (Zero CPU hit)
        # 2. nvvidconv: Scales and converts format on the hardware
        # 3. appsink drop=true: Prevents lag by dropping frames if the node is slow
        gst_pipeline = (
            "nvarguscamerasrc sensor-id=0 sensor-mode=4 ! "
            "video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=30/1 ! "
            "nvvidconv ! "
            "video/x-raw, width=640, height=480, format=BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=BGR ! appsink drop=true"
        )
        
        self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
        
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open IMX219. Is the cable correct?')
            return

        # Timer matches the 60fps framerate
        self.timer = self.create_timer(1.0/60.0, self.timer_callback)
        self.get_logger().info('IMX219 hardware-accelerated node started.')
    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # Create the ROS2 Image message manually
            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_link"
            msg.height = frame.shape[0]
            msg.width = frame.shape[1]
            msg.encoding = "bgr8"
            msg.is_bigendian = False
            msg.step = frame.shape[1] * 3
            msg.data = frame.tobytes() # Convert numpy array to raw bytes
            self.publisher_.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = JetsonCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
