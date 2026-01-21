#!/usr/bin/env python3
"""
Person Tracker Node for MARY Drone
Detects and tracks a person using the IMX219 camera with YOLO/pose estimation.
Publishes the target person's position relative to the drone.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Vector3Stamped
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np

# TODO: Import YOLO or pose estimation model
# from ultralytics import YOLO


class PersonTrackerNode(Node):
    """
    Detects and tracks a person using computer vision.

    Subscriptions:
        /mary/camera/image_raw (sensor_msgs/Image): Raw camera image

    Publications:
        /mary/perception/person_position (geometry_msgs/PointStamped): Person position in camera frame
        /mary/perception/person_velocity (geometry_msgs/Vector3Stamped): Person velocity estimate
        /mary/perception/person_detected (std_msgs/Bool): Whether a person is currently detected
        /mary/perception/annotated_image (sensor_msgs/Image): Debug image with bounding boxes
    """

    def __init__(self):
        super().__init__('person_tracker_node')

        # Parameters
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('target_class', 'person')
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('camera_fov_h', 62.2)  # IMX219 horizontal FOV in degrees
        self.declare_parameter('camera_fov_v', 48.8)  # IMX219 vertical FOV in degrees
        self.declare_parameter('publish_debug_image', True)

        # Get parameters
        self.model_path = self.get_parameter('model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        self.publish_debug = self.get_parameter('publish_debug_image').value

        # Initialize CV bridge
        self.bridge = CvBridge()

        # TODO: Initialize YOLO model
        # self.model = YOLO(self.model_path)
        self.model = None  # Placeholder

        # Tracking state
        self.last_detection_time = None
        self.last_position = None
        self.tracking_id = None  # For multi-person scenarios

        # QoS profile for sensor data
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/mary/camera/image_raw',
            self.image_callback,
            sensor_qos
        )

        # Publishers
        self.position_pub = self.create_publisher(
            PointStamped,
            '/mary/perception/person_position',
            10
        )
        self.velocity_pub = self.create_publisher(
            Vector3Stamped,
            '/mary/perception/person_velocity',
            10
        )
        self.detected_pub = self.create_publisher(
            Bool,
            '/mary/perception/person_detected',
            10
        )
        self.debug_image_pub = self.create_publisher(
            Image,
            '/mary/perception/annotated_image',
            10
        )

        self.get_logger().info('Person Tracker Node initialized')
        self.get_logger().warn('YOLO model not loaded - implement model loading!')

    def image_callback(self, msg: Image):
        """Process incoming camera image for person detection."""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # TODO: Run YOLO inference
            # results = self.model.predict(cv_image, verbose=False)
            # detections = self.process_detections(results)

            # Placeholder: No detection
            person_detected = False
            detections = []

            # Publish detection status
            detected_msg = Bool()
            detected_msg.data = person_detected
            self.detected_pub.publish(detected_msg)

            if person_detected and len(detections) > 0:
                # Get best detection (highest confidence or tracked ID)
                best_detection = self.select_target(detections)

                # Calculate position in camera frame
                position = self.calculate_position(best_detection, cv_image.shape)

                # Publish position
                pos_msg = PointStamped()
                pos_msg.header = msg.header
                pos_msg.point.x = position[0]  # Horizontal offset (positive = right)
                pos_msg.point.y = position[1]  # Vertical offset (positive = down)
                pos_msg.point.z = position[2]  # Estimated depth (if available)
                self.position_pub.publish(pos_msg)

                # Calculate and publish velocity
                velocity = self.estimate_velocity(position)
                if velocity is not None:
                    vel_msg = Vector3Stamped()
                    vel_msg.header = msg.header
                    vel_msg.vector.x = velocity[0]
                    vel_msg.vector.y = velocity[1]
                    vel_msg.vector.z = velocity[2]
                    self.velocity_pub.publish(vel_msg)

            # Publish debug image if enabled
            if self.publish_debug:
                annotated = self.draw_annotations(cv_image, detections)
                debug_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
                debug_msg.header = msg.header
                self.debug_image_pub.publish(debug_msg)

        except Exception as e:
            self.get_logger().error(f'Error in image callback: {e}')

    def process_detections(self, results):
        """Extract person detections from YOLO results."""
        detections = []
        # TODO: Implement based on YOLO output format
        # for r in results:
        #     boxes = r.boxes.xyxy.cpu().numpy()
        #     confidences = r.boxes.conf.cpu().numpy()
        #     class_ids = r.boxes.cls.cpu().numpy()
        #     for box, conf, cls_id in zip(boxes, confidences, class_ids):
        #         if self.model.names[int(cls_id)] == 'person' and conf > self.confidence_threshold:
        #             detections.append({'box': box, 'confidence': conf})
        return detections

    def select_target(self, detections):
        """Select which person to track (for multi-person scenarios)."""
        # TODO: Implement target selection logic
        # Options: largest bounding box, closest to center, tracked ID, etc.
        if len(detections) > 0:
            return detections[0]
        return None

    def calculate_position(self, detection, image_shape):
        """Calculate person position relative to camera center."""
        # TODO: Implement position calculation
        # Returns (horizontal_offset, vertical_offset, depth_estimate)
        return (0.0, 0.0, 0.0)

    def estimate_velocity(self, current_position):
        """Estimate person velocity from position history."""
        # TODO: Implement velocity estimation
        return None

    def draw_annotations(self, image, detections):
        """Draw bounding boxes and info on debug image."""
        annotated = image.copy()
        # TODO: Draw detections
        # Add crosshair at center
        h, w = image.shape[:2]
        cv2.drawMarker(annotated, (w//2, h//2), (0, 255, 0),
                       cv2.MARKER_CROSS, 20, 2)
        return annotated


def main(args=None):
    rclpy.init(args=args)
    node = PersonTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
