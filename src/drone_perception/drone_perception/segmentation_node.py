#!/usr/bin/env python3
import rclpy
import os
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory

class SegmentationNode(Node):
    def __init__(self):
        super().__init__('segmentation_node')
        
        # 1. Get Path
        package_share_directory = get_package_share_directory('drone_perception')
        model_path = os.path.join(package_share_directory, 'models', 'yolov8n-seg.onnx')
        
        # 2. DEBUG: List directory contents
        self.get_logger().info(f"Looking in directory: {os.path.dirname(model_path)}")
        try:
            files = os.listdir(os.path.dirname(model_path))
            self.get_logger().info(f"Files found in directory: {files}")
        except Exception as e:
            self.get_logger().error(f"Cannot list directory: {e}")

        # 3. Validation
        if not os.path.exists(model_path):
            self.get_logger().error(f"FATAL: Model not found at {model_path}")
            raise FileNotFoundError(f"Model file not found at: {model_path}")
            
        self.get_logger().info(f"Successfully validated model at: {model_path}")
        self.model = YOLO(model_path)
        # 4. ROS 2 Infrastructure
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/cam0/image_raw', self.image_callback, 10)
        self.pub = self.create_publisher(Image, '/semantic_image', 10)
        
        self.get_logger().info("Segmentation Node Initialized.")

    def image_callback(self, msg):
        # Convert ROS image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Perform inference
        results = self.model(cv_image, verbose=False)
        annotated_frame = results[0].plot()
        
        # Convert back to ROS image
        processed_msg = self.bridge.cv2_to_imgmsg(annotated_frame, 'bgr8')
        processed_msg.header = msg.header
        self.pub.publish(processed_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SegmentationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
