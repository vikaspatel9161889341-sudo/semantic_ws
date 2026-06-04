import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from cv_bridge import CvBridge
import cv2
import numpy as np
import std_msgs.msg

class SemanticPerceptionNode(Node):
    def __init__(self):
        super().__init__('semantic_perception_node')
        # Publishers for both the 2D masked image and the 3D grid stream
        self.image_pub = self.create_publisher(Image, 'semantic_image', 10)
        self.pc_pub = self.create_publisher(PointCloud2, 'semantic_pointcloud', 10)
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # REPLACE WITH YOUR SMARTPHONE'S IP
        self.cap = cv2.VideoCapture('http://10.136.7.143:8080/video')
        self.bridge = CvBridge()
        self.get_logger().info('Semantic Perception Node Operational.')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        # Downsample image slightly to optimize frame rate for the VM
        frame = cv2.resize(frame, (320, 240))
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # SEMANTIC SEGMENTATION MASK (Thresholding for Floor/Obstacles)
        # Adjust these HSV values later to target your specific floor color!
        lower_floor = np.array([0, 0, 50])
        upper_floor = np.array([180, 50, 255])
        floor_mask = cv2.inRange(hsv, lower_floor, upper_floor)

        # Create a visual display frame for validation
        semantic_display = frame.copy()
        # Paint the safe flight zone bright GREEN
        semantic_display[floor_mask > 0] = [0, 255, 0]
        # Paint obstacles (everything else) bright RED
        semantic_display[floor_mask == 0] = [0, 0, 255]

        # Publish the 2D Semantic Image
        msg = self.bridge.cv2_to_imgmsg(semantic_display, encoding="bgr8")
        self.image_pub.publish(msg)

        # GENERATE 3D MATHEMATICAL POINT CLOUD FOR THE RVIZ GRID
        self.publish_point_cloud(floor_mask)

    def publish_point_cloud(self, mask):
        SCALE_X = 0.005  # Stretches the width (Left/Right)
        SCALE_Y = 0.01   # Stretches the depth (Forward/Backward)
        CAMERA_Z = -1.0
        # Generate a mathematical 3D grid representation of the mask
        points = []
        h, w = mask.shape
        
        # Look down at the floor space in front of the drone
        for y in range(0, h, 4):  
            for x in range(0, w, 4):
                # Apply the calibration variables
                x_3d = (x - w/2) * SCALE_X
                y_3d = (h - y) * SCALE_Y
                z_3d = CAMERA_Z  # Assumed floor depth
                
                # Color code points based on safety status
                if mask[y, x] > 0:
                    rgb = (0 << 16) | (255 << 8) | 0  # Green: Safe to fly
                else:
                    rgb = (255 << 16) | (0 << 8) | 0  # Red: Obstacle

                # Pack spatial coordinates and color bitwise
                points.append([x_3d, y_3d, z_3d, rgb])

        # Pack data into an official ROS 2 binary PointCloud2 byte stream
        header = std_msgs.msg.Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'  # Anchoring to the global RViz grid coordinate

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1)
        ]

        pc2 = PointCloud2()
        pc2.header = header
        pc2.height = 1
        pc2.width = len(points)
        pc2.fields = fields
        pc2.is_bigendian = False
        pc2.point_step = 16
        pc2.row_step = 16 * len(points)
        pc2.is_dense = True
        
        # Convert floating array to raw binary buffer
        buffer = []
        for p in points:
            buffer.append(np.array([p[0], p[1], p[2]], dtype=np.float32).tobytes())
            buffer.append(np.array(p[3], dtype=np.int32).tobytes())
        pc2.data = b''.join(buffer)

        self.pc_pub.publish(pc2)

def main(args=None):
    rclpy.init(args=args)
    node = SemanticPerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
