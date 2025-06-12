#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from my_package.msg import DetectedCone, DetectedConeArray
import message_filters
from cv_bridge import CvBridge
import torch

class ConeDetectorNode(Node):
    def __init__(self):
        super().__init__('cone_detector')
        self.bridge = CvBridge()
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path='best.pt')
        self.model.conf = 0.4
        img_sub = message_filters.Subscriber(self, Image, '/zed/left/image_rect_color')
        depth_sub = message_filters.Subscriber(self, Image, '/zed/depth/depth_registered')
        sync = message_filters.ApproximateTimeSynchronizer([img_sub, depth_sub], queue_size=10, slop=0.1)
        sync.registerCallback(self.frame_callback)
        self.pub = self.create_publisher(DetectedConeArray, 'cones', 10)
        self.get_logger().info("ConeDetectorNode ready.")

    def frame_callback(self, img_msg: Image, depth_msg: Image):
        frame = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, '32FC1')
        results = self.model(frame)
        dets = results.xyxy[0]
        cones = DetectedConeArray(header=Header(stamp=self.get_clock().now().to_msg()))
        for *box, conf, cls in dets.cpu().numpy():
            x1, y1, x2, y2 = map(int, box)
            cx, cy = (x1 + x2)//2, (y1 + y2)//2
            distance = float(depth[cy, cx])
            cone = DetectedCone()
            cone.bbox.x = x1
            cone.bbox.y = y1
            cone.bbox.width = x2 - x1
            cone.bbox.height = y2 - y1
            cone.confidence = float(conf)
            cone.distance = distance
            cones.cones.append(cone)
        self.pub.publish(cones)
        self.get_logger().info(f"Published {len(cones.cones)} cones.")

def main(args=None):
    rclpy.init(args=args)
    node = ConeDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
