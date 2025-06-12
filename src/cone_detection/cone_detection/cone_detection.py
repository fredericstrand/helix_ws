import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from visualization_msgs.msg import MarkerArray, Marker
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np
from ultralytics import YOLO

class ConeNode(Node):
    def __init__(self):
        super().__init__('cone_node')
        self.bridge = CvBridge()

        # YOLO model setup
        self.model = YOLO('best.pt')
        self.conf_threshold = 0.25

        # Latest depth map
        self.depth_map = None

        # Subscribers
        self.create_subscription(
            Image,
            '/zed/zed_node/left/image_rect_color',
            self.image_callback,
            10
        )
        self.create_subscription(
            Image,
            '/zed/zed_node/depth/depth_registered',
            self.depth_callback,
            10
        )

        # Publisher for cone markers
        self.marker_pub = self.create_publisher(MarkerArray, 'cone_markers', 10)
        self.get_logger().info("ConeNode with YOLO initialized")

    def depth_callback(self, msg: Image):
        # Convert ROS depth Image to numpy array (in meters)
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        self.depth_map = np.array(depth, copy=False)

    def image_callback(self, msg: Image):
        # Convert ROS image to OpenCV
        cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        img_display = cv_img.copy()
        depth_display = cv.cvtColor(cv_img, cv.COLOR_BGR2GRAY)

        # Run YOLO detection
        results = self.model(cv_img, conf=self.conf_threshold)
        detections = results[0].boxes.data.cpu().numpy()

        markers = MarkerArray()
        marker_id = 0
        for det in detections:
            x1, y1, x2, y2, confidence, class_id = det.astype(float)
            x1, y1, x2, y2 = map(int, (x1, y1, x2, y2))
            # Determine label
            if hasattr(self.model, 'names') and self.model.names:
                label = self.model.names[int(class_id)]
            else:
                label = f"class_{int(class_id)}"

            # Depth ROI
            if self.depth_map is not None:
                roi = self.depth_map[y1:y2, x1:x2]
                valid = roi[(roi > 0) & (roi < 40)]
                if valid.size:
                    avg_depth = float(np.mean(valid))
                    display_label = f"{label}: {avg_depth:.2f}m ({confidence:.2f})"
                else:
                    display_label = f"{label}: Unknown ({confidence:.2f})"
            else:
                display_label = f"{label}: No depth ({confidence:.2f})"

            # Draw boxes & labels
            cv.rectangle(img_display, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv.putText(img_display, display_label, (x1, y1 - 10),
                       cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv.rectangle(depth_display, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # Create and append Marker for RViz
            m = Marker()
            m.header = msg.header
            m.id = marker_id
            m.type = Marker.CUBE
            m.action = Marker.ADD

            m.scale.x = m.scale.y = m.scale.z = 0.2
            m.color.r = 1.0
            m.color.g = 0.5
            m.color.a = 0.8
            markers.markers.append(m)
            marker_id += 1

        # Publish markers
        self.marker_pub.publish(markers)

        # Display windows
        cv.imshow("Object Detection with ZED", img_display)
        cv.imshow("Depth Display", depth_display)
        cv.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ConeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    cv.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
