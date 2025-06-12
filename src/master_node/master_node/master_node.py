import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

class ConeDistanceVisualizer(Node):
    def __init__(self):
        super().__init__('cone_distance_visualizer')
        # Subscribe to the detected cones poses (adjust topic and msg type as needed)
        self.subscription = self.create_subscription(
            PoseArray,
            '/cone_detector/cones',
            self.cone_callback,
            10)
        # Publisher for visualization markers
        self.marker_pub = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        self.get_logger().info('Cone Distance Visualizer started.')

    def cone_callback(self, msg: PoseArray):
        marker_array = MarkerArray()
        for idx, pose in enumerate(msg.poses):
            # Calculate Euclidean distance
            x = pose.position.x
            y = pose.position.y
            z = pose.position.z
            dist = (x**2 + y**2 + z**2)**0.5

            # Sphere marker at cone position
            sphere = Marker()
            sphere.header = msg.header
            sphere.ns = 'cones'
            sphere.id = idx * 2
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose = pose
            sphere.scale.x = 0.2
            sphere.scale.y = 0.2
            sphere.scale.z = 0.2
            sphere.color.a = 1.0
            sphere.color.r = 1.0
            sphere.color.g = 0.0
            sphere.color.b = 0.0
            marker_array.markers.append(sphere)

            # Text marker with distance above cone
            text = Marker()
            text.header = msg.header
            text.ns = 'cone_distances'
            text.id = idx * 2 + 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose = pose
            text.pose.position.z += 0.3  # offset text above the cone
            text.scale.z = 0.15
            text.color.a = 1.0
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.text = f"{dist:.2f} m"
            marker_array.markers.append(text)

        # Publish all markers
        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = ConeDistanceVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
