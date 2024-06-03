import rclpy
from rclpy.node import Node
from math import pi, sin, cos, sqrt
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped

class CupPublisher(Node):

    def __init__(self):
        super().__init__('cup_publisher')
        self.init_cup()
        
        self.input_listener = self.create_subscription(PointStamped, 'clicked_point', self.cup_request, 10)
        self.markers_publisher = self.create_publisher(MarkerArray, 'cup_pose', 10)
        # self.update_subscriber = self.create_subscription(, 'fill', 10)

    def init_cup(self):
        # cup
        self.cup_marker = Marker()
        self.cup_marker.header.frame_id = "base_link"
        self.cup_marker.header.stamp = self.get_clock().now().to_msg()
        self.cup_marker.type = Marker.CYLINDER
        self.cup_marker.id = 1
        self.cup_marker.action = Marker.ADD
        self.cup_marker.scale.x = 0.04
        self.cup_marker.scale.y = 0.04
        self.cup_marker.scale.z = 0.05
        self.cup_marker.color.r = 1.0
        self.cup_marker.color.g = 1.0
        self.cup_marker.color.b = 1.0
        self.cup_marker.color.a = 1.0
        
        self.cup_marker.pose.orientation.x = 0.0
        self.cup_marker.pose.orientation.y = 0.0
        self.cup_marker.pose.orientation.z = 0.0
        self.cup_marker.pose.orientation.w = 1.0

        self.fill_marker = Marker()
        self.fill_marker.header.frame_id = "base_link"
        self.fill_marker.header.stamp = self.get_clock().now().to_msg()
        self.fill_marker.type = Marker.CYLINDER
        self.fill_marker.id = 2
        self.fill_marker.action = Marker.ADD
        self.fill_marker.scale.x = 0.04
        self.fill_marker.scale.y = 0.04
        self.fill_marker.scale.z = 0.01
        self.fill_marker.color.r = 1.0
        self.fill_marker.color.g = 0.0
        self.fill_marker.color.b = 0.0
        self.fill_marker.color.a = 1.0
        
        self.fill_marker.pose.orientation.x = 0.0
        self.fill_marker.pose.orientation.y = 0.0
        self.fill_marker.pose.orientation.z = 0.0
        self.fill_marker.pose.orientation.w = 1.0

    def cup_request(self, pointMsg):
        self.fill_marker.pose.position.x = pointMsg.point.x
        self.fill_marker.pose.position.y = pointMsg.point.y
        self.fill_marker.pose.position.z = pointMsg.point.z + self.fill_marker.scale.z/2
        self.cup_marker.pose.position.x = pointMsg.point.x
        self.cup_marker.pose.position.y = pointMsg.point.y
        self.cup_marker.pose.position.z = self.fill_marker.pose.position.z + self.fill_marker.scale.z/2 + self.cup_marker.scale.z/2
        marker_array = MarkerArray()
        marker_array.markers.append(self.cup_marker)
        marker_array.markers.append(self.fill_marker)
        self.markers_publisher.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)

    cup_publisher = CupPublisher()

    rclpy.spin(cup_publisher)

    cup_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
