import rclpy
from rclpy.node import Node
from math import pi, sin, cos, sqrt
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped
from custom_interfaces.msg import Fill

class CupPublisher(Node):

    def __init__(self):
        super().__init__('cup_publisher')
        
        self.input_listener = self.create_subscription(PointStamped, 'clicked_point', self.new_cup, 10)
        self.markers_publisher = self.create_publisher(MarkerArray, 'cup_pose', 10)
        self.fill_subscriber = self.create_subscription(Fill, 'fill', self.fill_cup, 10)
        self.cup_height = 0.05
        self.z = 0
        self.init_cup()

    def init_cup(self):
        # cup
        cup_marker = Marker()
        cup_marker.header.frame_id = "base_link"
        cup_marker.header.stamp = self.get_clock().now().to_msg()
        cup_marker.type = Marker.CYLINDER
        cup_marker.id = 1
        cup_marker.action = Marker.ADD
        cup_marker.scale.x = 0.04
        cup_marker.scale.y = 0.04
        cup_marker.scale.z = self.cup_height
        cup_marker.color.r = 1.0
        cup_marker.color.g = 1.0
        cup_marker.color.b = 1.0
        cup_marker.color.a = 1.0
        
        cup_marker.pose.orientation.x = 0.0
        cup_marker.pose.orientation.y = 0.0
        cup_marker.pose.orientation.z = 0.0
        cup_marker.pose.orientation.w = 1.0

        fill_marker = Marker()
        fill_marker.header.frame_id = "base_link"
        fill_marker.header.stamp = self.get_clock().now().to_msg()
        fill_marker.type = Marker.CYLINDER
        fill_marker.id = 2
        fill_marker.action = Marker.ADD
        fill_marker.scale.x = 0.04
        fill_marker.scale.y = 0.04
        fill_marker.scale.z = 0.0
        fill_marker.color.r = 1.0
        fill_marker.color.g = 0.0
        fill_marker.color.b = 0.0
        fill_marker.color.a = 1.0
        
        fill_marker.pose.orientation.x = 0.0
        fill_marker.pose.orientation.y = 0.0
        fill_marker.pose.orientation.z = 0.0
        fill_marker.pose.orientation.w = 1.0

        self.marker_array = MarkerArray()
        self.marker_array.markers.append(cup_marker)
        self.marker_array.markers.append(fill_marker)

    def new_cup(self, pointMsg):
        fill_marker = self.marker_array.markers[1]
        cup_marker = self.marker_array.markers[0]

        fill_marker.pose.position.x = pointMsg.point.x
        fill_marker.pose.position.y = pointMsg.point.y
        fill_marker.pose.position.z = pointMsg.point.z + fill_marker.scale.z/2
        fill_marker.scale.z = 0.0

        cup_marker.pose.position.x = pointMsg.point.x
        cup_marker.pose.position.y = pointMsg.point.y
        cup_marker.pose.position.z = self.marker_array.markers[1].pose.position.z + fill_marker.scale.z/2 + cup_marker.scale.z/2
        cup_marker.scale.z = self.cup_height

        self.z = pointMsg.point.z
        self.markers_publisher.publish(self.marker_array)

    def fill_cup(self, fillMsg):
        fill_marker = self.marker_array.markers[1]
        cup_marker = self.marker_array.markers[0]

        fill_marker.scale.z = self.cup_height*fillMsg.filled
        cup_marker.scale.z = self.cup_height*(1-fillMsg.filled)
        fill_marker.pose.position.z = self.z + fill_marker.scale.z/2
        cup_marker.pose.position.z = fill_marker.pose.position.z + fill_marker.scale.z/2 + cup_marker.scale.z/2
        self.markers_publisher.publish(self.marker_array)

def main(args=None):
    rclpy.init(args=args)

    cup_publisher = CupPublisher()

    rclpy.spin(cup_publisher)

    cup_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
