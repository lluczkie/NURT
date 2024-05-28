import rclpy
from rclpy.node import Node
from math import pi, sin, cos, sqrt
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped
import os, yaml
import random

class CupPublisher(Node):

    def __init__(self):
        super().__init__('marker_publisher')
        
        self.input_listener = self.create_subscription(PointStamped, 'clicked_point', self.cup_request, 0)
        self.markers_publisher = self.create_publisher(Marker, 'cup_pose', 10)

    def cup_request(self, pointMsg):
        # cup
        cup_marker = Marker()
        cup_marker.header.frame_id = "base_link"
        cup_marker.header.stamp = self.get_clock().now().to_msg()
        cup_marker.type = Marker.CYLINDER
        cup_marker.id = 1
        cup_marker.action = Marker.ADD
        cup_marker.scale.x = 0.04
        cup_marker.scale.y = 0.04
        cup_marker.scale.z = 0.05
        cup_marker.color.r = 1.0
        cup_marker.color.g = 1.0
        cup_marker.color.b = 1.0
        cup_marker.color.a = 1.0
        
        beta = random.uniform(-pi, pi)
        
        cup_marker.pose.position.x = pointMsg.point.x
        cup_marker.pose.position.y = pointMsg.point.y
        cup_marker.pose.position.z = pointMsg.point.z + cup_marker.scale.z/2

        
        cup_marker.pose.orientation.x = 0.0
        cup_marker.pose.orientation.y = 0.0
        cup_marker.pose.orientation.z = 0.0
        cup_marker.pose.orientation.w = 1.0

        self.markers_publisher.publish(cup_marker)


def main(args=None):
    rclpy.init(args=args)

    cup_publisher = CupPublisher()

    rclpy.spin(cup_publisher)

    cup_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
