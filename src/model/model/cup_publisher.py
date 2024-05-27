import rclpy
from rclpy.node import Node
from math import pi, sin, cos, sqrt
from visualization_msgs.msg import Marker
import os, yaml
import random

class CupPublisher(Node):

    def __init__(self):
        super().__init__('marker_publisher')
        project_path = os.path.dirname("anro_lab_luczkiewicz_inkielman")
        file_path = project_path + "src/model/urdf/params.yaml"
        with open(file_path, 'r') as file:
            self.params = yaml.safe_load(file)
        self.servo = self.params['servo']
        
        self.markers_publisher = self.create_publisher(Marker, 'cup_pose', 10)
        self.timer_period = 2
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        # cup
        cup_marker = Marker()
        cup_marker.header.frame_id = "base_link"
        cup_marker.header.stamp = self.get_clock().now().to_msg()
        cup_marker.type = Marker.CYLINDER
        cup_marker.id = 1
        cup_marker.action = Marker.ADD
        cup_marker.scale.x = 0.05
        cup_marker.scale.y = 0.05
        cup_marker.scale.z = 0.1
        cup_marker.color.r = 1.0
        cup_marker.color.g = 1.0
        cup_marker.color.b = 1.0
        cup_marker.color.a = 1.0
        cup_marker.pose.position.x, cup_marker.pose.position.y = self.find_two_coordinates(0.25, 0.0, 0.5)
        cup_marker.pose.position.z = 0.025

        beta = random.uniform(0.0, 2*pi)
        self.get_logger().info(f'page rotation: {beta}')
        axis = [0,0,1]

        cup_marker.pose.orientation.x = axis[0] * sin(0.5 * beta)
        cup_marker.pose.orientation.y = axis[1] * sin(0.5 * beta)
        cup_marker.pose.orientation.z = axis[2] * sin(0.5 * beta)
        cup_marker.pose.orientation.w = cos(0.5 * beta)
        self.markers_publisher.publish(cup_marker)

    def find_two_coordinates(self, x0, y0, scale=1): 
        return x0 + scale*random.choice([-1,1])*random.uniform(0.0,0.08), y0 + scale*random.choice([-1,1])*random.uniform(0,0.03)   

def main(args=None):
    rclpy.init(args=args)

    cup_publisher = CupPublisher()

    rclpy.spin(cup_publisher)

    cup_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
