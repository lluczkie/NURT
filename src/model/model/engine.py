import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import JointState
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PointStamped
import yaml
import os
from math import atan2, copysign, pi, sqrt, acos, asin, cos, sin
from custom_interfaces.msg import Fill

class Engine(Node):

    def __init__(self):
        super().__init__('engine')
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.cup_subscription = self.create_subscription(PointStamped, 'clicked_point', self.cup_callback, 10)
        self.fill_publisher = self.create_publisher(Fill, 'fill', 10)

        self.cup_subscription
        
        project_path = os.path.dirname("NURT")
        file_path = project_path + "src/model/urdf/params.yaml"
        with open(file_path, 'r') as file:
            self.params = yaml.safe_load(file)
        self.servo = self.params['servo']
        self.bolt = self.params['bolt']
        self.nozzle = self.params['nozzle']
        self.leg = self.params['leg']
        self.cup_position = None

        self.j1_speed=0.15
        self.p1_speed=0.1
        self.period = 0.1
        self.state = 0
        self.filled = 0.0
        self.init_joints()
        self.timer = self.create_timer(self.period, self.go)

    def init_joints(self):
        self.jointMsg = JointState()
        self.jointMsg.header.frame_id = 'base_link'
        self.jointMsg.header.stamp = self.get_clock().now().to_msg()
        self.jointMsg.name = ["j1", "bolt_to_nozzle"]
        self.jointMsg.position = [0.0, 0.01]
        self.publisher.publish(self.jointMsg)

    def go(self):
        if self.state == 0:
            self.turn()
        if self.state == 1:
            if abs(self.cup_position[0] - self.jointMsg.position[0]) > self.j1_speed * self.period:
                self.turn()
            else:
                self.state = 2
        if self.state == 2:
            self.move_nozzle_to_target()
        if self.state == 3:
            self.fill()

        self.jointMsg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.jointMsg)
    
    def turn(self):
        self.jointMsg.position[0] += self.j1_speed * self.period
        self.jointMsg.position[0] %= (2*pi)

    def move_nozzle_to_target(self):
        diff = self.cup_position[1] - self.jointMsg.position[1]
        if abs(diff) > self.p1_speed * self.period:
            self.jointMsg.position[1] += copysign(self.p1_speed * self.period, diff)
        else:
            self.state = 3

    def cup_callback(self, pointMsg):
        x0 = pointMsg.point.x
        y0 = pointMsg.point.y
        [theta1, p1] = self.calculate_reverse_kinematic(x0, y0)

        if p1 < self.nozzle['l']/2 or p1 > self.bolt['l']-self.nozzle['l']/2-self.leg['x']:
            self.get_logger().warn("CUP outside of robot's workspace. Choose another location.")
            self.state = 0
        else:
            self.cup_position = [theta1, p1]
            self.state = 1

    def calculate_reverse_kinematic(self, x0, y0):
            theta1 = (atan2(y0, x0) + 2*pi) % (2*pi)
            x = x0-cos(theta1)*self.servo['x']/2
            y = y0-sin(theta1)*self.servo['x']/2
            p1 = sqrt(pow(x, 2) + pow(y, 2))
            return [theta1, p1]

    def fill(self):
        fillMsg = Fill()
        fillMsg.filled = self.filled
        self.fill_publisher.publish(fillMsg)
        if (0.9 - self.filled) <= 0.0001:
            self.filled = 0.0
            self.state = 4
        else:
            self.filled += 0.005

def main(args=None):
    rclpy.init(args=args)

    engine = Engine()

    rclpy.spin(engine)

    engine.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
