import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped
import yaml
import os
from math import atan2, copysign, pi, sqrt, acos, asin, cos, sin

class Engine(Node):

    def __init__(self):
        super().__init__('engine')
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.cup_subscription = self.create_subscription(Marker, 'cup_pose', self.cup_callback, 10)
        self.nozzle_subscription = self.create_subscription(PointStamped, 'nozzle_position', self.get_nozzle_position, 10)

        self.cup_subscription
        self.nozzle_subscription  # prevent unused variable warning
        project_path = os.path.dirname("NURT")
        file_path = project_path + "src/model/urdf/params.yaml"
        with open(file_path, 'r') as file:
            self.params = yaml.safe_load(file)
        self.servo = self.params['servo']
        self.bolt = self.params['bolt']
        self.nozzle = self.params['nozzle']
        self.leg = self.params['leg']

        self.init_joints()
        self.iter = 0
        self.state = 0
        self.speed = 500
        self.timer = self.create_timer(0.01, self.update_joints)

    def init_joints(self):
        self.jointMsg = JointState()
        self.jointMsg.header.frame_id = 'base_link'
        self.jointMsg.header.stamp = self.get_clock().now().to_msg()
        self.jointMsg.name = ["j1", "bolt_to_nozzle"]
        self.jointMsg.position = [0.0, 0.01]
        self.publisher.publish(self.jointMsg)

    def update_joints(self):
        self.jointMsg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.jointMsg)

    def get_nozzle_position(self, pointMsg):
        self.nozzle_position = [pointMsg.point.x, pointMsg.point.y, pointMsg.point.z]

    def cup_callback(self, markerMsg):
        x0 = markerMsg.pose.position.x
        y0 = markerMsg.pose.position.y
        z0 = markerMsg.pose.position.z

        len=sqrt(pow(x0, 2) + pow(y0, 2))-self.servo['x']/2
        if len < self.nozzle['l']/2 or len > self.bolt['l']-self.nozzle['l']/2-self.leg['x']:
            self.get_logger().warn(f"cannot reach position: \n\tx: {x0}\n\ty: {y0}\n\tz: {z0}\n")
        else:
            current_position = self.nozzle_position
            goal_position = [x0, y0, z0]
            goal_joints = self.calculate_reverse_kinematic(goal_position[0], goal_position[1])
            difference_vector = self.get_two_point_difference(current_position, goal_position)
            difference = 0
            for i in range(2):
                delta_i = difference_vector[i]
                difference += delta_i * delta_i
            difference = sqrt(difference)
            if abs(difference) < 0.0001:
                # reached target tolerance  
                self.jointMsg.position = goal_joints
                self.iter = 0
                self.state = 0     
            else:
                target_x = float(current_position[0] + (difference_vector[0]) / max(2, (self.speed - self.iter)))
                target_y = float(current_position[1] + (difference_vector[1]) / max(2, (self.speed - self.iter)))
                self.iter += 1
                position = self.calculate_reverse_kinematic(target_x, target_y)
                if self.state == 0:
                    self.speed = 500
                    self.jointMsg.position[0] = position[0]
                    if abs(self.jointMsg.position[0] - goal_joints[0]) < 0.0001:
                        self.state = 1
                if self.state == 1:
                    self.speed = 1000
                    self.jointMsg.position = position

    def calculate_reverse_kinematic(self, x0, y0):
            theta1 = atan2(y0, x0)
            x = x0-cos(theta1)*self.servo['x']/2
            y = y0-sin(theta1)*self.servo['x']/2
            p1 = sqrt(pow(x, 2) + pow(y, 2))
            return [theta1, p1]

    def get_two_point_difference(self, point1, point2):
        y = point2[1] - point1[1] 
        z = point2[2] - point1[2]
        x = point2[0] - point1[0]
        difference = [x, y, z]
        return difference 

def main(args=None):
    rclpy.init(args=args)

    engine = Engine()

    rclpy.spin(engine)

    engine.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
