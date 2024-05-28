import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped, PoseStamped
import yaml
import os
from math import atan2, copysign, pi, sqrt, acos, asin, cos, sin

class Engine(Node):

    def __init__(self):
        super().__init__('engine')
        self.publisher = self.create_publisher(JointState, 'joint_states', 0)
        self.subscription = self.create_subscription(PointStamped, 'clicked_point', self.point_callback, 0)
        self.subscription  # prevent unused variable warning
        project_path = os.path.dirname("NURT")
        file_path = project_path + "src/model/urdf/params.yaml"
        with open(file_path, 'r') as file:
            self.params = yaml.safe_load(file)
        self.servo = self.params['servo']

        self.init_joints()
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

    def point_callback(self, pointMsg):
        x0 = pointMsg.point.x
        y0 = pointMsg.point.y
        z0 = pointMsg.point.z

        try:
            theta1 = atan2(y0, x0)
            p1 = sqrt(pow(x0-cos(theta1)*self.servo['x']/2, 2) + pow(y0-sin(theta1)*self.servo['x']/2, 2))
            self.jointMsg.position = [theta1, p1]
        except ValueError:
            self.get_logger().warn(f"cannot reach position: \n\tx: {x0}\n\ty: {y0}\n\tz: {z0}\n")

def main(args=None):
    rclpy.init(args=args)

    engine = Engine()

    rclpy.spin(engine)

    engine.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
