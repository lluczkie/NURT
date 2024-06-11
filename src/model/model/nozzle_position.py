import rclpy
from rclpy.node import Node
from math import sin, cos
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped
import os, yaml

class NozzlePosition(Node):

    def __init__(self):
        super().__init__('nozzle_position_publisher')
        
        self.joint_listener = self.create_subscription(JointState, 'joint_states', self.calculate_nozzle_position, 10)
        self.nozzle_publisher = self.create_publisher(PointStamped, 'nozzle_position', 10)

        project_path = os.path.dirname("NURT")
        file_path = project_path + "src/model/urdf/params.yaml"
        with open(file_path, 'r') as file:
            self.params = yaml.safe_load(file)
        self.servo = self.params['servo']

    def calculate_nozzle_position(self, jointMsg):
        theta1 = jointMsg.position[0]
        r0 = jointMsg.position[1] + self.servo['x']/2 + self.servo['z']
        x = cos(theta1)*r0
        y = sin(theta1)*r0
        nozzlePos = PointStamped()
        nozzlePos.header.stamp = jointMsg.header.stamp
        nozzlePos.header.frame_id = 'base_link'
        nozzlePos.point.x = x
        nozzlePos.point.y = y
        nozzlePos.point.z = self.servo['z'] + self.servo['x']/2
        self.nozzle_publisher.publish(nozzlePos)
    
def main(args=None):
    rclpy.init(args=args)

    nozzle_position = NozzlePosition()

    rclpy.spin(nozzle_position)

    nozzle_position.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
