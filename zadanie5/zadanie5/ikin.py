import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from math import cos, sin, atan, atan2, sqrt
from rclpy.qos import QoSProfile 
import json
import transformations
import mathutils
from ament_index_python.packages import get_package_share_directory

class IKIN(Node):

    def __init__(self):
        super().__init__('ikin')
        qos_profile = QoSProfile(depth=10)
        self.pose_subscriber = self.create_subscription(PoseStamped, '/pose_ikin', self.listener_callback, qos_profile)
        self.joint_publisher = self.create_publisher(JointState, '/joint_interpolate', qos_profile)
        self.link1_length = self.get_length()[0]["d"]
        self.link2_length = self.get_length()[1]["d"]
        self.link3_length = self.get_length()[2]["d"]
        self.tool_length = self.get_length()[3]["tool_length"]
        self.base_length = self.get_length()[3]["base_length"]

    def listener_callback(self, msg):
        self.solve_inverse_kinematics(msg)
        
    def solve_inverse_kinematics (self, pose):
        joint_states = JointState()
        joint_states.name = ['joint_base_1', 'joint_1_2', 'joint_2_3']
        
        pose_x = pose.pose.position.x
        pose_y = pose.pose.position.y
        pose_z = pose.pose.position.z

        joint_base_1_trans = pose_z - self.link1_length - self.base_length
        joint_1_2_trans = -self.link2_length - pose_y
        joint_2_3_trans = pose_x - self.link3_length - self.tool_length

        if joint_base_1_trans > 0 or joint_base_1_trans < -1*self.link1_length:
            self.get_logger().info("joint_base_1 cannot move further")
        elif joint_1_2_trans > 0 or joint_1_2_trans < -1*self.link2_length:
            self.get_logger().info("joint_1_2 cannot move further")
        elif joint_1_2_trans > 0 or joint_1_2_trans < -1*self.link3_length:
            self.get_logger().info("joint_2_3 cannot move further")
        else:
            joint_states.position = [float(joint_base_1_trans), float(joint_1_2_trans), float(joint_2_3_trans)]
            self.joint_publisher.publish(joint_states)

    def get_length(self):
        path = get_package_share_directory('zadanie5') + "/dh_params.json"
        with open(path, "r") as read_file:
            data = json.load(read_file)
        return data


def main(args=None):
    rclpy.init(args=args)
    inv_kin_subscriber = IKIN()
    rclpy.spin(inv_kin_subscriber)
    rclpy.shutdown()


if __name__ == '__main__':
    main()