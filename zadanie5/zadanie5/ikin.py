import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from math import cos, sin, atan, atan2, sqrt
from rclpy.qos import QoSProfile 
import json
import transformations
import mathutils


class IKIN(Node):

    def __init__(self):
        super().__init__('ikin')
        qos_profile = QoSProfile(depth=10)
        self.pose_subscriber = self.create_subscription(PoseStamped, '/pose_ikin', self.listener_callback, qos_profile)
        self.joint_publisher = self.create_publisher(JointState, '/joint_interpolate', qos_profile)

    def listener_callback(self, msg):

        joint_states = self.solve_inverse_kinematics(msg)
        self.get_logger().info("dapu")
        self.joint_publisher.publish(joint_states)

    def solve_inverse_kinematics (self, pose):
        joint_states = JointState()
        joint_states.name = ['joint_base_1', 'joint_1_2', 'joint_2_3']
        
        pose_x = pose.pose.position.x
        pose_y = pose.pose.position.y
        pose_z = pose.pose.position.z

        # pose_x = 0.5 #(0,1)
        # pose_y = -0.5 #(-1,0)
        # pose_z = 2 #(1,2)

        joint_base_1_trans = pose_z - 2
        joint_1_2_trans = -1 - pose_y
        joint_2_3_trans = pose_x - 1
        # joint_base_1_trans = -1.
        # joint_1_2_trans = -1.
        # joint_2_3_trans = -1.
        joint_states.position = [float(joint_base_1_trans), float(joint_1_2_trans), float(joint_2_3_trans)]

        # if joint_base_1_trans < 0 or joint_base_1_trans > 1:
        #     pass
        #     # raise Exception("Niemożliwe do zrealizowania położenie stawu base -> 1")
        
        # if joint_1_2_trans < 0 or joint_1_2_trans > 1:
        #     pass
        #     # raise Exception("Niemożliwe do zrealizowania położenie stawu 1 -> 2")

        # if joint_2_3_trans < -1 or joint_2_3_trans > 0:
        #     pass
        #     # raise Exception("Niemożliwe do zrealizowania położenie stawu 2 -> 3")

        # else:
            
        return joint_states


def main(args=None):
    rclpy.init(args=args)

    inv_kin_subscriber = IKIN()

    rclpy.spin(inv_kin_subscriber)

    rclpy.shutdown()


if __name__ == '__main__':
    main()