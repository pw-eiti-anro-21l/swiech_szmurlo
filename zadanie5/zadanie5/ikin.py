import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState


class IKIN(Node):

    def __init__(self):
        super().__init__('ikin')
        self.srv = self.create_service(Pose, '/pose_stamped', self.service_callback)

        qos_profile = QoSProfile(depth=10)
        self.joint_publisher = self.create_publisher(JointState, '/joint_states', qos_profile)

    def service_callback(self, request, response):

        pose = request
        
        self.solve_inverse_kinematics(pose)
        self.get_logger()

        return response

    def solve_inverse_kinematics (self, pose):

        joint_states = JointState()
        joint_states.name = ['joint_base_1', 'joint_1_2', 'joint_2_3']
        
        pose_x = pose.pose.position.x
        pose_y = pose.pose.position.y
        pose_z = pose.pose.position.z

        joint_base_1_trans = pose_z + 1
        joint_1_2_trans = -pose_y
        joint_2_3_trans = pose_x

        joint_states.position = [float(joint_1_state), float(joint_2_state), float(joint_3_state)]
        self.joint_pub.publish(joint_states)


def main(args=None):
    rclpy.init(args=args)

    inv_kin_service = IKIN()

    rclpy.spin(inv_kin_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()