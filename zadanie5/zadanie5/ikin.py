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
        self.pose_subscriber = self.create_subscription(PoseStamped, '/pose_ikin', self.listener_callback)

        qos_profile = QoSProfile(depth=10)
        self.joint_publisher = self.create_publisher(JointState, '/joint_interpolate', qos_profile)

    def listener_callback(self, request, response):

        pose = request
        
        self.solve_inverse_kinematics(pose)
        self.get_logger().info("dupa")

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

        if joint_base_1_trans < 0 or joint_base_1_trans > 1:
            pass
            # raise Exception("Niemożliwe do zrealizowania położenie stawu base -> 1")
        
        if joint_1_2_trans < 0 or joint_1_2_trans > 1:
            pass
            # raise Exception("Niemożliwe do zrealizowania położenie stawu 1 -> 2")

        if joint_2_3_trans < -1 or joint_2_3_trans > 0:
            pass
            # raise Exception("Niemożliwe do zrealizowania położenie stawu 2 -> 3")

        else:
            joint_states.position = [float(joint_base_1_trans), float(joint_1_2_trans), float(joint_2_3_trans)]
            self.joint_pub.publish(joint_states)



    # def get_xyz_rpy(self, msg):
    #     with open("/home/piotr/anro1_ws/src/swiech_szmurlo/zadanie3/zadanie3/dh_params.json", "r") as file:
    #         dh_params = json.load(file)
    #         xyz_array = []
    #         rpy_array = []
    #         d_translations = []
    #         params_array = []
    #         trans_matrix_list = []
    #         msg_pos_it = 0
    #         for i in dh_params:
    #             dh_row = json.loads(json.dumps(i))
    #             a_translation = mathutils.Matrix.Translation((dh_row["a"], 0, 0))
    #             d_translation = mathutils.Matrix.Translation((0, 0, dh_row["d"]+msg.position[msg_pos_it]))
    #             alpha_rotation = mathutils.Matrix.Rotation (dh_row["alpha"], 4, 'X')
    #             theta_rotation = mathutils.Matrix.Rotation (dh_row["theta"], 4, 'Z')
    #             d_translations.append(d_translation)

    #             trans_matrix = a_translation @ alpha_rotation @ d_translation @ theta_rotation
    #             rpy = transformations.euler_from_matrix(trans_matrix)
    #             xyz = transformations.translation_from_matrix(trans_matrix)
    #             params_array.append({'xyz': xyz, 'rpy': rpy, 'd': dh_row['d']})
    #             xyz_array.append(xyz)
    #             rpy_array.append(rpy)
    #             trans_matrix_list.append(trans_matrix)
    #             base_tool_matrix = trans_matrix_list[0]
    #             for i in range(1, len(trans_matrix_list)):
    #                 base_tool_matrix = base_tool_matrix @ trans_matrix_list[i]
    #             msg_pos_it += 1
    #         return d_translations


def main(args=None):
    rclpy.init(args=args)
<<<<<<< HEAD
    inv_kin_service = IKIN()
    rclpy.spin(inv_kin_service)
=======

    inv_kin_subscriber = IKIN()

    rclpy.spin(inv_kin_subscriber)

>>>>>>> 11e2178d50188d08d87f1bd7b6b7911398407e9d
    rclpy.shutdown()


if __name__ == '__main__':
    main()