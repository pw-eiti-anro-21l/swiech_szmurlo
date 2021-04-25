import rclpy
from rclpy.node import Node

import json
import math
from math import cos, sin, atan, atan2, sqrt
import json
import transformations
import mathutils
import rclpy
import os
from rclpy.clock import ROSClock
import time


from std_msgs.msg import String
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState



class NONKDL_DKIN(Node):

    def __init__(self):
        super().__init__('NONKDL_DKIN')
        self.publisher = self.create_publisher(PoseStamped, '/nonkdl_fk', 10)
        self.subsciber = self.create_subscription(JointState, 'joint_states', self.listener_callback, 10)
        
    def listener_callback(self, msg):
        pose = self.solve_forward_kinematics(msg, 0.2)
        self.publisher.publish(pose)


    def get_xyz_rpy(self, msg):
        with open("/home/piotr/anro1_ws/src/swiech_szmurlo/zadanie3/zadanie3/dh_params.json", "r") as file:
            dh_params = json.load(file)
        rpy_xyz={}
        inter = 1
        iterator = 1

        xyz_array = []
        rpy_array = []
        params_array = []
        trans_matrix_list = []

        msg_pos_it = 0

        for i in dh_params:
            dh_row = json.loads(json.dumps(i))

            a_translation = mathutils.Matrix.Translation((dh_row["a"], 0, 0))
            d_translation = mathutils.Matrix.Translation((0, 0, dh_row["d"]+msg.position[msg_pos_it]))
            alpha_rotation = mathutils.Matrix.Rotation (dh_row["alpha"], 4, 'X')
            theta_rotation = mathutils.Matrix.Rotation (dh_row["theta"], 4, 'Z')
            
            trans_matrix = a_translation @ alpha_rotation @ d_translation @ theta_rotation
            rpy = transformations.euler_from_matrix(trans_matrix)
            xyz = transformations.translation_from_matrix(trans_matrix)

            params_array.append({'xyz': xyz, 'rpy': rpy, 'd': dh_row['d']})
            xyz_array.append(xyz)
            rpy_array.append(rpy)
            trans_matrix_list.append(trans_matrix)

            base_tool_matrix = trans_matrix_list[0]
        
            for i in range(1, len(trans_matrix_list)):
                base_tool_matrix = base_tool_matrix @ trans_matrix_list[i]

            msg_pos_it += 1
        return base_tool_matrix


    def solve_forward_kinematics(self, msg, tool_length):

        transformation_matrix = self.get_xyz_rpy(msg)
        xyz = transformation_matrix.to_translation()
        euler_angles = transformation_matrix.to_euler()
        base_tool_quaternion = euler_angles.to_quaternion()
        print (base_tool_quaternion)

        pose = PoseStamped()
        now = self.get_clock().now()
        pose.header.stamp = ROSClock().now().to_msg()
        pose.header.frame_id = "base_link"
        

        pose.pose.position.x = xyz[0]+tool_length/2
        pose.pose.position.y = xyz[1]
        pose.pose.position.z = xyz[2]+1

        pose.pose.orientation.x = base_tool_quaternion[0] 
        pose.pose.orientation.y = base_tool_quaternion[1] 
        pose.pose.orientation.z = base_tool_quaternion[2] 
        pose.pose.orientation.w = base_tool_quaternion[3] 

        print ("X")
        print (pose.pose.position.x)
        print ("Y")
        print (pose.pose.position.y)
        print ("Z")
        print (pose.pose.position.z)
        
        return pose




def main(args=None):

    rclpy.init(args=args)
    nonkdl_dkin = NONKDL_DKIN()

    rclpy.spin(nonkdl_dkin)

    nonkdl_dkin.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
