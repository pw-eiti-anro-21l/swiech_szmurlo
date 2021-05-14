import rclpy
from rclpy.node import Node
from zadanie5_interface.srv import ToolPosition
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from transforms3d.euler import euler2quat
from math import floor, pi
from rclpy.qos import QoSProfile
from time import sleep
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


class OpInterpolationServer(Node):

    def __init__(self):
        super().__init__('op_interpolation_server')
        self.srv = self.create_service(ToolPosition, 'op_interpolation', self.interpolation_callback)
        
        qos_profile = QoSProfile(depth=10)
        self.pose_publisher = self.create_publisher(PoseStamped, '/pose_ikin', qos_profile)

        self.initial_position = [0, 0, 0]

    def interpolation_callback(self, request, response):

        self.in_action = True

        pose = self.trapezoid_interpolation(request)

        return response
    

    def linear_interpolation(self, request):

        sample_time = 0.01
        steps = floor(request.interpolation_time/sample_time)
        pose = PoseStamped()

        initial_position = self.initial_position


        for step in range(steps + 1):

            pose_x = initial_position[0] + (request.x_goal - initial_position[0])/steps*step
            pose_y = initial_position[1] + (request.y_goal - initial_position[1])/steps*step
            pose_z = initial_position[2] + (request.z_goal - initial_position[2])/steps*step

            pose.header.frame_id = "base_link"
            pose.pose.position.x = pose_x
            pose.pose.position.y = pose_y
            pose.pose.position.z = pose_z

            sleep(sample_time)
            
            self.pose_publisher.publish(pose)

        self.initial_position = [pose_x, pose_y, pose_z]
            # return pose

        # self.initial_joint_states = [joint_1_state, joint_2_state, joint_3_state]


    def trapezoid_interpolation(self, request):
        sample_time = 0.01
        steps = floor(request.interpolation_time/sample_time)
        pose = PoseStamped()

        initial_position = self.initial_position

        max_vel_x = (request.x_goal - initial_position[0]) / (0.8*request.interpolation_time)
        max_vel_y = (request.y_goal - initial_position[1]) / (0.8*request.interpolation_time)
        max_vel_z = (request.z_goal - initial_position[2]) / (0.8*request.interpolation_time)

        last_vel_x = 0
        last_vel_y = 0
        last_vel_z = 0

        pose_x = initial_position[0]
        pose_y = initial_position[1]
        pose_z = initial_position[2]

        for step in range(steps + 1):
            if step < 0.2*steps:
                curr_vel_x = max_vel_x*step/(0.2*steps)
                curr_vel_y = max_vel_y*step/(0.2*steps)
                curr_vel_z = max_vel_z*step/(0.2*steps)

            elif step >= 0.2*steps and step <= 0.8*steps:
                
                curr_vel_x = max_vel_x
                curr_vel_y = max_vel_y
                curr_vel_z = max_vel_z

            elif step > 0.8 * steps:
                curr_vel_x = max_vel_x - max_vel_x * (step-0.8*steps)/(0.2*steps)
                curr_vel_y = max_vel_y - max_vel_y * (step-0.8*steps)/(0.2*steps)
                curr_vel_z = max_vel_z - max_vel_z * (step-0.8*steps)/(0.2*steps)

            pose_x = pose_x + (last_vel_x+curr_vel_x)*request.interpolation_time/steps
            pose_y = pose_y + (last_vel_y+curr_vel_y)*request.interpolation_time/steps
            pose_z = pose_z + (last_vel_z+curr_vel_z)*request.interpolation_time/steps

            pose.header.frame_id = "base_link"
            pose.pose.position.x = pose_x
            pose.pose.position.y = pose_y
            pose.pose.position.z = pose_z

            sleep(sample_time)
            self.pose_publisher.publish(pose)

        self.initial_position = [pose_x, pose_y, pose_z]


def main(args=None):
    rclpy.init(args=args)

    op_interpolation_server = OpInterpolationServer()

    rclpy.spin(op_interpolation_server)

    rclpy.shutdown()


if __name__ == '__main__':
    main()