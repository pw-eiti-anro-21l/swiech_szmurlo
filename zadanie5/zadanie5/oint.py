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
        self.initial_orientation = [0, 0, 0]

        # self.subsciber = self.create_subscription(JointState, 'joint_states', self.listener_callback, 10)
        self.in_action = False

    # def listener_callback(self, msg):
    #     if not self.in_action:
    #         self.initial_position[0] = msg.position[0]
    #         self.initial_position[1] = msg.position[1]
    #         self.initial_position[2] = msg.position[2]


    def interpolation_callback(self, request, response):

        self.in_action = True

        if request.interpolation_time <= 0:
            response.server_feedback = "Given interpolation time < 0. Defaulting to 5s"
            request.interpolation_time = 5
        
        if request.version != "std" and request.version != "ext":
            response.server_feedback = "Given version is wrong. Defaulting to standart"
            request.version = "std"

        if request.method != "linear" and request.method != "trapezoid":
            request.method = "linear"
            self.get_logger().info("Given interpolation method is wrong. Defaulting to linear")

        if request.method == "linear":
            pose = self.linear_interpolation(request)
            response.server_feedback = "Interpolation completed"

        elif request.method == "trapezoid":
            pose = self.trapezoid_interpolation(request)
            response.server_feedback = "Interpolation completed"

        # self.pose_publisher.publish(pose)
        return response
    

    def linear_interpolation(self, request):

        sample_time = 0.01
        steps = floor(request.interpolation_time/sample_time)
        pose = PoseStamped()

        initial_position = self.initial_position
        initial_orientation = self.initial_orientation


        for step in range(steps + 1):

            pose_x = initial_position[0] + (request.x_goal - initial_position[0])/steps*step
            pose_y = initial_position[1] + (request.y_goal - initial_position[1])/steps*step
            pose_z = initial_position[2] + (request.z_goal - initial_position[2])/steps*step

            ort_roll = initial_orientation[0] + (request.roll_goal - initial_orientation[0])/steps*step
            ort_pitch = initial_orientation[1] + (request.pitch_goal - initial_orientation[1])/steps*step
            ort_yaw = initial_orientation[2] + (request.yaw_goal - initial_orientation[2])/steps*step
            ort_quaternion = Quaternion(w=0.0, x=ort_roll, y=ort_pitch, z=ort_yaw)

            roll_rad = ort_roll * pi/180
            pitch_rad = ort_pitch * pi/180
            yaw_rad = ort_yaw * pi/180

            quat = euler2quat (roll_rad, pitch_rad, yaw_rad)
            
            if request.version == "ext":
                ort_quaternion = Quaternion(w=0.0, x=roll_rad, y=pitch_rad, z=yaw_rad)
            else:
                ort_quaternion = Quaternion(w=quat[0], x=quat[1], y=quat[2], z=quat[3])

            pose.header.frame_id = "base_link"
            pose.pose.position.x = pose_x
            pose.pose.position.y = pose_y
            pose.pose.position.z = pose_z
            pose.pose.orientation = ort_quaternion

            sleep(sample_time)
            
            self.pose_publisher.publish(pose)

        self.initial_position = [pose_x, pose_y, pose_z]
        self.initial_orientation = [ort_roll, ort_pitch, ort_yaw]

            # return pose

        # self.initial_joint_states = [joint_1_state, joint_2_state, joint_3_state]


    def trapezoid_interpolation(self, request):
        sample_time = 0.01
        steps = floor(request.interpolation_time/sample_time)
        pose = PoseStamped()

        initial_position = self.initial_position
        initial_orientation = self.initial_orientation

        max_vel_x = (request.x_goal - initial_position[0]) / (0.8*request.interpolation_time)
        max_vel_y = (request.y_goal - initial_position[1]) / (0.8*request.interpolation_time)
        max_vel_z = (request.z_goal - initial_position[2]) / (0.8*request.interpolation_time)

        max_vel_roll = (request.roll_goal - initial_orientation[0]) / (0.8*request.interpolation_time)
        max_vel_pitch = (request.pitch_goal - initial_orientation[1]) / (0.8*request.interpolation_time)
        max_vel_yaw = (request.yaw_goal - initial_orientation[2]) / (0.8*request.interpolation_time)

        last_vel_x = 0
        last_vel_y = 0
        last_vel_z = 0

        last_vel_roll = 0
        last_vel_pitch = 0
        last_vel_yaw = 0

        pose_x = initial_position[0]
        pose_y = initial_position[1]
        pose_z = initial_position[2]

        ort_roll = initial_orientation[0]
        ort_pitch = initial_orientation[1]
        ort_yaw = initial_orientation[2]

        for step in range(steps + 1):
            if step < 0.2*steps:
                curr_vel_x = max_vel_x*step/(0.2*steps)
                curr_vel_y = max_vel_y*step/(0.2*steps)
                curr_vel_z = max_vel_z*step/(0.2*steps)

                curr_vel_roll = max_vel_roll*step/(0.2*steps)
                curr_vel_pitch = max_vel_pitch*step/(0.2*steps)
                curr_vel_yaw = max_vel_yaw*step/(0.2*steps)


            elif step >= 0.2*steps and step <= 0.8*steps:
                
                curr_vel_x = max_vel_x
                curr_vel_y = max_vel_y
                curr_vel_z = max_vel_z

                curr_vel_roll = max_vel_roll
                curr_vel_pitch = max_vel_pitch
                curr_vel_yaw = max_vel_yaw

            elif step > 0.8 * steps:
                curr_vel_x = max_vel_x - max_vel_x * (step-0.8*steps)/(0.2*steps)
                curr_vel_y = max_vel_y - max_vel_y * (step-0.8*steps)/(0.2*steps)
                curr_vel_z = max_vel_z - max_vel_z * (step-0.8*steps)/(0.2*steps)

                curr_vel_roll = max_vel_roll - max_vel_roll * (step-0.8*steps)/(0.2*steps)
                curr_vel_pitch = max_vel_pitch - max_vel_pitch * (step-0.8*steps)/(0.2*steps)
                curr_vel_yaw = max_vel_yaw - max_vel_yaw * (step-0.8*steps)/(0.2*steps)

            pose_x = pose_x + (last_vel_x+curr_vel_x)*request.interpolation_time/steps
            pose_y = pose_y + (last_vel_y+curr_vel_y)*request.interpolation_time/steps
            pose_z = pose_z + (last_vel_z+curr_vel_z)*request.interpolation_time/steps

            ort_roll = ort_roll + (last_vel_roll + curr_vel_roll) * request.interpolation_time/steps
            ort_pitch = ort_pitch + (last_vel_pitch + curr_vel_pitch) * request.interpolation_time/steps
            ort_yaw = ort_yaw + (last_vel_yaw + curr_vel_yaw) * request.interpolation_time/steps
            
            roll_rad = ort_roll * pi/180
            pitch_rad = ort_pitch * pi/180
            yaw_rad = ort_yaw * pi/180

            quat = euler2quat (roll_rad, pitch_rad, yaw_rad)
            
            if request.version == "ext":
                ort_quaternion = Quaternion(w=0.0, x=roll_rad, y=pitch_rad, z=yaw_rad)
            else:
                ort_quaternion = Quaternion(w=quat[0], x=quat[1], y=quat[2], z=quat[3])

            pose.header.frame_id = "base_link"
            pose.pose.position.x = pose_x
            pose.pose.position.y = pose_y
            pose.pose.position.z = pose_z
            pose.pose.orientation = ort_quaternion

            sleep(sample_time)
            self.pose_publisher.publish(pose)

        self.initial_position = [pose_x, pose_y, pose_z]
        self.initial_orientation = [ort_roll, ort_pitch, ort_yaw]


def main(args=None):
    rclpy.init(args=args)

    op_interpolation_server = OpInterpolationServer()

    rclpy.spin(op_interpolation_server)

    rclpy.shutdown()


if __name__ == '__main__':
    main()