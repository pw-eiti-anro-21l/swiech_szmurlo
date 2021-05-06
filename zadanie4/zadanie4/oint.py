import rclpy
from rclpy.node import Node
from zadanie4_interface.srv import OpInterpolation
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from rclpy.qos import QoSProfile
from time import sleep
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from math import cos, sin, floor

class oint(Node):

    def __init__(self):
        super().__init__('op_interpolation_server')
        self.srv = self.create_service(OpInterpolation, 'op_interpolation', self.interpolation_callback)
        qos_profile = QoSProfile(depth=10)
        self.marker_publisher = self.create_publisher(MarkerArray, '/marker', qos_profile)
        self.pose_publisher = self.create_publisher(PoseStamped, 'oint_pose', qos_profile)
        self.initial_position = [0, 0, 0]
        self.initial_orientation = [0, 0, 0]

    def interpolation_callback(self, request, response):

        if request.interpolation_time <= 0:
            response.server_feedback = "Given interpolation time < 0. Defaulting to 5s"
            request.interpolation_time = 5
        
        if request.version != "std" and request.version != "ext":
            response.server_feedback = "Given version is wrong. Defaulting to standard"
            request.version = "std"

        if request.method != "linear" and request.method != "trapezoid":
            request.method = "linear"
            self.get_logger().info("Given interpolation method is wrong. Defaulting to linear")

        if request.method == "linear":
            self.linear_interpolation(request)
            response.server_feedback = "Interpolation completed"

        elif request.method == "trapezoid":
            self.trapezoid_interpolation(request)
            response.server_feedback = "Interpolation completed"

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

            if request.version == "ext":  
                ort_quaternion = self.euler_to_quaternion(ort_roll, ort_pitch, ort_yaw)
            else:
                ort_quaternion = self.euler_to_quaternion(0, 0, 0)
            pose.header.frame_id = "base_link"
            pose.pose.position.x = pose_x
            pose.pose.position.y = pose_y
            pose.pose.position.z = pose_z
            pose.pose.orientation = ort_quaternion
            sleep(sample_time)
            
            self.pose_publisher.publish(pose)

        self.initial_position = [pose_x, pose_y, pose_z]
        self.initial_orientation = [ort_roll, ort_pitch, ort_yaw]

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

        marker = Marker()
        marker_array = MarkerArray()
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.type = 1
        marker.action = 0
        marker.header.frame_id = "/base_link"

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

            if request.version == "ext":  
                ort_quaternion = self.euler_to_quaternion(ort_roll, ort_pitch, ort_yaw)
            else:
                ort_quaternion = self.euler_to_quaternion(0, 0, 0)

            pose.header.frame_id = "base_link"
            pose.pose.position.x = pose_x
            pose.pose.position.y = pose_y
            pose.pose.position.z = pose_z
            pose.pose.orientation = ort_quaternion
            sleep(sample_time)
            self.pose_publisher.publish(pose)
            marker.pose.position.x = pose_x
            marker.pose.position.y = pose_y
            marker.pose.position.z = pose_z
            marker.pose.orientation = ort_quaternion
            marker_array.markers.append(marker)
            id=0
            for marker in marker_array.markers:
                marker.id = id
                id += 1
            self.marker_publisher.publish(marker_array)

        self.initial_position = [pose_x, pose_y, pose_z]
        self.initial_orientation = [ort_roll, ort_pitch, ort_yaw]

    def euler_to_quaternion(self,roll, pitch, yaw):
        qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
        qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
        qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
        qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)


def main(args=None):
    rclpy.init(args=args)
    op_interpolation_server = oint()
    rclpy.spin(op_interpolation_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()