import rclpy
from rclpy.node import Node
from zadanie5_interface.srv import ToolPosition
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from math import floor, pi, sin, cos
from rclpy.qos import QoSProfile
from time import sleep
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import json
from ament_index_python.packages import get_package_share_directory
class OpInterpolationServer(Node):

    def __init__(self):
        super().__init__('op_interpolation_server')
        self.srv = self.create_service(ToolPosition, 'op_interpolation', self.interpolation_callback)
        qos_profile = QoSProfile(depth=10)
        self.marker_publisher = self.create_publisher(MarkerArray, '/marker', qos_profile)
        self.pose_publisher = self.create_publisher(PoseStamped, '/pose_ikin', qos_profile)
        self.subsciber = self.create_subscription(JointState, 'joint_states', self.listener_callback, 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_interpolate', qos_profile)
        self.tool_length = self.get_length()[3]['tool_length']
        self.base_length = self.get_length()[3]["base_length"]
        self.link1_length = self.get_length()[0]["d"]
        self.link2_length = self.get_length()[1]["d"]
        self.link3_length = self.get_length()[2]["d"]
        self.initial_position = [0.5, -0.5, 1.5]
        self.in_action = False
        self.initial_joint_states = [-0.5, -0.5, -0.6]
        self.position_dict = {}
        self.x_array = []
        self.y_array = []
        self.z_array = []

    def listener_callback(self, msg):
        if not self.in_action:
            self.initial_joint_states[0] = msg.position[0]
            self.initial_joint_states[1] = msg.position[1]
            self.initial_joint_states[2] = msg.position[2]

    def interpolation_callback(self, request, response):
        self.in_action = True
        pose = PoseStamped()
        if request.a <= 0 or request.b <= 0:
            response.server_feedback = "Length 'a' and height 'b' cannot be 0 or negative"
            return response
        # if request.a > 0.5 or request.b > 1.2:
        #     response.server_feedback = "Length 'a' and height 'b' cannot be greater than 0.5"
        #     return response
        if request.method != "rectangle" and request.method != "ellipse":
            response.server_feedback = "Bad method. Choose 'rectangle' or 'ellipse'"
            return response
        if request.interpolation_time <= 0:
            response.server_feedback = "Interpolation time cannot be 0 or negative"
            return response
        if self.initial_joint_states != [-0.5, -0.5, -0.6]:
            self.trapezoid_interpolation([-0.5, -0.5, -0.6], 2)
        if request.method == "rectangle":
            was_completed = self.interpolate_rectangle(request, self.tool_length)
        if request.method == "ellipse":
            was_completed = self.interpolate_ellipse(request, self.tool_length)
        self.in_action = False
        if was_completed:
            response.server_feedback = "Interpolation completed"
        else:
            response.server_feedback = "Interpolation failed: unreachable points"

        self.position_dict["x"] = self.x_array
        self.position_dict["y"] = self.y_array
        self.position_dict["z"] = self.z_array

        # with open ('/home/maciej/dev_ws/src/swiech_szmurlo/zadanie5/zadanie5/position_data.txt', 'w') as outfile:
        #     json.dump(self.position_dict, outfile) to do wykresow

        return response
    
    def interpolate_rectangle(self, request, tool_length):
        pose = PoseStamped()
        a = request.a
        b = request.b
        perimeter = 2*(a + b)
        sample_time = 0.01
        steps_a = floor(request.interpolation_time/sample_time*(a/perimeter))
        steps_b = floor(request.interpolation_time/sample_time*(b/perimeter))

        marker = Marker()
        marker_array = MarkerArray()
        marker.scale.x = 0.04
        marker.scale.y = 0.04
        marker.scale.z = 0.04
        marker.color.a = 1.
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.type = 2
        marker.action = 0
        marker.header.frame_id = "/base_link"
        pose_x = self.initial_position[0]

        for krok in [1,2,3,4]:
            initial_position = self.initial_position
            if krok == 1:
                y_goal = initial_position[1]
                z_goal = initial_position[2] - b
                steps = steps_b
            elif krok == 2:
                y_goal = initial_position[1] + a
                z_goal = initial_position[2]
                steps = steps_a
            elif krok == 3:
                y_goal = initial_position[1]
                z_goal = initial_position[2] + b
                steps = steps_b
            elif krok == 4:
                y_goal = initial_position[1] - a
                z_goal = initial_position[2]
                steps = steps_a
            for step in range(steps + 1):

                # pose_x = initial_position[0] + (x_goal - initial_position[0])/steps*step
                pose_y = initial_position[1] + (y_goal - initial_position[1])/steps*step
                pose_z = initial_position[2] + (z_goal - initial_position[2])/steps*step
                if pose_y > 0 or pose_y < -1*self.link1_length:
                    self.get_logger().info("Point is unreachable")
                    return False
                if pose_z > self.base_length+ self.link1_length or pose_z < self.base_length:
                    self.get_logger().info("Point is unreachable")
                    return False
                pose.header.frame_id = "base_link"
                pose.pose.position.x = pose_x
                pose.pose.position.y = pose_y
                pose.pose.position.z = pose_z

                sleep(sample_time)
                self.pose_publisher.publish(pose)
                marker.pose.position.x = pose_x
                marker.pose.position.y = pose_y
                marker.pose.position.z = pose_z
                marker_array.markers.append(marker)
                id=0
                for marker in marker_array.markers:
                    marker.id = id
                    id += 1
                self.marker_publisher.publish(marker_array)

                self.x_array.append(pose_x)
                self.y_array.append(pose_y)
                self.z_array.append(pose_z)

            self.initial_position = [pose_x, pose_y, pose_z]

        return True

    def interpolate_ellipse(self, request, tool_length):
        pose = PoseStamped()
        a = request.a/2
        b = request.b/2
        sample_time = 0.01
        steps = floor(request.interpolation_time/sample_time)
        initial_position = self.initial_position
        pose_x = initial_position[0]
        marker = Marker()
        marker_array = MarkerArray()
        marker.scale.x = 0.04
        marker.scale.y = 0.04
        marker.scale.z = 0.04
        marker.color.a = 1.
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.type = 2
        marker.action = 0
        marker.header.frame_id = "/base_link"
        for step in range(steps + 1):
            pose_y = initial_position[1] + a*cos(2*pi*step/steps) - a
            pose_z = initial_position[2] + b*sin(2*pi*step/steps)
            if pose_y > 0 or pose_y < -1:
                self.get_logger().info("Point is unreachable")
                return False
            if pose_z > 2 or pose_z < 1:
                self.get_logger().info("Point is unreachable")
                return False
            pose.header.frame_id = "base_link"
            pose.pose.position.x = pose_x
            pose.pose.position.y = pose_y
            pose.pose.position.z = pose_z

            sleep(sample_time)
            self.pose_publisher.publish(pose)
            marker.pose.position.x = pose_x
            marker.pose.position.y = pose_y
            marker.pose.position.z = pose_z
            marker_array.markers.append(marker)
            id=0
            for marker in marker_array.markers:
                marker.id = id
                id += 1
            self.marker_publisher.publish(marker_array)

            self.x_array.append(pose_x)
            self.y_array.append(pose_y)
            self.z_array.append(pose_z)

        self.initial_position = [pose_x , pose_y, pose_z]

        return True


    def trapezoid_interpolation(self, goal_joint_states, interpolation_time):
        sample_time = 0.01
        steps = floor(interpolation_time/sample_time)
        joint_states = JointState()
        joint_states.name = ['joint_base_1', 'joint_1_2', 'joint_2_3']
        start_joint_states = self.initial_joint_states
        max_vel_1 = (goal_joint_states[0] - start_joint_states[0]) / (0.8*interpolation_time)
        max_vel_2 = (goal_joint_states[1] - start_joint_states[1]) / (0.8*interpolation_time)
        max_vel_3 = (goal_joint_states[2] - start_joint_states[2]) / (0.8*interpolation_time)
        last_vel_1 = 0
        last_vel_2 = 0
        last_vel_3 = 0
        pos1 = start_joint_states[0]
        pos2 = start_joint_states[1]
        pos3 = start_joint_states[2]
        for step in range(steps + 1):
            if step < 0.2*steps:
                curr_vel_1 = max_vel_1*step/(0.2*steps)
                curr_vel_2 = max_vel_2*step/(0.2*steps)
                curr_vel_3 = max_vel_3*step/(0.2*steps)
            elif step >= 0.2*steps and step <= 0.8*steps:
                curr_vel_1 = max_vel_1
                curr_vel_2 = max_vel_2
                curr_vel_3 = max_vel_3
            elif step > 0.8 * steps:
                curr_vel_1 = max_vel_1 - max_vel_1 * (step-0.8*steps)/(0.2*steps)
                curr_vel_2 = max_vel_2 - max_vel_2 * (step-0.8*steps)/(0.2*steps)
                curr_vel_3 = max_vel_3 - max_vel_3 * (step-0.8*steps)/(0.2*steps)

            pos1 = pos1 + (last_vel_1+curr_vel_1)*interpolation_time/steps
            pos2 = pos2 + (last_vel_2+curr_vel_2)*interpolation_time/steps
            pos3 = pos3 + (last_vel_3+curr_vel_3)*interpolation_time/steps
            joint_1_state = pos1
            joint_2_state = pos2
            joint_3_state = pos3
            joint_states.position = [float(joint_1_state), float(joint_2_state), float(joint_3_state)]
            self.joint_pub.publish(joint_states)
            sleep(sample_time)
        self.initial_joint_states = [joint_1_state, joint_2_state, joint_3_state]
        self.initial_position = [0.5, -0.5, 1.5]
        pos1 = joint_1_state
        pos2 = joint_2_state
        pos3 = joint_3_state


    def get_length(self):
        path = get_package_share_directory('zadanie5') + "/dh_params.json"
        with open(path, "r") as read_file:
            data = json.load(read_file)
        return data
        


def main(args=None):
    rclpy.init(args=args)

    op_interpolation_server = OpInterpolationServer()

    rclpy.spin(op_interpolation_server)

    rclpy.shutdown()


if __name__ == '__main__':
    main()