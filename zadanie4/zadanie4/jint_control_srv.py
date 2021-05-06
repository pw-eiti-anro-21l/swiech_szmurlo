import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
from zadanie4_interface.srv import Interpolation
from sensor_msgs.msg import JointState
from math import floor
from rclpy.qos import QoSProfile
from time import sleep
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
class InterpolationServer(Node):

    def __init__(self):
        super().__init__('interpolation_server')
        self.srv = self.create_service(Interpolation, 'interpolation', self.interpolation_callback)
        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_interpolate', qos_profile)
        self.initial_joint_states = [0, 0, 0]
        self.marker_pub = self.create_publisher(MarkerArray, '/marker', qos_profile)

    def interpolation_callback(self, request, response):

        if request.interpolation_time <= 0:
            response.server_feedback = "time must be greater than 0"
        if request.position_joint1 > 0:
            request.position_joint1 = 0.
        elif request.position_joint1 < -1:
            request.position_joint1 = -1.
        if request.position_joint2 > 0:
            request.position_joint2 = 0.
        elif request.position_joint2 < -1:
            request.position_joint2 = -1.
        if request.position_joint3 > 0:
            request.position_joint3 = 0.
        elif request.position_joint3 < -1:
            request.position_joint3 = -1.
        if request.method != "linear" and request.method != "trapezoid":
            request.method = "linear"
            self.get_logger().info("bad method, defaulting to linear")
        if request.method == "linear":
            self.linear_interpolation(request)
        elif request.method == "trapezoid":
            self.trapezoid_interpolation(request)
        response.server_feedback = "Interpolation completed"
        # self.get_logger().info(f'Incoming request {position_joint1}, {position_joint2}, {position_joint3}, {interpolation_time}')
        return response
    

    def linear_interpolation(self, request):
        sample_time = 0.01
        steps = floor(request.interpolation_time/sample_time)
        joint_states = JointState()
        joint_states.name = ['joint_base_1', 'joint_1_2', 'joint_2_3']
        start_joint_states = self.initial_joint_states
        for step in range(steps + 1):
            joint_1_state = start_joint_states[0] + (request.position_joint1 - start_joint_states[0])/steps*step
            joint_2_state = start_joint_states[1] + (request.position_joint2 - start_joint_states[1])/steps*step
            joint_3_state = start_joint_states[2] + (request.position_joint3 - start_joint_states[2])/steps*step
            joint_states.position = [float(joint_1_state), float(joint_2_state), float(joint_3_state)]
            self.joint_pub.publish(joint_states)
            #self.get_logger().info(str(joint_1_state))
            sleep(sample_time)
        self.initial_joint_states = [joint_1_state, joint_2_state, joint_3_state]

    def trapezoid_interpolation(self, request):
        sample_time = 0.01
        steps = floor(request.interpolation_time/sample_time)
        joint_states = JointState()
        joint_states.name = ['joint_base_1', 'joint_1_2', 'joint_2_3']
        start_joint_states = self.initial_joint_states
        max_vel_1 = (request.position_joint1 - start_joint_states[0]) / (0.8*request.interpolation_time)
        max_vel_2 = (request.position_joint2 - start_joint_states[1]) / (0.8*request.interpolation_time)
        max_vel_3 = (request.position_joint3 - start_joint_states[2]) / (0.8*request.interpolation_time)
        last_vel_1 = 0
        last_vel_2 = 0
        last_vel_3 = 0
        pos1 = start_joint_states[0]
        pos2 = start_joint_states[1]
        pos3 = start_joint_states[2]

        markerArray = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "/tool"
        marker.type = 2
        marker.action = 0
        marker.color.a = 1.
        marker.color.r = 1.
        marker.color.g = 0.
        marker.color.b = 0.
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        # marker.pose.orientation.w = 1.0
        # marker.pose.orientation.x = 1.0
        # marker.pose.orientation.y = 1.0
        # marker.pose.orientation.z = 1.0

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

            pos1 = pos1 + (last_vel_1+curr_vel_1)*request.interpolation_time/steps
            pos2 = pos2 + (last_vel_2+curr_vel_2)*request.interpolation_time/steps
            pos3 = pos3 + (last_vel_3+curr_vel_3)*request.interpolation_time/steps
            self.get_logger().info(str(curr_vel_1))
            joint_1_state = pos1
            joint_2_state = pos2
            joint_3_state = pos3
            joint_states.position = [float(joint_1_state), float(joint_2_state), float(joint_3_state)]
            self.joint_pub.publish(joint_states)
            sleep(sample_time)
            marker.pose.position.y = 0
            marker.pose.position.z = 0
            marker.pose.position.x = 0
            markerArray.markers.append(marker)
            id = 0
            for marker in markerArray.markers:
                marker.id = id
                id += 1
            self.marker_pub.publish(markerArray)

        self.initial_joint_states = [joint_1_state, joint_2_state, joint_3_state]
        pos1 = joint_1_state
        pos2 = joint_2_state
        pos3 = joint_3_state

def main(args=None):
    rclpy.init(args=args)

    interpolation_server = InterpolationServer()

    rclpy.spin(interpolation_server)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
