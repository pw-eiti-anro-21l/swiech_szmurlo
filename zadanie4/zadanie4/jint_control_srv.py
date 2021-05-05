import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
from zadanie4_interface.srv import Interpolation
from sensor_msgs.msg import JointState
from math import floor
from rclpy.qos import QoSProfile
from time import sleep

class InterpolationServer(Node):

    def __init__(self):
        super().__init__('interpolation_server')
        self.srv = self.create_service(Interpolation, 'interpolation', self.interpolation_callback)
        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
    def interpolation_callback(self, request, response):
        if request.interpolation_time <= 0:
            response.server_feedback = "time must be greater than 0"
        self.linear_interpolation(request)

        response.server_feedback = "Interpolation completed"

        # self.get_logger().info(f'Incoming request {position_joint1}, {position_joint2}, {position_joint3}, {interpolation_time}')

        return response
    

    def linear_interpolation(self, request):
        sample_time = 0.1
        steps = floor(request.interpolation_time/sample_time)
        joint_states = JointState()
        start_joint_states = [0, 0, 0]
        for step in range(steps + 1):
            joint_1_state = start_joint_states[0] + (request.position_joint1 - start_joint_states[0])/steps*step
            joint_2_state = start_joint_states[1] + (request.position_joint2 - start_joint_states[1])/steps*step
            joint_3_state = start_joint_states[2] + (request.position_joint3 - start_joint_states[2])/steps*step
            joint_states = [joint_1_state, joint_2_state, joint_3_state]
            self.joint_pub.publish(joint_states)
            sleep(sample_time)


def main(args=None):
    rclpy.init(args=args)

    interpolation_server = InterpolationServer()

    rclpy.spin(interpolation_server)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
