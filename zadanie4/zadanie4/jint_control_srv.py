import rclpy
from rclpy.node import Node
from '../srv' import request_message


class InterpolationServer(Node):

    def __init__(self):
        super().__init__('interpolation_server')
        self.srv = self.create_service(request_message, 'interpolation', self.interpolation_callback)

    def interpolation_callback(self, request, response):

        solve_linear_interpolation()

        response.server_feedback = "Interpolation completed"

        self.get_logger().info(f'Incoming request {position_joint1}, {position_joint2}, {position_joint3}, {interpolation_time}')

        return response
    
    def solve_linear_interpolation():
        joint_positions = [0, 0, 0]
        


def main(args=None):
    rclpy.init(args=args)

    interpolation_server = InterpolationServer()

    rclpy.spin(interpolation_server)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
