import sys
from zadanie5_interface.srv import ToolPosition
import rclpy
from rclpy.node import Node


class oint(Node):

    def __init__(self):
        super().__init__('oint')
        self.client = self.create_client(ToolPosition, 'op_interpolation')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = ToolPosition.Request()


    def send_request(self):

        self.req.x_goal = float(sys.argv[1])
        self.req.y_goal= float(sys.argv[2])
        self.req.z_goal = float(sys.argv[3])

        self.req.roll_goal = float(sys.argv[4])
        self.req.pitch_goal= float(sys.argv[5])
        self.req.yaw_goal = float(sys.argv[6])
        self.req.interpolation_time = float(sys.argv[7])
        self.req.method = sys.argv[8]
        self.req.version = sys.argv[9]

        self.future = self.client.call_async(self.req)




def main(args=None):
    rclpy.init(args=args)

    oint_client = oint()
    oint_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(oint_client)
        if oint_client.future.done():
            try:
                response = oint_client.future.result()
            except Exception as e:
                oint_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                oint_client.get_logger().info(response.server_feedback)
            break

    oint_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
