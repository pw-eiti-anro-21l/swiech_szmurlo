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

        self.req.method = sys.argv[1]
        self.req.a= float(sys.argv[2])
        self.req.b = float(sys.argv[3])
        self.req.interpolation_time = float(sys.argv[4])
        if self.req.method != "rectangle" and self.req.method != "ellipse":
            self.get_logger().info("Bad method. Use 'rectangle' or 'ellipse'")
        elif self.req.a <=0 or self.req.a > 1 or self.req.b <=0 or self.req.b > 1:
            self.get_logger().info("Unreachable points in rectangle. a,b should be between 0 and 1")
        elif self.req.interpolation_time <= 0:
            self.get_logger().info("Time cannot be 0 or negative")
        else:
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

