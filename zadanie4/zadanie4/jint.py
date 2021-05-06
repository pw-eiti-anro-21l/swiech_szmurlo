import sys
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node
from zadanie4_interface.srv import Interpolation

class jint(Node):
    def __init__(self):
        super().__init__('jint')
        self.client = self.create_client(Interpolation, 'interpolation')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Interpolation.Request()

    def send_request(self):
        try:
            self.req.position_joint1 = float(sys.argv[1])
            self.req.position_joint2 = float(sys.argv[2])
            self.req.position_joint3 = float(sys.argv[3])
            self.req.interpolation_time = float(sys.argv[4])
            self.req.method = sys.argv[5]
            self.future = self.client.call_async(self.req)
        except ValueError:
            self.req.position_joint1 = -1.
            self.req.position_joint2 = -1.
            self.req.position_joint3 = -1.
            self.req.interpolation_time = -1.
            self.req.method = sys.argv[5]
            self.future = self.client.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)

    jint_client = jint()
    jint_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(jint_client)
        if jint_client.future.done():
            try:
                response = jint_client.future.result()
            except Exception as e:
                jint_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                jint_client.get_logger().info(response.server_feedback)
            break

    jint_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()