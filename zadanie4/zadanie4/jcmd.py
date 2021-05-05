# import sys

# from example_interfaces.srv import AddTwoInts
# import rclpy
# from rclpy.node import Node
# from '../srv' import request_message


# class InterpolationClient(Node):

#     def __init__(self):
#         super().__init__('interpolation_client')
#         self.cli = self.create_client(request_message, 'interpolation')
#         while not self.cli.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('service not available, waiting again...')
#         self.req = request_message.Request()

#     def send_request(self):
#         self.req.position_joint1 = int(sys.argv[1])
#         self.req.position_joint2 = int(sys.argv[2])
#         self.req.position_joint3 = int(sys.argv[3])
#         self.req.interpolation_time = int(sys.argv[4])

#         self.future = self.cli.call_async(self.req)


# def main(args=None):
#     rclpy.init(args=args)

#     interpolation_client = InterpolationClient()
#     interpolation_client.send_request()

#     while rclpy.ok():
#         rclpy.spin_once(interpolation_client)
#         if interpolation_client.future.done():
#             try:
#                 response = interpolation_client.future.result()
#             except Exception as e:
#                 interpolation_client.get_logger().info(
#                     'Service call failed %r' % (e,))
#             else:
#                 interpolation_client.get_logger().info(f'Result of interpolation:
#                     {interpolation_client.req.position_joint1, interpolation_client.req.position_joint2,
#                     interpolation_client.req.position_joint3, interpolation_client.req.interpolation_time}
#                     response.server_feedback')
#             break

#     interpolation_client.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()