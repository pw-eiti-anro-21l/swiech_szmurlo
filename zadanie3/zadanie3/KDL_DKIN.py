import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from PyKDL import *


class KDL_DKIN(Node):

    def __init__(self):
        super().__init__('KDL_DKIN')
        self.publisher_ = self.create_publisher(PoseStamped, 'kdl_pose', 10)
        self.subsciber = self.create_subscription(JointState, 'joint_states', self.listener_callback, 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        # msg = 0
        # self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
        
    def listener_callback(self, msg):
        pass

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = KDL_DKIN()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()