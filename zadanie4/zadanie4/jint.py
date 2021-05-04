import sys

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node
from '../srv' import request_message

class jint (Node):
    def __init__(self):

        self.subscription = self.create_subscription(JointState, 'joint_states', self.joint_position_callback, 10)
        self.service
        self.joint_start_position = [0, 0, 0]
        
