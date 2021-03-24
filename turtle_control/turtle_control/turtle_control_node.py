import rclpy
import rclpy.node
import curses
import signal
import sys
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType
from geometry_msgs.msg import Twist
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType

class TurtleControl(rclpy.node.Node):
    def __init__(self):
        super().__init__('turtle_control_node')
        timer_period = 0.01 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.i = 0
        self.stdscr = curses.initscr()
        self.declare_parameter('up_key', 'w')
        self.declare_parameter('down_key', 's')
        self.declare_parameter('left_key', 'a')
        self.declare_parameter('right_key', 'd')
        
    def timer_callback(self):
        self.i += 1
        msg = Twist()
        up_key, down_key, left_key, right_key = self.get_params()
        if self.i == 1:
            self.stdscr.addstr(f'Provide Turtle with steering "{up_key}" - go ahead, "{down_key}"- go backwards, "{left_key}"- turn left, "{right_key}"- turn right. To quit press "q"')
        self.stdscr.timeout(100)
        input_key = self.stdscr.getch()
        if input_key == ord(up_key):
            msg.linear.x = 1.5
            self.publisher_.publish(msg)
        elif input_key == ord(left_key):
            msg.angular.z = 1.5
            self.publisher_.publish(msg)
        elif input_key == ord(down_key):
            msg.linear.x = -1.5
            self.publisher_.publish(msg)
        elif input_key == ord(right_key):
            msg.angular.z = -1.5
            self.publisher_.publish(msg)
        elif input_key == ord('q'):
            curses.nocbreak()
            curses.echo()
            curses.endwin()
            rclpy.try_shutdown()
        elif input_key == -1:
            pass
        

    def get_params(self):
        up_key = self.get_parameter('up_key').get_parameter_value().string_value
        down_key = self.get_parameter('down_key').get_parameter_value().string_value
        left_key = self.get_parameter('left_key').get_parameter_value().string_value
        right_key = self.get_parameter('right_key').get_parameter_value().string_value
        return up_key, down_key, left_key, right_key

def main():
    rclpy.init()
    node = TurtleControl()
    curses.noecho()
    curses.cbreak()
    rclpy.spin(node)
    curses.nocbreak()
    curses.endwin()

if __name__ == '__main__':
    main()
