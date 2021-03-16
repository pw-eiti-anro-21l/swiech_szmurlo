import curses
import rclpy
import signal
import sys
from rclpy.node import Node

from geometry_msgs.msg import Twist


class VelocityValuePublisher(Node):

    def __init__(self):
        super().__init__('velocity_value_publisher')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        self.keyboard_input()

        #msg.data = 'Hello World: %d' % self.i
        #self.publisher_.publish(msg)
        #self.get_logger().info('Publishing: "%s"' % msg.data)
        #self.i += 1

    def keyboard_input(self):
        
        stdscr = curses.initscr()
        curses.noecho()
        curses.cbreak()
        self.get_logger().info('Provide Turtle with steering "w" - go ahead, "s"- go backwards, "a"- turn left, "d"- turn right. To quit press "q"')

        while True:
            msg = Twist()
            input_key = stdscr.getch()
            if input_key == ord('w'):
                msg.linear.x = 0.5
            elif input_key == ord('a'):
                msg.angular.z = 0.5
            elif input_key == ord('s'):
                msg.linear.x = -0.5
            elif input_key == ord('d'):
                msg.angular.z = -0.5
            elif input_key == ord('q'):
               # raise KeyboardInterrupt
                break
                curses.nocbreak()
                curses.echo()
                curses.endwin()
            else:
                pass

            self.publisher_.publish(msg)
        curses.nocbreak()
        curses.echo()
        curses.endwin()
        #raise KeyboardInterrupt
        exit()
#    def input_correct(self):
#        if keyboard.is_pressed('w'):
#            return True
#        if keyboard.is_pressed('a'):
#            return True
#        if keyboard.is_pressed('s'):
#            return True
#        if keyboard.is_pressed('d'):
#            return True
#        else:
#            return False

def main(args=None):
    rclpy.init(args=args)

    velocity_publisher = VelocityValuePublisher()

    rclpy.spin(velocity_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    velocity_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
