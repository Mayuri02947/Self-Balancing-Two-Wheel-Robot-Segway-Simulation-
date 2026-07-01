#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class UserNav(Node):
    def __init__(self):
        super().__init__('user_nav')
        self.pub = self.create_publisher(Twist, '/user_cmd_vel', 10)
        self.get_logger().info('Type command + Enter: w=forward s=back a=left d=right x=stop q=quit')
        self.loop()

    def loop(self):
        while rclpy.ok():
            try:
                key = input('cmd> ').strip()
            except EOFError:
                break
            cmd = Twist()
            if key == 'w':
                cmd.linear.x = 1.0
                print('Moving FORWARD')
            elif key == 's':
                cmd.linear.x = -1.0
                print('Moving BACK')
            elif key == 'a':
                cmd.angular.z = 1.0
                print('Turning LEFT')
            elif key == 'd':
                cmd.angular.z = -1.0
                print('Turning RIGHT')
            elif key == 'x':
                print('STOP')
            elif key == 'q':
                print('Quit')
                break
            else:
                print('Unknown key. Use w/s/a/d/x/q')
                continue
            self.pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = UserNav()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
