#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import math
import time

def quaternion_to_pitch(x, y, z, w):
    sinp = 2.0 * (w * y - z * x)
    sinp = max(min(sinp, 1.0), -1.0)
    return math.asin(sinp)

class PIDController:
    def __init__(self, kp, ki, kd, output_limit=2.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.prev_error = 0.0
        self.output_limit = output_limit
        self.prev_time = time.time()

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = time.time()

    def compute(self, error):
        now = time.time()
        dt = max(now - self.prev_time, 1e-3)
        self.integral += error * dt
        self.integral = max(min(self.integral, 1.0), -1.0)
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        output = max(min(output, self.output_limit), -self.output_limit)
        self.prev_error = error
        self.prev_time = now
        return output

class BalanceController(Node):
    def __init__(self):
        super().__init__('balance_controller')

        self.declare_parameter('kp', 4.0)
        self.declare_parameter('ki', 0.01)
        self.declare_parameter('kd', 0.6)
        self.declare_parameter('target_angle', 0.0)
        self.declare_parameter('output_limit', 2.0)

        kp    = self.get_parameter('kp').value
        ki    = self.get_parameter('ki').value
        kd    = self.get_parameter('kd').value
        limit = self.get_parameter('output_limit').value
        self.target_angle = self.get_parameter('target_angle').value

        self.pid = PIDController(kp=kp, ki=ki, kd=kd, output_limit=limit)
        self.user_speed = 0.0
        self.user_turn  = 0.0
        self.fallen     = False

        self.create_subscription(Imu, '/imu_plugin/out', self.imu_cb, 10)
        self.create_subscription(Twist, '/user_cmd_vel', self.user_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info(f'Balance controller: kp={kp} ki={ki} kd={kd} limit={limit}')

    def user_cb(self, msg):
        self.user_speed = msg.linear.x
        self.user_turn  = msg.angular.z

    def imu_cb(self, msg):
        pitch = quaternion_to_pitch(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )
        pitch_deg = math.degrees(pitch)

        if abs(pitch_deg) > 45.0:
            if not self.fallen:
                self.get_logger().warn(f'FALLEN at {pitch_deg:.1f} deg — respawn robot')
                self.fallen = True
                self.pid.reset()
            self.cmd_pub.publish(Twist())
            return

        self.fallen = False
        target = self.target_angle + self.user_speed * 0.1
        error  = target - pitch
        output = self.pid.compute(error)

        cmd = Twist()
        cmd.linear.x  = output
        cmd.angular.z = self.user_turn
        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f'pitch={pitch_deg:.2f}deg  err={math.degrees(error):.2f}deg  out={output:.4f}',
            throttle_duration_sec=0.3)

def main(args=None):
    rclpy.init(args=args)
    node = BalanceController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
