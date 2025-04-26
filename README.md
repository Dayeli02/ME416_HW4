#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Twist
from std_msgs.msg import Float64
from me416_utilities import stamp_difference
from controller import PID

class ControllerLine(Node):
    def __init__(self):
        super().__init__('controller_line')

        # 1. Initialize attributes
        self.lin_speed = 0.0
        self.gain_proportional = 0.0
        self.gain_derivative = 0.0
        self.gain_integral = 0.0
        self.image_width = 640  # Adjust based on your camera resolution

        # 2. Initialize subscriber and publishers
        self.subscription = self.create_subscription(
            PointStamped,
            '/image/centroid',
            self.listener_callback,
            10)
        self.publisher_twist = self.create_publisher(Twist, '/robot_twist', 10)
        self.publisher_error = self.create_publisher(Float64, '/control_error', 10)

        # 3. Initialize PID controller
        self.pid = PID(self.gain_proportional, self.gain_derivative, self.gain_integral)

        # 4. Initialize previous message attribute
        self.msg_previous = None

    def listener_callback(self, msg):
        # 1. Compute error signal
        error_signal = msg.point.x - (self.image_width / 2)

        # 2. Publish error signal
        error_msg = Float64()
        error_msg.data = error_signal
        self.publisher_error.publish(error_msg)

        # 3. Compute time delay
        time_delay = 0.0
        if self.msg_previous is not None:
            time_delay = stamp_difference(msg.header.stamp, self.msg_previous.header.stamp)

        # 4. Initialize Twist message
        msg_twist = Twist()

        # 5. Set linear speed
        msg_twist.linear.x = self.lin_speed

        # 6. Set angular speed using PID control
        if self.msg_previous is None:
            # First run - only proportional term
            angular_z = self.pid.proportional(error_signal)
        else:
            angular_z = (self.pid.proportional(error_signal) +
                        self.pid.derivative(error_signal, time_delay) +
                        self.pid.integral(error_signal, time_delay))

        msg_twist.angular.z = angular_z

        # 7. Publish twist message
        self.publisher_twist.publish(msg_twist)

        # Update previous message
        self.msg_previous = msg

def main(args=None):
    rclpy.init(args=args)
    controller_line = ControllerLine()
    rclpy.spin(controller_line)
    controller_line.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
