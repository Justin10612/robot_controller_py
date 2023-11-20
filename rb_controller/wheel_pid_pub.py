import rclpy
import time
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist


class WheelPidPublisher(Node):
    # LEFT MOTOR PID PARAM.
    l_kp = 0.01 
    l_ki = 0.012
    l_kd = 0.002
    # RIGHT MOTOR PID PARA.
    r_kp = 0.01
    r_ki = 0.012
    r_kd = 0.02

    def __init__(self):
        super().__init__('wheel_pid_publisher')
        # Create publisher
        self.motor_pid_pub_ = self.create_publisher(Twist, 'cmd_pid', 10)
        # Create subscriber
        self.robot_mode_sub_ = self.create_subscription(String, 'robot_mode', self.publish_pid, 10)
        self.robot_mode_sub_  # prevent unused variable warning

    def publish_pid(self, msg):
        # Publish Robot_state
        pid_msg = Twist()
        # left
        pid_msg.linear.x = self.l_kp
        pid_msg.linear.y = self.l_ki  
        pid_msg.linear.z = self.l_kd
        # right
        pid_msg.angular.x = self.r_kp
        pid_msg.angular.y = self.r_ki
        pid_msg.angular.z = self.r_kd
        # publish data
        self.motor_pid_pub_.publish(pid_msg)

def main(args=None):
    rclpy.init(args=args)
    pid_publisher = WheelPidPublisher()
    rclpy.spin(pid_publisher)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pid_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()