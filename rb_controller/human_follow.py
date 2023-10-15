import rclpy
from rclpy.node import Node
import math

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3


class HumanFollower(Node):
    # Constant
    #camera_fov = [86, 57] # [horizontal, vertical]
    # Veriable
    target_angle = 0.0
    target_depth = 0.0

    def __init__(self):
        super().__init__('human_follower')
        # Create publisher
        self.follow_vel_pub_ = self.create_publisher(Twist, 'follow_cmd_vel', 10)
        # Create subscriber
        self.robot_mode_sub_ = self.create_subscription(String, 'robot_mode', self.follow_mode, 10)
        self.robot_mode_sub_  # prevent unused variable warning
        self.human_pose_sub_ = self.create_subscription(Vector3, 'human_pose', self.pose_read, 10)
        self.human_pose_sub_  # prevent unused variable warning

    def follow_mode(self, mode_msg):
        output_cmd_vel_msgs = Twist()
        # Make sure other veriables are zero
        output_cmd_vel_msgs.linear.y = 0.0
        output_cmd_vel_msgs.linear.z = 0.0
        output_cmd_vel_msgs.angular.x = 0.0
        output_cmd_vel_msgs.angular.y = 0.0
        # Follow mode
        if mode_msg.data=='FOLLOW':
            output_cmd_vel_msgs.linear.x = 6.6
            output_cmd_vel_msgs.angular.z = 1.1
            # self.get_logger().info('Start Follow')   
        else:
            # Zeroing
            output_cmd_vel_msgs.linear.x = 0.0
            output_cmd_vel_msgs.angular.z = 0.0
        
        self.follow_vel_pub_.publish(output_cmd_vel_msgs)
        # self.get_logger().info('Follow cmd vel Published > <')
    
    def PID_controller(self, target, measure, kp, ki, kd):
        error = float(target - measure)
        error_sum += error
        output = kp*error + ki*error_sum + kd*(error-error_last)
        error_last = error
        return output

    def pose_read(self, pose_msgs):
        human_x = pose_msgs.x
        human_depth = pose_msgs.y
        self.target_angle = (abs(human_x-640)/640)*43 # 43 = horizontal_FOV / 2
        self.target_depth = human_depth / 1000.0
        

def main(args=None):
    rclpy.init(args=args)
    human_follower = HumanFollower()
    rclpy.spin(human_follower)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    human_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()