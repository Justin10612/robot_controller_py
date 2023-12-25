import rclpy
import numpy as np
from rclpy.node import Node
import math

from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3, PoseStamped

class DepthKalman(Node):
    # 
    last_x = 0.0
    last_y = 0.0
    
    def __init__(self):
        super().__init__('depth_kalman')
        # Create publisher
        self.target_pose_pub_ = self.create_publisher(PoseStamped, 'target_pose', 10)
        self.raw_depth_pub_ = self.create_publisher(Float64, 'depth_raw', 10)
        # Create subscriber
        self.robot_mode_sub_ = self.create_subscription(Vector3, 'human_pose', self.depth_callback, 10)
        self.robot_mode_sub_  # prevent unused variable warning

    def depth_callback(self, human_pose_msg):
        # input
        x_pixel = human_pose_msg.x
        target_radian = (x_pixel-640)*0.001225
        depth = max(min(human_pose_msg.y, 2.5), 0)
        target_state = human_pose_msg.z
        # Cal to Pose2D
        target_pose2D = PoseStamped()
        target_pose2D.header.frame_id = 'base_link'
        if target_state==1.0:
            target_pose2D._pose.position.x = depth
            target_pose2D.pose.position.y = depth*math.tan(target_radian)
            self.last_x = depth
            self.last_y = depth*math.tan(target_radian)
        else:
            target_pose2D._pose.position.x = self.last_x
            target_pose2D.pose.position.y =  self.last_y
        self.target_pose_pub_.publish(target_pose2D)


def main(args=None):
    rclpy.init(args=args)
    depth_kalman = DepthKalman()
    rclpy.spin(depth_kalman)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    depth_kalman.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()