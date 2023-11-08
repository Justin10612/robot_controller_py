import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

class HumanFollower(Node):
    # Constant
    MIN_CHASE_DISTANCE = 0.8     # Unit:center-meter
    MAX_CHASE_DISTANCE = 3    # Unit:center-meter
    MAX_LINEAR_VEL_OUTPUT = 1.2
    MAX_ANGULER_VEL_OUTPUT = 2.5
    # depth pid controller constant
    DEPTH_kp = 0.98
    DEPTH_kd = 0.1
    depth_error1 = 0
    # angle pid controller constant
    ANGLE_kp = 0.08
    ANGLE_kd = 0.08
    angle_error1 = 0 
    # Veriable
    target_angle = 0.0
    target_depth = 0.0
    target_state = 0.0
    follow_flag = False
    # Message
    output_cmd_vel_msgs = Twist()

    def __init__(self):
        super().__init__('human_follower')
        # Create publisher
        self.follow_vel_pub_ = self.create_publisher(Twist, 'follow_cmd_vel', 10)
        # Create subscriber
        self.robot_mode_sub_ = self.create_subscription(String, 'robot_mode', self.follow_mode, 10)
        self.robot_mode_sub_  # prevent unused variable warning
        self.human_pose_sub_ = self.create_subscription(Vector3, 'human_pose', self.pose_read, 10)
        self.human_pose_sub_  # prevent unused variable warning
        # PID controller setting

    def pose_read(self, pose_msgs):
        human_x = pose_msgs.x
        self.target_angle = round((human_x-640)*0.07, 2) # 43 = horizontal_FOV / 2
        self.target_depth = pose_msgs.y
        self.target_state = pose_msgs.z

    def follow_mode(self, mode_msg):
        # Follow mode
        if mode_msg.data=='FOLLOW':
            # self.get_logger().info('State: %f'%self.target_state)
            self.follow_flag = True
            # Stop Zone Detect
            if self.target_depth > self.MAX_CHASE_DISTANCE or self.target_state==0.0:
                # In the stop zone
                self.output_cmd_vel_msgs.linear.x = 0.0
                self.output_cmd_vel_msgs.angular.z = 0.0
            else:
                # In the chasing zone #
                # DEPTH PID Controller
                # depth_error = self.target_depth - self.MIN_CHASE_DISTANCE
                # depth_pid = self.DEPTH_kp*depth_error + self.DEPTH_kd*(depth_error-self.depth_error1)
                # if depth_pid > self.MAX_LINEAR_VEL_OUTPUT:
                #     depth_pid = self.MAX_LINEAR_VEL_OUTPUT
                # if depth_pid < -self.MAX_LINEAR_VEL_OUTPUT:
                #     depth_pid = -self.MAX_LINEAR_VEL_OUTPUT
                # self.depth_error1 = depth_error
                # self.output_cmd_vel_msgs.linear.x = depth_pid
                # ANGLE PID Controller
                self.get_logger().info('Angle:%fdegree'%self.target_angle)
                angle_error = self.target_angle
                angle_pid = -self.ANGLE_kp*angle_error + self.ANGLE_kd*(angle_error-self.angle_error1)
                if angle_pid > self.MAX_ANGULER_VEL_OUTPUT: angle_pid = self.MAX_ANGULER_VEL_OUTPUT
                if angle_pid < -self.MAX_ANGULER_VEL_OUTPUT: angle_pid = -self.MAX_ANGULER_VEL_OUTPUT
                self.output_cmd_vel_msgs.angular.z = angle_pid
                self.angle_error1 = angle_error
            # Publish Data
            self.follow_vel_pub_.publish(self.output_cmd_vel_msgs)
            # Log Data
            # self.get_logger().info('linear_x: %.2f, angular_z: %.2f'%(self.output_cmd_vel_msgs.linear.x, self.output_cmd_vel_msgs.angular.z))
            # self.get_logger().info('Start Follow')
        else:
            # Clean Output
            self.output_cmd_vel_msgs.linear.x = 0.0
            self.output_cmd_vel_msgs.angular.z = 0.0
            # Publish Data
            if self.follow_flag == True:
                self.follow_vel_pub_.publish(self.output_cmd_vel_msgs)
                self.follow_flag = False

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