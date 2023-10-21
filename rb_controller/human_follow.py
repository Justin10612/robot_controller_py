import rclpy
from rclpy.node import Node
from simple_pid import PID

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3


class HumanFollower(Node):
    # Constant
    MIN_CHASE_DISTANCE = 0.5     # Unit:center-meter
    MAX_CHASE_DISTANCE = 2    # Unit:center-meter
    # depth pid controller constant
    DEPTH_kp = 0.2
    DEPTH_ki = 0
    DEPTH_kd = 0
    # angle pid controller constant
    ANGLE_kp = 0
    ANGLE_ki = 0
    ANGLE_kd = 0
    # Veriable
    target_angle = 0.0
    target_depth = 0.0
    last_state = ''


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
        # self.pid_angle = PID(self.ANGLE_kp, self.ANGLE_ki, self.ANGLE_kd, setpoint=0)
        # self.pid_angle.output_limits = (-1000, 1000)
        self.pid_depth = PID(self.DEPTH_kp, self.DEPTH_ki, self.DEPTH_kd, setpoint=0)
        # self.pid_depth.output_limits = (-1000, 1000)

    def pose_read(self, pose_msgs):
        human_x = pose_msgs.x
        # human_depth = 
        self.target_angle = (abs(human_x-640)/640)*43 # 43 = horizontal_FOV / 2
        self.target_depth = pose_msgs.y

    def follow_mode(self, mode_msg):
        output_cmd_vel_msgs = Twist()
        # Make sure other veriables are zero
        # output_cmd_vel_msgs.linear.y = 0.0
        # output_cmd_vel_msgs.linear.z = 0.0
        # output_cmd_vel_msgs.angular.x = 0.0
        # output_cmd_vel_msgs.angular.y = 0.0
        # Follow mode
        if mode_msg.data=='FOLLOW':
            # Dummy Data for Testing
            # output_cmd_vel_msgs.linear.x = 6.6
            # output_cmd_vel_msgs.angular.z = 1.1
            # Stop Zone Detect
            if (self.target_depth < self.MIN_CHASE_DISTANCE 
                or self.target_depth > self.MAX_CHASE_DISTANCE):
                # In the stop zone
                output_cmd_vel_msgs.linear.x = 0.0
                output_cmd_vel_msgs.angular.z = 0.0
            else:
                # In the chasing zone
                # PID Controller Calculation
                output_cmd_vel_msgs.linear.x = float(self.pid_depth(-self.target_depth))
                # output_cmd_vel_msgs.angular.z = float(self.pid_angle(-self.target_angle))
                self.get_logger().info('linear_x: %.2f, angular_z: %.2f'%(output_cmd_vel_msgs.linear.x, output_cmd_vel_msgs.angular.z))
            # Publish Data
            self.follow_vel_pub_.publish(output_cmd_vel_msgs)
            # Log Data
            # self.get_logger().info('Start Follow')
        else:
            # Zeroing
            output_cmd_vel_msgs.linear.x = 0.0
            output_cmd_vel_msgs.angular.z = 0.0
            # Publish Data
            if self.last_state != mode_msg.data:
                self.follow_vel_pub_.publish(output_cmd_vel_msgs)
            # Log Data
            # self.get_logger().info('Follow cmd vel Published > <')
        self.last_state = mode_msg.data


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