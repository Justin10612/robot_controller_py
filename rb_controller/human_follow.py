import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

class HumanFollower(Node):
    # Depth Distance Constant
    MAX_CHASE_DISTANCE = 3.0    # Unit:meter
    MIN_CHASE_DISTANCE = 0.8     # Unit:meter
    MIN_SKID_DISTANCE = 0.5
    MAX_LINEAR_VEL_OUTPUT = 1.5
    # depth pid controller constant
    DEPTH_kp = 2.0
    DEPTH_kd = 0.5
    depth_error1 = 0
    # Angle 
    MAX_ANGULER_VEL_OUTPUT = 2
    # angle pid controller constant
    ANGLE_kp = 0.5
    ANGLE_kd = 0.25
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
        self.target_state = pose_msgs.z
        if self.target_state == 1.0:
            self.target_angle = round((human_x-640)*0.0122, 2) # 43 = horizontal_FOV / 2
            self.target_depth = pose_msgs.y

    def follow_mode(self, mode_msg):
        # Follow mode
        if mode_msg.data=='FOLLOW':
            # Log Data
            # self.get_logger().info('Start Follow')
            self.follow_flag = True
            if (self.target_depth > self.MAX_CHASE_DISTANCE):
                # In the stop zone
                self.output_cmd_vel_msgs.linear.x = 0.0
                self.output_cmd_vel_msgs.angular.z = 0.0
            elif(self.target_depth > self.MIN_CHASE_DISTANCE or self.target_depth < self.MIN_SKID_DISTANCE):
                # In the Chasing Zone #
                # DEPTH PID Controller
                # depth_error = self.target_depth - self.MIN_CHASE_DISTANCE
                # depth_pid = self.PD_Controller(depth_error, self.depth_error1, self.MAX_LINEAR_VEL_OUTPUT, self.DEPTH_kp, self.DEPTH_kd)
                # self.depth_error1 = depth_error
                # self.output_cmd_vel_msgs.linear.x = depth_pid
                # ANGLE PID Controller
                # self.get_logger().info('Angle:%fdegree'%self.target_angle)
                if abs(self.target_angle)<1.5:
                    angle_error = 0
                else:
                    angle_error = -self.target_angle
                angle_pid = self.PD_Controller(angle_error, self.angle_error1, self.MAX_ANGULER_VEL_OUTPUT, self.ANGLE_kp, self.ANGLE_kd)
                self.angle_error1 = angle_error
                self.output_cmd_vel_msgs.angular.z = angle_pid
            else:
                # In the Skid Zone
                self.output_cmd_vel_msgs.linear.x = 0.0
                # ANGLE PID Controller
                if abs(self.target_angle)<1.5:
                    angle_error = 0
                else:
                    angle_error = -self.target_angle
                angle_pid = self.PD_Controller(angle_error, self.angle_error1, self.MAX_ANGULER_VEL_OUTPUT, self.ANGLE_kp, self.ANGLE_kd)
                self.angle_error1 = angle_error
                self.output_cmd_vel_msgs.angular.z = angle_pid
            # Publish Data
            self.follow_vel_pub_.publish(self.output_cmd_vel_msgs)
        else:
            # Clean Output
            self.output_cmd_vel_msgs.linear.x = 0.0
            self.output_cmd_vel_msgs.angular.z = 0.0
            # Publish Data
            if self.follow_flag == True:
                self.follow_vel_pub_.publish(self.output_cmd_vel_msgs)
                self.follow_flag = False

    def PD_Controller(self, error, error1, max, kp, kd):
        output = kp*error + kd*(error-error1)
        if output > max:
            output = max
        if output < -self.MAX_LINEAR_VEL_OUTPUT:
            output = -max
        return output*1.0

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