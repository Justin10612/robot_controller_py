import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy


class RobotController(Node):
    # Parameter
    __FOLLOW_MODE_BTN = 1 #a
    __TELEOP_MODE_BTN = 2 #b
    __IDLE_MODE_BTN = 3 #y
    # Mode
    follow_btn_state = False
    teleop_btn_state = False
    idle_btn_state = False
    robot_state = 'IDLE'
    last_robot_state = 'IDLE'
    target_state = True
    # Button State
    follow_btn_flag = True
    teleop_btn_flag = True
    idle_btn_flag = True

    def __init__(self):
        super().__init__('robot_controller')
        # Create publisher
        self.idle_bool_pub_ = self.create_publisher(Bool, 'idle_bool', 10)
        self.follow_bool_pub_ = self.create_publisher(Bool, 'follow_bool', 10)
        self.teleop_bool_pub_ = self.create_publisher(Bool, 'teleop_bool', 10)
        self.robot_state_pub_ = self.create_publisher(String, 'robot_mode', 10)
        # Create subscriber
        self.subscription = self.create_subscription(Joy, 'joy', self.button_read, 10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Robot State: "%s "' % self.robot_state)

    def button_read(self, joy_msg):
        # ############# Idle operate mode btn #############
        if joy_msg.buttons[self.__IDLE_MODE_BTN] == 1 and self.idle_btn_flag :
            self.idle_btn_flag = False
            # self.get_logger().info('IDLE_btn_pressed')
            # Clear status
            self.idle_btn_state = True
        elif joy_msg.buttons[self.__IDLE_MODE_BTN] == 0 and not(self.idle_btn_flag):
            self.idle_btn_flag = True
        else:
            self.idle_btn_state = False
        # ############# Follow btn btn ####################
        if joy_msg.buttons[self.__FOLLOW_MODE_BTN] == 1 and self.follow_btn_flag :
            self.follow_btn_flag = False
            # self.get_logger().info('Follow_btn_pressed')
            # Change State
            self.follow_btn_state = True
        elif joy_msg.buttons[self.__FOLLOW_MODE_BTN] == 0 and not(self.follow_btn_flag):
            self.follow_btn_flag = True
        else:
            self.follow_btn_state = False
        # ############# Teleop operate btn btn #############
        if joy_msg.buttons[self.__TELEOP_MODE_BTN] == 1 and self.teleop_btn_flag :
            self.teleop_btn_flag = False
            # self.get_logger().info('Teleop_btn_pressed')
            # Change State
            self.teleop_btn_state = True
        elif joy_msg.buttons[self.__TELEOP_MODE_BTN] == 0 and not(self.teleop_btn_flag):
            self.teleop_btn_flag = True
        else:
            self.teleop_btn_state = False

        # ############## FSM ##############
        teleop_msg = Bool()
        follow_msg = Bool()
        idle_msg = Bool()
        if self.robot_state=='IDLE' :
            # Set MUX boolean enable
            idle_msg.data = True
            follow_msg.data = False
            teleop_msg.data = False
            # Switch to Follow Mode
            if self.follow_btn_state==True and self.target_state==True:
                self.robot_state = 'FOLLOW'
            # Switch to Teleoperating Mode
            if self.teleop_btn_state==True:
                self.robot_state = 'TELEOP'
        else:
            # Set MUX boolean enable
            idle_msg.data = False
            if self.robot_state=='FOLLOW' :
                follow_msg.data = True
            if self.robot_state=='TELEOP' :
                teleop_msg.data = True
            # Go back to Idel Mode
            if self.idle_btn_state == True:
                self.robot_state = 'IDLE'
    
        # Publish Robot_state
        mode_msg = String()
        mode_msg.data = self.robot_state
        self.robot_state_pub_.publish(mode_msg)
        # Publish mux bool
        self.idle_bool_pub_.publish(idle_msg)
        self.follow_bool_pub_.publish(follow_msg)
        self.teleop_bool_pub_.publish(teleop_msg)
        # Log Robot_state
        if (self.last_robot_state != self.robot_state):
            self.get_logger().info('Robot State: "%s "' % self.robot_state)
        self.last_robot_state = self.robot_state

def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()
    rclpy.spin(robot_controller)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robot_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()