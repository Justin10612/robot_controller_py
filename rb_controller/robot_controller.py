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
    follow_mode_state = False
    teleop_mode_state = False
    idle_mode_state = True
    target_state = True
    # Button State
    follow_btn_flag = True
    teleop_btn_flag = True
    idle_btn_flag = True

    def __init__(self):
        super().__init__('robot_controller')
        # Create publisher
        self.idle_bool_pub = self.create_publisher(Bool, 'idle_bool', 10)
        self.follow_bool_pub = self.create_publisher(Bool, 'follow_bool', 10)
        self.teleop_bool_pub = self.create_publisher(Bool, 'teleop_bool', 10)
        # Create subscriber
        self.subscription = self.create_subscription(Joy, 'joy', self.button_read, 10)
        self.subscription  # prevent unused variable warning

    def button_read(self, joy_msg):
        # ############# Idle operate mode btn #############
        if joy_msg.buttons[self.__IDLE_MODE_BTN] == 1 and self.idle_btn_flag :
            self.idle_btn_flag = False
            self.get_logger().info('IDLE_btn_pressed')
            # Clear status
            self.idle_mode_state = True
            self.follow_mode_state = False
            self.teleop_mode_state = False
        elif joy_msg.buttons[self.__IDLE_MODE_BTN] == 0 and not(self.idle_btn_flag):
            self.idle_btn_flag = True
        # ############# Follow btn btn ####################
        if joy_msg.buttons[self.__FOLLOW_MODE_BTN] == 1 and self.follow_btn_flag :
            self.follow_btn_flag = False
            self.get_logger().info('Follow_btn_pressed')
            # Change State
            self.idle_mode_state = False
            self.follow_mode_state = True
        elif joy_msg.buttons[self.__FOLLOW_MODE_BTN] == 0 and not(self.follow_btn_flag):
            self.follow_btn_flag = True
        # ############# Teleop operate btn btn #############
        if joy_msg.buttons[self.__TELEOP_MODE_BTN] == 1 and self.teleop_btn_flag :
            self.teleop_btn_flag = False
            self.get_logger().info('Teleop_btn_pressed')
            # Change State
            self.idle_mode_state = False
            self.teleop_mode_state = True
        elif joy_msg.buttons[self.__TELEOP_MODE_BTN] == 0 and not(self.teleop_btn_flag):
            self.teleop_btn_flag = True

        if self.idle_mode_state==True :
            self.get_logger().info('IDLE')
        elif self.follow_mode_state==True and self.target_state==True and self.teleop_mode_state==False:
            self.get_logger().info('FOLLOW')
        elif self.teleop_mode_state==True and self.follow_mode_state==False:
            self.get_logger().info('TELEOP')

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