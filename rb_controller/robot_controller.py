import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Joy


class RobotController(Node):
    # Parameter
    __FOLLOW_MODE_BTN = 1
    __TELEOP_MODE_BTN = 2
    follow_mode_flag = True
    teleop_mode_flag = True

    def __init__(self):
        super().__init__('robot_controller')
        # Create publisher
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        # Create subscriber
        self.subscription = self.create_subscription(Joy, 'joy', self.button_read, 10)
        self.subscription  # prevent unused variable warning

    def button_read(self, joy_msg):
        # Follow mode btn
        if joy_msg.buttons[self.__FOLLOW_MODE_BTN] == 1 and self.follow_mode_flag :
            self.follow_mode_flag = False
            self.get_logger().info('Follow_mode')
        elif joy_msg.buttons[self.__FOLLOW_MODE_BTN] == 0 and not(self.follow_mode_flag):
            self.follow_mode_flag = True
        # Teleop operate mode btn
        if joy_msg.buttons[self.__TELEOP_MODE_BTN] == 1 and self.teleop_mode_flag :
            self.teleop_mode_flag = False
            self.get_logger().info('Teleop_mode')
        elif joy_msg.buttons[self.__TELEOP_MODE_BTN] == 0 and not(self.teleop_mode_flag):
            self.teleop_mode_flag = True

    def timer_callback(self):
        msga = String()
        msga.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msga)
        self.get_logger().info('Publishing: "%s"' % msga.data)
        self.i += 1

    # def mode_FSM():
        


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