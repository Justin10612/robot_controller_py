import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.subscription = self.create_subscription(String, 'topic', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

    def timer_callback(self):
        msga = String()
        msga.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msga)
        self.get_logger().info('Publishing: "%s"' % msga.data)
        self.i += 1


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