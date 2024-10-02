import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class UserInputPublisher(Node):
    def __init__(self):
        super().__init__('user_input_publisher')
        self.publisher_ = self.create_publisher(String, 'operator_cmd', 10)
        self.timer = self.create_timer(1.0, self.publish_user_input)

    def publish_user_input(self):
        user_input = input("What can I do for you: ")
        msg = String()
        msg.data = user_input
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UserInputPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
