import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
import sys
import termios
import tty

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_publisher = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.twist = Twist()
        self.arm_twist = TwistStamped()

    def get_key(self):
        """Capture keyboard input from the terminal."""
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))
        return key

    def run(self):
        """Main loop to read keyboard inputs and publish corresponding cmd_vel messages."""
        try:
            print("Use WASD to move, Q to quit")
            while True:
                key = self.get_key()

                if key == 'w':
                    self.arm_twist.header.frame_id="link2"
                    self.arm_twist.header.stamp = self.get_clock().now().to_msg()
                    self.arm_twist.twist.linear.x = 0.0
                    self.arm_twist.twist.linear.y = 0.0
                    self.arm_twist.twist.linear.z = 0.2
                    self.arm_twist.twist.angular.x = 0.0
                    self.arm_twist.twist.angular.y = 0.0
                    self.arm_twist.twist.angular.z = 0.0


                elif key == 's':
                    self.arm_twist.header.frame_id="link2"
                    self.arm_twist.header.stamp = self.get_clock().now().to_msg()
                    self.arm_twist.twist.linear.x = 0.0
                    self.arm_twist.twist.linear.y = 0.0
                    self.arm_twist.twist.linear.z = -0.2
                    self.arm_twist.twist.angular.x = 0.0
                    self.arm_twist.twist.angular.y = 0.0
                    self.arm_twist.twist.angular.z = 0.0


                elif key == 'a':
                    self.arm_twist.header.frame_id="link2"
                    self.arm_twist.header.stamp = self.get_clock().now().to_msg()
                    self.arm_twist.twist.linear.x = -0.2
                    self.arm_twist.twist.linear.y = 0.0
                    self.arm_twist.twist.linear.z = 0.0
                    self.arm_twist.twist.angular.x = 0.0
                    self.arm_twist.twist.angular.y = 0.0
                    self.arm_twist.twist.angular.z = 0.0

                elif key == 'd':
                    self.arm_twist.header.frame_id="link2"
                    self.arm_twist.header.stamp = self.get_clock().now().to_msg()
                    self.arm_twist.twist.linear.x = 0.2
                    self.arm_twist.twist.linear.y = 0.0
                    self.arm_twist.twist.linear.z = 0.0
                    self.arm_twist.twist.angular.x = 0.0
                    self.arm_twist.twist.angular.y = 0.0
                    self.arm_twist.twist.angular.z = 0.0

                elif key == 'q':
                    self.arm_twist.header.frame_id="link2"
                    self.arm_twist.header.stamp = self.get_clock().now().to_msg()
                    self.arm_twist.twist.linear.x = 0.0
                    self.arm_twist.twist.linear.y = 0.0
                    self.arm_twist.twist.linear.z = 0.0
                    self.arm_twist.twist.angular.x = 0.0
                    self.arm_twist.twist.angular.y = 0.0
                    self.arm_twist.twist.angular.z = -1.0

                elif key == 'e':
                    self.arm_twist.header.frame_id="link2"
                    self.arm_twist.header.stamp = self.get_clock().now().to_msg()
                    self.arm_twist.twist.linear.x = 0.0
                    self.arm_twist.twist.linear.y = 0.0
                    self.arm_twist.twist.linear.z = 0.0
                    self.arm_twist.twist.angular.x = 0.0
                    self.arm_twist.twist.angular.y = 0.0
                    self.arm_twist.twist.angular.z = 1.0

                elif key == 'z':
                    self.arm_twist.header.frame_id="link2"
                    self.arm_twist.header.stamp = self.get_clock().now().to_msg()
                    self.arm_twist.twist.linear.x = 0.0
                    self.arm_twist.twist.linear.y = 0.0
                    self.arm_twist.twist.linear.z = 0.0
                    self.arm_twist.twist.angular.x = 0.0
                    self.arm_twist.twist.angular.y = -1.0
                    self.arm_twist.twist.angular.z = 0.0

                elif key == 'c':
                    self.arm_twist.header.frame_id="link2"
                    self.arm_twist.header.stamp = self.get_clock().now().to_msg()
                    self.arm_twist.twist.linear.x = 0.0
                    self.arm_twist.twist.linear.y = 0.0
                    self.arm_twist.twist.linear.z = 0.0
                    self.arm_twist.twist.angular.x = 0.0
                    self.arm_twist.twist.angular.y = 1.0
                    self.arm_twist.twist.angular.z = 0.0

                elif key == 'm':  # Quit
                    break
                else:
                    self.twist.linear.x = 0.0
                    self.twist.angular.z = 0.0

                # Publish the Twist message
                # self.publisher.publish(self.twist)
                self.arm_publisher.publish(self.arm_twist)

        except Exception as e:
            self.get_logger().error(f"Error: {e}")
        finally:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.publisher.publish(self.twist)
            print("Exiting...")

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
