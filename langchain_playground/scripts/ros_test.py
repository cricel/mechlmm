import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
import threading

class LongRunningFunctionNode(Node):
    def __init__(self):
        super().__init__('long_running_function_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        # Lock to ensure only one thread runs the long-running task at a time
        self.lock = threading.Lock()

    def listener_callback(self, msg):
        # self.get_logger().info('Received message: "%s"' % msg.data)
        
        # Start long-running function, only if the lock is available
        thread = threading.Thread(target=self.run_with_lock, args=(msg.data,))
        thread.start()

    def run_with_lock(self, data):
        # Acquire the lock before running the long task
        with self.lock:
            self.long_running_function(data)

    def long_running_function(self, data):
        self.get_logger().info('Starting long-running task...')
        # Simulate a long-running process
        import time
        time.sleep(5)  # Replace this with your actual long-running code
        self.get_logger().info('Finished long-running task with data:')

def main(args=None):
    rclpy.init(args=args)

    long_running_function_node = LongRunningFunctionNode()

    rclpy.spin(long_running_function_node)

    long_running_function_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
