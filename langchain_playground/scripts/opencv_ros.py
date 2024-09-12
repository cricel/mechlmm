import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

import base64

from langchain_playground.ollama_core import OllamaCore

import concurrent.futures


class VisionCore:
    def __init__(self, node):
        self.ollama_core = OllamaCore()

        self.executor = concurrent.futures.ThreadPoolExecutor(max_workers=1)
        self.last_future = None

        self.node = node
        self.subscription = node.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        cv2.imshow("Camera Image", cv_image)
        
        if cv2.waitKey(1) == ord('q'):
            self.node.get_logger().info('Quitting...')
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    
    node = Node('image_subscriber_node')
    
    image_subscriber = VisionCore(node)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()