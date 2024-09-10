import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2

import base64

from ollama_image import OllamaCore

import concurrent.futures


class VisionCore:
    def __init__(self, node):
        self.ollama_core = OllamaCore()

        self.executor = concurrent.futures.ThreadPoolExecutor(max_workers=1)
        self.last_future = None

        self.node = node
        self.subscription = node.create_subscription(
            Image,
            '/camera/image_raw',  # Replace with the image topic you are subscribing to
            self.image_callback,
            10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # Convert ROS2 Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Display the image using OpenCV
        cv2.imshow("Camera Image", cv_image)
        
        # Add waitKey to keep the display window open
        if cv2.waitKey(1) == ord('q'):
            self.node.get_logger().info('Quitting...')
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    
    # Create the ROS2 node separately
    node = Node('image_subscriber_node')
    
    # Pass the node to the ImageSubscriber class
    image_subscriber = VisionCore(node)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    # Destroy OpenCV windows
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()




# ollama_core = OllamaCore()

# cam = cv2.VideoCapture(0)

# frame_width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
# frame_height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))

# executor = concurrent.futures.ThreadPoolExecutor(max_workers=1)

# last_future = None

# while True:
#     ret, frame = cam.read()

#     if not ret:
#         break
    
#     _, buffer = cv2.imencode('.jpg', frame)
#     frame_base64 = base64.b64encode(buffer).decode('utf-8')
    
#     if last_future is None or last_future.done():
#         last_future = executor.submit(ollama_core.chat_img, frame_base64)
    
#     cv2.imshow('Camera', frame)

#     if cv2.waitKey(1) == ord('q'):
#         break

# cam.release()
# cv2.destroyAllWindows()