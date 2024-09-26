import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from mechllm_core import MechLLMCore
import utilities_core

import threading

class ImageSubscriber(Node):
    """
    Create an ImageSubscriber class, which is a subclass of the Node class.
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """
        super().__init__('image_subscriber')
        
        self.subscription = self.create_subscription(
            Image, 
            '/camera/image_raw', 
            self.listener_callback, 
            10
        )
        self.subscription
        
        self.br = CvBridge()

        self.mechllm_core = MechLLMCore()
        # self.executor = concurrent.futures.ThreadPoolExecutor(max_workers=1)
        # self.image_analyse_thread = None

        self.lock = threading.Lock()

        self.is_llm_processing = False
   
    def listener_callback(self, data):
        """
        Callback function.
        """
        current_frame = self.br.imgmsg_to_cv2(data)

        thread = threading.Thread(target=self.run_with_lock, args=(current_frame,))
        thread.start()

        # Display image
        cv2.imshow("camera", current_frame)
        
        cv2.waitKey(1)

    def run_with_lock(self, _frame):
        with self.lock:
            self.long_running_function(_frame)

    def long_running_function(self, _frame):
        base64_image = utilities_core.opencv_frame_to_base64(_frame)
        image_url = f"data:image/jpeg;base64,{base64_image}"

        tag = {"filename": "test"}

        question = "analysis this image, and give me a detail break down of list of objects in the image"

        json_schema = {
            "title": "image_analysis",
            "description": "give a detail analysis of what happen in the image",
            "type": "object",
            "properties": {
                "objects": {
                    "type": "object",
                    "description": "the list of objects",
                    "properties": {
                        "name": {
                            "type": "string",
                            "description": "the name of the object detected",
                        },
                        "position": {
                            "type": "array",
                            "items": {
                                "type": "number"
                            },
                            "description": "the bounding box coordinate of the object detected, such as ['top_left_x', 'top_left_y', 'bottom_right_x', 'bottom_right_y']",
                        },
                        "features": {
                            "type": "array",
                            "items": {
                                "type": "string"
                            },
                            "description": "the key features of the object detected",
                        },
                    }
                },
                "description": {
                    "type": "string",
                    "description": "Overall description of what is seen in the image"
                }
            },
            "required": ["objects", "description"]
        }

        self.mechllm_core.chat_img(question, image_url, json_schema, tag)
  

def main(args=None):
    rclpy.init(args=args)
    
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    
    rclpy.shutdown()
  
if __name__ == '__main__':
    main()