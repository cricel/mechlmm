import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header
from visualization_msgs.msg import Marker

class PointWithLabelPublisher(Node):

    def __init__(self):
        super().__init__('point_with_label_publisher')
        
        # Publishers for Point and Marker
        self.point_publisher = self.create_publisher(PointStamped, '/point', 10)
        self.marker_publisher = self.create_publisher(Marker, '/marker', 10)
        
        # Timer to repeatedly publish
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        # Publish the point
        point_msg = PointStamped()
        point_msg.header = Header()
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.header.frame_id = "map"  # Frame ID should match Rviz2
        point_msg.point.x = 1.0  # Set your desired coordinates
        point_msg.point.y = 2.0
        point_msg.point.z = 0.0
        self.point_publisher.publish(point_msg)
        self.get_logger().info(f"Publishing Point: x={point_msg.point.x}, y={point_msg.point.y}, z={point_msg.point.z}")
        
        # Publish the label (marker)
        marker_msg = Marker()
        marker_msg.header.frame_id = "map"  # Same frame as the point
        marker_msg.header.stamp = self.get_clock().now().to_msg()
        marker_msg.ns = "labels"
        marker_msg.id = 0
        marker_msg.type = Marker.TEXT_VIEW_FACING
        marker_msg.action = Marker.ADD

        # Position the label close to the point
        marker_msg.pose.position.x = point_msg.point.x
        marker_msg.pose.position.y = point_msg.point.y
        marker_msg.pose.position.z = point_msg.point.z + 0.2  # Slight offset above the point

        # Set label text
        marker_msg.text = "Point 1"

        # Set marker properties
        marker_msg.scale.z = 0.2  # Height of the text
        marker_msg.color.r = 1.0  # Red color
        marker_msg.color.g = 1.0  # Green color
        marker_msg.color.b = 1.0  # Blue color
        marker_msg.color.a = 1.0  # Fully opaque

        self.marker_publisher.publish(marker_msg)
        self.get_logger().info("Publishing Marker with Label 'Point 1'")

def main(args=None):
    rclpy.init(args=args)
    node = PointWithLabelPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
