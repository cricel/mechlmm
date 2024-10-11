#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

class RVizPointPublisher:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('rviz_point_publisher', anonymous=True)

        # Create publishers for both points and text markers
        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

        # List of points to display
        self.points = [
            Point(1, 1, 1),
            Point(2, 2, 2),
            Point(3, 3, 3),
            Point(4, 4, 4)
        ]
        
        # Corresponding names for the points
        self.point_names = ['Point 1', 'Point 2', 'Point 3', 'Point 4']

        # Timer to call the callback function every 1 second
        rospy.Timer(rospy.Duration(1.0), self.timer_callback)

    def create_points_marker(self):
        """Create a Marker message for the points."""
        points_marker = Marker()
        points_marker.header.frame_id = "map"  # Make sure RViz is set to this frame
        points_marker.header.stamp = rospy.Time.now()
        points_marker.ns = "points"
        points_marker.id = 0
        points_marker.type = Marker.POINTS
        points_marker.action = Marker.ADD

        # Set the scale of the points (x and y should be equal for spherical points)
        points_marker.scale.x = 0.1  # size of the point
        points_marker.scale.y = 0.1

        # Set the color (r, g, b, a)
        points_marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)  # Red color, fully opaque

        # Add the points to the marker
        points_marker.points = self.points

        return points_marker

    def create_text_marker(self, point, name, marker_id):
        """Create a Marker message for the text labels."""
        text_marker = Marker()
        text_marker.header.frame_id = "map"
        text_marker.header.stamp = rospy.Time.now()
        text_marker.ns = "point_labels"
        text_marker.id = marker_id  # Unique ID for each text marker
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD

        # Set the position of the text (slightly above the point)
        text_marker.pose.position.x = point.x
        text_marker.pose.position.y = point.y
        text_marker.pose.position.z = point.z + 0.2  # Offset above the point

        # Set the text string
        text_marker.text = name

        # Set the scale of the text (size of the font)
        text_marker.scale.z = 0.2  # Height of text

        # Set the color of the text
        text_marker.color = ColorRGBA(0.0, 1.0, 0.0, 1.0)  # Green color, fully opaque

        return text_marker

    def timer_callback(self, event):
        """Callback function that gets called by the timer every second."""
        # Publish the points marker
        points_marker = self.create_points_marker()
        self.marker_pub.publish(points_marker)

        # Publish a text marker for each point
        for i, point in enumerate(self.points):
            text_marker = self.create_text_marker(point, self.point_names[i], i + 1)
            self.marker_pub.publish(text_marker)

if __name__ == '__main__':
    try:
        # Create the point publisher object
        rviz_publisher = RVizPointPublisher()
        
        # Keep the node running
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
