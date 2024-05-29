#!/usr/bin/env python3

import os

import cv2  # Needed on arm64 systems because cv_bridge is commonly amd64
import rospkg
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String


class ImagePublisher:
    def __init__(self):
        rospy.init_node("actor_status_display_node", anonymous=True)
        self.image_pub = rospy.Publisher("wled_bridge/image", Image, queue_size=1)
        rospy.Subscriber("led_panel_image_filename", String, self.filename_callback)
        self.bridge = CvBridge()

    def filename_callback(self, msg):
        filename = msg.data
        image_path = rospkg.RosPack().get_path("actor_status_display") + f"/images/{filename}.png"

        if os.path.exists(image_path):
            # Read the image
            cv_image = cv2.imread(image_path)

            # Convert the image to ROS Image message
            ros_image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")

            # Publish the ROS Image message
            self.image_pub.publish(ros_image_msg)

            rospy.loginfo(f"Image '{filename}' published to wled_bridge/image topic")
        else:
            rospy.logwarn(f"Image file '{image_path}' does not exist")

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        image_publisher = ImagePublisher()
        image_publisher.run()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
        pass
