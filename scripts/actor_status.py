#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2  # Needed on arm64 systems because cv_bridge is commonly amd64
from cv_bridge import CvBridge
import os
import rospkg


def publish_image():
    rospy.init_node("actor_status_node", anonymous=True)
    image_pub = rospy.Publisher("wled_bridge/image", Image, queue_size=1)
    rate = rospy.Rate(0.1)  # Adjust the publishing Hz rate as needed

    bridge = CvBridge()

    while not rospy.is_shutdown():
        # Load a sample PNG image
        image_path = rospkg.RosPack().get_path("actor_status") + "/images/E_Stop.png"

        if os.path.exists(image_path):
            # Read the image
            cv_image = cv2.imread(image_path)

            # Show the image
            #cv2.imshow("Image", cv_image)
            #cv2.waitKey(0)
            #cv2.destroyAllWindows()

            # Convert the image to ROS Image message
            ros_image_msg = bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")

            # Publish the ROS Image message
            image_pub.publish(ros_image_msg)

            rospy.loginfo("Image published to wled_bridge/image topic")

        rate.sleep()


if __name__ == "__main__":
    try:
        publish_image()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
        pass

