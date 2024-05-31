#!/usr/bin/env python3

import os

import cv2  # Needed on arm64 systems because cv_bridge is commonly amd64
import rospkg
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

image_dict={
    # Route Shorthands
    "BLANK":"Black.png",
    "STOP":"E_Stop.png",
    "F3-1":"Route_F3-1.png",
    "F3-2":"Route_F3-2.png",
    "F3-3":"Route_F3-3.png",
    "F4-1":"Route_F4-1.png",
    "F4-2":"Route_F4-2.png",
    "F4-3":"Route_F4-3.png",
    "F5-1":"Route_F5-1.png",
    "F5-2":"Route_F5-2.png",
    "F5-3":"Route_F5-3.png",
    "F5-4":"Route_F5-4.png",
    "F6-2":"Route_F6-2.png",
    "F7-1":"Route_F7-1.png",
    "F7-2":"Route_F7-2.png",
    "MAIN":"Route_Main.png",
    "Q1":"Route_Q1.png",
    "Q2":"Route_Q2.png",
    "Q3":"Route_Q3.png",
    "Q4":"Route_Q4.png",
    "Q5":"Route_Q5.png",
    "Q6":"Route_Q6.png",

    # Sign Shorthands
    "SBLANK":"Lane_Follow_Blank.png",
    "BARREL":"Sign_Barrel.png",
    "PARK":"Sign_Parking.png",
    "PED":"Sign_Pedestrian.png",
    "POTHOLE":"Sign_Pothole.png",
    "STOPSIGN":"Sign_Stop.png",
    "TIRE":"Sign_Tire.png",
    
    # Lane Shorthands
    "LBLANK":"Lane_Follow_Blank.png",
    "LACTIVE":"Lane_Follow_Active.png",
    "LWARN":"Lane_Follow_Warning.png",

}

class ImagePublisher:
    def __init__(self):
        rospy.init_node("actor_status_display_node", anonymous=True)
        self.image_pub = rospy.Publisher("wled_bridge/image", Image, queue_size=1)
        rospy.Subscriber("led_panel_image_filename", String, self.filename_callback)
        self.bridge = CvBridge()

    def filename_callback(self, msg):
        filename = msg.data
        # image_path = rospkg.RosPack().get_path("actor_status_display") + f"/images/{filename}.png"
        
        #New Dynamic Image Generator
        cv_image = self.create_image("MAIN","PED","LBLANK") # ADD INPUT MESSAGES HERE TO CHANGE THEM
        ros_image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        self.image_pub.publish(ros_image_msg)
        # if os.path.exists(image_path):
        #     # Read the image
        #     cv_image = cv2.imread(image_path)

        #     # Convert the image to ROS Image message
        #     ros_image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")

        #     # Publish the ROS Image message
        #     self.image_pub.publish(ros_image_msg)

        #     rospy.loginfo(f"Image '{filename}' published to wled_bridge/image topic")
        # else:
        #     rospy.logwarn(f"Image file '{image_path}' does not exist")
            
    def create_image(self,route_name = "BLANK", detected_name = "SBLANK", lane_name = "LBLANK"):
        image_path = rospkg.RosPack().get_path("actor_status_display") + "/"
        #Grab the file name if it uses shorthand, otherwise use the name as is
        if(route_name in image_dict.keys()):
            route = cv2.imread(f"{image_path}images/{image_dict[route_name]}")
        else:
            route = cv2.imread(f"{image_path}images/{route_name}")
        if(detected_name in image_dict.keys()):
            obj = cv2.imread(f"{image_path}images/{image_dict[detected_name]}")
        else:
            obj = cv2.imread(f"{image_path}images/{detected_name}")
        if(lane_name in image_dict.keys()):
            lane = cv2.imread(f"{image_path}images/{image_dict[lane_name]}")
        else:
            lane = cv2.imread(f"{image_path}images/{lane_name}")

        # Concat Sign and Lane images
        obj_lane = cv2.hconcat([obj,lane])

        # Concat the combined sign and lane image with the route image
        final = cv2.hconcat([route,obj_lane])

        # Display image for testing
        # cv2.imshow("Finished",im_f)
        # cv2.waitKey(0)

        return final
    
    def run(self):
        rospy.spin()
        
    # def create_image(self, route_name, detected_name, lane_name):
        


if __name__ == "__main__":
    try:
        image_publisher = ImagePublisher()
        image_publisher.run()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
        pass
