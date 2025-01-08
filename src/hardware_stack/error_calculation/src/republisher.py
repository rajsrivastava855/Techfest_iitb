#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CameraInfo

class CameraRepublisher:
    def __init__(self):
       
        rospy.init_node('camera_republisher', anonymous=True)
        
       
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        self.info_sub = rospy.Subscriber('/camera/camera_info', CameraInfo, self.info_callback)
        
       
        self.image_pub = rospy.Publisher('/front_camera/image_raw', Image, queue_size=10)
        self.info_pub = rospy.Publisher('/front_camera/camera_info', CameraInfo, queue_size=10)

    def image_callback(self, msg):
       
        self.image_pub.publish(msg)

    def info_callback(self, msg):
      
        self.info_pub.publish(msg)

    def spin(self):
       
        rospy.spin()

if __name__ == '__main__':
    try:
        node = CameraRepublisher()
        node.spin()
    except rospy.ROSInterruptException:
        pass
