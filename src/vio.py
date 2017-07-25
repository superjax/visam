#!/usr/bin/python

import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from pyquaternion import Quaternion
from nav_msgs.msg import Odometry
import random
from klt_tracker import KLT_tracker
# from visam import visam

class VIO(KLT_tracker):
    def __init__(self, num_features):
        KLT_tracker.__init__(self, num_features)


    def image_callback(self, msg):
        KLT_tracker.image_callback(self, msg)

if __name__ == "__main__":
    rospy.init_node("rovio")

    thing = VIO(20)

    rospy.spin()
