#!/usr/bin/python
#================================ ros01basic ===============================
## @file
# @brief  Basic usage of image stream capture from ROS topic. 
# 
# Demonstrate basic usage of camera class to obtain and display images
# from a ROS topic.  This is not necessarily the best approach, but it
# retains compatibility with other camera class instances, thereby
# minimizing code changes when going from direct camera reading to ROS bag
# reading to ROS robot usage.
# 
# The more critical one being from direct camera reading to ROS bag
# reading.
# 
# @quitf

#================================ ros01basic ===============================
#
# @author         Patricio A. Vela,     pvela@gatech.edu
# @date           2023/07/13            [Created]
#
#================================ ros01basic ===============================


import cv2
import time
import rospy
import numpy as np

# import camera.utils.display as display # BAD
import ivapy.display_cv as display
import camera.rostopic as roscam


cfgCamera = roscam.CfgROSCam()
cfgCamera.topicPath = '/device_0/sensor_1/Color_0/image'
cfgCamera.topicName = 'data'

theCamera = roscam.Color(cfgCamera)

theCamera.start()

time.sleep(0.2)

while not rospy.is_shutdown():
    rgb = theCamera.get_frames()

    print(np.shape(rgb))
    display.rgb(rgb, ratio=0.5, window_name="Camera Topic." )

    time.sleep(0.25)
    opKey = cv2.waitKey(1)
    if opKey == ord('q'):
        break


theCamera.stop()

#
#================================ ros01basic ===============================
