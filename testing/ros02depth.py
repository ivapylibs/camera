#!/usr/bin/python
#================================ ros02depth ===============================
'''!
@brief  Example usage of depth stream subscription from ROS topic. 

Demonstrate basic usage of camera API to obtain and display depth images
from a ROS topic.  Approach simplifies coding of such an implementation.

'''
#================================ ros02depth ===============================
#
# @author         Patricio A. Vela,     pvela@gatech.edu
# @date           2023/07/13            [Created]
#
#================================ ros02depth ===============================


import cv2
import time
import rospy

import camera.utils.display as display
import camera.rostopic as roscam


cfgCamera = roscam.CfgROSCam()
cfgCamera.topicPath = '/device_0/sensor_1/Color_0/image'
cfgCamera.topicName = 'data'

theCamera = roscam.Depth(cfgCamera)

theCamera.start()


while not rospy.is_shutdown():
    dIm = theCamera.get_frames()


    #display.depth_cv(dIm, ratio=0.5, window_name="Depth Camera Topic." )

    time.sleep(0.25)
    #opKey = cv2.waitKey(1)
    #if opKey == ord('q'):
    #    break


theCamera.stop()

#
#================================ ros02depth ===============================
