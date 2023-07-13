#!/usr/bin/python
#================================ ros03depth ===============================
'''!
@brief  Example usage of depth stream subscription from ROS topic. 

Demonstrate basic usage of camera API to obtain and display depth images
from a ROS topic.  Approach simplifies coding of such an implementation.

'''
#================================ ros03depth ===============================
#
# @author         Patricio A. Vela,     pvela@gatech.edu
# @date           2023/07/13            [Created]
#
#================================ ros03depth ===============================


import cv2
import time
import rospy

import camera.utils.display as display
import camera.rostopic as roscam


cfgCamera = roscam.CfgROSRGBD()
cfgCamera.colorPath = '/device_0/sensor_1/Color_0/image'
cfgCamera.colorName = 'data'
cfgCamera.depthPath = '/device_0/sensor_1/Depth_0/image'
cfgCamera.depthName = 'data'

theCamera = roscam.RGBD(cfgCamera)

theCamera.start()


while not rospy.is_shutdown():
    cIm, dIm = theCamera.get_frames()


    #display.rgb_depth_cv(cIm, dIm, ratio=0.25, window_name="RGBD Camera Topics." )

    time.sleep(0.25)
    #opKey = cv2.waitKey(1)
    #if opKey == ord('q'):
    #    break


theCamera.stop()

#
#================================ ros03depth ===============================
