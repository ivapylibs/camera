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
from ivapy import display_cv
import numpy as np

cfgCamera = roscam.CfgROSCam()
cfgCamera.colorPath = '/camera/color'
cfgCamera.colorName = 'image_raw'
cfgCamera.depthPath = '/camera/aligned_depth_to_color'
cfgCamera.depthName = 'image_raw'

theCamera = roscam.RGBD(cfgCamera)

theCamera.start()

theFrame = theCamera.get_frames()

while not rospy.is_shutdown():
    image = theCamera.get_frames()
    
    if image is not None:
        display_cv.rgb_depth(image.color, image.depth, ratio=0.50, window_name="RGBD" )

    # time.sleep(0.25)
    opKey = cv2.waitKey(1)
    if opKey == ord('q'):
       break


theCamera.stop()

#
#================================ ros03depth ===============================
