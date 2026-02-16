#!/usr/bin/python
#================================ ros04process ===============================
'''!
@brief  Example usage of process loop run using ros topic subscribe 
        to camera. Ideally should be done with callbacks, but since
        both depth and color are needed for processing, using a loop.

'''
#================================ ros04process ===============================
#
# @author         Nihit Agarwal,        nagarwal90@gatech.edu
# @date           2025/02/10            [Created]
#
#================================ ros04process ===============================


import cv2
import time
import rospy

import camera.utils.display as display
import camera.rostopic as roscam
from ivapy import display_cv
import numpy as np

count =0 
def dummyFunc(theFrame):
    global count
    count = count + 1
    print(f"Frame count: {count}")
cfgCamera = roscam.CfgROSCam()
cfgCamera.colorPath = '/camera/color'
cfgCamera.colorName = 'image_raw'
cfgCamera.depthPath = '/camera/aligned_depth_to_color'
cfgCamera.depthName = 'image_raw'
cfgCamera.align = True
cfgCamera.process = dummyFunc
cfgCamera.depth_scale = 0.001
cfgCamera.bad = 2.2

theCamera = roscam.RGBDListener(cfgCamera)

if __name__ == '__main__':
    theCamera.start()
    while not rospy.is_shutdown():
        if theCamera.frame is None:
            continue
        display_cv.rgb_depth(theCamera.frame.color, theCamera.frame.depth)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()
            
#
#================================ ros04process ===============================
