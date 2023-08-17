#!/usr/bin/python
#=============================== bag05align ==============================
'''!
@brief  Replay a ROS bag recorded using the D435 ivapylibs API.  
        The bag needs to be decompressed (original bag is decompressed).

Incorporate for loop to go through entire bag. Plot both the color and depth
imagery in two windows if not aligned and in single window if aligned.
This script permit adjusting the align flag.

@todo   Record a brief ROS bag for being the source file. Say like 5 seconds
        to not trigger git large.
'''
#=============================== bag05align ==============================

#
# @author   Patricio A. Vela        pvela@gatech.edu
# @date     2023/08/17              
#
#=============================== bag05align ==============================

import numpy as np
import cv2

import camera.utils.rs_utils as rs_utils

from camera.d435.runner2 import CfgD435
from camera.d435.runner2 import Replay

import camera.utils.display as display


cfgStream = CfgD435.builtForReplay('bagsource.bag')
cfgStream.camera.align = True
theStream = Replay(cfgStream)

# @note Will crash here if the bag file is compressed.
theStream.start()

while True:
  color_image, depth_image, allgood = theStream.get_frames()

  if theStream.configs.camera.align:
    display.rgb_depth_cv(color_image,depth_image,ratio=0.5,window_name="RGBD")
  else:
    display.rgb_cv(color_image,ratio=0.5,window_name="color")
    display.depth_cv(depth_image,ratio=0.5,window_name="depth")

  opKey = cv2.waitKey(1)
  if opKey == ord('q'):
    break

theStream.stop()
display.close_cv("RGBD")
#
#=============================== bag05align ==============================
