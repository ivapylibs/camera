#!/usr/bin/python
#============================== bag03replay ==============================
'''!
@brief  Replay a ROS bag recorded using the realsense API.  
        The bag needs to be decompressed (original bag is decompressed).

Incorporate for loop to go through entire bag. Plot both the color and depth
imagery.
'''
#============================== bag03replay ==============================

#
# @author   Patricio A. Vela        pvela@gatech.edu
# @date     2023/08/03              [from realsense read_bag_example.py]
#
#============================== bag03replay ==============================



import numpy as np
import pyrealsense2 as rs
import cv2

import camera.utils.rs_utils as rs_utils

from camera.d435.runner2 import CfgD435

import camera.utils.display as display

pipe   = rs.pipeline()
config = rs.config()
config.enable_device_from_file('bagsource.bag')
config.enable_all_streams()

profile = pipe.start(config)
# @note Will crash here is the bag file is compressed.

while True:
  frame = pipe.wait_for_frames()

  color_frame = frame.get_color_frame()
  color_image = np.asanyarray(color_frame.get_data())

  depth_frame = frame.get_depth_frame()
  depth_image = np.asanyarray(depth_frame.get_data())

  display.rgb_depth_cv(color_image,depth_image,ratio=0.5,window_name="RGBD")

  opKey = cv2.waitKey(1)
  if opKey == ord('q'):
    break

display.close_cv("RGBD")
#
#============================== bag03replay ==============================
