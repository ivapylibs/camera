#!/usr/bin/python
#============================== bag02replay ==============================
'''!
@brief  Replay a ROS bag recorded using the realsense API.  
        The bag needs to be decompressed (original bag is decompressed).

Incorporate for loop to go through entire bag. Display color image topic
only.
'''
#============================== bag02replay ==============================

#
# @author   Patricio A. Vela        pvela@gatech.edu
# @date     2023/08/03              [from realsense read_bag_example.py]
#
#============================== bag02replay ==============================



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
# @note Will crash here if the bag file is compressed.

while True:
  frame = pipe.wait_for_frames()

  color_frame = frame.get_color_frame()
  color_image = np.asanyarray(color_frame.get_data())

  display.rgb_cv(color_image,ratio=0.5,window_name="color")

  opKey = cv2.waitKey(1)
  if opKey == ord('q'):
    break

display.close_cv("color")
#
#============================== bag02replay ==============================
