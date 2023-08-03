#!/usr/bin/python
#============================== bag01replay ==============================
'''!
@brief  Snag a frame for a ROS bag recorded using the realsense API.  
        The bag needs to be decompressed (original bag is decompressed).

Simple test to grab a frame and move on.
'''
#============================== bag01replay ==============================

#
# @author   Patricio A. Vela        pvela@gatech.edu
# @date     2023/08/03              [from realsense read_bag_example.py]
#
#============================== bag01replay ==============================


import numpy as np
import pyrealsense2 as rs
import cv2

import camera.utils.rs_utils as rs_utils

from camera.d435.runner2 import CfgD435

import camera.utils.display as display

pipe   = rs.pipeline()
config = rs.config()
config.enable_device_from_file('/home/mary/Downloads/16fd1967d2_Garrison.bag')
config.enable_all_streams()

profile = pipe.start(config)
# @note Will crash here is the bag file is compressed.

frame = pipe.wait_for_frames()
color_frame = frame.get_color_frame()
color_image = np.asanyarray(color_frame.get_data())

display.rgb_cv(color_image,ratio=0.5,window_name="color")

opKey = cv2.waitKey(-1)

#
#============================== bag01replay ==============================
