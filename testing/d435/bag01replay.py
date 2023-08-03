#!/usr/bin/python


import pyrealsense2 as rs
import camera.utils.rs_utils as rs_utils

from camera.d435.runner2 import CfgD435

import camera.utils.display as display

pipe   = rs.pipeline()
config = rs.config()
config.enable_device_from_file('/home/mary/Downloads/16fd1967d2_Garrison.bag')
config.enable_stream(rs.stream.color, rs.format.bgr8, 30)

print("Starting")
profile = pipe.start(config)
# CRASHES HERE. REALSENSE-VIEWER CRASHES WITH BAG FILES TOO.
# NOT SURE WHY. GARRISON SHOULD CONFIRM AND PROVIDE VERSION NUMBER.

#align = rs.align(rs.stream.color)
print("Entering loop")

while True:
  frame = pipe.wait_for_frames()
  print('Got frame')
  color_frame = frame.get_color_frame()
  print('Got color')

  display.rgb_cv(color_frame,ratio=0.5,window_name="color")

  opKey = cv2.waitKey(1)
  if opKey == ord('q'):
    break
