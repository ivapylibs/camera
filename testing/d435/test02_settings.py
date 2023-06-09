#!/usr/bin/python
#============================= test02_settings =============================
"""
@brief          More advanced setup of D435 camera from stored JSON settings.

Expands on the test01 implementation by permitting the loading of setting
information from a YAML file with an option JSON file for more detailed D435
settings (as obtained from the realsense_viewer application).
"""
#============================= test02_settings =============================
#
# @author         Patricio A. Vela,       pvela@gatech.edu
# @date           2023/05/26              [created]
#
#============================= test02_settings =============================

import cv2
import camera.utils.display as display
import camera.d435.runner2 as d435

d435_configs = d435.CfgD435()
d435_configs.merge_from_file('test02_setup.yaml')
d435_starter = d435.D435_Runner(d435_configs)
d435_starter.start()

while(True):
    rgb, dep, success = d435_starter.get_frames()
    if not success:
        print("Cannot get the camera signals. Exiting...")
        exit()

    display.display_rgb_dep_cv(rgb, dep, ratio=0.5, \
                   window_name="Camera signals. (color-scaled depth). Press \'q\' to exit")

    opKey = cv2.waitKey(1)
    if opKey == ord('q'):
        break

#
#============================= test02_settings =============================
