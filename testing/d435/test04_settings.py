#!/usr/bin/python
#============================= test04_settings =============================
"""
@brief          Usage of YAML file to control camera streams used.

Expands on the test03 implementation by loading YAML file that only requests
depth imagery.
"""
#============================= test04_settings =============================
#
# @author         Patricio A. Vela,       pvela@gatech.edu
# @date           2023/06/08              [created]
#
#============================= test04_settings =============================

import cv2
import camera.utils.display as display
import camera.d435.runner2 as d435

d435_configs = d435.CfgD435()
d435_configs.merge_from_file('test04_setup.yaml')
d435_starter = d435.D435_Runner(d435_configs)
d435_starter.start()

while(True):
    rgb, dep, success = d435_starter.get_frames()
    if not success:
        print("Cannot get the camera signals. Exiting...")
        exit()

    display.display_dep_cv(dep, ratio=0.5, \
                   window_name="Depth image. Press \'q\' to exit")

    opKey = cv2.waitKey(1)
    if opKey == ord('q'):
        break

#
#============================= test04_settings =============================
