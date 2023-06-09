#!/usr/bin/python
#=============================== test01_basic ==============================
'''!
@brief  Basic usage of the D435 camera for image stream capture.

Simple implementation that uses the default image capture settings and 
captures with existing D435 low-level settings.  If these are changed
using an exernal application, then they should continue to apply when
this basic steam capture test is run.

'''
#=============================== test01_basic ==============================
#
# @author         Yiye Chen.            yychen2019@gatech.edu
# @author         Patricio A. Vela,     pvela@gatech.edu
#
# @date           2021/10/07            [Created]
# @date           2023/05/26            [Modified]
#
#=============================== test01_basic ==============================

import cv2

import camera.utils.display as display
import camera.d435.runner2 as d435


d435_starter = d435.D435_Runner()
d435_starter.start()


while(True):
    rgb, dep, success = d435_starter.get_frames()
    if not success:
        print("Cannot get D435 camera signals. Exiting...")
        exit()

    display.display_rgb_dep_cv(rgb, dep, ratio=0.5, \
                   window_name="Camera signals. Press \'q\' to exit")

    opKey = cv2.waitKey(1)
    if opKey == ord('q'):
        break

d435_starter.stop()

#
#=============================== test01_basic ==============================
