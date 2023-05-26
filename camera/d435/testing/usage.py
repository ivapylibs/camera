"""

    @brief          The basic usage of the d435 runner class

    @author         Yiye Chen.          yychen2019@gatech.edu
    @date           10/07/2021

"""
import matplotlib.pyplot as plt
import cv2

import camera.d435.runner2 as d435

import camera.utils.display as display

d435_configs = d435.CfgD435()
d435_starter = d435.D435_Runner(d435_configs)

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

    #display.display_rgb_dep_plt(rgb, dep, suptitle=None, fh=fh)
    #plt.draw()
    #plt.pause(1)



