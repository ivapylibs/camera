"""

    @brief          The basic usage of the d435 runner class

    @author         Yiye Chen.          yychen2019@gatech.edu
    @date           10/07/2021

"""
import matplotlib.pyplot as plt
import cv2

import camera.d435.d435_runner as d435

import Surveillance.utils.display as display

d435_configs = d435.D435_Configs(
    W_dep=1280,
    H_dep=720,
    W_color=1920,
    H_color=1080,
    exposure = 100,
    gain = 50
)

d435_starter = d435.D435_Runner(d435_configs)

#fh = plt.figure()
#plt.show()
#plt.ion()
while(True):
    rgb, dep, success = d435_starter.get_frames()
    #print("The camera gain: {}. The camera exposure: {}".format(d435_starter.get("gain"), d435_starter.get("exposure")))
    if not success:
        print("Cannot get the camera signals. Exiting...")
        exit()

    display.display_rgb_dep_cv(rgb, dep, ratio=0.5, window_name="THe camera signals. (color-scaled depth). Press \'q\' to exit")

    opKey = cv2.waitKey(1)
    if opKey == ord('q'):
        break

    #display.display_rgb_dep_plt(rgb, dep, suptitle=None, fh=fh)
    #plt.draw()
    #plt.pause(1)



