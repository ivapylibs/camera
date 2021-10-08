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
    H_color=1080
)

d435_starter = d435.D435_Runner(d435_configs)

fh = plt.figure()
while(True):
    rgb, dep, success = d435_starter.get_frames()
    if not success:
        print("Cannot get the camera signals. Exiting...")
        exit()

    display.display_rgb_dep(rgb, dep, suptitle="THe D435 camera signals. Press any key to end", fh=fh)
    plt.draw()
    plt.show(block=False)
    if plt.waitforbuttonpress(0.01):
        break



