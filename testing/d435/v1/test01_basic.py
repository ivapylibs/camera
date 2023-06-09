#!/usr/bin/python

import cv2

import camera.d435.runner as d435
import camera.utils.display as display

# settings
vidname = "puzzle_play"        # save video name
W = 1920
H = 1080

# prepare
d435_configs = d435.D435_Configs(
    W_depth=848,
    H_depth=480,
    W_color=W,
    H_color=H,
    exposure = 100,
    gain = 50
)

d435_starter = d435.D435_Runner(d435_configs)

# get started
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


