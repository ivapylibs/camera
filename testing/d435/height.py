#=================================== testing/height ==================================
#
"""

    @brief:     The demo of the tabletop height estimator on the Realsense d435

    @author:    Yiye Chen
    @date:      10/26/2021

"""
#=================================== testing/height ==================================

from struct import error
import matplotlib.pyplot as plt
from mpl_toolkits.axes_grid1 import make_axes_locatable
import numpy as np
import cv2

import camera.d435.runner as d435
from camera.tabletop.height_estimator import HeightEstimator
from camera.utils.display import display_rgb_dep_plt

import camera.utils.display as display

# The D435 starter
d435_configs = d435.D435_Configs(
    W_dep=1280,
    H_dep=720,
    W_color=1920,
    H_color=1080,
    exposure = 100,
    gain = 50
)

d435_starter = d435.D435_Runner(d435_configs)

# The tabletop plane estimator
height_estimator = HeightEstimator(intrinsic=d435_starter.intrinsic_mat)

# Get started
calibrated = False
fh = plt.figure()
ax = fh.add_subplot(1, 1, 1)

divider = make_axes_locatable(ax)
cax     = divider.append_axes("right", size="5%", pad=0.05)

plt.show(block=False)
plt.ion()

# Enter loop.
while(True):
    rgb, dep, success = d435_starter.get_frames()
    #print("The camera gain: {}. The camera exposure: {}".format(d435_starter.get("gain"), d435_starter.get("exposure")))
    if not success:
        print("Cannot get the camera signals. Exiting...")
        exit()

    # Wait for user to trigger calibration.
    if not calibrated:
        display.display_rgb_dep_cv(rgb, dep, ratio=0.5, \
            window_name="Please clear the table and press \"c\" to estimate the tabletop plane. \
                Or press \"q\" to exit.")

        opKey = cv2.waitKey(1)
        if opKey == ord('q'):
            break
        elif opKey == ord('c'):
            # calibrate the height estimator
            height_estimator.calibrate(depth_map=dep)
            calibrated = True
            cv2.destroyAllWindows()

    # Once calibrated, can perform height estimation.
    else:
        height_map = height_estimator.apply(dep)

        #ax.clear()
        #cax.clear()
        im = ax.imshow(height_map)
        ax.set_title("The estimated height")
        plt.colorbar(im, cax=cax)
        plt.draw()
        plt.pause(1)

#
#=================================== testing/height ==================================
