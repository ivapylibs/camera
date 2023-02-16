"""

    @brief          The testing script for the extrinsic calibration for the D435 camera
                    using an Aruco tag

    @author         Yiye Chen.          yychen2019@gatech.edu
    @date           10/08/2021

"""
import matplotlib.pyplot as plt
import numpy as np
import cv2

import camera.d435.d435_runner as d435
from camera.extrinsic.aruco import CtoW_Calibrator_aruco
from camera.utils.utils import BEV_rectify_aruco 

import camera.utils.display as display

# The D435 starter
d435_configs = d435.D435_Configs(
    W_dep=1280,
    H_dep=720,
    W_color=1920,
    H_color=1080
)

d435_starter = d435.D435_Runner(d435_configs)

# The aruco calibrator
calibrator_CtoW = CtoW_Calibrator_aruco(
    d435_starter.intrinsic_mat,
    distCoeffs=np.array([0.0, 0.0, 0.0, 0.0, 0.0]),
    markerLength_CL = 0.08,
    maxFrames = 30,
    flag_vis_extrinsic = True,
    flag_print_MCL = True,
    stabilize_version =True 
)

fh = plt.figure()
while(True):
    # get frames
    rgb, dep, success = d435_starter.get_frames()
    if not success:
        print("Cannot get the camera signals. Exiting...")
        exit()

    # calibrate
    M_CL, corners_aruco, img_with_ext, status = calibrator_CtoW.process(rgb, dep) 

    # Bird-eye-view rectification
    if status is True:
        topDown_image, BEV_mat = BEV_rectify_aruco(rgb, corners_aruco,
                target_size=200, mode="same") 
        topDown_image2, BEV_mat2 = BEV_rectify_aruco(rgb, corners_aruco, 
                target_size=200, mode="full") 
    else:
        topDown_image = np.zeros_like(rgb, dtype=np.uint8)


    # visualization
    display.display_images_cv([img_with_ext[:,:,::-1], topDown_image[:,:,::-1]], ratio=0.5, \
        window_name="The extrinsic calibration(right, the aruco frame) and the Bird-eye-view rectification(right). Press \'q\' to exit")

    display.display_images_cv([topDown_image2[:,:,::-1]], ratio=0.5, \
        window_name="The top-down view with the full size")
    

    opKey = cv2.waitKey(1)
    if opKey == ord('q'):
        break



