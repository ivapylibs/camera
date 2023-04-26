"""

@brief      Intel Realsense D435 camera interfaces.

@author     Yiye Chen.          yychen2019@gatech.edu
            Patricio A. Vela    pvela@gatech.edu

@date       2023/04/26  [created from D435/runner.py, to be deleted]
"""

from dataclasses import dataclass

import numpy as np
import pyrealsense2 as rs

import camera.base as base
import camera.utils.rs_utils as rs_utils

@dataclass
class Config_D435():
    '''!
    Simple configuration structure for the D435 camera.  Has the most basic elements, which
    are the resolutions for the cameras and an optional configuration JSON file to load
    that contains the richer set of parameters.

    NOTE: Supposedly the default depth resolution of 848x480 is optimal for the d435
    camera. [See notes](https://dev.intelrealsense.com/docs/tuning-depth-cameras-for-best-performance)

    @param[in] W_depth  (int). Width of depth image.    (Optional: Defaults to 848)
    @param[in] H_depth  (int). Height of depth image.   (Optional: Defaults to 480)
    @param[in] W_color  (int). Width of color image.    (Optional: Defaults to 1920)
    @param[in] H_color  (int). Height of color image.   (Optional: Defaults to 1080)
    @param[in] cfgfile  (str). Configuration file location (Camera unchanged if empty).
    ''' 
    # IGNORING CONFIG PARMS BELOW BECAUSE NOT USED. INSTEAD GOING WITH CONFIG FILE.
    #    exposure (int. Optional.) The camera exposure. Defaults to None, in which case will \
    #        enable auto_exposure (if gain is also None) or use the default value 50 (according to Yiye's test)
    #    gain (int. Optional.) The camera gain. Defaults to None, in which case will \
    #        enable auto_exposure (if gain is also None) or use the default value 64 (according to Yiye's test)
    
    W_depth: int = 848
    H_depth: int = 480
    W_color: int = 1920
    H_color: int = 1080
    cfgfile: str = ""


#====================================== Color ======================================
#
#
class Color(base.Color):
    """D435 Color image capture class.
    """

    #============================ Color __init___ ============================
    #
    def __init__(self, configs:D435_Configs=D435_Configs()) -> None:
        super().__init__(configs=configs)

        # Should connect and do all kinds of other stuff here.

        return

#====================================== Depth ======================================
#
#
class Depth(base.Color):
    """D435 Depth image capture class.
    """

    #============================ Depth __init___ ============================
    #
    def __init__(self, configs:D435_Configs=D435_Configs()) -> None:
        super().__init__(configs=configs)

        # Should connect and do all kinds of other stuff here.

        return


#====================================== RGBD =====================================
#
#
class RGBD(base.Base):
    '''!
    @brief  D435 Color + Depth (RGB+D) image capture class.  This version captures the raw
            color and depth imagery without performing registration.
    '''

    #============================= RGBD __init___ ============================
    #
    def __init__(self, configs:D435_Configs=D435_Configs()) -> None:
        super().__init__(configs=configs)

        # Should connect and do all kinds of other stuff here.
        return



#=================================== RGBD_Aligned ==================================
#
#

class RGBD_Aligned(RGBD):
    '''!D435 Color + Depth (RGB+D) image capture class.
    '''

    #========================= RGBD_Aligned __init___ ========================
    #
    def __init__(self, configs:D435_Configs=D435_Configs()) -> None:
        super().__init__(configs=configs)

        # Should connect and do all kinds of other stuff here.
        # Pretty much following improved version of code below.
        # See detector realsense testing scripts for how to proceed.
        # See what differs in code below.

        return

        # Ignore the rest for now.
        # This part should do different stuff based on the testing scripts.

        # Configure realsense depth and color streams
        self.pipeline  = rs.pipeline()
        self.rs_config = rs.config()

        self.rs_config.enable_stream(rs.stream.depth, self.configs.W_depth, self.configs.H_dep, 
                                     rs.format.z16, 30)
        self.rs_config.enable_stream(rs.stream.color, self.configs.W_color, self.configs.H_color, rs.format.bgr8, 30)

        # Start streaming
        self.profile = self.pipeline.start(self.rs_config)

        # prepare depth scale: convert sensor unit to meters
        depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()

        # prepare depth2color aligner
        align_to = rs.stream.color
        self.align = rs.align(align_to)

        # set color sensor exposure & camera gain
        # NOTE:color sensor is always the second one: https://github.com/IntelRealSense/librealsense/issues/3558#issuecomment-549405382
        # NOTE: auto_exposure will automatically set both the gain and the exposure. 
        # Setting the exposure or gain will automatically disable exposure. See: https://dev.intelrealsense.com/docs/high-dynamic-range-with-stereoscopic-depth-cameras
        # NOTE:see all available options here:https://intelrealsense.github.io/librealsense/python_docs/_generated/pyrealsense2.option.html
        self.auto_exposure = True   #<- auto exposure mode
        self.color_sensor = self.profile.get_device().query_sensors()[1]
        if self.configs.exposure is not None:
            self.color_sensor.set_option(rs.option.exposure, self.configs.exposure)
        if self.configs.gain is not None:
            self.color_sensor.set_option(rs.option.gain, self.configs.gain)

        # the 3-by-3 intrinsic matrix
        self.intrinsic_mat = None       #<- Need to be set when the frame is get. Didn't find other methods
        for i in range(20):
            self.get_frames()           #<- run several steps of get_frames to initialize the intrinsic. Not sure whether there are better methods

    def get_frames(self, before_scale=False):
        """Get the next frames

        Args:  
            before_scale (bool). Before the scaling. If True, will get the integer depth map before scaling.

        Returns:
            rgb [np.ndarray]: The rgb image
            dep [np.ndarray]: The depth map in meter
            succ_flag [bool]: The indicator for the status of fetching the next frames. \
                If true, then both rgb and depth frames are successfully fetched.
        """
        # Wait for a coherent pair of frames: depth and color
        frames = self.pipeline.wait_for_frames()

        # align depth2color
        frames = self.align.process(frames)

        # split depth and color. <class 'pyrealsense2.video_frame'>
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            return None, None, False
            
        # update the intrinsic matrix stored in the calibrator, in case of the camera profile change
        if self.intrinsic_mat is None:
            intrinsic =  color_frame.profile.as_video_stream_profile().intrinsics
            intrinsic_Mat = rs_utils.rs_intrin_to_M(intrinsic)
            self.intrinsic_mat = intrinsic_Mat

        # Convert realsense images to numpy arrays. Depth image in meters
        depth_raw = np.asanyarray(depth_frame.get_data())
        if before_scale:
            depth_image = depth_raw
        else:
            depth_image = depth_raw * self.depth_scale
        color_image = np.asanyarray(color_frame.get_data())[:,:,::-1]   #<- bgr to rgb

        return color_image, depth_image, True
    
    def get(self, key):
        """Get attributes specified by the key

        Args:
            key (str):      Choices: [exposure, gain, W_rgb, H_rgb, W_dep, H_dep, depth_scale, intrinsic_mat]
        Returns:
            value
        """
        if key == "exposure" or key == "gain":
            value = self.color_sensor.get_option(
                eval("rs.option." + key)
            )
        elif key == "W_rgb" or key == "H_rgb" or key == "W_dep" or key == "H_dep":
            value = eval("self.configs." + key)
        else:
            value = eval("self." + key)

        return value
