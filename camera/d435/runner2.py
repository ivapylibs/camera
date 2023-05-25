#=============================== camera/d435 ===============================
'''!
@brief          The d435 camera runner.

'''
#=============================== camera/d435 ===============================
#
# @author         Yiye Chen.          yychen2019@gatech.edu
# @author         Patricio A. Vela    pvela@gatech.edu
# @date           [created] 10/07/2021
#
# NOTE: python 4 space indent. tab converts to 4 spaces. 100 columns.

from dataclasses import dataclass
from yacs.config import CfgNode


import numpy as np
import pyrealsense2 as rs

import camera.base as base
import camera.utils.rs_utils as rs_utils

class CfgD435(CfgNode):
    '''!

    @brief  Configuration setting specifier for the D435 camera.

    For detailed explanations and options, please refer to the D435 official document.
    NOTE: The default depth resolution 848x480 is explained to be optimal for the d435 camera. 
    See [realsense documentation](https://dev.intelrealsense.com/docs/tuning-depth-cameras-for-best-performance)
    '''

    #=============================== __init__ ==============================
    #
    '''!
    @brief        Constructor of perception configuration node for Mary.

    @param[in]    cfg_files   List of config files to load to merge settings.
    '''
    # exposure The camera exposure. Defaults to None, in which case will enable
    # auto_exposure (if gain is also None) or use the default value 50
    # (according to Yiye's test) 
    # gain (int. Optional.) The camera gain.
    # Defaults to None, in which case will enable auto_exposure (if gain is
    # also None) or use the default value 64 (according to Yiye's test)
    def __init__(self, init_dict=None, key_list=None, new_allowed=True):
      
      # SHOULD SET BASIC ELEMENTS FOR THE REALSENSE. HARD CODED AND NOT FROM
      # A FILE.
      # camera.depth_on  = False
      # camera.depth_res = [848, 480]
      # camera.depth_fps = 30
      #
      # camera.color_on  = True
      # camera.color_res = [1920, 1080]
      # camera.color_fps = 30
      #
      # exposure ???
      # gain ???
      #
      super().__init__(init_dict, key_list, new_allowed)

      # self.merge_from_lists(XX)

    
class D435_Runner(base.Base):
    """D435_Runner
    """

    def __init__(self, configs:camConfig=CfgD435()) -> None:
        super().__init__(configs=configs)

        # Configure realsense depth and color streams
        self.pipeline = rs.pipeline()
        self.rs_config = rs.config()

        if self.settings.camera.loadConfig:
            adv_mode = rs.rs400_advanced_mode(device)
            if not adv_mode.is_enabled():
                print('Issues with advanced mode')
                quit()

            configFile = open(os.path.expanduser(self.settings.camera.config_file) )
            configStr = json.load(configFile)
            configStr = str(configStr).replace("'", '\"')
            configFile.close()

            print('Loaded JSON configuration and applying.')
            adv_mode.load_json(configStr)

        # Enable / start streaming 
        if (self.settings.camera.depth.on):
          self.rs_config.enable_stream(rs.stream.depth,  \
                  self.configs.depth.res(1), self.configs.depth.res(2), \
                  rs.format.z16, self.settings.depth_fps)

        if (self.settngs.camera.color.on):
          self.rs_config.enable_stream(rs.stream.color, \
                  self.configs.depth.res(1), self.configs.depth.res(2), \
                  rs.format.bgr8, self.configs.depth.fps)


        # SHOULD THIS REALLY BE IN INIT?? OR SHOULD IT BE IN A START ROUTINE??
        # I PREFER A START ROUTINE.

        # Start streaming
        self.profile = self.pipeline.start(self.rs_config)

        # prepare depth scale: convert sensor unit to meters
        depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()

        # prepare depth2color aligner

        # HAVE THIS BE PART OF camera.align FLAG.
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
