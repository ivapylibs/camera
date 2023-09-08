#=================================== camera/d435 ===================================
'''!
@brief          The Intel Realsense D435 camera runner.

This class API serves as a simpler wrapper for the D435 python interface,
which itself is a wrapper for the C++ implementation.

'''
#=================================== camera/d435 ===================================
#
# @author         Yiye Chen.          yychen2019@gatech.edu
# @author         Patricio A. Vela    pvela@gatech.edu
# @date           [created] 10/07/2021
#
# NOTE: python 4 space indent. tab converts to 4 spaces. 100 columns.
#
#=================================== camera/d435 ===================================

from dataclasses import dataclass
from yacs.config import CfgNode

import numpy as np
import pyrealsense2 as rs

import camera.base as base
import camera.utils.rs_utils as rs_utils
from camera.base import ImageRGBD


import camera.utils.display as display


#================================== CfgD435 ==================================
#
class CfgD435(CfgNode):
    '''!

    @brief  Configuration setting specifier for the D435 camera.

    For detailed explanations and options, refer to official D435 documentation.

    @note   Currently not using a CfgCamera super class. Probably best to do so.
    '''

    #=============================== __init__ ==============================
    #
    '''!
    @brief        Constructor of perception configuration node for Mary.

    @param[in]    cfg_files   List of config files to load to merge settings.
    '''
    def __init__(self, init_dict=None, key_list=None, new_allowed=True):
      
      if (init_dict == None):
        init_dict = CfgD435.get_default_settings()

      super().__init__(init_dict, key_list, new_allowed)

      # self.merge_from_lists(XX)


    #========================= get_default_settings ========================
    #
    # @brief    Recover the default settings in a dictionary.
    #
    # The default depth resolution 848x480 is explained to be optimal for the d435 camera. 
    # See [realsense documentation](https://dev.intelrealsense.com/docs/tuning-depth-cameras-for-best-performance)
    #
    # The camera exposure defaults to None, in which case will enable auto_exposure (if gain 
    # is also None) or use the default value 50 (according to Yiye's test).
    # The camera gain defaults to None, in which case will enable auto_exposure (if gain is
    # also None) or use the default value 64 (according to Yiye's test)
    #
    @staticmethod
    def get_default_settings():
        '''!
        @brief  Defines most basic, default settings for RealSense D435.

        @param[out] default_dict    Dictionary populated with minimal set of default settings.
        '''
        default_dict = dict( camera = dict ( 
              config = dict(load = False, file = "", ros = ""),
              depth  = dict(use = True, res = [848, 480], fps = 30),
              color  = dict(use = True , res = [1920, 1080], fps = 30) ,
              align  = False, exposure = None, gain = None,
              ros    = dict(enable = False, depth = dict(pub = False, topic = ''), \
                                            color = dict(pub = False, topic = ''), \
                            load = False, file = '') ) )
        return default_dict

    # @note Need to fix it so that defaults permit not using one of the streams.
    #       Ignoring for now in the interest of progress.  Will need to circle
    #       back eventually.

    #=========================== builtForReplay ==========================
    #
    @staticmethod
    def builtForReplay(rosFile = None):

        replayCfg = CfgD435()
        replayCfg.camera.ros.enable = True
        replayCfg.camera.ros.depth.pub = replayCfg.camera.depth.use
        replayCfg.camera.ros.depth.topic = '/device_0/sensor_0/Depth_0/image/data'
        replayCfg.camera.ros.color.pub = replayCfg.camera.color.use
        replayCfg.camera.ros.color.topic = '/device_0/sensor_1/Color_0/image/data'

        if rosFile is not None:
            replayCfg.camera.ros.load = True
            replayCfg.camera.ros.file = rosFile

        return replayCfg
    
#=============================== D435_Runner ===============================
#

class D435_Runner(base.Base):
    """D435_Runner
    """

    #============================== __init__ =============================
    #
    def __init__(self, configs=CfgD435()) -> None:
        '''!
        @brief  Constructor for Intel Realsense D435 camera interface.

        @param[in]  configs     Settings to apply.
        '''
        super().__init__(configs=configs)

        self.Kdepth = None
        self.gCD    = None

        # Configure realsense depth and color streams. Load file if specified.
        self.pipeline = rs.pipeline()
        self.rs_config = rs.config()

        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = self.rs_config.resolve(pipeline_wrapper)
        self.profile = pipeline_profile


        if self.configs.camera.config.load:
            import os
            import json

            device = pipeline_profile.get_device()

            adv_mode = rs.rs400_advanced_mode(device)
            if not adv_mode.is_enabled():
                print('Issues with advanced mode')
                quit()

            if self.configs.camera.config.ros in (None,''):
                configFile = open(os.path.expanduser(self.configs.camera.config.file) )
            else:
                import rospkg
               
                rpkg       = rospkg.RosPack()
                pkgPath    = rpkg.get_path(self.configs.camera.config.ros)
                configFile = os.path.join(pkgPath,self.configs.camera.config.file)

            configStr = json.load(configFile)
            configStr = str(configStr).replace("'", '\"')
            configFile.close()

            print('Loaded JSON configuration and applying.')
            adv_mode.load_json(configStr)

        # Configure streaming sources.
        if (self.configs.camera.depth.use):
          self.rs_config.enable_stream(rs.stream.depth,  \
                  self.configs.camera.depth.res[0], self.configs.camera.depth.res[1], \
                  rs.format.z16, self.configs.camera.depth.fps)

          # Recover depth scale to convert sensor units to meters.
          depth_sensor      = self.profile.get_device().first_depth_sensor()
          self.depth_scale  = depth_sensor.get_depth_scale()
          self.depth_sensor = depth_sensor

        if (self.configs.camera.color.use):
          self.rs_config.enable_stream(rs.stream.color, \
                  self.configs.camera.color.res[0], self.configs.camera.color.res[1], \
                  rs.format.bgr8, self.configs.camera.color.fps)

        # Set color sensor exposure & camera gain
        # The color sensor is always [the second one](https://github.com/IntelRealSense/librealsense/issues/3558#issuecomment-549405382)
        # auto_exposure will automatically set both the gain and the exposure. 
        # Setting the exposure or gain will [automatically disable exposure](https://dev.intelrealsense.com/docs/high-dynamic-range-with-stereoscopic-depth-cameras)
        # See all available options [here](https://intelrealsense.github.io/librealsense/python_docs/_generated/pyrealsense2.option.html)

        self.auto_exposure = True   #<- auto exposure mode
        self.color_sensor = self.profile.get_device().query_sensors()[1]

        if self.configs.camera.exposure is not None:
            self.color_sensor.set_option(rs.option.exposure, self.configs.camera.exposure)
            self.auto_exposure = False

        if self.configs.camera.gain is not None:
            self.color_sensor.set_option(rs.option.gain, self.configs.camera.gain)
            self.auto_exposure = False

        self.align = None;


    #=============================== start ===============================
    #
    def start(self):
        '''!
        @brief  Start capturing the stream.

        @note   Right now the construction does this, which is poor design.
        @todo   Should implement start/stop functionality and capture boolean.
        '''

        # Start streaming
        self.profile = self.pipeline.start(self.rs_config)

        # Once started, the camera intrinsics are available.
        # @note Not sure how well code works for multiple cameras. Assuming one for now.
        if (self.configs.camera.color.use):
            profC   = self.profile.get_stream(rs.stream.color)          # Fetch color profile
            intr    = profC.as_video_stream_profile().get_intrinsics()  # Fetch intrinsics
            self.K  = rs_utils.rs_intrin_to_M(intr)

        if (self.configs.camera.depth.use):
            if (self.configs.camera.align):
                self.Kdepth = self.K                                        # Depth = color 
                self.gCD    = np.eye(4);
            else:
                profD   = self.profile.get_stream(rs.stream.depth)          # Fetch depth profile
                intr    = profD.as_video_stream_profile().get_intrinsics()  # Fetch intrinsics
                self.Kdepth = rs_utils.rs_intrin_to_M(intr)

                if (self.configs.camera.color.use):                         # g^C_D extrinsic
                    extr      = profD.get_extrinsics_to(profC) 
                    self.gCD  = rs_utils.rs_extrin_to_M(extr)

        if (self.configs.camera.align):
            align_to   = rs.stream.color
            self.align = rs.align(align_to)


    #================================ stop ===============================
    #
    def stop(self):
        '''!
        @brief  Stop capturing the stream.

        @note   Right now not implemented.
        @todo   Should implement start/stop functionality and capture boolean.
        '''
        pass

    #============================= get_frames ============================
    #
    # @todo Rename to "capture" so that it is agnostic to what is being
    #       captured.  The setup should determine that and the code should
    #       be consistent with the setup, or at least sanity check the
    #       results (by checking for None when this code is upgraded).
    #
    def get_frames(self, before_scale=False):
        """!
        @brief  Get the next frame(s)

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

        # Align depth to color if enabled prior to starting.
        if (self.configs.camera.align):
            frames = self.align.process(frames)

        # split depth and color. <class 'pyrealsense2.video_frame'>
        # Convert realsense images to numpy arrays. Depth image in meters
        allGood = True
        if (self.configs.camera.depth.use):
            depth_frame = frames.get_depth_frame()
            if not depth_frame:
                allGood = False
            else:
                depth_raw = np.asanyarray(depth_frame.get_data())
                if before_scale:
                    depth_image = depth_raw
                else:
                    depth_image = depth_raw * self.depth_scale
        else:
            depth_image = None

        if (self.configs.camera.color.use):
            color_frame = frames.get_color_frame()
            if not color_frame:
                allGood = False
            else:
                color_image = np.asanyarray(color_frame.get_data())[:,:,::-1] # BGR to RGB
        else:
            color_image = None

        # Update intrinsic matrix stored in calibrator, in case of camera profile change.
        #if self.K is None:
        #    intrinsic =  color_frame.profile.as_video_stream_profile().intrinsics
        #    intrinsic_Mat = rs_utils.rs_intrin_to_M(intrinsic)
        #    self.K = intrinsic_Mat

        return color_image, depth_image, allGood
    
    #============================ captureRGBD ============================
    #
    #
    def captureRGBD(self, before_scale=False):
        """!
        @brief  Snag the next set of RGB and D frames. 

        Captures the color and depth pair as an RGBD image instance. The depth
        map should be returned in standard units (meters).

        @param[in]  before_scale    [bool] True = integer depth map before scaling.

        @param[out] data        RGBD image data as an ImageRGBD element.
        @param[out] flSuccess   Success flag. True = both frames fetched.
        """

        theImage = ImageRGBD()
        theImage.color, theImage.depth, flSuccess = self.get_frames(before_scale)

        return (theImage, flSuccess)

    #================================ get ================================
    #
    # @brief    Get attributes based on keys.
    #
    # @todo     Need to recode this part.  It is not up to date.
    def get(self, key):
        """Get attributes specified by the key

        Args:
            key (str):      Choices: [exposure, gain, W_rgb, H_rgb, W_dep, H_dep, depth_scale, K]
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

    #================================= display =================================
    #
    # @brief    Display latest frames.
    #
    def display(self, rgb, depth):

        display.display_rgb_dep_cv(rgb, dep, ratio=0.5, \
                   window_name="Camera signals. Press \'q\' to exit")



#====================================== Replay =====================================
#

class Replay(D435_Runner):
    """!
    @brief  Replay a recorded stream from a bag file.
    """

    #============================== __init__ =============================
    #
    def __init__(self, configs) -> None:
        '''!
        @brief  Constructor for Intel Realsense D435 camera bag replay instance.

        @param[in]  configs     Settings to apply (indicates topics and alignment).
        '''
        super(D435_Runner,self).__init__(configs=configs)    # Skip D435 init. Use its superclass.

        self.Kdepth = None
        self.gCD    = None
        self.depth_scale = 0.0010000000474974513
        # @todo Snag from /device_0/sensor_0/option/Depth_Units/value
        #       How to read without having to subscribe and be stuck?? Talk to Justin.

        # Configure realsense depth and color streams. Load file if specified.
        self.pipeline  = rs.pipeline()
        self.rs_config = rs.config()


        if (self.configs.camera.ros.load):
            self.rs_config.enable_device_from_file(self.configs.camera.ros.file)
        else:
            raise Exception("Error, confguration indicates not to load file.")


    #=============================== start ===============================
    #
    def start(self):
        '''!
        @brief  Start capturing the stream.

        @note   Right now the construction does this, which is poor design.
        @todo   Should implement start/stop functionality and capture boolean.
        '''

        self.rs_config.enable_all_streams()

        if (self.configs.camera.align):
          self.align = rs.align(rs.stream.color)
        else:
          self.align = None

        self.pipeline.start(self.rs_config)


        # Should snag from ros topic information about intrinsics.
        # What is best place for that.
        # @note Not grabbing instrinsic / calib data about camera from topics.
        #       Pushing to later, when needed since not sure how to snag
        #       in advance.  Maybe through custom ros bag loading then file closure.
        #       Reopen for actual streaming. Talk to Justin about this.
        #
        #if self.K is None:
        #    intrinsic =  color_frame.profile.as_video_stream_profile().intrinsics
        #    intrinsic_Mat = rs_utils.rs_intrin_to_M(intrinsic)
        #    self.K = intrinsic_Mat

    #============================= get_frames ============================
    #
    # @todo Rename to "capture" so that it is agnostic to what is being
    #       captured.  The setup should determine that and the code should
    #       be consistent with the setup, or at least sanity check the
    #       results (by checking for None when this code is upgraded).
    #
    def get_frames(self, before_scale=False):
        """!
        @brief  Get the next frame(s)

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

        # Align depth to color if enabled prior to starting.
        if (self.configs.camera.align):
            frames = self.align.process(frames)

        # split depth and color. <class 'pyrealsense2.video_frame'>
        # Convert realsense images to numpy arrays. Depth image in meters
        allGood = True
        if (self.configs.camera.depth.use):
            depth_frame = frames.get_depth_frame()
            if not depth_frame:
                allGood = False
            else:
                depth_raw = np.asanyarray(depth_frame.get_data())
                if before_scale:
                    depth_image = depth_raw
                else:
                    depth_image = depth_raw * self.depth_scale
        else:
            depth_image = None

        if (self.configs.camera.color.use):
            color_frame = frames.get_color_frame()
            if not color_frame:
                allGood = False
            else:
                color_image = np.asanyarray(color_frame.get_data()) # already in RGB
        else:
            color_image = None


        return color_image, depth_image, allGood
    
#
#=================================== camera/d435 ===================================
