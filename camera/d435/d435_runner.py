"""

    @brief          The d435 camera runner.

    @author         Yiye Chen.          yychen2019@gatech.edu
    @date           [created] 10/07/2021

"""

from dataclasses import dataclass

import numpy as np
import pyrealsense2 as rs

import camera.base as base
import camera.utils.rs_utils as rs_utils

@dataclass
class D435_Configs():
    """The configurations for the D435 camera
    For the detailed explanations and options, please refer to the D435 official document.

    Args:
        W_dep (int. Optional). The width of the depth sensor. Defaults to 1280.
        H_dep (int. Optional). The height of the depth sensor. Defaults to 720.
        W_color (int. Optional). The width of the color sensor. Defaults to 1920.
        H_color (int. Optional). The height of the color sensor. Defaults to 1080.
        exposure (int. Optional.) The camera exposure. Defaults to None, in which case will \
            enable auto_exposure (if gain is also None) or use the default value 50 (according to Yiye's test)
        gain (int. Optional.) The camera gain. Defaults to None, in which case will \
            enable auto_exposure (if gain is also None) or use the default value 64 (according to Yiye's test)
    
    """ 
    W_dep: int = 1280
    H_dep: int = 720
    W_color: int = 1920
    H_color: int = 1080
    exposure: int = None
    gain: int = None


class D435_Runner(base.Base):
    """D435_Runner
    """

    def __init__(self, configs:D435_Configs=D435_Configs()) -> None:
        super().__init__(configs=configs)
        # Configure realsense depth and color streams
        self.pipeline = rs.pipeline()
        self.rs_config = rs.config()
        self.rs_config.enable_stream(rs.stream.depth, self.configs.W_dep, self.configs.H_dep, rs.format.z16, 30)
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

    def get_frames(self):
        """Get the next frames

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
        depth_image = depth_raw * self.depth_scale
        color_image = np.asanyarray(color_frame.get_data())[:,:,::-1]   #<- bgr to rgb

        return color_image, depth_image, True
    
    def get(self, key):
        if key == "exposure" or "gain":
            value = self.color_sensor.get_option(
                eval("rs.option." + key)
            )
        elif key == "W_rgb" or key == "H_rgb" or key == "W_dep" or key == "H_dep":
            value = eval("self.configs." + key)
        else:
            raise NotImplementedError

        return value
