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
    
    """ 
    W_dep: int = 1280
    H_dep: int = 720
    W_rgb: int = 1920
    H_rgb: int = 1080


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

        # the intrinsic
        self.intrinsic = None       #<- Need to be set when the frame is get. Didn't find other methods

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
        frames = self.alignalign.process(frames)

        # split depth and color. <class 'pyrealsense2.video_frame'>
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            return None, None, False
            
        # update the intrinsic matrix stored in the calibrator, in case of the camera profile change
        if self.intrinsic is None:
            intrinsic =  color_frame.profile.as_video_stream_profile().intrinsics
            intrinsic_Mat = rs_utils.rs_intrin_to_M(intrinsic)

        # Convert realsense images to numpy arrays. Depth image in meters
        depth_raw = np.asanyarray(depth_frame.get_data())
        depth_image = depth_raw * self.depth_scale
        color_image = np.asanyarray(color_frame.get_data())
