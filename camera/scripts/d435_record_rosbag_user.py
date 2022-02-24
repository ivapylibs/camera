#!/usr/bin/env python
"""

    @ brief:        A more user-friendly D435 signals rosbag recorder

    @ author:       Yiye Chen
    @ date:         02/22/2022

"""

import cv2
import numpy as np
import os
import argparse

import camera.d435.d435_runner as d435
import camera.utils.display as display
from camera.utils.writer_ros import vidWriter_ROS

from d435_recorders import D435RecRosbag



# prepare
d435_configs = d435.D435_Configs(
    W_dep=848,
    H_dep=480,
    W_color=1920,
    H_color=1080,
    exposure = 100,
    gain = 50
)

# create the recorder
recorder = D435RecRosbag(
    d435_configs=d435_configs,
    color_topic_name="color",
    depth_topic_name="depth",
    depth_scale_topic_name="depth_scale"
)

recorder.run()


