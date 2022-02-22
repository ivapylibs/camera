"""

    @ brief:        Record the camera rgb and depth as the rosbag file

    @ author:       Yiye Chen
    @ date:         02/16/2022

"""

import cv2
import numpy as np
import os
import argparse

import camera.d435.d435_runner as d435
import camera.utils.display as display
from camera.utils.writer_ros import vidWriter_ROS

# the save_path from the command line
default_path = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "d435_record.bag"
)
parser = argparse.ArgumentParser(description='The d435 camera rosbag recorder.')
parser.add_argument('--target_file_path', default=default_path, type=str,
                    help='The path to save the bag file')
parser.add_argument('--frame_rate', default=20, type=int,
                    help='The frame rate')
parser.add_argument("--H", default=1080, type=int,
                    help="The frame height")
parser.add_argument("--W", default=1920, type=int,
                    help="The frame width")

args = parser.parse_args()
if not args.target_file_path.endswith(".bag"):
    args.target_file_path = args.target_file_path + ".bag"

# settings
W = args.W
H = args.H

# prepare
d435_configs = d435.D435_Configs(
    W_dep=848,
    H_dep=480,
    W_color=W,
    H_color=H,
    exposure = 100,
    gain = 50
)

d435_starter = d435.D435_Runner(d435_configs)
depth_scale = d435_starter.get("depth_scale")

vid_writer = vidWriter_ROS(
    save_file_path=args.target_file_path,
    depth_scale=depth_scale,
    rgb_topic="color",
    depth_topic="depth",
    depth_scale_topic="depth_scale",
    frame_rate=args.frame_rate
)

#### Get Started

# get and save the frames
while(True):
    rgb, dep, success = d435_starter.get_frames(before_scale=True)
    #print("The camera gain: {}. The camera exposure: {}".format(d435_starter.get("gain"), d435_starter.get("exposure")))
    if not success:
        print("Cannot get the camera signals. Exiting...")
        exit()

    display.display_rgb_dep_cv(rgb, dep*depth_scale, depth_clip=0.08, ratio=0.5, window_name="THe camera signals. (color-scaled depth). Press \'q\' to exit")
    vid_writer.save_frame(rgb,  dep)

    opKey = cv2.waitKey(1)
    if opKey == ord('q'):
        break

vid_writer.finish()