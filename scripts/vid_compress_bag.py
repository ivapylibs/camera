"""
    @ brief:     Reduce the rosbag file size. 

    @ author:   Yiye Chen
    @ date:     02/17/2022

"""

import os
import argparse
import cv2
import numpy as np

from cv_bridge import CvBridge
import rosbag

import sys
sys.path.append("/home/cyy/Research/ivapylibs/camera/camera")
from utils.writer_ros import vidWriter_ROS
#from camera.utils.writer_ros import vidWriter_ROS

# the save_path from the command line
load_path_default = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "d435_record.bag"
)
parser = argparse.ArgumentParser(description='The d435 camera rosbag recorder.')
parser.add_argument('--load_file_path', default=load_path_default, type=str, nargs=1,
                    help='The path to save the bag file')
parser.add_argument('--save_file_path', default=None, type=str, nargs=1,
                    help='The path to save the bag file. Default to load_file_path plus the term compressed at the end.')
parser.add_argument('--frame_rate', default=5, type=int, nargs=1,
                    help='The frame rate.')
args = parser.parse_args()
if not args.load_file_path.endswith(".bag"):
    args.load_file_path = args.load_file_path + ".bag"
if args.save_file_path is None:
    args.save_file_path = os.path.splitext(args.load_file_path)[0] + "_compressed.bag"

# the ros-cv converter
bridge = CvBridge()

# the bag file
bag_load = rosbag.Bag(args.load_file_path)
vidname = os.path.basename(args.save_file_path)

# the video writer
vid_writer = vidWriter_ROS(
    save_file_path=args.save_file_path,
    rgb_topic="color",
    depth_topic="depth",
    depth_format=np.float32,
    frame_rate=args.frame_rate
)

# Original file size
print("\n The size of the original file: {} GB \n".format(os.path.getsize(args.load_file_path)/1e9))

# get started
rgb = None
depth = None
print("Compressing...")
for topic, msg, t in bag_load.read_messages():
    if topic == "color":
        rgb = bridge.imgmsg_to_cv2(msg)[:,:,::-1]
    elif topic == "depth":
        depth = bridge.imgmsg_to_cv2(msg)
    
    # display if gathered both data
    if rgb is not None and depth is not None:

        vid_writer.save_frame(rgb, depth, t=t)

        rgb = None
        depth = None

vid_writer.finish()

print("The size of the compressed file: {} GB \n".format(os.path.getsize(args.save_file_path)/1e9))
