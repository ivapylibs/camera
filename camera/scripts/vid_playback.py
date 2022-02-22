"""

    @ brief:        The video playback from the rosbag file

    @ author:       Yiye Chen
    @ date:         02/16/2022

"""

import os
import argparse
from time import sleep
import cv2

from cv_bridge import CvBridge
import rosbag

import camera.d435.d435_runner as d435
from camera.utils.display import display_rgb_dep_cv
from camera.utils.writer_ros import vidWriter_ROS

# the save_path from the command line
default_path = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "d435_record.bag"
)
parser = argparse.ArgumentParser(description='The d435 camera rosbag recorder.')
parser.add_argument('--save_file_path', default=default_path, type=str,
                    help='The path to save the bag file')
args = parser.parse_args()
if not args.save_file_path.endswith(".bag"):
    args.save_file_path = args.save_file_path + ".bag"

# the ros-cv converter
bridge = CvBridge()

# the bag file
bag = rosbag.Bag(args.save_file_path)
vidname = os.path.basename(args.save_file_path)

# get frame rate
total_time = bag.get_end_time() - bag.get_start_time()      # in seconds
frame_num = bag.get_message_count(topic_filters="color")
frame_time_interval = total_time / frame_num

# get started
rgb = None
depth = None
depth_scale = None
for topic, msg, t in bag.read_messages():
    if topic == "color":
        rgb = bridge.imgmsg_to_cv2(msg)[:,:,::-1]
    elif topic == "depth":
        depth = bridge.imgmsg_to_cv2(msg)
    elif topic == "depth_scale":
        depth_scale = msg.data
    
    # display if gathered both data
    if rgb is not None and depth is not None:
        if depth_scale is not None:
            depth = depth * depth_scale
        display_rgb_dep_cv(rgb, depth, depth_clip=0.08, ratio=0.4, window_name="The playback of {}. Press \'q\' to quit.".format(vidname))
    
        opKey = cv2.waitKey(1)
        if opKey == ord('q'):
            break
        sleep(frame_time_interval)
        rgb = None
        depth = None

