"""

    @ brief:        The video playback of the rosbag file

    @ author:       Yiye Chen
    @ date:         02/16/2022

"""

import os
import argparse
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
parser.add_argument('--save_file_path', default=default_path, type=str, nargs=1,
                    help='The path to save the bag file')
args = parser.parse_args()
if not args.save_file_path.endswith(".bag"):
    args.save_file_path = args.save_file_path + ".bag"

# the ros-cv converter
bridge = CvBridge()

# the bag file
bag = rosbag.Bag(args.save_file_path)
vidname = os.path.basename(args.save_file_path)


# get started
rgb = None
depth = None
for topic, msg, t in bag.read_messages():
    if topic == "color":
        rgb = bridge.imgmsg_to_cv2(msg)[:,:,::-1]
    elif topic == "depth":
        depth = bridge.imgmsg_to_cv2(msg)
    
    # display if gathered both data
    if rgb is None and depth is None:
        display_rgb_dep_cv(rgb, depth, ratio=0.4, window_name="The playback of {}. Press \'q\' to quit.".format(vidname))
    
        opKey = cv2.waitKey(1)
        if opKey == ord('q'):
            break

        rgb = None
        depth = None

