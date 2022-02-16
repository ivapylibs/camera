import cv2
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
parser.add_argument('--target_file_path', default=default_path, type=str, nargs=1,
                    help='The path to save the bag file')
args = parser.parse_args()
if not args.target_file_path.endswith(".bag"):
    args.target_file_path = args.target_file_path + ".bag"

# settings
W = 1920
H = 1080

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

vid_writer = vidWriter_ROS(
    save_file_path=args.target_file_path,
    rgb_topic="color",
    dep_topic="depth"
)

# get started
while(True):
    rgb, dep, success = d435_starter.get_frames()
    #print("The camera gain: {}. The camera exposure: {}".format(d435_starter.get("gain"), d435_starter.get("exposure")))
    if not success:
        print("Cannot get the camera signals. Exiting...")
        exit()

    display.display_rgb_dep_cv(rgb, dep, ratio=0.5, window_name="THe camera signals. (color-scaled depth). Press \'q\' to exit")
    vid_writer.save_frame(rgb,  dep)

    opKey = cv2.waitKey(1)
    if opKey == ord('q'):
        break

    #display.display_rgb_dep_plt(rgb, dep, suptitle=None, fh=fh)
    #plt.draw()
    #plt.pause(1)

