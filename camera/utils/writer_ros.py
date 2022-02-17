"""

    @brief              The video writer that saved the rgb frame and the depth map into a rosbag file

    @author             Yiye Chen.          yychen2019@gatech.edu
    @date               02/16/2022[created]

"""
import numpy as np
import time

from cv_bridge import CvBridge
import genpy
import rosbag

class vidWriter_ROS(object):

    def __init__(self, save_file_path, rgb_topic, depth_topic, depth_format=None,
        frame_rate = None):
        """
        Args:
            save_file_path (str): The path to save the file
            rgb_topic (str): The topic name for the rgb images
            dep_topic (str): The topic name for the depth maps
            depth_format (np.dtype): The depth format. If not none, will cast the depth map to this format (mostly to reduce size). \
                defautls to None
            frame_rate (int): The frame rate (frame per second.). If not None, will wait for 1/frame_rate after writing out each frame
        """
        self.file_path = save_file_path
        self.depth_format = depth_format
        self.rgb_topic = rgb_topic
        self.dep_topic = depth_topic

        self.bridge = CvBridge()
        self.bag = rosbag.Bag(self.file_path, 'w')

        self.frame_rate = frame_rate
        if self.frame_rate is None:
            self.wait_time = None
        else:
            self.wait_time = 1./self.frame_rate  # The wait time between frames in seconds
        self.wait_time_counter = 0  # count the wait time
        self.cur_time = 0           # store the time of the current frame save
    
    #def save_frame(self, rgb:np.ndarray, depth:np.ndarray, t=None):
    def save_frame(self, rgb, depth, t=None):
        """Save a frame of the rgb and depth data into the rosbag

        Args:
            rgb (array): The rgb image. Will be converted to cv2 bgr format and then the ros image msg
            depth (_type_): The depth map
            t : The timestamp. Defaults to None, which will use the current time.
        """
        if self.depth_format is not None:
            depth = depth.astype(self.depth_format)
        
        if t is None:
            t = genpy.Time.from_sec(time.time())

        # determine whether the frame comes too quick
        if self.wait_time is not None:
            if self.cur_time == 0:
                interval = 0
            else:
                interval = t.to_time() - self.cur_time.to_time()
            # update the current time
            self.cur_time = t
            # increment the wait time counter
            self.wait_time_counter = self.wait_time_counter + interval
            if self.wait_time_counter >= self.wait_time:
                # if satisfy, then can process to save the frame, and reset the counter
                self.wait_time_counter = 0
            else:
                # Do not save
                return

        # write frames
        rgb_ros = self.bridge.cv2_to_imgmsg(
            rgb[:,:,::-1],
            encoding="passthrough"
        )
        depth_ros = self.bridge.cv2_to_imgmsg(
            depth,
            encoding="passthrough"
        )
        self.bag.write(
            self.rgb_topic,
            rgb_ros,
            t
        )
        self.bag.write(
            self.dep_topic,
            depth_ros,
            t
        )

    
    def finish(self):
        self.bag.close()
        # clear the counter
        self.cur_time = 0
        self.wait_time_counter = 0
