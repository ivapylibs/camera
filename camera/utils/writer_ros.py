"""

    @brief              The video writer that saved the rgb frame and the depth map into a rosbag file

    @author             Yiye Chen.          yychen2019@gatech.edu
    @date               02/16/2022[created]

"""

from cv_bridge import CvBridge
import rosbag

class vidWriter_ROS(object):

    def __init__(self, save_file_path, rgb_topic, depth_topic, depth_format):
        """
        Args:
            save_file_path (str): The path to save the file
            rgb_topic (str): The topic name for the rgb images
            dep_topic (str): The topic name for the depth maps
            depth_format (): The deopth format
        """
        self.file_path = save_file_path
        self.depth_format = depth_format
        self.rgb_topic = rgb_topic
        self.dep_topic = depth_topic

        self.bridge = CvBridge()
        self.bag = rosbag.Bag(self.file_path, 'w')
    
    def save_frame(self, rgb, depth):
        """Save a frame of the rgb and depth data into the rosbag

        Args:
            rgb (array): The rgb image. Will be converted to cv2 bgr format and then the ros image msg
            depth (_type_): The depth map
        """
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
            rgb_ros
        )
        self.bag.write(
            self.dep_topic,
            depth_ros
        )
