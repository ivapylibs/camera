#============================= camera/rostopic ===========================
'''!

@brief      Get camera information from a ROS stream.

@author     Patricio A. Vela,   pvela@gatech.edu
@date       2023/07/13

'''


import numpy as np
import rospy

import camera.Base as Camera

@dataclass
class ImageRGBD:
  color: any = None
  depth: any = None


class CfgROSCam(Camera.CfgCamera):
    '''!
    @brief  Configuration setting specifier for ROS connected camera.
    '''

    #=============================== __init__ ==============================
    #
    '''!
    @brief        Constructor of camera.

    @param[in]  init_dict   Configuration dictionary.
    @param[in]  key_list    Key listing (claim: for debugging only)
    @param[in]  new_allowed OK to add new configuration items?
    '''
    def __init__(self, init_dict=None, key_list=None, new_allowed=True):
      
      if (init_dict is None):
          init_dict = CfGROSCam.get_default_settings();

      super().__init__(init_dict, key_list, new_allowed)

    #========================= get_default_settings ========================
    #
    # @brief    Recover the default settings in a dictionary.
    #
    @staticmethod
    def get_default_settings():
        '''!
        @brief  Defines most basic, default settings for RealSense D435.

        @param[out] default_dict  Dictionary populated with minimal set of
                                  default settings.
        '''
  
        default_dict = dict(topicPath = '', topicName = '', type = Image)
        return default_dict


class Color(Camera.Color):
    """!
    @brief  Class for ROS-based camera runners.

    """

    #============================= __init__ ============================
    #
    #
    def __init__(self, configs = None, K = None) -> None:
        '''!
        @brief  Base class instantiator for roscam.

        '''
        super().__init__(configs, K)

        self.camsub = None
        self.I1 = None
        self.I2 = None
        self.last = False
        # Unsure if implementation will have access clashes, so creating
        # a primitive avoidance approach.


    #============================== start ==============================
    #
    #
    def start(self):

      rospy.init_node('listener')

      theTopic = self.config.topicPath + "/" + self.config.topicName
      self.camsub = rospy.Subscriber(theTopic,                      \
                                     numpy_msg(self.config.topic),  \
                                     self.streamCB)

    #=============================== stop ==============================
    #
    #
    def stop(self):

      self.camsub.unregister()
      

    #============================= streamCB ============================
    #
    #
    def streamCB(self, img_msg):

      # log some info about the image topic
      rospy.loginfo(img_msg.header)

      # Try to convert the ROS Image message to a CV2 Image
      if (self.last)
        self.I1 = img_msg.data
        self.last = False
      else:
        self.I2 = img_msg.data
        self.last = True

    #============================ get_frames ===========================
    #
    #
    def get_frames(self):
        """Get the next frames
        """
        if (self.last)
          return self.I2
        else:
          return self.I1

