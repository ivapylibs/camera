#============================= camera/rostopic ===========================
'''!

@brief      Get camera information from a ROS stream.

@author     Patricio A. Vela,   pvela@gatech.edu
@date       2023/07/13

'''


import numpy as np
import rospy
from rospy.numpy_msg import numpy_msg

from sensor_msgs.msg import Image

import camera.base as Camera

#
#-------------------------------------------------------------------------
#==================== ROSTopic / Camera Configuration ====================
#-------------------------------------------------------------------------
#

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
          init_dict = CfgROSCam.get_default_settings();

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
  
        default_dict = dict(topicPath = '', topicName = '')
        return default_dict

#
#-------------------------------------------------------------------------
#======================== ROSTopic / Color Camera ========================
#-------------------------------------------------------------------------
#

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

      theTopic = self.configs.topicPath + "/" + self.configs.topicName
      self.camsub = rospy.Subscriber(theTopic,                      \
                                     numpy_msg(Image),  \
                                     self.streamCB) 

    #=============================== stop ==============================
    #
    #
    def stop(self):

      self.camsub.unregister()
      

    #============================= streamCB ============================
    #
    #
    # @param[in]    imsg    image message.
    #
    def streamCB(self, imsg):

      # log some info about the image topic
      rospy.loginfo(imsg.header)

      # Try to convert the ROS Image message to a CV2 Image
      if (self.last):
        self.I1 = np.frombuffer(imsg.data, dtype=np.uint8).reshape(imsg.height, imsg.width, -1)
        self.last = False
      else:
        self.I2 = np.frombuffer(imsg.data, dtype=np.uint8).reshape(imsg.height, imsg.width, -1)
        self.last = True

    #============================ get_frames ===========================
    #
    #
    def get_frames(self):
        """Get the next frames
        """
        if (self.last):
          return self.I2
        else:
          return self.I1


#
#-------------------------------------------------------------------------
#======================== ROSTopic / Depth Camera ========================
#-------------------------------------------------------------------------
#

class Depth(Camera.Grayscale):
    """!
    @brief  Class for ROS-based depth camera subsrciber.

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
        self.D1 = None
        self.D2 = None
        self.last = False
        # Unsure if implementation will have access clashes, so creating
        # a primitive avoidance approach.


    #============================== start ==============================
    #
    #
    def start(self):

      rospy.init_node('listener')

      theTopic = self.configs.topicPath + "/" + self.configs.topicName
      self.camsub = rospy.Subscriber(theTopic,                      \
                                     numpy_msg(Image),  \
                                     self.streamCB) 

    #=============================== stop ==============================
    #
    #
    def stop(self):

      self.camsub.unregister()
      

    #============================= streamCB ============================
    #
    #
    # @param[in]    imsg    image message.
    #
    def streamCB(self, imsg):

      # log some info about the image topic
      rospy.loginfo(imsg.header)

      # Try to convert the ROS Image message to a CV2 Image
      if (self.last):
        self.D1 = np.frombuffer(imsg.data, dtype=np.uint16).reshape(imsg.height, imsg.width, -1)
        self.last = False
      else:
        self.D2 = np.frombuffer(imsg.data, dtype=np.uint16).reshape(imsg.height, imsg.width, -1)
        self.last = True

    #============================ get_frames ===========================
    #
    #
    def get_frames(self):
        """Get the next frames
        """
        if (self.last):
          return self.D2
        else:
          return self.D1

#
#-------------------------------------------------------------------------
#========================= ROSTopic / RGBD Camera ========================
#-------------------------------------------------------------------------
#

class RGBD(Camera.Base):
    """!
    @brief  Class for ROS-based depth camera subsrciber.

    """

    #============================= __init__ ============================
    #
    #
    def __init__(self, configs = None, K = None) -> None:
        '''!
        @brief  Base class instantiator for roscam.

        '''
        super().__init__(configs, K)

        self.colorSub = None
        self.depthSub = Nond
        self.I1 = None
        self.I2 = None

        self.D1 = None
        self.D2 = None

        self.dLast = False
        self.cLast = False

        # Unsure if implementation will have access clashes, so creating
        # a primitive avoidance approach.


    #============================== start ==============================
    #
    #
    def start(self):

      rospy.init_node('listener')

      theTopic = self.configs.colorPath + "/" + self.configs.colorName
      self.colorSub = rospy.Subscriber(theTopic,                      \
                                       numpy_msg(Image),  \
                                       self.colorCB) 
      
      theTopic = self.configs.depthPath + "/" + self.configs.depthName
      self.depthSub = rospy.Subscriber(theTopic,                      \
                                       numpy_msg(Image),  \
                                       self.depthCB) 

    #=============================== stop ==============================
    #
    #
    def stop(self):

      self.colorSub.unregister()
      self.depthSub.unregister()
      

    #============================= colorCB ============================
    #
    #
    # @param[in]    imsg    image message.
    #
    def colorCB(self, imsg):

      # log some info about the image topic
      rospy.loginfo(imsg.header)

      # Try to convert the ROS Image message to a CV2 Image
      if (self.cLast):
        self.I1 = np.frombuffer(imsg.data, dtype=np.uint8).reshape(imsg.height, imsg.width, -1)
        self.cLast = False
      else:
        self.I2 = np.frombuffer(imsg.data, dtype=np.uint8).reshape(imsg.height, imsg.width, -1)
        self.cLast = True


    #============================= depthCB ============================
    #
    #
    # @param[in]    imsg    image message.
    #
    def depthCB(self, imsg):

      # log some info about the image topic
      rospy.loginfo(imsg.header)

      # Try to convert the ROS Image message to a CV2 Image
      if (self.dLast):
        self.D1 = np.frombuffer(imsg.data, dtype=np.uint16).reshape(imsg.height, imsg.width, -1)
        self.dLast = False
      else:
        self.D2 = np.frombuffer(imsg.data, dtype=np.uint16).reshape(imsg.height, imsg.width, -1)
        self.dLast = True

    #============================ get_frames ===========================
    #
    #
    def get_frames(self):
        """Get the next frames
        """
        if (self.cLast):
          C = self.D2
        else:
          C = self.D1

        if (self.dLast):
          D = self.D2
        else:
          D = self.D1

        return C, D

