#================================== camera/cv2cam ==================================
'''!
@brief          OpenCV based camera interface. 

This class API serves as a simpler wrapper for the python camera capture interfaces
for webcams, which themselves are usually wrappers for the C++ implementation.

'''
#================================== camera/cv2cam ==================================
#
# @author         Patricio A. Vela      pvela@gatech.edu
# @date           2023/12/21            [created]
#
# NOTE: python 2 space indent (tab converts). 90 columns. wrap margin = 6.
#
#================================== camera/cv2cam ==================================

import numpy as np
import cv2

import camera.base as cam
import ivapy.display_cv as cvdisp


#================================== CfgColor ==================================
#
class CfgColor(cam.CfgCamera):
  '''!
  @ingroup Camera
  @brief  Configuration setting specifier for OpenCV2 camera interface.

  For detailed explanations and options, refer to official OpenCV documentation.

  @note   Currently not using a CfgCamera super class. Probably best to do so.
  '''

  #=============================== __init__ ==============================
  #
  '''!
  @brief        Constructor of camera configuration node for OpenCV interface.

  @param[in]    cfg_files   List of config files to load to merge settings.
  '''
  def __init__(self, init_dict=None, key_list=None, new_allowed=True):
    
    if (init_dict == None):
      init_dict = CfgColor.get_default_settings()

    super().__init__(init_dict, key_list, new_allowed)

  #========================= get_default_settings ========================
  #
  # @brief    Recover the default settings in a dictionary.
  #
  @staticmethod
  def get_default_settings():
      '''!
      @brief  Defines most basic, default settings for RealSense D435.

      @param[out] default_dict    Dictionary populated with minimal set of default settings.
      '''
      default_dict = dict(camera = dict ( 
            config = dict(load = False, file = "", ros = ""),
            color  = dict(use = True, id = 0, res = [640, 480], fps = 30, toRGB = False),
            settings = None, 
            ros    = dict(enable = False, color = dict(pub = False, topic = ''), \
                          load = False, file = '') ) )
      return default_dict

  #=========================== builtForROSBag ==========================
  #
  @staticmethod
  def builtForROSBag(rosFile = None, camTopic='/device_0/sensor_0/Color_0/imega/data'):

    bagCfg = CfgColor()
    bagCfg.camera.ros.enable = True
    bagCfg.camera.ros.color.pub = replayCfg.camera.color.use
    bagCfg.camera.ros.color.topic = camTopic

    if rosFile is not None:
      bagCfg.camera.ros.load = True
      bagCfg.camera.ros.file = rosFile

    return bagCfg
    
#=============================== D435_Runner ===============================
#

class Color(cam.Color):
  """!
  @brief          OpenCV based camera interface. 

  This class API serves as a simpler wrapper for the cv2 python camera capture
  interface, which itself is a wrapper for the C++ implementation.
  """

  #============================== __init__ =============================
  #
  def __init__(self, configs=CfgColor(), K = None) -> None:
      '''!
      @brief  Constructor for OpenCV2 camera interface.

      @param[in]  configs     Settings to apply.
      '''
      super().__init__(configs=configs, K=K)

      self.gCW = None

      # Configure color streams. Load file if specified.
      # Configure streaming sources.
      # @todo   Figure out how to use. Ignoring for now to go faster.
      if (self.configs.camera.color.use):
        pass

      self.color_sensor  = None 
      self.ready = False


  #=============================== start ===============================
  #
  def start(self):
      '''!
      @brief  Start capturing the stream.

      @note   Right now the construction does this, which is poor design.
      @todo   Should implement start/stop functionality and capture boolean.
      '''

      if not self.configs.camera.color.use:
        return

      # Start streaming
      self.color_sensor = cv2.VideoCapture(self.configs.camera.color.id)

      if not self.color_sensor.isOpened():
        print("Cannot open camera")
        exit()

        self.applySettings(self.configs.camera.color)

      self.ready = True


  #================================ stop ===============================
  #
  def stop(self):
      '''!
      @brief  Stop capturing the stream.

      @note   Right now not implemented.
      @todo   Should implement start/stop functionality and capture boolean.
      '''
      if (self.color_sensor is not None) and self.color_sensor.isOpened():
        self.color_sensor.release()
        self.ready = False

  #============================== capture ==============================
  #
  #
  def capture(self):
      """!
      @brief  Get the next frame

      @param[out] color_image     The color image
      @param[out] succ_flag       Success flag of fetching next frame.
      """

      if self.ready:
        succ_flag, color_image = self.color_sensor.read()
        #if allGood 
        #  color_image = np.asanyarray(color_frame.get_data())[:,:,::-1] # BGR to RGB
        # @todo  Figure out how to post-process if needed.
        if succ_flag and self.configs.camera.color.toRGB:
          color_image = color_image[:,:,::-1]
      else:
        color_image = None
        succ_flag   = False

      return (color_image, succ_flag)
  
  #============================= captureRGB ============================
  #
  #
  def captureRGB(self):
      """!
      @brief  Snag the next set of RGB. 

      Captures the color and depth pair as an RGBD image instance. The depth
      map should be returned in standard units (meters).

      @param[out] data        RGBD image data as an ImageRGBD element.
      @param[out] flSuccess   Success flag. True = both frames fetched.
      """

      (color_image, succ_flag) = self.get_frame()
      if succ_flag and not self.configs.camera.color.toRGB:
        return (color_image[:,:,::-1], succ_flag)

  #================================ get ================================
  #
  # @brief    Get attributes based on keys.
  #
  # @todo     Need to recode this part.  It is not up to date.
  def get(self, key):
      """Get attributes specified by the key

      Args:
          key (str):      Choices: [exposure, gain, W_rgb, H_rgb, W_dep, H_dep, depth_scale, K]
      Returns:
          value
      """

      return None
      # @todo   Not implemented.  Need to read up to do so.  Bad D435 code here
      #if key == "exposure" or key == "gain":
      #    value = self.color_sensor.get_option(
      #        eval("rs.option." + key)
      #    )
      #elif key == "W_rgb" or key == "H_rgb" or key == "W_dep" or key == "H_dep":
      #    value = eval("self.configs." + key)
      #else:
      #    value = eval("self." + key)
      #return value

  #================================= display =================================
  #
  # @brief    Display latest frames.
  #
  def display(self, image, ratio = 0.5, window_name = "Color"):

    if self.configs.camera.color.toRGB:
      cvdisp.rgb(image, ratio, window_name)
    else:
      cvdisp.bgr(image, ratio, window_name)


  #============================== buildFromFile ==============================
  #
  @staticmethod
  def buildFromFile(configFile):

    configs = CfgColor()
    configs.merge_from_file(configFile)
    return Color(configs)

#
#================================== camera/cv2cam ==================================
