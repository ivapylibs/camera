#===================================== base ====================================
"""!
@package    base
@brief      Define the base classes for camera interface.
"""
#===================================== base ====================================
"""!
@author Patricio A. Vela,       pvela@gatech.edu
@date           [created] 10/07/2021

"""

#
#===================================== base ====================================

from dataclasses import dataclass
import numpy as np

from ivapy.Configuration import AlgConfig

@dataclass
class ImageRGBD:
  color: any = None
  depth: any = None


class CfgCamera(AlgConfig):
    '''!

    @brief  Configuration setting specifier for generic camera.

    '''

    #=============================== __init__ ==============================
    #
    '''!
    @brief        Constructor of camera.

    @param[in]    cfg_files   List of config files to load to merge settings.
    '''
    # 
    # NOTE: NEEDS TO BE REDONE. TODO TODO.
    #
    def __init__(self, init_dict=None, key_list=None, new_allowed=True):
      
      super().__init__(init_dict, key_list, new_allowed)
      # self.merge_from_lists(XX)


class Base():
    """!
    @brief  Base class for camera runners.

    Defines some shared functionality interfaces
    """


    #============================= __init__ ============================
    #
    #
    def __init__(self, configs, K = None) -> None:
        '''!
        @brief  Base class instantiator for camera runners.

        '''
        self.configs = configs
        if K is None:
            self.K = np.identity(3)
        else:
            self.K = K

    #========================== set_intrinsics =========================
    #
    #
    def set_intrinsic(self, K):
        self.K = K

    #============================== start ==============================
    #
    #
    def start(self):
        pass

    #=============================== stop ==============================
    #
    #
    def stop(self):
        pass

    #============================ get_frames ===========================
    #
    #
    def get_frames(self):
        """Get the next frames
        """
        raise NotImplementedError
    
    #=========================== get_configs ===========================
    #
    #
    def get_configs(self):
        """Get all the configurations
        """
        return self.configs
    
    #=========================== set_configs ===========================
    #
    #
    def set_configs(self, configs):
        """set all the configurations

        Args:
            configs (Any): The desired configurations
        """
        self.configs = configs
    
    #=============================== get ===============================
    #
    #
    def get(self, key):
        """Get a particular configuration.

        Args:
            key (Any): The configuration name
        """
        raise NotImplementedError

    #=============================== set ===============================
    #
    #
    def set(self, key, value):
        """Set a particular configuration.

        Args:
            key (Any): The configuration name
            value (Any): The value to be set
        """
        raise NotImplementedError



class Color(Base):
  '''!
  @brief  Expands on base class to specialize to color images.

  What might this do that is unique?
  '''
  #============================ Color __init___ ============================
  #
  def __init__(self, configs, K = None) -> None:
    super().__init__(configs)
    self.K = K


class Grayscale(Base):
   '''!
   @brief   Expands on base class to specialize to color images.
     
   What might this do that is unique?
   '''
   #============================ Color __init___ ============================
   #
   def __init__(self, configs, K = None) -> None:
        super().__init__(configs, K)


#====================================== Replay =====================================
#

class ReplayColor(Color):
  """!
  @brief  Replay a recorded color stream from a bag file.
  """

  #============================== __init__ =============================
  #
  def __init__(self, configs) -> None:
      '''!
      @brief  Constructor for color camera bag replay instance.

      @param[in]  configs     Settings to apply (indicates topics and alignment).
      '''
      super(Color,self).__init__(configs=configs) 

      if (self.configs.camera.ros.load):
        #self.rs_config.enable_device_from_file(self.configs.camera.ros.file)
        # @todo Add ROS bag loading.
        pass
      else:
        raise Exception("Error, confguration indicates not to load file.")


  #=============================== start ===============================
  #
  def start(self):
    '''!
    @brief  Start capturing the stream.

    @note   Right now the construction does this, which is poor design.
    @todo   Should implement start/stop functionality and capture boolean.
    '''

    #self.rs_config.enable_all_streams()
    # @todo   start the ros bag.

    # Should snag from ros topic information about intrinsics.
    # What is best place for that.
    # @note Not grabbing instrinsic / calib data about camera from topics.
    #       Pushing to later, when needed since not sure how to snag
    #       in advance.  Maybe through custom ros bag loading then file closure.
    #       Reopen for actual streaming. Talk to Justin about this.
    #
    #if self.K is None:
    #    intrinsic =  color_frame.profile.as_video_stream_profile().intrinsics
    #    intrinsic_Mat = rs_utils.rs_intrin_to_M(intrinsic)
    #    self.K = intrinsic_Mat

  #============================== capture ==============================
  #
  #
  def capture(self):
    """!
    @brief  Get the next frame(s)

    @param[out] color_image     The color image.
    @param[out] succ_flag       Frame fetch success flag.
    """
    # Wait for next frame.
    #frames = self.pipeline.wait_for_frames()
    # @todo Put ROS bag equivalent.

    # @todo Code is totally bad.  Should not run.
    color_image = None
    succ_flag = False
    return color_image, succ_flag
    # WIPE ABOVE WHEN CODE PROPERLY.

    if (self.configs.camera.color.use):
      #color_frame = frames.get_color_frame()
      if not color_frame:
        allGood = False
      #else:
      #   color_image = np.asanyarray(color_frame.get_data()) # already in RGB
    else:
      color_image = None
      succ_flag = False


    return color_image, succ_flag
 
#
#===================================== base ====================================
