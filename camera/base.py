"""

    @brief          Define the base class for the camera runners

    @author         Yiye Chen.          yychen2019@gatech.edu
    @date           [created] 10/07/2021

"""


from dataclasses import dataclass
import numpy as np

from yacs.config import CfgNode

@dataclass
class ImageRGBD:
  color: any = None
  depth: any = None


class CfgCamera(CfgNode):
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
        super().__init__(configs, K)



class Grayscale(Base):
   '''!
   @brief   Expands on base class to specialize to color images.
     
   What might this do that is unique?
   '''
   #============================ Color __init___ ============================
   #
   def __init__(self, configs, K = None) -> None:
        super().__init__(configs, K)



