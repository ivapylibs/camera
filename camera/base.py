"""

    @brief          Define the base class for the camera runners

    @author         Yiye Chen.          yychen2019@gatech.edu
    @date           [created] 10/07/2021

"""


class Base():
    """The base class for the camera runners

    Defines some shared functionality interfaces
    """
    def __init__(self) -> None:
        self.configs = None

    def get_frames(self):
        """Get the next frames
        """
        raise NotImplementedError
    
    def get_configs(self):
        """Get all the configurations
        """
        return self.configs
    
    def set_configs(self, configs):
        """set all the configurations

        Args:
            configs (Any): The desired configurations
        """
        self.configs = configs
    
    def get(self, key):
        """Get a particular configuration.

        Args:
            key (Any): The configuration name
        """
        raise NotImplementedError

    def set(self, key, value):
        """Set a particular configuration.

        Args:
            key (Any): The configuration name
            value (Any): The value to be set
        """
        raise NotImplementedError


