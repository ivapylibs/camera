'''

@brief          Realsense SR 305 Camera Interface

@Author         Kyle de Nobel           knobel3@gatech.edu

@date           11/5/2025

'''

import camera.base as base
import pyrealsense2 as rs
import cv2
import numpy as np
import math
import ivapy.display_cv as display




class ConfigRS205():
    '''config general parameters of OAK camera
    '''
    def __init__(self, config=None):
        if config is None:
            # setup config struct
            self.config = {}
            # define each type of camera
            
        else:
            self.config = config

    def setConfig(self, config):
        '''allows setting all configs at once
        '''
        self.config = config

    def getConfig(self):
        '''returns config dictionary
        '''
        return self.config
    
    def setConfigParameter(self, cam, param, value):
        '''allows the setting of a specific parameter of a camera to a certain value

        Args:
            cam (any): name of camera
            param (any): name of camera parameter
            value (any): value to set parameter to
        '''
        self.config[cam][param] = value


class Color(base.Color):
    '''OAK class to caputre images (there are no color images for this camera)
    '''
    def __init__(self, configs=ConfigRS205()):
        super().__init__(configs=configs)
        

    def start(self):
        '''Start the capture stream. This must be called before get_frames() or capture()
        '''
        # setup device and device queues
        
        self.ready = True
        return self.ready

    def stop(self):
        '''Stop the capture stream and release the device from use
        '''
        self.device = None
        self.ready = False

    def get_configs(self):
        return super().get_configs().getConfig()
    
    def set_configs(self, configs):
        super().set_configs(ConfigRS205(configs))
        # update the cameras. Ignore all depthCam values bc no depth cam
        

    def set(self, cam, key, value):
        '''Set a particular configuration

        Args:
            cam (any): name of the camera to config (monoLeftCam / monoRightCam)
            key (any): The configuration name
            value (any): The value to be set
        '''
        self.configs.setConfigParameter(cam, key, value)

    def get_frames(self, getBothMonoFrames=False, scaleFactor=1):
        ''' Gets the next frame

        Args:
            getBothMonoFrames (optional) : 
                - if True -> returns both left and right frame
                - if False -> returns only left frame
            scaleFactor (optional) : scaling factor on outputted frame

        Returns:
            - if getBothMonoFrames = False -> leftCameraImage
            - if getBothMonoFrames = True -> tuple (leftCameraImage, rightCameraImage)
        '''
        if not self.ready:
            return None, False
    

    def capture(self):
        '''Alias for get_frames
        '''
        return self.get_frames
    

    def display(self, frame, windowName='frame'):
        cv2.imshow(windowName, frame)
    

    def fancyPrintConfigs(self):
        '''Prints out the configurations in a more human readable way
        '''
        cfg = self.configs.getConfig()
        keys = list(cfg.keys())
        for key in keys:
            subKeys = list(cfg[key].keys())
            print(key)
            for subKey in subKeys:
                print("  - ", subKey, " "*(22 - len(subKey)), ": ", cfg[key][subKey])


class Depth(base.Base):
    '''OAK class to capture depth frames

    The depth images are aligned to the left camera by default class initialization
    '''
    def __init__(self, configs=ConfigRS205()):
        super().__init__(configs=configs)

        # setup pipelines and cameras
        

        # configure the cameras with all of the configs
        

        # not ready until start is called
        self.ready = False

    def start(self):
        '''Start the capture stream. This must be called before get_frames() or capture()
        '''
        # setup device and device queues
        
        
        self.ready = True
        return self.ready

    def stop(self):
        '''Stop the capture stream and release the device from use
        '''
        self.device = None
        self.ready = False

    def get_configs(self):
        return super().get_configs().getConfig()
    
    def set_configs(self, configs):
        super().set_configs(ConfigRS205(configs))
        # update the cameras
        

    def set(self, cam, key, value):
        '''Set a particular configuration

        Args:
            cam (any): name of the camera to config 
                - (monoLeftCam / monoRightCam / depthCam / spatialLocationCalculator)
            key (any): The configuration name
            value (any): The value to be set
        '''
        self.configs.setConfigParameter(cam, key, value)

    def get_frames(self, normalization=False, scaleFactor=1):
        '''Gets the next frame

        Args:
            normalization (optional) : 
                - if True -> will normalize the depth frame [0,255]
                - if False -> raw depth frame will be returned
            scaleFactor (optional) : scaling factor on outputted frame
        Returns:
            depth frame in units of [mm] if not normalized
                - if normalized, range will be from [0,255]
        '''
        if not self.ready:
            return None
        

        # normalize the depth frame
        pass
    
    def capture(self):
        '''Alias for get_frames
        '''
        return self.get_frames
    
    def display(self, frame, windowName='frame'):
        cv2.imshow(windowName, frame)

    def fancyPrintConfigs(self):
        '''Prints out the configurations in a more human readable way
        '''
        cfg = self.configs.getConfig()
        keys = list(cfg.keys())
        for key in keys:
            subKeys = list(cfg[key].keys())
            print(key)
            for subKey in subKeys:
                print("  - ", subKey, " "*(22 - len(subKey)), ": ", cfg[key][subKey])


class RGBD(Depth):
    '''OAK class to capture images and depth (there are no color images for this camera, only black and white)

    The depth images are aligned to the left camera by default class initialization
    '''
    def __init__(self, configs=ConfigRS205()):
        super().__init__(configs=configs)

        # create new xlinks for mono cam
        

        # link the mono cams to xlink
        

    def start(self):
        '''Start the capture stream. This must be called before get_frames() or capture()
        '''
        # setup device queues
        
        
        self.ready = True
        return self.ready

    def stop(self):
        '''Stop the capture stream and release the device from use
        '''
        self.device = None
        self.ready = False

    def get_frames(self, normalization=False, scaleFactor=1):
        '''Gets the next frame.

        Args:
            normalization (optional) : 
                - if True -> will normalize the depth frame [0,255]
                - if False -> raw depth frame will be returned
            scaleFactor (optional) : scaling factor on outputted frame
        Returns:
            - if getBothMonoFrames = True -> ((monoLeftFrame, monoRightFrame), depthFrame, True)
            - if getBothMonoFrames = False -> (monoLeftFrame, depthFrame, True)

            depth frame in units of [mm] if not normalized
                - if normalized, range will be from [0,255]
        '''
        if not self.ready:
            return (None, None)
        

        
        
    def capture(self, normalization=True):
        '''Gets RGBD frames in ImageRGBD() class format
        '''
        frames = self.get_frames(normalization=normalization)
        images = base.ImageRGBD()
        images.color = frames[0]
        images.depth = frames[1]
        return images
    
    
    def process_loop(self, theProcessor, figOut=False):
        '''Will loop through indefinitely and send the obtained data to the passed function.
        The raw data can be visualized if set, otherwise the processing function is 
        responsible for handling output of raw, intermediat, or final data
            
        Args:
            theProcessor (any) : 
                - RGBD stream data processor. Should handle input
            figOut (optional) : 
                - true -> automatically show raw data
                - false -> do NOT automatically show raw data
        '''
        
        self.start()

        while True:
            images = self.capture()

            theProcessor(images)

            if figOut:
                self.display(images.color, "color")
                self.display(images.depth, "depth")
            
            opKey = display.wait(1)
            if opKey == ord('q'):
                break

        self.stop()
        if figOut:
            display.close("color")
            display.close("depth")


    def process_frames_selected(self, theProcessor, figOut=True):
        '''Replay and process data from the bag file attached to this instance.
        Will loop through the bag file and send obtained data to the passed function.
        The raw data can be visulaized if set, otherwise the processing function is responsible
        for handling output of raw, intermediate, or final data.

        Args:
            theProcessor (any) :
                - RGBD stream data processor. Should handle input.
            figOut (optional) :
                - true -> automatically show raw data
                - false -> do NOT automatically show raw data
        '''
        print("Replaying bag video. Hit 'q' to quit. Hit 's' to select.")
        print("Selection will trigger additional processing. It might involve")
        print("additional user input from keyboard or mouse.")

        self.start()

        while True:
            images = self.capture()

            if figOut:
                self.display(images.color, "color")
                self.display(images.depth, "depth")

            opKey = display.wait(1)
            if opKey == ord('q'):
                break
            elif opKey == ord('s'):
                theProcessor(images)


        self.stop()


    def process_frame(self, theProcessor, figOut=False):
        images = self.capture()

        theProcessor(images)

        if figOut:
            self.display(images.color, "color")
            self.display(images.depth, "depth")

        if figOut:
            display.close("color")
            display.close("depth")
