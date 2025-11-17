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
import os
import yaml




class ConfigRS305(base.CfgCamera):
    '''config general parameters of Realsense 305 camera
    Args:
        filePath (str):
            path of yaml configuration file
    '''
    def __init__(self, yamlFilePath:str=None):
        super().__init__(new_allowed=True)
        # check if filePath is empty, if so we use default initialization
        if yamlFilePath is None:
            currentPath = os.path.dirname(os.path.realpath(__file__))
            yamlFilePath = currentPath+"/utils/cameraInitializationFiles/realsense305.yaml"
            
        with open(yamlFilePath) as stream:
            init = yaml.load(stream, yaml.SafeLoader)

        super().__init__(init)
     

    def setConfig(self, filePath:str):
        '''allows setting all configs at once
        Args:
        filePath (str):
            path of yaml configuration file
        '''
        self.clear()
        self.merge_from_file(filePath)

    
    def setConfigParameter(self, cam, param, value):
        '''allows the setting of a specific parameter of a camera to a certain value

        Args:
            cam (any): name of camera
            param (any): name of camera parameter
            value (any): value to set parameter to
        '''
        self[cam][param] = value


class Color(base.Color):
    '''Realsense305 class to caputre images
    Args:
        yamlInitFilePath (str):
            path of yaml file to use for camera initialization
    '''
    def __init__(self, yamlInitFilePath:str=None):
        configs = ConfigRS305(yamlInitFilePath)
        super().__init__(configs=configs)

        # setup the camera pipeline
        self.pipeline = rs.pipeline()

        # enable the color camera
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 
                                  self.configs["colorCamera"]["imageWidth"],
                                  self.configs["colorCamera"]["imageHeight"], 
                                  self.configs["colorCamera"]["dataFormat"], 
                                  self.configs["colorCamera"]["FPS"])

        # get camera intrinsics
        intrinsics = rs.intrinsics
        self.cameraMatrix = np.array([[intrinsics.fx, 0, intrinsics.ppx],
                                      [0, intrinsics.fy, intrinsics.ppy],
                                      [0,0,1]])
        self.distortionCoeffs = intrinsics.coeffs
        self.K = self.cameraMatrix

        # put ready to false since we have not started the camera
        self.ready = False
        

    def start(self):
        '''Start the capture stream. This must be called before get_frames() or capture()
        '''
        # setup device and device queues
        self.pipeline.start(self.config)
        
        self.ready = True
        return self.ready
    

    def stop(self):
        '''Stop the capture stream and release the device from use
        '''
        self.pipeline.stop()

        self.ready = False


    def get_configs(self):
        '''Returns all of the configs for the camera
        '''
        return super().get_configs()
    
    
    def set_configs(self, yamlFilePath:str):
        '''Re-sets all of the configs for the camera
        Args:
            yamlFilePath (str):
                path of yaml configuration file
        '''
        super().set_configs(ConfigRS305(yamlFilePath))
        # update the cameras. Ignore all depthCam values bc no depth cam
        previouslyReady = False
        # stop the pipeline if the stream is currently on
        if self.ready:
            previouslyReady = True
            self.stop()
        # disable the stream
        self.config.disable_stream(rs.stream.color)
        # re-enable the stream with new parameters
        self.config.enable_stream(rs.stream.color,
                                  self.configs["colorCamera"]["imageWidth"],
                                  self.configs["colorCamera"]["imageHeight"],
                                  self.configs["colorCamera"]["dataFormat"],
                                  self.configs["colorCamera"]["FPS"])
        # if the pipeline was previously on, re-enable the pipeline
        if previouslyReady:
            self.start()

    def get_frames(self):
        ''' Gets the next frame

        Returns:
            frame:
                rgb color frame
        '''
        if not self.ready:
            return None, False
        
        # get the latest frame
        frames = self.pipeline.wait_for_frames()
        colorFrame = frames.get_color_frame()
        colorFrame = np.asanyarray(colorFrame.get_data())

        return colorFrame

    def capture(self):
        '''Alias for get_frames
        '''
        return self.get_frames()
    

    def display(self, frame, windowName='frame'):
        '''Displays an image
        Args:
            frame (ndarray (N,M)):
                an image frame
            windowName (str):
                the name of the window
        '''
        cv2.imshow(windowName, frame)
    

    def printConfigs(self):
        '''Prints out the configurations in a more human readable way
        '''
        print(self.configs)


    def getCameraIntrinsics(self):
        '''Returns the intrinsic camera matrix and distortion coefficients

        Returns:
            cameraMatrix:
            distortionCoeffs :
                (list (3,3)) intrinsice camera matrix distortionCoeffs 
                (list (,14)) distortion coefficients of the camera
        '''
        return self.cameraMatrix, self.distortionCoeffs


class Depth(base.Base):
    '''Realsense305 class to capture depth frames

    The depth images are aligned to the left camera by default class initialization
    Args:
        yamlInitFilePath (str):
            path of yaml file to use for camera initialization
    '''
    def __init__(self, yamlInitFilePath:str=None):
        if yamlInitFilePath is None:
            configs = ConfigRS305(yamlInitFilePath)
            super().__init__(configs=configs)

        # setup the camera pipeline
        self.pipeline = rs.pipeline()

        # enable the depth camera
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 
                                  self.configs["depthCamera"]["imageWidth"],
                                  self.configs["depthCamera"]["imageHeight"], 
                                  rs.format(self.configs["depthCamera"]["dataFormat"]), 
                                  self.configs["depthCamera"]["FPS"])
        
        # enable the color camera for depth alignment
        self.config.enable_stream(rs.stream.color,
                                  self.configs["colorCamera"]["imageWidth"],
                                  self.configs["colorCamera"]["imageHeight"],
                                  rs.format(self.configs["colorCamera"]["dataFormat"]),
                                  self.configs["colorCamera"]["FPS"])

        # get camera intrinsics
        intrinsics = rs.intrinsics
        self.cameraMatrix = np.array([[intrinsics.fx, 0, intrinsics.ppx],
                                      [0, intrinsics.fy, intrinsics.ppy],
                                      [0,0,1]])
        self.distortionCoeffs = intrinsics.coeffs
        self.K = self.cameraMatrix

        # need to align the depth frame with the color camera
        self.align = rs.align(rs.stream.color)

        # conversion of sensor units to meters
        self.depthScale = None

        # put ready to false since we have not started the camera
        self.ready = False


    def start(self):
        '''Start the capture stream. This must be called before get_frames() or capture()
        '''
        # setup device and device queues
        device = self.pipeline.start(self.config)

        # get the depth scale
        self.depthScale = device.get_device().first_depth_sensor().get_depth_scale()
        
        self.ready = True
        return self.ready
    

    def stop(self):
        '''Stop the capture stream and release the device from use
        '''
        self.pipeline.stop()
        self.depthScale = None

        self.ready = False


    def get_configs(self):
        '''Returns all of the configs for the camera
        '''
        return super().get_configs()
    
    
    def set_configs(self, yamlFilePath:str):
        '''Re-sets all of the configs for the camera
        Args:
            yamlFilePath (str):
                path of yaml configuration file
        '''
        super().set_configs(ConfigRS305(yamlFilePath))
        # update the cameras. Ignore all depthCam values bc no depth cam
        previouslyReady = False
        # stop the pipeline if the stream is currently on
        if self.ready:
            previouslyReady = True
            self.stop()
        # disable the stream
        self.config.disable_stream(rs.stream.color)
        # re-enable the stream with new parameters
        self.config.enable_stream(rs.stream.color,
                                  self.configs["depthCamera"]["imageWidth"],
                                  self.configs["depthCamera"]["imageHeight"],
                                  self.configs["depthCamera"]["dataFormat"],
                                  self.configs["depthCamera"]["FPS"])
        # if the pipeline was previously on, re-enable the pipeline
        if previouslyReady:
            self.start()


    def get_frames(self, normalization=False):
        '''Gets the next frame.
        Args:
            normalization (bool):
                - if True, the depth frame will be normalized to 0-255 for better visulaization
                - if False, the depth frame will not be normalized and is returned in units of meters
        Returns:
            (colorFrame, depthFrame):
                - colorFrame = color frame of camera
                - depthFrame = depth frame of camera
        '''
        if not self.ready:
            return None
        
        # get the latest frame
        frames = self.pipeline.wait_for_frames()
        # align the depth frame  to the color frame
        frames = self.align.process(frames)
        depthFrame = frames.get_depth_frame()
        depthFrame = np.asanyarray(depthFrame.get_data())
        
        if normalization:
            depthFrame = np.interp(depthFrame, (min(depthFrame), max(depthFrame)), (0, 255)).astype(np.uint8)
        else:
            depthFrame = depthFrame * self.depthScale

        return depthFrame
        
    
    def capture(self):
        '''Alias for get_frames
        '''
        return self.get_frames()
    
    
    def display(self, frame, windowName='frame'):
        '''Displays an image
        Args:
            frame (ndarray (N,M)):
                an image frame
            windowName (str):
                the name of the window
        '''
        cv2.imshow(windowName, frame)


    def printConfigs(self):
        '''Prints out the configurations in a more human readable way
        '''
        print(self.configs)
        

    def getCameraIntrinsics(self):
        '''Returns the intrinsic camera matrix and distortion coefficients

        Returns:
            cameraMatrix:
            distortionCoeffs :
                (list (3,3)) intrinsice camera matrix distortionCoeffs 
                (list (,14)) distortion coefficients of the camera
        '''
        return self.cameraMatrix, self.distortionCoeffs


class RGBD(Depth):
    '''Realsense305 class to capture color images and depth

    The depth images are aligned to the color camera
    Args:
        yamlInitFilePath (str):
            path of yaml file to use for camera initialization
    '''
    def __init__(self, yamlInitFilePath:str=None):
        super().__init__(yamlInitFilePath)
        

    def get_frames(self, normalization=False):
        '''Gets the next frame.
        Args:
            normalization (bool):
                - if True, the depth frame will be normalized to 0-255 for better visulaization
                - if False, the depth frame will not be normalized and is returned in units of meters
        Returns:
            (colorFrame, depthFrame):
                - colorFrame = color frame of camera
                - depthFrame = depth frame of camera
        '''
        if not self.ready:
            return (None, None)
        
        # get the latest frame
        frames = self.pipeline.wait_for_frames()
        # align the depth frame  to the color frame
        frames = self.align.process(frames)
        depthFrame = frames.get_depth_frame()
        depthFrame = np.asanyarray(depthFrame.get_data())

        if normalization:
            depthFrame = np.interp(depthFrame, (np.min(depthFrame), np.max(depthFrame)), (0, 255)).astype(np.uint8)
        else:
            depthFrame = depthFrame * self.depthScale
        
        # get the color frame
        colorFrame = frames.get_color_frame()
        colorFrame = np.asanyarray(colorFrame.get_data())

        return colorFrame, depthFrame

        
    def capture(self, normalization=True):
        '''Gets RGBD frames in ImageRGBD() class format
        Args:
            normalization (bool):
                - if True, the depth frame will be normalized to 0-255 for better visulaization
                - if False, the depth frame will not be normalized and is returned in units of meters
        '''
        colorImage, depthImage = self.get_frames(normalization=normalization)
        images = base.ImageRGBD()
        images.color = colorImage
        images.depth = depthImage
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
