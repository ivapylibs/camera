'''

@brief          OAK-D SR Camera Interface

@Author         Kyle de Nobel           knobel3@gatech.edu

@date           

'''

import camera.base as base
import depthai as dai
import cv2
import numpy as np
import math


class ConfigOak():
    '''config general parameters of OAK camera
    '''
    def __init__(self, config=None):
        if config is None:
            # setup config struct
            self.config = {}
            # define each type of camera
            self.config["monoLeftCam"] = {}
            self.config["monoRightCam"] = {}
            self.config["depthCam"] = {}

            # config monoLeftCam properies
            self.config["monoLeftCam"]["resolution"] = dai.MonoCameraProperties.SensorResolution.THE_400_P
            self.config["monoLeftCam"]["setSide"] = "left"

            # config monoRightCam Properties
            self.config["monoRightCam"]["resolution"] = dai.MonoCameraProperties.SensorResolution.THE_400_P
            self.config["monoRightCam"]["setSide"] = "right"

            # config depthCam properties
            self.config["depthCam"]["leftRightCheck"] = True
            self.config["depthCam"]["extendedDisparity"] = False
            self.config["depthCam"]["subpixel"] = True
            self.config["depthCam"]["subpixelFractionalBits"] = 3
            self.config["depthCam"]["medianFilter"] = dai.MedianFilter.KERNEL_7x7
            self.config["depthCam"]["defaultProfilePreset"] = dai.node.StereoDepth.PresetMode.DEFAULT
            self.config["depthCam"]["confidenceLevel"] = 150
            self.config["depthCam"]["maxDistanceCM"] = 500  # this value is in Centimeters
            self.config["depthCam"]["baselineCM"] = 2       # this value is in Centimeters
            self.config["depthCam"]["HFOV"] = 80            # this value is in degrees
            self.config["depthCam"]["imageWidthInPixels"] = 640 # 640 due to the resolution selection
            self.config["depthCam"]["focalLengthInPixels"] = self.config["depthCam"]["imageWidthInPixels"] * 0.5 / math.tan(self.config["depthCam"]["HFOV"] * 0.5 * math.pi/180)
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

# TODO: rename to aligned as well bc color should only have color
class Color(base.Color):
    '''OAK class to capture images (there are no color images for this camera, only black and white)
    '''
    def __init__(self, configs=ConfigOak()):
        super().__init__(configs=configs)

        # setup pipelines and cameras
        self.pipeline = dai.Pipeline()
        self.monoLeftCam = self.pipeline.create(dai.node.MonoCamera)
        self.monoRightCam = self.pipeline.create(dai.node.MonoCamera)
        self.depthCam = self.pipeline.create(dai.node.StereoDepth)

        # setup output from device links
        self.monoLeftXlink = self.pipeline.create(dai.node.XLinkOut)
        self.monoLeftXlink.setStreamName('monoLeftCam')
        self.monoRightXlink = self.pipeline.create(dai.node.XLinkOut)
        self.monoRightXlink.setStreamName('monoRightCam')
        self.depthXlink = self.pipeline.create(dai.node.XLinkOut)
        self.depthXlink.setStreamName('disparity')

        # link devices
        self.monoLeftCam.out.link(self.monoLeftXlink.input)         # link monoLeftCam output to monoLeftXlink input
        self.monoRightCam.out.link(self.monoRightXlink.input)       # link monoRightCam output to monoRightXlink input
        self.monoLeftCam.out.link(self.depthCam.left)               # link monoLeftCam output to left of the depthCam disparity
        self.monoRightCam.out.link(self.depthCam.right)             # link monoRightCam output to right of the depthCam disparity
        self.depthCam.disparity.link(self.depthXlink.input)         # link depthCam disparity output to depthXlink input

        # configure the cameras with all of the configs
        self.monoLeftCam.setResolution(self.get_configs()['monoLeftCam']['resolution'])
        self.monoLeftCam.setCamera(self.get_configs()['monoLeftCam']['setSide'])
        self.monoRightCam.setResolution(self.get_configs()['monoRightCam']['resolution'])
        self.monoRightCam.setCamera(self.get_configs()['monoRightCam']['setSide'])
        self.depthCam.setDefaultProfilePreset(self.get_configs()['depthCam']['defaultProfilePreset'])
        self.depthCam.initialConfig.setMedianFilter(self.get_configs()['depthCam']['medianFilter'])
        self.depthCam.setLeftRightCheck(self.get_configs()['depthCam']['leftRightCheck'])
        self.depthCam.setExtendedDisparity(self.get_configs()['depthCam']['extendedDisparity'])
        self.depthCam.setSubpixel(self.get_configs()['depthCam']['subpixel'])
        self.depthCam.setSubpixelFractionalBits(self.get_configs()['depthCam']['subpixelFractionalBits'])
        self.depthCam.initialConfig.setConfidenceThreshold(self.get_configs()['depthCam']['confidenceLevel'])

    def get_configs(self):
        return super().get_configs().getConfig()
    
    def set_configs(self, configs):
        super().set_configs(ConfigOak(configs))
        # update the cameras
        self.monoLeftCam.setResolution(self.get_configs()['monoLeftCam']['resolution'])
        self.monoLeftCam.setCamera(self.get_configs()['monoLeftCam']['setSide'])
        self.monoRightCam.setResolution(self.get_configs()['monoRightCam']['resolution'])
        self.monoRightCam.setCamera(self.get_configs()['monoRightCam']['setSide'])
        self.depthCam.setDefaultProfilePreset(self.get_configs()['depthCam']['defaultProfilePreset'])
        self.depthCam.initialConfig.setMedianFilter(self.get_configs()['depthCam']['medianFilter'])
        self.depthCam.setLeftRightCheck(self.get_configs()['depthCam']['leftRightCheck'])
        self.depthCam.setExtendedDisparity(self.get_configs()['depthCam']['extendedDisparity'])
        self.depthCam.setSubpixel(self.get_configs()['depthCam']['subpixel'])
        self.depthCam.setSubpixelFractionalBits(self.get_configs()['depthCam']['subpixelFractionalBits'])
        self.depthCam.initialConfig.setConfidenceThreshold(self.get_configs()['depthCam']['confidenceLevel'])

    def set(self, cam, key, value):
        '''Set a particular configuration

        Args:
            cam (any): name of the camera to config (monoLeftCam / monoRightCam / depthCam)
            key (any): The configuration name
            value (any): The value to be set
        '''
        self.configs.setConfigParameter(cam, key, value)

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

    def get_frames(self):
        with dai.Device(self.pipeline) as device:
            monoLeftQueue = device.getOutputQueue('monoLeftCam')
            monoRightQueue = device.getOutputQueue('monoRightCam')
            disparityQueue = device.getOutputQueue('disparity')

            # TODO: Fix
            inDisparity = disparityQueue.get()
            inMonoLeftCam = monoLeftQueue.get()
            inMonoRightCam = monoRightQueue.get()

            monoLeftImage = inMonoLeftCam.getCvFrame()

        return monoLeftImage
            




cam = Color()
# print(cam.get_configs())
cfg = cam.get_configs()
cfg['monoLeftCam']['setSide'] = "test"
cam.set_configs(cfg)

cam.fancyPrintConfigs()

cfg = cam.get_configs()
cfg['monoLeftCam']['setSide'] = "left"
cam.set_configs(cfg)


while True:
    cv2.imshow("image", cam.get_frames())