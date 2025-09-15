'''

@brief          OAK-D SR Camera Interface

@Author         Kyle de Nobel           knobel3@gatech.edu

@date           9/15/2025

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
            self.config["spatialLocationCalculator"] = {}

            # config monoLeftCam properies
            self.config["monoLeftCam"]["resolution"] = dai.MonoCameraProperties.SensorResolution.THE_400_P
            self.config["monoLeftCam"]["setSide"] = "left"

            # config monoRightCam Properties
            self.config["monoRightCam"]["resolution"] = dai.MonoCameraProperties.SensorResolution.THE_400_P
            self.config["monoRightCam"]["setSide"] = "right"

            # config depthCam properties
            self.config["depthCam"]["leftRightCheck"] = True
            self.config["depthCam"]["subpixel"] = True
            self.config["depthCam"]["defaultProfilePreset"] = dai.node.StereoDepth.PresetMode.DEFAULT
            
            # config spatialLocationCalculator
            self.config["spatialLocationCalculator"]["roiTopLeft"] = dai.Point2f(0.4, 0.4)
            self.config["spatialLocationCalculator"]["roiBottomRight"] = dai.Point2f(0.6, 0.6)
            self.config["spatialLocationCalculator"]["lowerDepthThreshold"] = 100           # this is in mm
            self.config["spatialLocationCalculator"]["upperDepthThreshold"] = 1000          # this is in mm
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
    def __init__(self, configs=ConfigOak()):
        super().__init__(configs=configs)

        # setup pipelines and cameras
        self.pipeline = dai.Pipeline()
        self.monoLeftCam =  self.pipeline.create(dai.node.MonoCamera)
        self.monoRightCam = self.pipeline.create(dai.node.MonoCamera)

        # setup output from device links
        self.monoLeftXlink = self.pipeline.create(dai.node.XLinkOut)
        self.monoLeftXlink.setStreamName('monoLeftCam')
        self.monoRightXlink = self.pipeline.create(dai.node.XLinkOut)
        self.monoRightXlink.setStreamName('monoRightCam')

        # link devices
        self.monoLeftCam.out.link(self.monoLeftXlink.input)
        self.monoRightCam.out.link(self.monoRightXlink.input)

        # configure the cameras with all of the configs
        self.monoLeftCam.setResolution(self.get_configs()['monoLeftCam']['resolution'])
        self.monoLeftCam.setCamera(self.get_configs()['monoLeftCam']['setSide'])
        self.monoRightCam.setResolution(self.get_configs()['monoRightCam']['resolution'])
        self.monoRightCam.setCamera(self.get_configs()['monoRightCam']['setSide'])

        # not ready until start is called
        self.ready = False

    def start(self):
        '''Start the capture stream. This must be called before get_frames() or capture()
        '''
        # setup device and device queues
        self.device = dai.Device(self.pipeline)
        self.monoLeftQueue = self.device.getOutputQueue('monoLeftCam', 4, blocking=False)
        self.monoRightQueue = self.device.getOutputQueue('monoRightCam', 4, blocking=False)
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
        super().set_configs(ConfigOak(configs))
        # update the cameras. Ignore all depthCam values bc no depth cam
        self.monoLeftCam.setResolution(self.get_configs()['monoLeftCam']['resolution'])
        self.monoLeftCam.setCamera(self.get_configs()['monoLeftCam']['setSide'])
        self.monoRightCam.setResolution(self.get_configs()['monoRightCam']['resolution'])
        self.monoRightCam.setCamera(self.get_configs()['monoRightCam']['setSide'])

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
        monoLeftIn = self.monoLeftQueue.get()
        monoRightIn = self.monoRightQueue.get()

        monoLeftFrame = monoLeftIn.getCvFrame()
        monoRightFrame = monoRightIn.getCvFrame()

        # scale the image if desired
        monoLeftFrame = np.repeat(monoLeftFrame, scaleFactor, axis=1)
        monoLeftFrame = np.repeat(monoLeftFrame, scaleFactor, axis=0)
        monoRightFrame = np.repeat(monoRightFrame, scaleFactor, axis=1)
        monoRightFrame = np.repeat(monoRightFrame, scaleFactor, axis=0)

        if getBothMonoFrames:
            return (monoLeftFrame, monoRightFrame)
        else:
            return monoLeftFrame
    
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
    '''
    def __init__(self, configs=ConfigOak()):
        super().__init__(configs=configs)

        # setup pipelines and cameras
        self.pipeline = dai.Pipeline()
        self.monoLeftCam =  self.pipeline.create(dai.node.MonoCamera)
        self.monoRightCam = self.pipeline.create(dai.node.MonoCamera)
        self.depthCam = self.pipeline.create(dai.node.StereoDepth)
        self.spatialLocationCalculator = self.pipeline.create(dai.node.SpatialLocationCalculator)

        # setup output from device links
        self.depthXLink = self.pipeline.create(dai.node.XLinkOut)
        self.depthXLink.setStreamName('depth')
        self.spatialDataXLink = self.pipeline.create(dai.node.XLinkOut)
        self.spatialDataXLink.setStreamName('spatialData')
        self.spatialCalcConfigXLink = self.pipeline.create(dai.node.XLinkIn)
        self.spatialCalcConfigXLink.setStreamName('spatialCalcConfig')

        # link devices
        self.monoLeftCam.out.link(self.depthCam.left)           # output left camera to depth left
        self.monoRightCam.out.link(self.depthCam.right)         # output right camera to depth right
        self.spatialLocationCalculator.passthroughDepth.link(self.depthXLink.input)
        self.depthCam.depth.link(self.spatialLocationCalculator.inputDepth)
        self.spatialLocationCalculator.out.link(self.spatialDataXLink.input)
        self.spatialCalcConfigXLink.out.link(self.spatialLocationCalculator.inputConfig)

        # configure the cameras with all of the configs
        self.monoLeftCam.setResolution(self.get_configs()['monoLeftCam']['resolution'])
        self.monoLeftCam.setCamera(self.get_configs()['monoLeftCam']['setSide'])
        self.monoRightCam.setResolution(self.get_configs()['monoRightCam']['resolution'])
        self.monoRightCam.setCamera(self.get_configs()['monoRightCam']['setSide'])
        self.depthCam.setDefaultProfilePreset(self.get_configs()['depthCam']['defaultProfilePreset'])
        self.depthCam.setLeftRightCheck(self.get_configs()['depthCam']['leftRightCheck'])
        self.depthCam.setSubpixel(self.get_configs()['depthCam']['subpixel'])
        config = dai.SpatialLocationCalculatorConfigData()
        config.depthThresholds.lowerThreshold = self.get_configs()["spatialLocationCalculator"]["lowerDepthThreshold"]
        config.depthThresholds.upperThreshold = self.get_configs()["spatialLocationCalculator"]["upperDepthThreshold"]
        self.calculationAlgorithm = dai.SpatialLocationCalculatorAlgorithm.MEDIAN
        config.roi = dai.Rect(self.get_configs()["spatialLocationCalculator"]["roiTopLeft"], self.get_configs()["spatialLocationCalculator"]["roiBottomRight"])
        self.spatialLocationCalculator.inputConfig.setWaitForMessage(False)
        self.spatialLocationCalculator.initialConfig.addROI(config)

        ### TODO:: NOTE:: add this in configs
        self.depthCam.setDepthAlign(dai.RawStereoDepthConfig.AlgorithmControl.DepthAlign.CENTER)

        # not ready until start is called
        self.ready = False

    def start(self):
        '''Start the capture stream. This must be called before get_frames() or capture()
        '''
        # setup device and device queues
        self.device = dai.Device(self.pipeline)
        self.depthQueue = self.device.getOutputQueue('depth', 4, blocking=False)
        self.spatialCalcQueue = self.device.getOutputQueue(name='spatialData', maxSize=4, blocking=False)
        self.spatialCalcConfigInQueue = self.device.getInputQueue(name='spatialCalcConfig')
        
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
        super().set_configs(ConfigOak(configs))
        # update the cameras
        self.monoLeftCam.setResolution(self.get_configs()['monoLeftCam']['resolution'])
        self.monoLeftCam.setCamera(self.get_configs()['monoLeftCam']['setSide'])
        self.monoRightCam.setResolution(self.get_configs()['monoRightCam']['resolution'])
        self.monoRightCam.setCamera(self.get_configs()['monoRightCam']['setSide'])
        self.depthCam.setDefaultProfilePreset(self.get_configs()['depthCam']['defaultProfilePreset'])
        self.depthCam.setLeftRightCheck(self.get_configs()['depthCam']['leftRightCheck'])
        self.depthCam.setSubpixel(self.get_configs()['depthCam']['subpixel'])
        config = dai.SpatialLocationCalculatorConfigData()
        config.depthThresholds.lowerThreshold = self.get_configs()["spatialLocationCalculator"]["lowerDepthThreshold"]
        config.depthThresholds.upperThreshold = self.get_configs()["spatialLocationCalculator"]["upperDepthThreshold"]
        self.calculationAlgorithm = dai.SpatialLocationCalculatorAlgorithm.MEDIAN
        config.roi = dai.Rect(self.get_configs()["spatialLocationCalculator"]["roiTopLeft"], self.get_configs()["spatialLocationCalculator"]["roiBottomRight"])
        self.spatialLocationCalculator.inputConfig.setWaitForMessage(False)
        self.spatialLocationCalculator.initialConfig.addROI(config)

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
        depth = self.depthQueue.get()
        depthFrame = depth.getFrame() # in millimeters

        depthFrame = np.repeat(depthFrame, scaleFactor, axis=1)
        depthFrame = np.repeat(depthFrame, scaleFactor, axis=0)

        # normalize the depth frame
        if normalization:
            normalizedDepthFrame = np.interp(depthFrame, (100, 1000), (0,255)).astype(np.uint8)
            return normalizedDepthFrame

        return depthFrame
    
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
    '''
    def __init__(self, configs=ConfigOak()):
        super().__init__(configs=configs)

        # create new xlinks for mono cam
        self.monoLeftXLink = self.pipeline.create(dai.node.XLinkOut)
        self.monoLeftXLink.setStreamName('monoLeftCam')
        self.monoRightXLink = self.pipeline.create(dai.node.XLinkOut)
        self.monoRightXLink.setStreamName('monoRightCam')

        # link the mono cams to xlink
        self.monoLeftCam.out.link(self.monoLeftXLink.input)
        self.monoRightCam.out.link(self.monoRightXLink.input)

    def start(self):
        '''Start the capture stream. This must be called before get_frames() or capture()
        '''
        # setup device queues
        self.device = dai.Device(self.pipeline)
        self.monoLeftQueue = self.device.getOutputQueue('monoLeftCam', 4, blocking=False)
        self.monoRightQueue = self.device.getOutputQueue('monoRightCam', 4, blocking=False)
        self.depthQueue = self.device.getOutputQueue('depth', 4, blocking=False)
        self.spatialCalcQueue = self.device.getOutputQueue(name='spatialData', maxSize=4, blocking=False)
        self.spatialCalcConfigInQueue = self.device.getInputQueue(name='spatialCalcConfig')
        
        self.ready = True
        return self.ready

    def stop(self):
        '''Stop the capture stream and release the device from use
        '''
        self.device = None
        self.ready = False

    def get_frames(self, normalization=False, getBothMonoFrames=False, scaleFactor=1):
        '''Gets the next frame.

        Args:
            normalization (optional) : 
                - if True -> will normalize the depth frame [0,255]
                - if False -> raw depth frame will be returned
            getBothMonoFrames (optional) :
                - if True -> will return both left and right image frames
                - if False -> will only return the left image frame
            scaleFactor (optional) : scaling factor on outputted frame
        Returns:
            - if getBothMonoFrames = True -> ((monoLeftFrame, monoRightFrame), depthFrame, True)
            - if getBothMonoFrames = False -> (monoLeftFrame, depthFrame, True)

            depth frame in units of [mm] if not normalized
                - if normalized, range will be from [0,255]
        '''
        if not self.ready:
            return (None, None)
        monoLeftIn = self.monoLeftQueue.get()
        monoRightIn = self.monoRightQueue.get()

        monoLeftFrame = monoLeftIn.getCvFrame()
        monoRightFrame = monoRightIn.getCvFrame()

        # scale the image if desired
        monoLeftFrame = np.repeat(monoLeftFrame, scaleFactor, axis=1)
        monoLeftFrame = np.repeat(monoLeftFrame, scaleFactor, axis=0)
        monoRightFrame = np.repeat(monoRightFrame, scaleFactor, axis=1)
        monoRightFrame = np.repeat(monoRightFrame, scaleFactor, axis=0)

        if getBothMonoFrames:
            return (monoLeftFrame, monoRightFrame), super().get_frames(normalization=normalization, scaleFactor=scaleFactor*2)
        else:
            return (monoLeftFrame, super().get_frames(normalization=normalization, scaleFactor=scaleFactor*2))
        
    def capture(self):
        '''Alias for get_frames
        '''
        return self.get_frames
    
    def process_loop(self, theProcessor, figOut=False):
        '''Will loop through indefinitely and send the obtained data to the passed function.
            The raw data can be visualized if set, otherwise the processing function is 
            responsible for handling output of raw, intermediat, or final data
        '''
        # TODO: implement
        pass

    def process_frames_selected():
        # TODO: implement
        pass

    def process_frame():
        #TODO: implement
        pass







## ======= testing cam implementation ======= ##
'''

Testing functionality of RGBD camera and Color camera
    - RGBD inherits Depth camera, so this intrinsically tests the depth camera as well

Creating multiple camera objects in one file, stop and start allowing them to not conflic with each other
Using depth frames to configure images
Displaying both depth frames and image frames

'''
cam = RGBD()

cam.start()

while True:
    monoFrame, depthFrame = cam.get_frames(normalization=True)
    # show black as far away
    blackFarAwayDepthFrame = 255 - depthFrame

    cam.display(blackFarAwayDepthFrame, windowName='depth')
    cam.display(monoFrame, windowName="image from cam")


    # only show objects that are close to the camera
    closeDepthFrame = np.empty_like(depthFrame)
    # (150/255)*1000 = 588 mm = 58.8 CM
    mask = (depthFrame < 150)
    closeDepthFrame[~mask] = 0
    closeDepthFrame[mask] = monoFrame[mask]

    cam.display(closeDepthFrame, windowName="only show close up objects")

    # cv2.imshow("only show close up objects", closeDepthFrame)

    if cv2.waitKey(1) == ord('q'):
        break

cam.stop()

cv2.destroyAllWindows()

cam2 = Color()
cam2.start()

while True:
    leftMonoFrame, rightMonoFrame = cam2.get_frames(getBothMonoFrames=True)
    # show black as far away
    # cv2.imshow("left image from cam2", monoFrame)


    cam2.display(leftMonoFrame, "left image from cam2")
    cam2.display(rightMonoFrame, "right image from cam2")

    if cv2.waitKey(1) == ord('q'):
        break

cam2.stop()