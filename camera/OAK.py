'''

@brief          OAK-D SR Camera Interface

@Author         Kyle de Nobel           knobel3@gatech.edu

@date           9/15/2025

@note: Comments above file are for Doxygen support and are the same as the docstrings

'''

import camera.base as base
import depthai as dai
import cv2
import numpy as np
import math
import ivapy.display_cv as display
import os
import yaml

## @brief config general parameters of OAK camera
class ConfigOak(base.CfgCamera):
    '''config general parameters of OAK camera
    '''

    ## @brief the constructor. If yamlFilePath is not give, default initialization is done
    # @param[in] yamlFilePath the file path of the yaml to read camera initailization parameters from
    def __init__(self, yamlFilePath:str=None):
        '''config general parameters of OAK camera
        Args:
            filePath (str):
                path of yaml configuration file
        '''
        super().__init__(new_allowed=True)
        # check if filePath is empty, if so we use default initialization
        if yamlFilePath is None:
            currentPath = os.path.dirname(os.path.realpath(__file__))
            yamlFilePath = currentPath+"/utils/cameraInitializationFiles/oak.yaml"
            
        with open(yamlFilePath) as stream:
            init = yaml.load(stream, yaml.SafeLoader)

        super().__init__(init)

    ## @brief allows setting all configs at once
    # @param[in] config a configuration dictionary
    def setConfig(self, config):
        '''allows setting all configs at once
        Args:
            config: the configuration dictionary
        '''
        self.config = config


    ## @brief returns config dictionary
    # @return config the config dictionary
    def getConfig(self):
        '''returns config dictionary
        Returns:
            congif: the config dictionary
        '''
        return self.config
    

    ## @brief allows the setting of a specific parameter of a camera to a certain value
    # @param[in] cam name of the camera to change the value of
    # @param[in] param the name of the camera parameters
    # @value[in] value to set parameter to
    # @note if parameters are unclear, print out config
    def setConfigParameter(self, cam, param, value):
        '''allows the setting of a specific parameter of a camera to a certain value

        Args:
            cam (any): name of camera
            param (any): name of camera parameter
            value (any): value to set parameter to
        '''
        self.config[cam][param] = value


## @brief OAK class to capture only images (there are no color images for this camera)
class Color(base.Color):
    '''OAK class to capture only images (there are no color images for this camera)
    '''

    ## @brief constructor
    # @param[in] configs ConfigOak class. Optional argument
    def __init__(self, configs:ConfigOak=ConfigOak()):
        super().__init__(configs=configs)

        # setup pipelines and cameras
        self.pipeline = dai.Pipeline()
        ## @var pipeline
        # camera pipeline (dai.Pipeline object)
        self.monoLeftCam =  self.pipeline.create(dai.node.MonoCamera)
        ## @var monoLeftCam
        # left mono camera node (dai.node.MonoCamera object)
        self.monoRightCam = self.pipeline.create(dai.node.MonoCamera)
        ## @var monoRightCam
        # right mono camera node (dai.node.MonoCamera object)

        # setup output from device links
        self.monoLeftXlink = self.pipeline.create(dai.node.XLinkOut)
        ## @var monoLeftXlink
        # link for left mono camera (dai.node.XlinkOut object)
        self.monoLeftXlink.setStreamName('monoLeftCam')
        self.monoRightXlink = self.pipeline.create(dai.node.XLinkOut)
        ## @var monoRightXlink
        # link for right mono camera (dai.node.XlinkOut object)
        self.monoRightXlink.setStreamName('monoRightCam')

        # link devices
        self.monoLeftCam.out.link(self.monoLeftXlink.input)
        self.monoRightCam.out.link(self.monoRightXlink.input)

        # configure the cameras with all of the configs
        self.monoLeftCam.setResolution(dai.MonoCameraProperties.SensorResolution(self.configs['monoLeftCam']['resolution']))
        self.monoLeftCam.setCamera("left") #self.configs['monoLeftCam']['setSide'])
        self.monoRightCam.setResolution(dai.MonoCameraProperties.SensorResolution(self.configs['monoRightCam']['resolution']))
        self.monoRightCam.setCamera(self.configs['monoRightCam']['setSide'])

        # not ready until start is called
        self.ready = False
        ## @var ready
        # indicates if the camera is ready to stream images

        # get and store the camera intrinsics
        with dai.Device(self.pipeline) as device:
            calibdata = device.readCalibration()
            leftMtx = calibdata.getCameraIntrinsics(dai.CameraBoardSocket.CAM_B, resizeWidth=640, resizeHeight=400)
            distCoeffs = calibdata.getDistortionCoefficients(dai.CameraBoardSocket.CAM_B)
        self.K = leftMtx
        ## @var K
        # camera intrinsic matrix
        self.cameraMatrix = leftMtx
        ## @var cameraMatrix
        # camera intrinsic matrix
        self.distortionCoeffs = distCoeffs
        ## @var distortionCoeffs
        # camera distortion coefficients


    ## @brief Start the capture stream. This must be called before get_frames() or capture()
    def start(self):
        '''Start the capture stream. This must be called before get_frames() or capture()
        '''
        # setup device and device queues
        self.device = dai.Device(self.pipeline)
        self.monoLeftQueue = self.device.getOutputQueue('monoLeftCam', 4, blocking=False)
        self.monoRightQueue = self.device.getOutputQueue('monoRightCam', 4, blocking=False)
        self.ready = True
        return self.ready


    ## @brief Stop the capture stream and release the device from use
    def stop(self):
        '''Stop the capture stream and release the device from use
        '''
        self.device = None
        self.ready = False


    ## @brief returns the configs of the camera
    # @return configs the config dictionary of the camera
    def get_configs(self):
        ''' Returns the configs of the camera
        Returns:
            configs: the config dictionary of the camera
        '''
        return super().configs.getConfig()
    

    ## @brief sets all of the configs of the camera
    # @param[in] configs the config dictionary for the camera
    def set_configs(self, configs):
        '''sets all of the configs of the camera
        Args:
            configs: the config dictionary for the camera
        '''
        super().set_configs(ConfigOak(configs))
        # update the cameras. Ignore all depthCam values bc no depth cam
        self.monoLeftCam.setResolution(dai.MonoCameraProperties.SensorResolution(self.configs['monoLeftCam']['resolution']))
        self.monoLeftCam.setCamera(self.configs['monoLeftCam']['setSide'])
        self.monoRightCam.setResolution(dai.MonoCameraProperties.SensorResolution(self.configs['monoRightCam']['resolution']))
        self.monoRightCam.setCamera(self.configs['monoRightCam']['setSide'])


    ## @brief Set a particular configuration
    # @param cam name of the camera to config (monoLeftCam / monoRightCam)
    # @param key The configuration name
    # @param value The value to be set
    def set(self, cam, key, value):
        '''Set a particular configuration

        Args:
            cam (any): name of the camera to config (monoLeftCam / monoRightCam)
            key (any): The configuration name
            value (any): The value to be set
        '''
        self.configs.setConfigParameter(cam, key, value)


    ## @brief Gets the next frame
    # @param[in] getBothMonoFrames if True, returns both left and right frames, False returns only left
    # @param[in] scaleFactor scaling factor on outputted frame
    # @return out the requested frames
    def get_frames(self, getBothMonoFrames:bool=False, scaleFactor:int=1):
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
    

    ## @brief Alias for get_frames
    def capture(self):
        '''Alias for get_frames
        '''
        return self.get_frames()
    

    ## @brief displays a frame
    # @param frame the frame to display
    # @param windowName the name of the window to display the frame on
    def display(self, frame:np.ndarray, windowName:str='frame'):
        '''displays a frame
        Args:
            frame (ndarray): frame the frame to display
            windowName (str): the name of the window to display the frame on
        '''
        cv2.imshow(windowName, frame)
    

    ## @brief Prints out the configurations in a more human readable way
    def printConfigs(self):
        '''Prints out the configurations in a more human readable way
        '''
        print(self.configs)


    ## @brief Returns the intrinsic camera matrix and distortion coefficients
    # @return out the camera intrinsic matrix and distortion coefficients
    def getCameraIntrinsics(self):
        '''Returns the intrinsic camera matrix and distortion coefficients

        Returns:
            cameraMatrix:
            distortionCoeffs :
                (list (3,3)) intrinsice camera matrix distortionCoeffs 
                (list (,14)) distortion coefficients of the camera
        '''
        return self.cameraMatrix, self.distortionCoeffs
    

## @brief OAK class to capture depth frames. 
# The depth images are aligned to the left camera by default class initialization
class Depth(base.Base):
    '''OAK class to capture depth frames

    The depth images are aligned to the left camera by default class initialization
    '''

    ## @breif constructor
    # @param[in] configs ConfigOak object. Optional argument
    def __init__(self, configs=ConfigOak()):
        super().__init__(configs=configs)

        # setup pipelines and cameras
        self.pipeline = dai.Pipeline()
        ## @var pipeline
        # camera pipeline (dai.Pipeline object)
        self.monoLeftCam =  self.pipeline.create(dai.node.MonoCamera)
        ## @var monoLeftCam
        # left mono camera node (dai.node.MonoCamera object)
        self.monoRightCam = self.pipeline.create(dai.node.MonoCamera)
        ## @var monoRightCam
        # right mono camera node (dai.node.MonoCamera object)
        self.depthCam = self.pipeline.create(dai.node.StereoDepth)
        ## @var depthCam
        # depth camera node (dai.node.StereoDetph object)
        self.spatialLocationCalculator = self.pipeline.create(dai.node.SpatialLocationCalculator)
        ## @var spatialLocationCalculator
        # spatial location calculator node (dai.node.SpatialLocationCalculator object)

        # setup output from device links
        self.depthXLink = self.pipeline.create(dai.node.XLinkOut)
        ## @var depthXLink
        # link for depth camera (dai.node.XLinkOut object)
        self.depthXLink.setStreamName('depth')
        self.spatialDataXLink = self.pipeline.create(dai.node.XLinkOut)
        ## @var spatialDataXLink
        # link for spatial location data (dai.node.XLinkOut object)
        self.spatialDataXLink.setStreamName('spatialData')
        self.spatialCalcConfigXLink = self.pipeline.create(dai.node.XLinkIn)
        ## @var spatialCalcConfigXLink
        # link for spatial loocation calculator (dai.node.XLinkOut object)
        self.spatialCalcConfigXLink.setStreamName('spatialCalcConfig')

        # link devices
        self.monoLeftCam.out.link(self.depthCam.left)           # output left camera to depth left
        self.monoRightCam.out.link(self.depthCam.right)         # output right camera to depth right
        self.spatialLocationCalculator.passthroughDepth.link(self.depthXLink.input)
        self.depthCam.depth.link(self.spatialLocationCalculator.inputDepth)
        self.spatialLocationCalculator.out.link(self.spatialDataXLink.input)
        self.spatialCalcConfigXLink.out.link(self.spatialLocationCalculator.inputConfig)

        # configure the cameras with all of the configs
        self.monoLeftCam.setResolution(dai.MonoCameraProperties.SensorResolution(self.configs['monoLeftCam']['resolution']))
        self.monoLeftCam.setCamera(self.configs['monoLeftCam']['setSide'])
        self.monoRightCam.setResolution(dai.MonoCameraProperties.SensorResolution(self.configs['monoRightCam']['resolution']))
        self.monoRightCam.setCamera(self.configs['monoRightCam']['setSide'])
        self.depthCam.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode(self.configs['depthCam']['defaultProfilePreset']))
        self.depthCam.setLeftRightCheck(self.configs['depthCam']['leftRightCheck'])
        self.depthCam.setSubpixel(self.configs['depthCam']['subpixel'])
        config = dai.SpatialLocationCalculatorConfigData()
        config.depthThresholds.lowerThreshold = self.configs["spatialLocationCalculator"]["lowerDepthThreshold"]
        config.depthThresholds.upperThreshold = self.configs["spatialLocationCalculator"]["upperDepthThreshold"]
        self.calculationAlgorithm = dai.SpatialLocationCalculatorAlgorithm.MEDIAN
        left = self.configs["spatialLocationCalculator"]["roiTopLeft"]
        right = self.configs["spatialLocationCalculator"]["roiBottomRight"]
        config.roi = dai.Rect(dai.Point2f(left[0], left[1]), dai.Point2f(right[0], right[1]))
        self.spatialLocationCalculator.inputConfig.setWaitForMessage(False)
        self.spatialLocationCalculator.initialConfig.addROI(config)
        self.depthCam.setDepthAlign(dai.RawStereoDepthConfig.AlgorithmControl.DepthAlign(self.configs["depthCam"]["setDepthAlign"]))

        # not ready until start is called
        self.ready = False
        ## @var ready
        # indicates if the camera is ready to stream images

        # get and store the camera intrinsics
        with dai.Device(self.pipeline) as device:
            calibdata = device.readCalibration()
            leftMtx = calibdata.getCameraIntrinsics(dai.CameraBoardSocket.CAM_B, resizeWidth=640, resizeHeight=400)
            distCoeffs = calibdata.getDistortionCoefficients(dai.CameraBoardSocket.CAM_B)
        self.K = leftMtx
        ## @var K
        # camera intrinsic matrix
        self.cameraMatrix = leftMtx
        ## @var cameraMatrix
        # camera intrinsic matrix
        self.distortionCoeffs = distCoeffs
        ## @var distortionCoeffs
        # camera distortion coefficients


    ## @brief Start the capture stream. This must be called before get_frames() or capture()
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


    ## @brief Stop the capture stream and release the device from use
    def stop(self):
        '''Stop the capture stream and release the device from use
        '''
        self.device = None
        self.ready = False


    ## @brief returns the configs of the camera
    # @return configs the config dictionary of the camera
    def get_configs(self):
        return super().configs.getConfig()
    

    ## @brief sets all of the configs of the camera
    # @param[in] configs the config dictionary for the camera
    def set_configs(self, configs):
        super().set_configs(ConfigOak(configs))
        # update the cameras
        self.monoLeftCam.setResolution(dai.MonoCameraProperties.SensorResolution(self.configs['monoLeftCam']['resolution']))
        self.monoLeftCam.setCamera(self.configs['monoLeftCam']['setSide'])
        self.monoRightCam.setResolution(dai.MonoCameraProperties.SensorResolution(self.configs['monoRightCam']['resolution']))
        self.monoRightCam.setCamera(self.configs['monoRightCam']['setSide'])
        self.depthCam.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode(self.configs['depthCam']['defaultProfilePreset']))
        self.depthCam.setLeftRightCheck(self.configs['depthCam']['leftRightCheck'])
        self.depthCam.setSubpixel(self.configs['depthCam']['subpixel'])
        config = dai.SpatialLocationCalculatorConfigData()
        config.depthThresholds.lowerThreshold = self.configs["spatialLocationCalculator"]["lowerDepthThreshold"]
        config.depthThresholds.upperThreshold = self.configs["spatialLocationCalculator"]["upperDepthThreshold"]
        self.calculationAlgorithm = dai.SpatialLocationCalculatorAlgorithm.MEDIAN
        left = self.configs["spatialLocationCalculator"]["roiTopLeft"]
        right = self.configs["spatialLocationCalculator"]["roiBottomRight"]
        config.roi = dai.Rect(dai.Point2f(left[0], left[1]), dai.Point2f(right[0], right[1]))
        self.spatialLocationCalculator.inputConfig.setWaitForMessage(False)
        self.spatialLocationCalculator.initialConfig.addROI(config)
        self.depthCam.setDepthAlign(dai.RawStereoDepthConfig.AlgorithmControl.DepthAlign(self.configs["depthCam"]["setDepthAlign"]))


    ## @brief Set a particular configuration
    # @param cam name of the camera to config (monoLeftCam / monoRightCam / depthCam / spatialLocationCalculator)
    # @param key The configuration name
    # @param value The value to be set
    def set(self, cam, key, value):
        '''Set a particular configuration

        Args:
            cam (any): name of the camera to config 
                - (monoLeftCam / monoRightCam / depthCam / spatialLocationCalculator)
            key (any): The configuration name
            value (any): The value to be set
        '''
        self.configs.setConfigParameter(cam, key, value)


    ## @brief Gets the next frame
    # @param[in] normalization if True, will normalize depth frame [0,255] for viewing. If False, depth frame is in meters
    # @param[in] scaleFactor scaling factor on outputted frame
    # @return depthFrame the depth frame
    def get_frames(self, normalization:bool=False, scaleFactor:int=1):
        '''Gets the next frame

        Args:
            normalization (optional) : 
                - if True -> will normalize the depth frame [0,255]
                - if False -> raw depth frame will be returned
            scaleFactor (optional) : scaling factor on outputted frame
        Returns:
            depth frame in units of [m] if not normalized
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
            normalizedDepthFrame = np.interp(depthFrame, (10, 1000), (0,255)).astype(np.uint8)
            return normalizedDepthFrame

        # convert to meters
        depthFrame = depthFrame/1000
        return depthFrame
    

    ## @brief Alias for get_frames
    def capture(self):
        '''Alias for get_frames
        '''
        return self.get_frames()
    

    ## @brief displays a frame
    # @param frame the frame to display
    # @param windowName the name of the window to display the frame on
    def display(self, frame, windowName='frame'):
        cv2.imshow(windowName, frame)


    ## @brief Prints out the configurations in a more human readable way
    def printConfigs(self):
        '''Prints out the configurations in a more human readable way
        '''
        print(self.configs)


    ## @brief Returns the intrinsic camera matrix and distortion coefficients
    # @return out the camera intrinsic matrix and distortion coefficients
    def getCameraIntrinsics(self):
        '''Returns the intrinsic camera matrix and distortion coefficients

        Returns:
            cameraMatrix:
            distortionCoeffs :
                (list (3,3)) intrinsice camera matrix distortionCoeffs 
                (list (,14)) distortion coefficients of the camera
        '''
        return self.cameraMatrix, self.distortionCoeffs


## @brief OAK class to capture images and depth (there are no color images for this camera, only black and white)
# The depth images are aligned to the left camera by default class initialization
class RGBD(Depth):
    '''OAK class to capture images and depth (there are no color images for this camera, only black and white)

    The depth images are aligned to the left camera by default class initialization
    '''

    ## @brief constructor
    # @param[in] configs ConfigOak object. Optional argument
    def __init__(self, configs=ConfigOak()):
        super().__init__(configs=configs)

        # create new xlinks for mono cam
        self.monoLeftXLink = self.pipeline.create(dai.node.XLinkOut)
        ## @var monoLeftXlink
        # link for left mono camera (dai.node.XlinkOut object)
        self.monoLeftXLink.setStreamName('monoLeftCam')
        self.monoRightXLink = self.pipeline.create(dai.node.XLinkOut)
        ## @var monoRightXlink
        # link for right mono camera (dai.node.XlinkOut object)
        self.monoRightXLink.setStreamName('monoRightCam')

        # link the mono cams to xlink
        self.monoLeftCam.out.link(self.monoLeftXLink.input)
        self.monoRightCam.out.link(self.monoRightXLink.input)

        # get and store the camera intrinsics
        with dai.Device(self.pipeline) as device:
            calibdata = device.readCalibration()
            leftMtx = calibdata.getCameraIntrinsics(dai.CameraBoardSocket.CAM_B, resizeWidth=640, resizeHeight=400)
            distCoeffs = calibdata.getDistortionCoefficients(dai.CameraBoardSocket.CAM_B)
        self.K = leftMtx
        ## @var K
        # camera intrinsic matrix
        self.cameraMatrix = leftMtx
        ## @var cameraMatrix
        # camera intrinsic matrix
        self.distortionCoeffs = distCoeffs
        ## @var distortionCoeffs
        # camera distortion coefficients


    ## @brief Start the capture stream. This must be called before get_frames() or capture()
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


    ## @brief Stop the capture stream and release the device from use
    def stop(self):
        '''Stop the capture stream and release the device from use
        '''
        self.device = None
        self.ready = False


    ## @brief Gets the next frame.
    # @param[in] normalization if True, will normalize depth frame [0,255] for viewing. If False, depth frame is in meters
    # @param[in] getBothMonoFrames if True, returns both left and right frames, False returns only left
    # @param[in] scaleFactor scaling factor on outputted frame
    # @return out the requested frames
    def get_frames(self, normalization:bool=False, getBothMonoFrames:bool=False, scaleFactor:int=1):
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

            depth frame in units of [m] if not normalized
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
        

    ## @brief Gets RGBD frames in ImageRGBD() class format
    # @param[in] normalization if True, will normalize depth frame [0,255] for viewing. If False, depth frame is in meters
    # @return images ImageRGBD object with .color and .depth images populated
    def capture(self, normalization:bool=True):
        '''Gets RGBD frames in ImageRGBD() class format.
        Args:
            normalization (bool):
                - if True, the depth frame will be normalized to 0-255 for better visulaization
                - if False, the depth frame will not be normalized and is returned in units of mm
        '''
        frames = self.get_frames(normalization=normalization)
        images = base.ImageRGBD()
        images.color = cv2.cvtColor(frames[0], cv2.COLOR_GRAY2RGB)
        images.depth = frames[1]
        return images
    

    ## @brief ill loop through indefinitely and send the obtained data to the passed function.
    # The raw data can be visualized if set, otherwise the processing function is 
    # responsible for handling output of raw, intermediat, or final data
    # @param[in] theProcessor RGBD stream data processor. Should handle input
    # @param[in] figOut if True, show raw data. if False, do NOT show raw data
    def process_loop(self, theProcessor, figOut:bool=False):
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


    ## @brief Replay and process data from the bag file attached to this instance.
    # Will loop through the bag file and send obtained data to the passed function.
    # The raw data can be visulaized if set, otherwise the processing function is responsible
    # for handling output of raw, intermediate, or final data.
    # @param[in] theProcessor RGBD stream data processor. Should handle input.
    # @param[in] figOut if True, show raw data. if False, do NOT show raw data
    def process_frames_selected(self, theProcessor, figOut:bool=True):
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


    ## @brief process a single frame
    # @param[in] theProcessor RGBD stream data processor. Should handle input.
    # @param[in] figOut if True, show raw data. if False, do NOT show raw data
    def process_frame(self, theProcessor, figOut=False):
        '''Processes a single frame
        Args:
            theProcessor (any) :
                - RGBD stream data processor. Should handle input.
            figOut (optional) :
                - true -> automatically show raw data
                - false -> do NOT automatically show raw data
        '''
        images = self.capture()

        theProcessor(images)

        if figOut:
            self.display(images.color, "color")
            self.display(images.depth, "depth")

        if figOut:
            display.close("color")
            display.close("depth")


    ## @brief Returns the intrinsic camera matrix and distortion coefficients
    # return out camera intrinsic matrix, camera distortion coefficients
    def getCameraIntrinsics(self):
        '''Returns the intrinsic camera matrix and distortion coefficients

        Returns:
            cameraMatrix:
            distortionCoeffs :
                (list (3,3)) intrinsice camera matrix distortionCoeffs 
                (list (,14)) distortion coefficients of the camera
        '''
        return self.cameraMatrix, self.distortionCoeffs
