'''
@brief: class for aruco calibraiton and point transformations from workspace to camera

@author: Kyle de Nobel

@note: Comments above file are for Doxygen support and are the same as the docstrings.
'''

import cv2
import os
import yaml
import numpy as np
import camera.OAK as oak
from camera.base import Base
import time


## @brief Class for aruco calibration and point transformations from workspace to camera
# @param[in] markerLength length of the aruco marker in meters
# @param[in] cam camera used to gather frames
# @param[in] cameraMatrix the intrinsic camera matrix
# @param[in] distortionCoeffs the distortion coefficients of the camera
# @param[in] calibrationLoops the number of times a frame needs to be read from the camera before the image is stable
# @param[in] arucoDict the predefined aruco dictionary that the marker belongs to
class CameraToWorkspace:
    def __init__(
            self,
            markerLength:float,
            cam:Base = None,
            cameraMatrix = None,
            distortionCoeffs = None,
            calibrationLoops:int = 1000,
            arucoDict=cv2.aruco.DICT_6X6_100
    ):
        '''
        Args:
            markerLenght (any): length of the aruco marker in meters
            cam (optional): camera used to gather frames
                - if not provided, OAK-D SR camera from camera.OAK class will be used
            cameraMatrix (optional): the intrinsic camera matrix
                - if not provided, matrix will be from yaml file for OAK-D SR camera
            distortionCoeffs (optional): the distortion coefficients of the camera
                - if not provided, coefficients will be from yaml file for OAK-D SR camera
            calibrationLoops (optional): the number of times a frame needs to be read from the camera before the image is stable
            arucoDict (optional): the predefined aruco dictionary that the marker belongs to
        '''
        if cam is None:
            cam = oak.RGBD()
        
        if cameraMatrix is None or distortionCoeffs is None:
            intrinsicDict = {}
            # need to extract from yaml #
            # check if yaml exists yet
            try:
                dir_path = os.path.dirname(os.path.realpath(__file__))
                stream = open(dir_path+'/data/intrinsic_data.yaml', 'r')
                intrinsicDict = yaml.load(stream, Loader=yaml.FullLoader)
            # file does not exist or has not yet been created, so we must create it for future
            except FileNotFoundError:
                try:
                    mtx, distCoeffs = cam.getCameraIntrinsics()
                    intrinsicDict["mtx"] = mtx
                    intrinsicDict['distCoeffs'] = distCoeffs
                except NameError:
                    intrinsicDict['mtx'] = cam.K
                    intrinsicDict['distCoeffs'] = [0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        
        # load camera Matrix if needed
        if cameraMatrix is None:
            cameraMatrix = intrinsicDict['mtx']
            cameraMatrix = cv2.Mat(np.array(cameraMatrix))
        
        # load distortion coefficients if needed
        if distortionCoeffs is None:
            distortionCoeffs = intrinsicDict['distCoeffs']
            distortionCoeffs = cv2.Mat(np.array(distortionCoeffs))

        # store parameters
        self.cameraMatrix = cameraMatrix
        ## @var cameraMatrix 
        # the intrinsic camera matrix
        self.distortionCoeffs = distortionCoeffs
        ## @var distortionCoeffs 
        # the camera distortion coefficients
        self.arucoDict = arucoDict
        ## @var arucoDict
        # the dictionary the aruco tag belongs to
        self.markerLength = markerLength
        ## @var markerLength
        # the length of the aruco marker in meters
        self.calibrationLoops = calibrationLoops
        ## @var calibrationLoops 
        # the number of frames to get before calibrating
        self.numberOfLoops = 0
        ## @var numberOfLoops
        # the number of loops currently run
        self.gCW = None
        ## @var gCW
        # the camera to world transformation matrix
        self.cam = cam
        ## @var cam
        # the camera to be used for calibration

        # make coordinate in middle of marker
        self.objPoints = np.array([[-self.markerLength/2, self.markerLength/2, 0],
                                   [self.markerLength/2, self.markerLength/2, 0],
                                   [self.markerLength/2, -self.markerLength/2, 0],
                                   [-self.markerLength/2, -self.markerLength/2, 0]])
        ## @var objPoints
        # the coordinates of the middle of the marker


    ## @brief Process function to calibrate extrinsic matrix from rgb and depth frame
    # @param[in] img rgb image
    # @param[in] depth depth map
    # @return gCW
    def process(self, img:np.ndarray, depth:np.ndarray =None):
        '''Process function to calibrate extrinsic matrix from rgb and depth frame

        Args:
            img (np.ndarray (H, W, 3)): rgb image
            depth (np.ndarray (H, W), optional): Depth map. Not used
        Returns:
            gCW (np.ndarray (4,4)): Extrinsic aruco-to-camera transformation matrix
        '''
        # need to first stall process for some time for camera to clean image out
        if self.numberOfLoops < self.calibrationLoops:
            self.numberOfLoops = self.numberOfLoops + 1
        else:
            self.gCW,  = self.calibrate(img, depth)

        return self.gCW


    ## @brief Calculate and return the transformation gCW
    # @return gCW
    def calibrate(self):
        '''Calculate and return the transformation gCW
        Returns:
            gCW: transformation in homogeneous coordinates
        '''
        self.cam.start()
        # wait for 5 seconds for camera to correct itself
        start = time.time()
        while start+5 > time.time():
            self.cam.capture()
        images = self.cam.capture(normalization=False)
        # get the gCW transformation
        greyImg = cv2.cvtColor(images.color, cv2.COLOR_RGB2GRAY)
        self.gCW = self.getgCW(greyImg)
        self.cam.stop()
        return self.gCW


    ## @brief gets the transformation from workspace to camera
    # @param[in] bwImg black and white image containing an aruco tag
    # @return gCW
    def getgCW(self, bwImg):
        '''gets the transformation from workspace to camera
        Args:
            bwImg (np.array (H, W)): black and white image containing an aruco tag
        '''
        # detect the markers
        detectorParams = cv2.aruco.DetectorParameters()
        dictionary = cv2.aruco.getPredefinedDictionary(self.arucoDict)
        detector = cv2.aruco.ArucoDetector(dictionary, detectorParams)
        (markerCorners, markerIds, rejectedCandidates) = detector.detectMarkers(bwImg)
        
        # ensure that there was an aruco tag detected
        if markerCorners == 0:
            # no marker
            return None
    
        # get camera pose relative to the marker
        for c in markerCorners:
            (_, rvecs, tvecs) = cv2.solvePnP(self.objPoints, c, self.cameraMatrix, self.distortionCoeffs, False, flags=cv2.SOLVEPNP_IPPE_SQUARE)

        # convert to homogeneous coordinates
        roationMTX, _ = cv2.Rodrigues(rvecs)
        gCW = np.identity(4)
        gCW[:3, :3] = roationMTX
        gCW[:3, 3] = np.reshape(tvecs, (3,))
        return gCW
    

    ## @brief gets the intrinsic camera matrix
    # @return cameraMatrix
    def getCameraMatrix(self):
        '''Gets the intrinsic camera matrix
        Returns:
            cameraMatrix: the intrinsic camera matrix
        '''
        return self.cameraMatrix
    

    ## @brief gets the camera distortion coefficients
    # @return distCoeffs
    def getDistoritionCoeffs(self):
        '''Gets the camera distortion coefficients
        Returns:
            distCoeffs: the camera distortion coefficients
        '''
        return self.distortionCoeffs
        

