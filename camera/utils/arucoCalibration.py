'''
@brief: class for aruco calibraiton and point transformations from workspace to camera

@author: Kyle de Nobel
'''

import cv2
import os
import yaml
import numpy as np
import camera.OAK as oak
import time

class CameraToWorkspace:
    def __init__(
            self,
            markerLength,
            cam = None,
            cameraMatrix = None,
            distortionCoeffs = None,
            calibrationLoops = 1000,
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
                # import depthai as dai
                # # make pipeline
                # pipeline = dai.Pipeline()

                # # dictionary for output
                # intrinsicDict = dict()

                # # grab intrinsic data from device
                # with dai.Device(pipeline) as device:
                #     calibdata = device.readCalibration()
                #     mtx = calibdata.getCameraIntrinsics(dai.CameraBoardSocket.CAM_B)
                #     distCoeffs = calibdata.getDistortionCoefficients(dai.CameraBoardSocket.CAM_B)
                # intrinsicDict["mtx"] = mtx
                # intrinsicDict['distCoeffs'] = distCoeffs

                try:
                    mtx, distCoeffs = cam.getCameraIntrinsics()
                    intrinsicDict["mtx"] = mtx
                    intrinsicDict['distCoeffs'] = distCoeffs
                except NameError:
                    intrinsicDict['mtx'] = cam.K
                    intrinsicDict['distCoeffs'] = [0,0,0,0,0,0,0,0,0,0,0,0,0,0]

                # # place data in yaml file
                # try:
                #     os.mkdir(dir_path+"/data")
                # except FileExistsError:
                #     pass
                # stream = open(dir_path+'/data/intrinsic_data.yaml', 'x')
                # yaml.dump(intrinsicDict, stream)
        
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
        self.distortionCoeffs = distortionCoeffs
        self.arucoDict = arucoDict
        self.markerLength = markerLength
        self.calibrationLoops = calibrationLoops
        self.numberOfLoops = 0
        self.gCW = None
        self.cam = cam

        # make coordinate in middle of marker
        self.objPoints = np.array([[-self.markerLength/2, self.markerLength/2, 0],
                                   [self.markerLength/2, self.markerLength/2, 0],
                                   [self.markerLength/2, -self.markerLength/2, 0],
                                   [-self.markerLength/2, -self.markerLength/2, 0]])

    def process(self, img, depth=None):
        '''Process function to calibrate extrinsic matrix from rgb and depth frame

        Args:
            img (np.ndarray (H, W, 3)): rgb image
            depth (np.ndarray (H, W), optional): Depth map. Not used
        Returns:
            gCW (np.ndarray (4,4)): Extrinsic aruco-to-camera transformation matrix
            cornersAruco (np.ndarray): Detected aruco tag corners
            imgWithExt (np.ndarray (H, W, 3)): Color image with aruco tag corrdinates
        '''
        # need to first stall process for some time for camera to clean image out
        if self.numberOfLoops < self.calibrationLoops:
            self.numberOfLoops = self.numberOfLoops + 1
        else:
            self.gCW,  = self.calibrate(img, depth)

        return self.gCW



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
    
    def getCameraMatrix(self):
        return self.cameraMatrix
    
    def getDistoritionCoeffs(self):
        return self.distortionCoeffs
        

