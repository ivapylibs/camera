''' Generate yaml file with transformation gCW
@author: Kyle de Nobel
'''

from camera.utils.arucoCalibration import CameraToWorkspace
import camera.OAK as oak
import cv2
import numpy as np
import os
import yaml



def calibrate(outFileDir, yamlFilePath="", cam=None, cameraMatrix=None, distortionCoefficients=None):
    '''Perform calibration of the camera to world frame

    Args:
        outFilePath (string): directory to write the output yaml file containing extrinsic transformations to
        yamlFilePath (string): file path of yaml containing aruco information
        cam (optional): any camera child class of camera.base
        cameraMatrix (optional): camera intrinsic matrix
        distortionCoefficients (optional): camera distortion coefficients

    Example:
        aruco yaml structure:
            * arucoTagSize:
            * - 0.07
            * arucoDictionary:
            * - 9
            * - Note -> cv2.aruco.DICT_6X6_100 = 9. You must convert the dict enum to an integer before storing here
            * arucoID:
            * - 20

    '''
    # read in aruco tag characteristics
    #dirPath = os.path.dirname(os.path.realpath(__file__))
    try:
        stream = open(yamlFilePath, 'r')
        arucoTag = yaml.load(stream, Loader=yaml.FullLoader)

        arucoTagSize = float(arucoTag['arucoTagSize'][0])
        arucoDictionary = int(arucoTag['arucoDictionary'][0])

        # setup camera to workspace
        c2w = CameraToWorkspace(cam=cam,\
                            cameraMatrix=cameraMatrix,\
                            distortionCoeffs=distortionCoefficients,\
                            markerLength=arucoTagSize,\
                            arucoDict=arucoDictionary)
    except FileNotFoundError:
        print("file not found, default tag is used")
        c2w = CameraToWorkspace(cam=cam,\
                            cameraMatrix=cameraMatrix,\
                            distortionCoeffs=distortionCoefficients)

    # get transformation
    gCW = c2w.calibrate()

    # calculate inverse transform
    gWC = np.block([
                [gCW[:3,:3].T, np.reshape(-gCW[:3, :3].T @ gCW[:3, 3], (3,1))],
                [0,0,0,1]
                ])

    # get the camera's intrinsic matrix
    camMTX = c2w.getCameraMatrix()

    # get the camera's distortion coefficients
    distCoeffs = c2w.getDistoritionCoeffs()

    # package the transformation and intrinsic matrix
    extrinsics = {}
    extrinsics['gCW'] = gCW.tolist()
    extrinsics['gWC'] = gWC.tolist()
    extrinsics['camMTX'] = camMTX.tolist()
    extrinsics['distortionCoeffs'] = distCoeffs.tolist()

    # store in yaml file in folder 'mycobotExtrinsics'
    try:
        os.remove(outFileDir+"/extrinsics.yaml")
    except FileNotFoundError:
        pass

    with open(outFileDir+"/extrinsics.yaml", 'x') as stream:
        yaml.dump(extrinsics, stream)

    print("NEED ARUCO TO BASE OF ROBOT STILL")



def getExtrinsics(filePath):
    '''Get the pre-calibrated 
                    gCW = world to camera transformation, 
                    gWC = camera to world transformation,
                    camMTX = camera intrinsic matrix, 
                    distortionCoeffs = camera distortion coefficients
    Args:
        filePath (string) : file path of yaml file contianing extrinsic transformations

    Returns:
        gCW, gWC, camMTX, distortionCoeffs
    '''
    currentDir = os.path.dirname(os.path.realpath(__file__))
    with open(filePath, 'r') as stream:
        extrinsics = yaml.load(stream, Loader=yaml.FullLoader)

    gCW = extrinsics['gCW']
    gWC = extrinsics['gWC']
    camMTX = extrinsics['camMTX']
    distortionCoeffs = extrinsics['distortionCoeffs']

    return gCW, gWC, camMTX, distortionCoeffs
