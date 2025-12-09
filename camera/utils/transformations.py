'''
@brief: useful transformations

@author: Kyle de Nobel

@note: Comments above file are for Doxygen support and are the same as the docstrings
'''


import numpy as np


## @brief calculates the pixel coordinates given the world coordinates
# @param[in] coordinates an ndarray (3,1) of the X,Y,Z coordinates of an object in the workspace frame
# @param[in] gCW an ndarray (4, 4) of the transformation from workspace to camera
# @param[in] camMTX an ndarray (3,3) of the camera's intrinsic parameters
# @return out an ndarray (2,1) of the pixel coordinates -> np.array([[u],[v]])
def workspaceToPixel(coordinates:np.ndarray, gCW = None, camMTX = None):
    '''calculates the pixel coordinates given the world coordinates.
    Args:
        coordinates (ndarray (3,1)) : the X,Y,Z coordinates of an object in the workspace frame
        gCW (ndarray (4,4)) : an ndarray (4, 4) of the transformation from workspace to camera
        camMTX (ndarray (3,3)) : an ndarray (3,3) of the camera's intrinsic parameters
    Returns:
        an ndarray (2,1) of the pixel coordinates -> np.array([[u],[v]])
    '''
    # setup coordinates as homogeneous coordinates
    coords = np.block([[coordinates], [1]])

    # perform calculation
    imageCoords = camMTX @ (gCW @ coords)[:3]
    u = imageCoords[0]/imageCoords[2]
    v = imageCoords[1]/imageCoords[2]

    return np.array([[u[0]],[v[0]]])


## @brief calculates the workspace coordinates given pixel coordinates and a depth map
# @param[in] coordinates an ndarray of the u,v coordinates of a pixel in an image
# @param[in] gWC an ndarray (4, 4) of the transformation from camera to workspace
# @param[in] camMTX an ndarray (3,3) of the camera's intrinsic parameters
# @param[in] depthMap a 2D ndarray of the depth map of your image. This must be the same size as your color image
# @return out an ndarray (3,1) of the X,Y,Z coordinates in the workspace frame
def pixelToWorkspace(coordinates:np.ndarray, depthMap:np.ndarray, gWC:np.ndarray, camMTX:np.ndarray):
    '''calculates the workspace coordinates given pixel coordinates and a depth map
    Args:
        coordinates (ndarray (2,1)) : the u,v coordinates of a pixel in your image
        depthMap (ndarray (N,M)) : the depth map of your N by M image
        gWC (ndarray (4,4)) : an ndarray (4, 4) of the transformation from camera to workspace
        camMTX (ndarray (3,3)) : an ndarray (3,3) of the camera's intrinsic parameters
    Returns:
        an ndarray (3,1) of the X,Y,Z coordinates in the workspace frame
    '''
    # grab z in camera frame
    zFromDepth = depthMap[coordinates[1][0], coordinates[0][0]]

    # setup coordinates as homogeneous coordinates
    coords = np.array([[coordinates[0][0]*zFromDepth], [coordinates[1][0]*zFromDepth], [zFromDepth]])

    # calculate the camera frame position
    camFrame = np.linalg.inv(camMTX) @ coords

    # calculate the workspace frame position
    workspacePosition = np.array(gWC) @ np.block([[camFrame], [1]])
    return np.reshape(workspacePosition[:3, 0], (3,1))