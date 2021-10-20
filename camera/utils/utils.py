"""

    @brief:             The other utility functions

    @author:            Yiye Chen,      yychen2019@gatech.edu
    @date:              10/07/2021

"""

import numpy as np
import cv2

def BEV_rectify_aruco(image, corners, target_pos="down", target_size = 100, margin=100, returnMode=2):
    '''The bird-eye-view rectification based on the aruco tag
    Project the image to the aruco frame by estimating an affine transformation matrix that maps the Aruco corners to the desired coordinates 

    @param[in]         image:   The image to be rectified. np.ndarray
    @param[in]       corners:   The coordinates of the four corners of the aruco tag, starting from top-left and goes clock-wise. (4, 2)
    @param[in]    target_pos:   The target position of the Aruco corners after rectification. The following options are provided:
                                \"down\"(Default): The bottom middle of the frame. The margin is 100. The Aruco size is 100; 
                                \"top\": The upper middle of the frame. The margin is 100. The Aruco size is 100; 
                                Coordinates (np.ndarray. (4, 2)): The customized desired location
    @param[in]    target_size:  The target size of the Aruco after the rectification. Unit is pixel
    @param[in]          margin: The margin of the tag from the image side
    @param[in]    returnMode:   1-return rectified image only; 
                                2-return composed origin-rectified image
    '''
    # numpy img -> opencv image. scale the color range to (0, 1)
    if isinstance(image, np.ndarray):
        src = image/255.0
    else:
        raise NotImplementedError("Only np.ndarray input is supported for the top-down rectification function")

    # first sort the corners. assumption is that the aruco is positioned at the top-middle of the field
    dist_square = np.sum(corners**2, axis=1, keepdims=False)
    idx_min = np.argmin(dist_square)
    corners = corners[[x%4 for x in [idx_min, idx_min+1, idx_min+2, idx_min+3]]]

    # create the perspective transformation matrix from the aruco corners. Put the point at the top-middle.     
    pDes_top = np.float32([[image.shape[1]/2-target_size/2, margin], [image.shape[1]/2+target_size/2, margin], 
                        [image.shape[1]/2+target_size/2, target_size + margin], [ image.shape[1]/2-target_size/2, target_size + margin]])
    pDes_down = np.float32([[image.shape[1]/2-target_size/2, image.shape[0] - margin - target_size], [image.shape[1]/2+target_size/2, image.shape[0] - margin - target_size], 
                        [image.shape[1]/2+target_size/2, image.shape[0] - margin], [ image.shape[1]/2-target_size/2, image.shape[0] - margin]])
    if target_pos == "down":
        pDes = pDes_down
    elif target_pos == "top":
        pDes = pDes_top
    elif isinstance(target_pos, np.ndarray):
        assert np.all(target_pos.shape == [4,2])
        pDes = target_pos
    
    # find the warping matrix
    warp_mat, gb = cv2.findHomography(corners, pDes)

    # calculate the transformed image
    image_rec = cv2.warpPerspective( image, warp_mat, (image.shape[1], image.shape[0]))
    
    # return
    if returnMode == 1:
        image_return = image_rec 
    elif returnMode == 2:
        image_return = cv2.hconcat([image, image_rec])
    
    return image_return, warp_mat