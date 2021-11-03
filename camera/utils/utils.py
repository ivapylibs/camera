"""

    @brief:             The other utility functions

    @author:            Yiye Chen,      yychen2019@gatech.edu
    @date:              10/07/2021

"""

import numpy as np
import cv2

def BEV_rectify_aruco(image, corners, target_pos="down", target_size = 100, margin=100, mode="full"):
    '''The bird-eye-view rectification based on the aruco tag
    Project the image to the aruco frame by estimating an affine transformation matrix that maps the Aruco corners to the desired coordinates .

    @param[in]         image:   The image to be rectified. np.ndarray
    @param[in]       corners:   The coordinates of the four corners of the aruco tag, starting from top-left and goes clock-wise. (4, 2)
    @param[in]    target_pos:   The target position of the Aruco corners after rectification. The following options are provided:
                                \"down\"(Default): The bottom middle of the frame. The margin is 100. The Aruco size is 100; 
                                \"top\": The upper middle of the frame. The margin is 100. The Aruco size is 100; 
                                Coordinates (np.ndarray. (4, 2)): The customized desired location
    @param[in]    target_size:  The target size of the Aruco after the rectification. Unit is pixel
    @param[in]          margin: The margin of the tag from the image side
    @param[in]          mode:   "same". Rectify the aruco to the desired location, then crop the result image to have the same size as the original image.
                                "full". The rectified image will be translated and crop with different size to contain the full visual field. 
                                        Method follows the following link: https://stackoverflow.com/questions/13063201/how-to-show-the-whole-image-when-using-opencv-warpperspective

    @param[out]     image_rec:  The rectified image.
    @param[out]     warp_mat:   The warping matrix
    '''
    # input size
    H, W = image.shape[:2]

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
    if mode == "same":
        image_rec = cv2.warpPerspective( image, warp_mat, (W, H))
    elif mode == "full":
        # determine the minimum and maximum x and y value
        img_corners = np.float32([[0,0],[0,H],[W,H],[W,0]]).reshape(-1,1,2)
        img_corners_ = cv2.perspectiveTransform(img_corners, warp_mat)
        [xmin, ymin] = np.int32(img_corners_.min(axis=0).ravel() - 0.5)
        [xmax, ymax] = np.int32(img_corners_.max(axis=0).ravel() + 0.5)

        # Define the translation matrix to get all pixel coordinates to be positive
        warp_mat_translate = np.array([[1,0,-xmin],[0,1,-ymin],[0,0,1]])
        warp_mat = warp_mat_translate @ warp_mat

        # determine the cropping size
        xsize = xmax - xmin
        ysize = ymax - ymin

        # rectify
        image_rec = cv2.warpPerspective(image, warp_mat, (xsize ,ysize))
    else: raise NotImplementedError
    
    # return
    return image_rec, warp_mat