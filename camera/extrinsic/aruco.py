"""
    @brief:         The aruco-based extrinsic matrix

    @author:        Yiye Chen,      yychen2019@gatech.edu
    @date:          10/07/2021

"""

import cv2
import cv2.aruco as aruco
import numpy as np
import copy

class CtoW_Calibrator_aruco():
    """
    A wrapper for the camera-to-workspace extrinsic matrix calibration (M_CL) with automatic 

    The camera intrinsic matrix and distortion coefficients should be obtained priorly
    Currently the wrapper only supports the workspace origin being marked by one single aruco marker.
    In case the calibration result is unstable, the wrapper updates the M_CL based on the result of each new frame 
        Currently the updating method is simply counting the occurance of the calibration result for a consecutive frames
        and then keeping the most frequent one. L2-distance is used to determine the equalness of two matrices

    The image and the camera coordinate systems are defined the same way as in the Realsense documentation:
    https://github.com/IntelRealSense/librealsense/blob/5e73f7bb906a3cbec8ae43e888f182cc56c18692/include/librealsense2/rsutil.h#L46


    @param[in]  cameraMatrix            - The camera intrinsic matrix
    @param[in]  distCoeffs              - The camera distortion matrix
    @param[in]  markerLength_CL         - The marker side length in meters
    @param[in]  stabilize_version       - stabilize_version version or not. When set to true, then only the first maxFrames number of frames will be used for calibration, 
                                          and the result will be fixed for the future frames.
    @param[in]  (optional)maxFrames     - The max frame number after which the M_CL updating ceases. If None, then will always update.
                                          (Default: 5000)

    """
    def __init__(self, cameraMatrix, markerLength_CL, \
                    distCoeffs=np.array([0.0, 0.0, 0.0, 0.0, 0.0]),
                    maxFrames=100, flag_vis_extrinsic=True, flag_print_MCL=True, 
                    stabilize_version = True):

        self.cameraMatrix = cameraMatrix
        self.distCoeffs = distCoeffs
        self.markerLength_CL = markerLength_CL

        self.maxFrames = maxFrames
        self.frame_counter = 0

        self.flag_vis_ext= flag_vis_extrinsic
        self.flag_print_MCL = flag_print_MCL

        # calibration result. 
        self.detected = False       # whether an Aruco has been detected
        self.M_CL = None
        self.corners_aruco = None 
        self.other_infos = {}  # for visualization
        self.img_with_ext = None    # use to store the image with the extrinsic frame drawn on it if self.flag_vis_ext

        # indicate whether the result is the stable version or not
        self.stabilize_version = stabilize_version
        self.stable_status = False

        # cache list. M_CL and occurance number. Should be corresponded. 
        self.cache_M_CL = []
        self.cache_occr = []
        self.cache_other_infos = []

    def process(self, rgb, depth=None):
        """
        Process function that calibrate the extrinsic matrix from the corresponding rgb and depth frame

        Args:
            img (np.ndarray, (H, W, 3)). The rgb image
            depth (np.ndarray, (H, W), optional). The depth map. Not used for now so set to optional. Defaults to None,
                it is a placeholder for the future improvement based on the depth
        Returns:
            M_CL [np.ndarray, (4, 4)]. The extrinsic aruco-to-camera transformation matrix
            corners_aruco [np.ndarray]. The detected aruco tag corners
            img_with_ext. [np.ndarray, (H, W, 3)]. The color image with the aruco tag coordinate. Useful for visualization. 
                In the stablize version, if no aruco is detected, then will draw the 
            status [binary]. True if an aruco is detected (non-stablize version) or a result has been stored (stablize version).
        """
        if not self.stabilize_version:
            """
            Not stabilize version, meaning calibrate with no memory
            """
            M_CL_cur, other_infos = self.calibrate(rgb, depth)
            # if detect a new aruco in the non-stablize version, then simply replace the new result
            if M_CL_cur is not None:
                self.M_CL = M_CL_cur
                self.corners_aruco = other_infos["corners_aruco"][0].squeeze()
                self.other_infos = other_infos
                self.detected = True
            else:
                self.detected = False
        elif self.frame_counter > self.maxFrames:
            """
            Stabilize version, but the calibration window has past
            """
            self.stable_status = True
        else:
            M_CL_cur, other_infos = self.calibrate(rgb, depth)
            if M_CL_cur is not None:
                self.frame_counter = self.frame_counter + 1
                self.detected = True
                self.update(M_CL_cur, other_infos)

        if (self.flag_print_MCL) and self.detected:
            print(self.M_CL)

        if self.flag_vis_ext:
            self._vis_ext(rgb)

        return self.M_CL, self.corners_aruco, self.img_with_ext, self.detected
    
    def calibrate(self, rgb, depth=None):
        '''
        Process a single frame
        '''
        gray = rgb.astype(np.uint8)
        M_CL, other_infos = self._get_M_CL(gray, rgb)

        return M_CL,other_infos
        
    def update(self, M_CL_new, other_infos):
        flag_exist = False
        for idx, M_CL in enumerate(self.cache_M_CL):
            if self._same_ext_mat(M_CL, M_CL_new):
                self.cache_occr[idx] += 1
                flag_exist = True
        if not flag_exist:
            self.cache_occr.append(1)
            self.cache_M_CL.append(M_CL_new)
            self.cache_other_infos.append(other_infos)

        # find the maximum occurance of the result
        occur_max = 0
        for idx, M_CL in enumerate(self.cache_M_CL):
            if self.cache_occr[idx] > occur_max:
                self.M_CL =  M_CL
                self.corners_aruco = self.cache_other_infos[idx]["corners_aruco"][0].squeeze()
                self.other_infos = self.cache_other_infos[idx]
                occur_max = self.cache_occr[idx]

    def update_intrinsic(self, cameraMatrix):
        """
        Now simply replace the original one
        """
        self.cameraMatrix = cameraMatrix

    # get transformation matrix from camera to the aruco tag
    def _get_M_CL(self, gray, rgb):
        '''
        Function: get the T matrix from camera to aruco tag
        :param gray:
        :param image_init:
        :param visualize:
        param[out]       M_CL:  The extrinsic matrix. (4, 4). If no aruco is detected then will return the stored matrix
        param[out] corners_CL:  The detected aruco corners in the image frame. (4, 2). If 
        '''
        # parameters
        #markerLength_CL = 0.076
        aruco_dict_CL = aruco.Dictionary_get(aruco.DICT_5X5_250)
        # aruco_dict_CL = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
        #aruco_dict_CL = aruco.Dictionary_get(aruco.DICT_6X6_250)
        parameters = aruco.DetectorParameters_create()

        corners_CL, ids_CL, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict_CL, parameters=parameters)

        # for the first frame, it may contain nothing
        if ids_CL is None:
            self.img_with_ext = copy.deepcopy(rgb)
            return None, None

        # see the doc for explanation http://amroamroamro.github.io/mexopencv/matlab/cv.estimatePoseSingleMarkers.html
        rvec_CL, tvec_CL, _objPoints_CL = aruco.estimatePoseSingleMarkers(corners_CL[0], self.markerLength_CL,
                                                                          self.cameraMatrix, self.distCoeffs)
        
        dst_CL, _ = cv2.Rodrigues(rvec_CL) 
        M_CL = np.zeros((4, 4))
        M_CL[:3, :3] = dst_CL
        M_CL[:3, 3] = tvec_CL
        M_CL[3, :] = np.array([0, 0, 0, 1])

        other_infos = {}
        other_infos["corners_aruco"] = corners_CL
        other_infos["ids_CL"] = ids_CL
        other_infos["rvec_CL"] = rvec_CL
        other_infos["tvec_CL"] = tvec_CL
        return M_CL, other_infos 
    
    def _vis_ext(self, rgb):
        bgr_copy = copy.deepcopy(rgb[:,:,::-1])
        if self.detected:
            cv2.aruco.drawDetectedMarkers(bgr_copy, self.other_infos["corners_aruco"], self.other_infos["ids_CL"])
            aruco.drawAxis(bgr_copy, self.cameraMatrix, self.distCoeffs,
                self.other_infos["rvec_CL"], self.other_infos["tvec_CL"], self.markerLength_CL)
        self.img_with_ext = bgr_copy[:,:,::-1] 
    
    def _same_ext_mat(self, M_CL1, M_CL2):
        #M_CL1_RT = M_CL1[:3, :]
        #M_CL2_RT = M_CL2[:3, :]
        #L2_dist = np.linalg.norm(M_CL1_RT - M_CL2_RT) 
        #print(L2_dist)
        #same = L2_dist > 1e-2
        same = np.allclose(M_CL1, M_CL2)
        return same 
