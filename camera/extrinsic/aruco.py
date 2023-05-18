"""
    @brief:         The aruco-based extrinsic matrix

    @author:        Yiye Chen,      yychen2019@gatech.edu
    @date:          10/07/2021

"""

import cv2
import cv2.aruco as aruco
import numpy as np
import copy

class CtoW_Calibrator_aruco:
    """!
    @brief  Wrapper for aruco tag camera-to-workspace extrinsic matrix calibration (M_CL) 

    The camera intrinsic matrix and distortion coefficients should generally be
    obtained from the camera_info topic.  Currently the wrapper only supports
    the workspace origin being marked by one single aruco marker. In case the
    calibration result is unstable, the wrapper updates the M_CL based on the
    result of each new frame. Currently the updating method simply counts
    calibration results for consecutive frames and keeps the most frequent one.
    L2-distance is used to determine the similarity of two matrices.

    The image and the camera coordinate systems are defined the same way as in
    the [Realsense documentation](https://github.com/IntelRealSense/librealsense/blob/5e73f7bb906a3cbec8ae43e888f182cc56c18692/include/librealsense2/rsutil.h#L46).
    """

    #=================================== __init___ ===================================
    #
    def __init__(
        self,
        cameraMatrix,
        markerLength_CL,
        distCoeffs=np.array([0.0, 0.0, 0.0, 0.0, 0.0]),
        maxFrames=100,
        flag_vis_extrinsic=True,
        flag_print_MCL=True,
        stabilize_version = True,
        aruco_dict=aruco.DICT_5X5_250,
    ):
        """!
        @brief  Constructor of the CtoW_Calibrator_aruco class.

        @param[in]  cameraMatrix                Camera intrinsic matrix
        @param[in]  markerLength_CL             Marker side length in meters
        @param[in]  (optional)distCoeffs        Camera distortion matrix (Default: np.array([0.0, 0.0, 0.0, 0.0, 0.0]))
        @param[in]  (optional)maxFrames         Max frame number after which M_CL updating ceases. 
                                                If None, then will always update. (Default: 5000)
        @param[in]  (optional)flag_vis_ext      Visualize extrinsic frame on image. (Default: True)
        @param[in]  (optional)flag_print_MCL    Print out M_CL. (Default: True)
        @param[in]  (optional)stabilize_version Use "stabilized" version. (Default: True)
                                                True: only first maxFrames used for calibration. 
        @param[in]  (optional)aruco_dict        Aruco dictionary. (Default: aruco.DICT_5X5_250)
        """

        # Store operational parameters.
        self.cameraMatrix    = cameraMatrix
        self.distCoeffs      = distCoeffs
        self.markerLength_CL = markerLength_CL
        self.aruco_dict      = aruco_dict

        self.maxFrames      = maxFrames

        self.flag_vis_ext   = flag_vis_extrinsic
        self.flag_print_MCL = flag_print_MCL

        self.stabilize_version = stabilize_version  

        # Calibration process variables.
        self.detected       = False     # Aruco tag detected
        self.corners_aruco  = None

        self.frame_counter  = 0
        self.stable_status  = False

        self.other_infos    = {}        # for visualization
        self.img_with_ext   = None      # image with extrinsic frame drawn, if being output

        # Cache list. M_CL and occurance number. Implicitly in correspondence by index.
        self.cache_M_CL = []
        self.cache_occr = []
        self.cache_other_infos = []

        # Calibration result.
        self.M_CL = None

    #==================================== process ====================================
    #
    def process(self, rgb, depth=None):
        """!
        @brief  Process function to calibrate extrinsic matrix from rgb and depth frames.

        @param[in] img      (np.ndarray, (H, W, 3)) Rgb image
        @param[in] depth    (np.ndarray, (H, W), optional). Depth map. Not used for now. 
                            It is a placeholder for future improvement based on depth.
                            Defaults to None

        @param[out] M_CL            [np.ndarray, (4, 4)] Extrinsic aruco-to-camera transformation 
                                    matrix.
        @param[out] corners_aruco   [np.ndarray]. Detected aruco tag corners.
        @param[out] img_with_ext    [np.ndarray, (H, W, 3)] Color image with aruco tag coordinate. 
                                    Useful for visualization.
                                    With stablize_version, if no aruco detected, will draw status.
                                    [binary]. True if aruco detected (non-stablize version) or 
                                              a result has been stored (stabilize version).
        """
        if not self.stabilize_version:
            # Not stabilized version, meaning calibrate with no memory.
            # When aruco detected, recovered transformation replaces existing result.
            M_CL_cur, other_infos = self.calibrate(rgb, depth)
            if M_CL_cur is not None:
                self.detected   = True
                self.M_CL       = M_CL_cur

                self.corners_aruco = other_infos["corners_aruco"][0].squeeze()
                self.other_infos   = other_infos
            else:
                self.detected = False

        elif self.frame_counter > self.maxFrames:
            # Running stabilize version and calibration time window has past.
            self.stable_status = True
        else:
            # Running stabilize version and still within calibration time window.
            M_CL_cur, other_infos = self.calibrate(rgb, depth)
            if M_CL_cur is not None:
                self.frame_counter = self.frame_counter + 1
                self.detected      = True
                self.update(M_CL_cur, other_infos)

        if (self.flag_print_MCL) and self.detected:
            print(self.M_CL)

        if self.flag_vis_ext:
            self._vis_ext(rgb)

        return self.M_CL, self.corners_aruco, self.img_with_ext, self.detected

    #=================================== calibrate ===================================
    #
    def calibrate(self, rgb, depth=None):
        '''!
        @brief  Process a single frame and recover transformation.

        @param[in]  rgb     Color image.
        @param[in]  depth   Depth image [optional]. Not used in this class instance.

        @param[out] M_CL    The camera to workspace matrix.
        '''
        gray = rgb.astype(np.uint8)
        M_CL, other_infos = self._get_M_CL(gray, rgb)

        return M_CL,other_infos

    #===================================== update ====================================
    #
    def update(self, M_CL_new, other_infos):
        '''!
        @brief  If using a running calibration, provide latest measurement and update estimate.

        @param[in]  M_CL_new    Latest measurement.
        @param[in]  other_inofs Additional information.
        '''

        # Compare to stored estimates and update count if similar to existing one.
        flag_exist = False
        for idx, M_CL in enumerate(self.cache_M_CL):
            if self._same_ext_mat(M_CL, M_CL_new):
                self.cache_occr[idx] += 1
                flag_exist = True

        # If not similar, then add to stored estimates.
        if not flag_exist:
            self.cache_occr.append(1)
            self.cache_M_CL.append(M_CL_new)
            self.cache_other_infos.append(other_infos)

        # Find mode of estimated outcomes. 
        occur_max = 0
        for idx, M_CL in enumerate(self.cache_M_CL):
            if self.cache_occr[idx] > occur_max:
                self.M_CL =  M_CL
                self.corners_aruco = self.cache_other_infos[idx]["corners_aruco"][0].squeeze()
                self.other_infos = self.cache_other_infos[idx]
                occur_max = self.cache_occr[idx]

    #================================ update_intrinsic ===============================
    #
    def update_intrinsic(self, cameraMatrix):
        """
        @brief  Simply replace the original one
        """
        self.cameraMatrix = cameraMatrix

    #=================================== _get_M_CL ===================================
    #
    def _get_M_CL(self, gray, rgb):
        '''
        @brief   Get transformation matrix from camera to the aruco tag

        @param[in]  gray    Grayscale image (for ARUCO detection)
        @param[in]  rgb     Original color image
        
        @param[out] M_CL        Extrinsic matrix. If no aruco detected, returns stored matrix.
        @param[out] corners_CL  Detected aruco corners in image frame. [4, 2]
        '''

        aruco_dict_CL = aruco.Dictionary_get(self.aruco_dict)
        parameters    = aruco.DetectorParameters_create()
        corners_CL, ids_CL, rejectedImgPoints \
                      = aruco.detectMarkers(gray, aruco_dict_CL, parameters=parameters)

        # First frame may contain nothing.
        if ids_CL is None:
            self.img_with_ext = copy.deepcopy(rgb)
            return None, None

        # For [explanation](http://amroamroamro.github.io/mexopencv/matlab/cv.estimatePoseSingleMarkers.html)
        rvec_CL, tvec_CL, _objPoints_CL \
                    = aruco.estimatePoseSingleMarkers(corners_CL[0], self.markerLength_CL,
                                                  self.cameraMatrix, self.distCoeffs     )

        dst_CL, _ = cv2.Rodrigues(rvec_CL)      # Convert twist to matrix.
        M_CL = np.identity(4)                   # Package into SE(3) homogeneous matrix.
        M_CL[:3, :3] = dst_CL
        M_CL[:3, 3]  = tvec_CL

        other_infos = {}
        other_infos["corners_aruco"] = corners_CL
        other_infos["ids_CL"]  = ids_CL
        other_infos["rvec_CL"] = rvec_CL
        other_infos["tvec_CL"] = tvec_CL

        return M_CL, other_infos

    #==================================== _vis_ext ===================================
    #
    def _vis_ext(self, rgb):
        '''!
        @brief  Create visualization of extrinsic camera estimate.

        @param[in]  rgb     Measured color image.
        '''

        bgr_copy = copy.deepcopy(rgb[:,:,::-1])
        if self.detected:
            cv2.aruco.drawDetectedMarkers(bgr_copy, self.other_infos["corners_aruco"], 
                                                    self.other_infos["ids_CL"]       )
            aruco.drawAxis(bgr_copy, self.cameraMatrix, self.distCoeffs,
                                     self.other_infos["rvec_CL"], self.other_infos["tvec_CL"], 
                                                                  self.markerLength_CL       )

        self.img_with_ext = bgr_copy[:,:,::-1]

    #================================= _same_ext_mat =================================
    #
    def _same_ext_mat(self, M_CL1, M_CL2):
        '''!
        @brief  Determine if two matrices are close to each other.
        '''
        # Old code used matrix norm of difference.  Now using allclose instead.
        same = np.allclose(M_CL1, M_CL2)
        return same

