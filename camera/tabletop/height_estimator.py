"""
    @brief: Estimate the height towards the perceived tabletop surface

    Assuming the visual field contains only one surface,
    Given the depth map and the camera intrinsic matrix,
    The height estimator derive the transformation to get the distance map of each pixel to the surface

    Build upon the tabletop plane estimator
        
    @Author: Yiye Chen                  yychen2019@gatech.edu

    @Date:      07/13/2021[created]
                10/25/2021[moved]

"""

import numpy as np
from numpy.core.fromnumeric import sort
from sklearn.decomposition import PCA

from camera.tabletop.tabletop_plane import tabletopPlaneEstimator

class HeightEstimator(tabletopPlaneEstimator):
    """ Estimate the height towards the perceived tabletop surface

    Assuming the visual field contains only one surface,
    Given the depth map and the camera intrinsic matrix,
    The height estimator derive the transformation to get the distance map of each pixel to the surface

    Args:
        intrinsic (np.ndarray. (3,3)): The camera intrinsic matrix.
    """
    def __init__(self, intrinsic):
        self.intrinsic = intrinsic

        # R: (1, 3). T:(1,). the naming is a little misleading. Consider changing them
        self.R = None
        self.T = None

    def calibrate(self, depth_map):
        R, T = self._measure_params(depth_map)
        self._update_params(R, T)
    
    def apply(self, depth_frame):
        """
        apply the calibrated transformation to a new frame
        """
        H, W = depth_frame.shape[:2]
        uv_map = self._get_uv(depth_frame)

        # (H, W, 3). where 3 is (uz, vz, z). Make use of the numpy multiplication broadcast mechanism
        uvz_map = np.concatenate(
            (uv_map, np.ones_like(uv_map[:, :, :1])),
            axis=2
        ) * depth_frame[:, :, None]

        # get the height
        height = self.R @ (uvz_map.reshape((-1, 3)).T) + self.T
        height = height.T.reshape(H, W)
        
        return height 

    def _measure_params(self, depth_map):
        """
        Measure the transformation parameters from a new training frame

        The plane parameters: (a, b, c, d)
        The intrinsic matrix: M_{int}

        Then the formula is:
        R = (a, b, c) M_{int}^{-1}
        T = -d
        """
        # (H, W, 3)
        plane_params = self.get_plane_params()
        R, T = self._get_RT(plane_params)
        return R, T

    def _update_params(self, R, T):
        """
        update the stored transformation

        right now just store the new R and T
        """
        self.R = R 
        self.T = T 

    def _get_RT(self, plane_param):
        """
        Get the transformation parameters that transforms a pixel with 1.pixel coordinate 2. depth information
        to the height w.r.t the plane(i.e. the norm direction of the plane) depicted by the plane_param.

        @param[in] plane_param
        """
        R = plane_param[:3].reshape([1,3]) @ np.linalg.inv(self.intrinsic)
        T = plane_param[-1]
        return R, T

        

