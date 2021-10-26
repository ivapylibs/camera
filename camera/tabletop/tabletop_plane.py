"""
======================================= tabletop_plane =========================================

        @brief: The estimator of the tabletop plane in the camera frame.
        
        @Author:    Yiye Chen                  yychen2019@gatech.edu
        @Date:      07/13/2021[created]
                    10/25/2021[moved]

======================================= tabletop_plane =========================================
"""

import numpy as np
from numpy.core.fromnumeric import sort
from sklearn.decomposition import PCA

class tabletopPlaneEstimator():
    """ The estimator of the tabletop plane in the camera frame.

    A flat tabletop forms a 3D plane in the camera frame that can be described by the linear equation:
        ax + by + cz + d = 0
    The estimator recovers the parameters (a, b, c, d) in the linear equation from the depth frame

    The calibrator requires:
    1. The camera intrinsic matrix to map the image pixel coordinates to the camera coordinates
    2. Depth frames

    Args:
        intrinsic (np.ndarray. (3,3)): The camera intrinsic matrix.
    """
    def __init__(self, intrinsic):
        self.intrinsic = intrinsic

        # use PCA to fit the 3d plane
        self.pca = PCA(n_components=3)

        # the result
        self.plane_params = None        #<- (4, ) Params [a, b, c, d] of the plane equation
    
    def measure_plane(self, depth_map):
        """Get the plane parameters from the depth map.
        Requires the tabletop to occupy the majority content.

        Args:
            depth_map (np.ndarray, (H, W)): The height map

        Returns:
            error_map (np.ndarray, (H, W)): The distance between each pixel and the estimated plane. 
                Expected to be low for the tabletop pixel
        """
        p_cam_map = self._recover_p_Cam(depth_map)
        plane_params = self._estimate_plane(p_cam_map)
        self._update_plane(plane_params)

        error_map = self._produce_error_map(p_cam_map)
        return error_map
    
    def get_plane_params(self):
        """Get the estimated plane parameters

        Returns:
            plane_params [np.ndarray, (4,)]. The estimated plane parameters. Will be None if no plane has been estimated
        """
        return self.plane_params

    def _get_uv(self, img, vec=False):
        """
        Get the pixel coordinates of an input image.

        The origin (0, 0) is the upperleft corner, with right as u and down as vA
        
        @param[in]  img     The input image of the shape (H, W)
        @param[out] vec     Vectorize the outputs? Default is False

        @param[out] uv_map  The (u, v) coordinate of the image pixels. (H, W, 2), where 2 is (u, v)
        """
        H, W = img.shape[:2]
        rows, cols = np.indices((H, W))
        U = cols
        V = rows
        uv_map = np.concatenate(
            (U[:, :, None], V[:, :, None]),
            axis=2
        )

        # TODO: Vectorize the output instead as a map?
        if vec:
            pass

        return uv_map

    def _recover_p_Cam(self, depth_map):
        """
        Recover the 3d camera coordinate with the input depth map and the stored camera intrinsic matrix

        Assuming the 2d frame coordinate used by the intrinsic matrix set the upper left as (0, 0), right is x, down is y.
        This is true for the realsense cameras according to the github issue below:

        https://github.com/IntelRealSense/librealsense/issues/8221#issuecomment-765335642

        @param[out] pC_map       (H, W, 3), where 3 is (xc, yc, zc)
        """
        H, W = depth_map.shape[:2]
        uv_map = self._get_uv(depth_map)

        # (H, W, 3). where 3 is (uz, vz, z). Make use of the numpy multiplication broadcast mechanism
        uvz_map = np.concatenate(
            (uv_map, np.ones_like(uv_map[:, :, :1])),
            axis=2
        ) * depth_map[:, :, None]

        # recover the camera coordinates
        p_Cam_map = np.linalg.inv(self.intrinsic) @ \
            uvz_map.reshape(-1, 3).T
        p_Cam_map = p_Cam_map.T.reshape(H, W, 3)

        return p_Cam_map

    def _estimate_plane(self, p_Cam_map):
        """
        Estimate a 3D plane from a map of 3d points
        
        TODO: RANSAC or PCA? PCA first. Remove extrame values

        @param[in]  p_Cam_map       (H, W, 3), where 3 is (x_c, y_c, z_c)

        @param[out] plane_param     (4,). (a, b, c, d) that depicts a 3d plane: ax+by+cz+d = 0
        """

        p_Cam_vec = p_Cam_map.reshape((-1, 3))

        # remove extreme value depth, retain only the middle part. Lets be bold and only take the middle 50%. There are too many pixels anyway
        # NOTE: this is different from ivapylibs.improcessor.clipTails. It replace the extreme values with the upper and lower threshold, 
        # where as this one removes them.
        z_c_vec = p_Cam_vec[:, 2]
        N = z_c_vec.size
        sorted_z_c_vec = np.sort(z_c_vec)
        th_low = sorted_z_c_vec[int(N*0.25)]
        th_high = sorted_z_c_vec[int(N*0.75)]
        mask = (z_c_vec >= th_low) & (z_c_vec <= th_high)
        p_Cam_vec_mid = p_Cam_vec[mask, :]

        # fit the plane
        self.pca.fit(p_Cam_vec_mid)
        print("\n Got the fitted plane from the PCA. The three variance ratios are:{} \n".format(self.pca.explained_variance_ratio_))
        normal = self.pca.components_[-1, :] #(a, b, c)
        # use the mean to calculate d
        mean = np.mean(p_Cam_vec_mid, axis=0) 
        d = -normal.reshape((1, -1)) @ mean.reshape((-1, 1))

        plane_param = np.zeros((4,))
        plane_param[:3] = normal
        plane_param[3] = d
        return plane_param
    
    def _update_plane(self, plane_params_new):
        """Update the plane parameters based on the freshly estimated ones

        Args:
            plane_params (np.array, (4, )): The newly estimated plane parameters
        """
        #TODO: for now simply replace the stored params
        self.plane_params = plane_params_new 

    def _produce_error_map(self, p_cam_map):
        """Produce the error map, defined as the distance of each pixel to the estimated plane in the camera frame.
        The tabletop plane pixels are expected to be close to zero.

        Args:
            p_cam_map (np.ndarray, (H, W, 3)): The camera frame coordinate map, where 3 is (x_c, y_c, z_c)

        Returns:
            error_map (np.ndarray, (H, W)): The plane estimation error per pixel
        """
        H, W, _ = p_cam_map.shape
        p_cam_map_homogeneous = np.concatenate(
            (p_cam_map, np.ones(shape=(H, W, 1))),
            axis=2
        )
        error_map = (p_cam_map_homogeneous @ self.plane_params) / np.linalg.norm(self.plane_params[:3])
        return error_map

