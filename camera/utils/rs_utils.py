"""

    @brief:     The utility functions for the realsense sensors

    @author:    Yiye Chen,          yychne2019@gatech.edu
    @date:      [created] 10/07/2021

"""

import numpy as np

def rs_intrin_to_M(rs_intrin):
    """
    Convert class <'pyrealsense2.intrinsics'> instance into a matrix, s.t.
    [i * d, j * d, d] = M * [x, y, d]  
    Intrinsic class structure is described [here](https://intelrealsense.github.io/librealsense/python_docs/_generated/pyrealsense2.intrinsics.html?highlight=intrinsic)
    as duplicated from the [C struct version](https://github.com/IntelRealSense/librealsense/blob/5e73f7bb906a3cbec8ae43e888f182cc56c18692/include/librealsense2/h/rs_types.h#L55)

    @param[out]     M       3-by-3 camera intrinsic matrix. Last row is [0, 0, 1]
    """

    M = np.eye(3)
    M[0, 0] = rs_intrin.fx
    M[0, 2] = rs_intrin.ppx
    M[1, 1] = rs_intrin.fy
    M[1, 2] = rs_intrin.ppy

    return M

def rs_extrin_to_M(rs_extrin):
    """
    Convert class <'pyrealsense2.extrinsics'> instance into a matrix, s.t.
    M = [R, d | 0, 1] 
    The extrinsics class stores rotation matrix as a linear array per
    [documentation](https://intelrealsense.github.io/librealsense/python_docs/_generated/pyrealsense2.extrinsics.html?highlight=extrin#pyrealsense2.extrinsics).

    @param[out]     M       4-by-4 SE(3) matrix. Last row is [0, 0, 0, 1]
    """

    M = np.eye(4)
    M[0:3,0:3] = np.reshape(rs_extrin.rotation, (3,3), 'F')
    M[0:3,3]   = rs_extrin.translation

    return M
