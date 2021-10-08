"""

    @brief:     The utility functions for the realsense sensors

    @author:    Yiye Chen,          yychne2019@gatech.edu
    @date:      [created] 10/07/2021

"""

import numpy as np

def rs_intrin_to_M(rs_intrin):
    """
    convert the class <'pyrealsense2.intrinsics'> into a matrix, s.t.
    [i * d, j * d, d] = M * [x, y, d]  
    The class description is not available due to the crappy document of the pyrealsense2,
    but it is similary to the C struct version described in the link below:
    https://github.com/IntelRealSense/librealsense/blob/5e73f7bb906a3cbec8ae43e888f182cc56c18692/include/librealsense2/h/rs_types.h#L55
    @param[out]     M       3-by-3 camera intrinsic matrix. Last row is [0, 0, 1]
    """

    M = np.zeros((3, 3))
    M[0, 0] = rs_intrin.fx
    M[0, 2] = rs_intrin.ppx
    M[1, 1] = rs_intrin.fy
    M[1, 2] = rs_intrin.ppy
    M[2, 2] = 1.

    return M