"""Script for testing behavior of the tabletop estimator.

In particular, there are dependencies on a visualization tool like mayavi or
pptk. This is to help debug the situation.

usage: test_tabletop_estimator.py [-h] [--force-matplotlib] [--random] [--dim DIM]
"""
from argparse import ArgumentParser
import numpy as np
from camera.tabletop.tabletop_plane import tabletopPlaneEstimator

if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("--force-matplotlib", action="store_true", help="Force matplotlib")
    parser.add_argument("--random", action="store_true", help="Use random data")
    parser.add_argument("--dim", type=int, default=100, help="The dimension of the depth map")
    args = parser.parse_args()

    dims = (args.dim, args.dim)
    if args.random:
        intrinsic = np.random.rand(3, 3)
        depth_map = np.random.random(dims)
        rgb = np.random.random((*dims, 3))
    else:
        intrinsic = np.array(
            [
                [100, 0, 10],
                [0, 100, 10],
                [0, 0, 1],
            ]
        )
        depth_map = np.ones(dims)
        rgb = np.ones((*dims, 3))

    estimator = tabletopPlaneEstimator(intrinsic=intrinsic)
    estimator.measure_plane(depth_map)
    estimator.vis_plane(rgb, force_matplotlib=args.force_matplotlib)
