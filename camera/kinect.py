'''
@brief          Kinect v1 Camera Interface

@Author         Xuanyou Chen              xchen3019@gatech.edu

@date           5/5/2026

@note: Comments above file are for Doxygen support and are the same as the docstrings
'''

import camera.base as base
import cv2
import numpy as np
import os
import yaml
import ivapy.display_cv as display

try:
    import freenect
except ImportError as e:
    raise ImportError(
        "Could not import freenect.\n\n"
        "Prerequisite: libfreenect must already be built/installed, and its shared "
        "libraries must be visible, e.g.:\n"
        "    export LD_LIBRARY_PATH=\"$HOME/.local/lib:$LD_LIBRARY_PATH\"\n\n"
        "Then install the Python wrapper from the cloned libfreenect repo:\n"
        "    pip install numpy \"Cython<3\"\n"
        "    cd ~/src/libfreenect/wrappers/python\n"
        "    python setup.py install\n\n"
        "The path ~/src/libfreenect assumes you cloned the repo there.\n\n"
        f"Original import error: {e}"
    ) from e

## @brief Pads distortion coefficients to a fixed length.
# @param[in] coeffs distortion coefficient list or array
# @param[in] length target number of coefficients
# @return out distortion coefficients as ndarray with length entries
def _pad_distortion_coeffs(coeffs, length=14):
    '''Pads distortion coefficients to a fixed length.

    Args:
        coeffs:
            distortion coefficient list or array
        length:
            target number of coefficients

    Returns:
        coeffs:
            distortion coefficients as ndarray with length entries
    '''
    coeffs = list(coeffs)
    if len(coeffs) < length:
        coeffs = coeffs + [0.0] * (length - len(coeffs))
    return np.array(coeffs[:length], dtype=np.float64)


## @brief Safely reads a matrix/list from the YAML config.
# @param[in] configs config dictionary
# @param[in] key config key to read
# @param[in] default default matrix/list used if key is not present
# @return out matrix as ndarray
def _read_matrix_from_config(configs, key, default):
    '''Safely reads a matrix/list from the YAML config.

    Args:
        configs:
            config dictionary
        key:
            config key to read
        default:
            default matrix/list used if key is not present

    Returns:
        matrix:
            matrix as ndarray
    '''
    value = configs.get(key, default)
    return np.array(value, dtype=np.float64)


## @brief Normalizes a depth image to uint8 [0, 255] for visualization.
# @param[in] depth depth image in meters
# @return out normalized depth image as uint8
def _normalize_depth_for_display(depth):
    '''Normalizes a depth image to uint8 [0, 255] for visualization.

    Uses robust percentiles instead of raw min/max to reduce flickering.

    Args:
        depth:
            depth image in meters

    Returns:
        depth_vis:
            normalized depth image as uint8
    '''
    depth = np.asarray(depth)

    valid = depth[np.isfinite(depth) & (depth > 0)]
    if valid.size == 0:
        return np.zeros(depth.shape, dtype=np.uint8)

    lo, hi = np.percentile(valid, [1, 99])
    if hi <= lo:
        hi = lo + 1e-6

    depth_vis = np.clip((depth - lo) / (hi - lo), 0.0, 1.0)
    return (depth_vis * 255).astype(np.uint8)


## @brief Converts raw Kinect v1 depth values to approximate meters.
# @param[in] depth_raw raw Kinect depth frame from freenect.sync_get_depth()
# @return out depth frame in approximate meters
def raw_kinect_depth_to_meters(depth_raw):
    '''Converts raw Kinect v1 depth values to approximate meters.

    libfreenect's default depth frame is a raw 11-bit-ish Kinect depth value,
    not meters. This formula is a practical approximation from:
    https://graphics.stanford.edu/~mdfisher/Kinect.html.

    Args:
        depth_raw:
            raw Kinect depth frame from freenect.sync_get_depth()

    Returns:
        depth_m:
            depth frame in approximate meters
    '''
    depth_raw = np.asarray(depth_raw, dtype=np.float32)

    with np.errstate(divide="ignore", invalid="ignore"):
        depth_m = 1.0 / (depth_raw * -0.0030711016 + 3.3309495161)

    depth_m[~np.isfinite(depth_m)] = 0.0
    depth_m[depth_raw <= 0] = 0.0

    return depth_m.astype(np.float32)


## @brief config general parameters of Kinect v1 camera
class ConfigKinect(base.CfgCamera):
    '''config general parameters of Kinect v1 camera
    '''

    ## @brief the constructor. If yamlFilePath is not given, default initialization is done
    # @param[in] yamlFilePath the file path of the yaml to read camera initialization parameters from
    def __init__(self, yamlFilePath: str = None):
        '''config general parameters of Kinect v1 camera

        Args:
            yamlFilePath (str):
                path of yaml configuration file
        '''
        super().__init__(new_allowed=True)

        if yamlFilePath is None:
            currentPath = os.path.dirname(os.path.realpath(__file__))
            yamlFilePath = currentPath+"/utils/cameraInitializationFiles/realsense305.yaml"

        with open(yamlFilePath) as stream:
            init = yaml.load(stream, yaml.SafeLoader)

        super().__init__(init)

    ## @brief allows setting all configs at once
    # @param[in] filePath path of yaml configuration file
    def setConfig(self, filePath: str):
        '''allows setting all configs at once

        Args:
            filePath (str):
                path of yaml configuration file
        '''
        self.clear()
        self.merge_from_file(filePath)

    ## @brief returns config dictionary
    # @return config the config dictionary
    def getConfig(self):
        '''returns config dictionary

        Returns:
            config:
                the config dictionary
        '''
        return self.config

    ## @brief allows the setting of a specific parameter of a camera to a certain value
    # @param[in] cam name of the camera/config section to change the value of
    # @param[in] param the name of the camera parameter
    # @param[in] value value to set parameter to
    # @note if parameters are unclear, print out config
    def setConfigParameter(self, cam, param, value):
        '''allows the setting of a specific parameter of a camera to a certain value

        Args:
            cam:
                name of camera/config section
            param:
                name of camera parameter
            value:
                value to set parameter to
        '''
        self[cam][param] = value


## @brief Shared helper logic for Kinect color, depth, and RGBD classes.
class _KinectCommon:
    '''Shared helper logic for Kinect color, depth, and RGBD classes.

    This class is intentionally internal. The public API stays as Color, Depth,
    and RGBD.
    '''

    ## @brief Initializes RGB and depth intrinsics from the YAML config.
    def _init_intrinsics_from_config(self):
        '''Initializes RGB and depth intrinsics from the YAML config.

        Kinect v1 does not expose RealSense-style intrinsics through the Python
        API in the same convenient way. We therefore read them from YAML.

        The default_color_K matrix is a common approximate Kinect v1 RGB matrix (found here: https://cvg.cit.tum.de/data/datasets/rgbd-dataset/intrinsic_calibration)
        The default_depth_K matrix is a placeholder and should be replaced with
        calibration results for accurate metric 3D work.
        '''
        default_color_K = [
            [525.0, 0.0, 319.5],
            [0.0, 525.0, 239.5],
            [0.0, 0.0, 1.0],
        ]

        default_depth_K = [
            [580.0, 0.0, 319.5],
            [0.0, 580.0, 239.5],
            [0.0, 0.0, 1.0],
        ]

        default_dist = [0.0] * 14

        self.cameraMatrix = _read_matrix_from_config(
            self.configs,
            "cameraMatrix",
            default_color_K,
        )
        ## @var cameraMatrix
        # camera intrinsic matrix

        self.distortionCoeffs = _pad_distortion_coeffs(
            self.configs.get("distortionCoeffs", default_dist)
        )
        ## @var distortionCoeffs
        # distortion coefficients for the camera

        self.K = self.cameraMatrix
        ## @var K
        # camera intrinsic matrix

        self.depthCameraMatrix = _read_matrix_from_config(
            self.configs,
            "depthCameraMatrix",
            default_depth_K,
        )
        ## @var depthCameraMatrix
        # depth camera intrinsic matrix

        self.depthDistortionCoeffs = _pad_distortion_coeffs(
            self.configs.get("depthDistortionCoeffs", default_dist)
        )
        ## @var depthDistortionCoeffs
        # depth camera distortion coefficients

    ## @brief returns whether color frames should be converted from RGB to BGR
    # @return out True if color frames should be returned as BGR
    def _return_bgr(self):
        '''Returns whether color frames should be converted from RGB to BGR.

        Returns:
            returnBGR:
                True if color frames should be returned as BGR
        '''
        return bool(self.configs.get("returnBGR", True))

    ## @brief gets one color frame from the Kinect
    # @return colorFrame color image frame
    def _get_color_frame(self):
        '''Gets one color frame from the Kinect.

        Returns:
            colorFrame:
                color image frame
        '''
        colorFrame, _ = freenect.sync_get_video()

        if colorFrame is None:
            return None

        colorFrame = np.asarray(colorFrame)

        if self._return_bgr():
            colorFrame = cv2.cvtColor(colorFrame, cv2.COLOR_RGB2BGR)

        return colorFrame

    ## @brief gets one raw depth frame from the Kinect
    # @return depthFrame raw depth image frame
    def _get_depth_raw_frame(self):
        '''Gets one raw depth frame from the Kinect.

        Returns:
            depthFrame:
                raw depth image frame
        '''
        depthFrame, _ = freenect.sync_get_depth()

        if depthFrame is None:
            return None

        return np.asarray(depthFrame)

    ## @brief converts raw Kinect depth to meters or normalized uint8 display image
    # @param[in] depthRaw raw Kinect depth frame
    # @param[in] normalization if True, normalize depth frame to [0,255] for visualization
    # @return out converted depth frame
    def _convert_depth(self, depthRaw, normalization=False):
        '''Converts raw Kinect depth to meters or normalized uint8 display image.

        Args:
            depthRaw:
                raw Kinect depth frame
            normalization:
                if True, normalize depth frame to [0,255] for visualization

        Returns:
            depthFrame:
                converted depth frame
        '''
        if depthRaw is None:
            return None

        depth_m = raw_kinect_depth_to_meters(depthRaw)

        if normalization:
            return _normalize_depth_for_display(depth_m)

        return depth_m

    ## @brief stops the libfreenect sync thread if the binding exposes sync_stop()
    def _stop_freenect_sync(self):
        '''Stops the libfreenect sync thread if the binding exposes sync_stop().
        '''
        try:
            freenect.sync_stop()
        except Exception:
            pass


## @brief Kinect class to capture only color images
class Color(base.Color, _KinectCommon):
    '''Kinect class to capture only color images
    '''

    ## @brief the constructor
    # @param[in] yamlInitFilePath path of yaml file to use for camera initialization
    def __init__(self, yamlInitFilePath: str = None):
        '''Kinect class to capture only color images

        Args:
            yamlInitFilePath (str):
                path of yaml file to use for camera initialization
        '''
        configs = ConfigKinect(yamlInitFilePath)
        super().__init__(configs=configs)

        self._init_intrinsics_from_config()

        self.ready = False
        ## @var ready
        # indicates if the camera is ready to stream images

    ## @brief Start the capture stream. This must be called before get_frames() or capture()
    def start(self):
        '''Start the capture stream. This must be called before get_frames() or capture()
        '''
        self.ready = True
        return self.ready

    ## @brief Stop the capture stream and release the device from use
    def stop(self):
        '''Stop the capture stream and release the device from use
        '''
        self._stop_freenect_sync()
        self.ready = False

    ## @brief returns the configs of the camera
    # @return configs the config dictionary of the camera
    def get_configs(self):
        '''Returns all of the configs for the camera.

        Returns:
            configs:
                the config dictionary of the camera
        '''
        return super().get_configs()

    ## @brief sets all of the configs of the camera
    # @param[in] yamlFilePath path of yaml configuration file
    def set_configs(self, yamlFilePath: str):
        '''Re-sets all of the configs for the camera.

        Args:
            yamlFilePath (str):
                path of yaml configuration file
        '''
        wasReady = self.ready
        if wasReady:
            self.stop()

        super().set_configs(ConfigKinect(yamlFilePath))
        self._init_intrinsics_from_config()

        if wasReady:
            self.start()

    ## @brief gets the next frame
    # @return frame color frame. By default this is BGR for OpenCV compatibility.
    def get_frames(self):
        '''Gets the next frame.

        Returns:
            frame:
                color frame. By default this is BGR for OpenCV compatibility.
        '''
        if not self.ready:
            return None, False

        return self._get_color_frame()

    ## @brief Alias for get_frames
    def capture(self):
        '''Alias for get_frames.
        '''
        return self.get_frames()

    ## @brief displays a frame
    # @param[in] frame the frame to display
    # @param[in] windowName the name of the window to display the frame on
    def display(self, frame: np.ndarray, windowName: str = 'frame'):
        '''Displays an image.

        Args:
            frame (ndarray):
                an image frame
            windowName (str):
                the name of the window
        '''
        cv2.imshow(windowName, frame)

    ## @brief Prints out the configurations in a more human readable way
    def printConfigs(self):
        '''Prints out the configurations in a more human readable way.
        '''
        print(self.configs)

    ## @brief Returns the intrinsic camera matrix and distortion coefficients
    # @return out the camera intrinsic matrix and distortion coefficients
    def getCameraIntrinsics(self):
        '''Returns the intrinsic camera matrix and distortion coefficients.

        Returns:
            cameraMatrix:
                intrinsic camera matrix
            distortionCoeffs:
                distortion coefficients of the camera
        '''
        return self.cameraMatrix, self.distortionCoeffs

    ## @brief returns the width and height of the color image
    # @return out image width, image height
    def getImagePixelWidthAndHeight(self) -> tuple:
        '''Returns the width and height of the color image.

        Returns:
            (imageWidth, imageHeight) (int, int)
        '''
        return (
            self.configs["colorCamera"]["imageWidth"],
            self.configs["colorCamera"]["imageHeight"],
        )


## @brief Kinect class to capture depth frames
class Depth(base.Base, _KinectCommon):
    '''Kinect class to capture depth frames.

    The depth images are not automatically aligned to the color camera by
    default class initialization.
    '''

    ## @brief constructor
    # @param[in] yamlInitFilePath path of yaml file to use for camera initialization
    def __init__(self, yamlInitFilePath: str = None):
        '''Kinect class to capture depth frames.

        Args:
            yamlInitFilePath (str):
                path of yaml file to use for camera initialization
        '''
        configs = ConfigKinect(yamlInitFilePath)
        super().__init__(configs=configs)

        self._init_intrinsics_from_config()

        self.depthScale = None
        ## @var depthScale
        # scale factor placeholder kept for API familiarity. Kinect raw depth uses
        # a conversion formula instead of a RealSense-style hardware depth scale.

        self.ready = False
        ## @var ready
        # indicates if the camera is ready to stream images

    ## @brief Start the capture stream. This must be called before get_frames() or capture()
    def start(self):
        '''Start the capture stream. This must be called before get_frames() or capture()
        '''
        self.depthScale = 1.0
        self.ready = True
        return self.ready

    ## @brief Stop the capture stream and release the device from use
    def stop(self):
        '''Stop the capture stream and release the device from use.
        '''
        self._stop_freenect_sync()
        self.depthScale = None
        self.ready = False

    ## @brief returns the configs of the camera
    # @return configs the config dictionary of the camera
    def get_configs(self):
        '''Returns all of the configs for the camera.

        Returns:
            configs:
                the config dictionary of the camera
        '''
        return super().get_configs()

    ## @brief sets all of the configs of the camera
    # @param[in] yamlFilePath path of yaml configuration file
    def set_configs(self, yamlFilePath: str):
        '''Re-sets all of the configs for the camera.

        Args:
            yamlFilePath (str):
                path of yaml configuration file
        '''
        wasReady = self.ready
        if wasReady:
            self.stop()

        super().set_configs(ConfigKinect(yamlFilePath))
        self._init_intrinsics_from_config()

        if wasReady:
            self.start()

    ## @brief gets the next frame
    # @param[in] normalization if True, normalize depth frame [0,255]. If False, depth frame is in meters
    # @return frame depth frame
    def get_frames(self, normalization: bool = False):
        '''Gets the next frame.

        Args:
            normalization (bool):
                - if True, depth frame is normalized to 0-255 for visualization
                - if False, depth frame is returned in approximate meters

        Returns:
            depthFrame:
                depth frame
        '''
        if not self.ready:
            return None

        depthRaw = self._get_depth_raw_frame()
        return self._convert_depth(depthRaw, normalization=normalization)

    ## @brief Alias for get_frames
    def capture(self):
        '''Alias for get_frames.
        '''
        return self.get_frames()

    ## @brief displays a frame
    # @param[in] frame the frame to display
    # @param[in] windowName the name of the window to display the frame on
    def display(self, frame: np.ndarray, windowName: str = 'frame'):
        '''Displays an image.

        Args:
            frame (ndarray):
                an image frame
            windowName (str):
                the name of the window
        '''
        cv2.imshow(windowName, frame)

    ## @brief Prints out the configurations in a more human readable way
    def printConfigs(self):
        '''Prints out the configurations in a more human readable way.
        '''
        print(self.configs)

    ## @brief Returns the intrinsic camera matrix and distortion coefficients
    # @return out the camera intrinsic matrix and distortion coefficients
    def getCameraIntrinsics(self):
        '''Returns the intrinsic camera matrix and distortion coefficients.

        Returns:
            cameraMatrix:
                intrinsic camera matrix
            distortionCoeffs:
                distortion coefficients of the camera
        '''
        return self.cameraMatrix, self.distortionCoeffs

    ## @brief Returns the depth camera intrinsic matrix and distortion coefficients
    # @return out the depth camera intrinsic matrix and depth distortion coefficients
    def getDepthCameraIntrinsics(self):
        '''Returns the depth camera intrinsic matrix and distortion coefficients.

        Returns:
            depthCameraMatrix:
                depth camera intrinsic matrix
            depthDistortionCoeffs:
                depth distortion coefficients
        '''
        return self.depthCameraMatrix, self.depthDistortionCoeffs

    ## @brief returns the width and height of the color image
    # @return out image width, image height
    def getImagePixelWidthAndHeight(self) -> tuple:
        '''Returns the width and height of the color image.

        Returns:
            (imageWidth, imageHeight) (int, int)
        '''
        return (
            self.configs["colorCamera"]["imageWidth"],
            self.configs["colorCamera"]["imageHeight"],
        )


## @brief Kinect class to capture color images and depth
class RGBD(Depth):
    '''Kinect class to capture color images and depth.

    The output format is preserved from the RealSense wrapper:
        get_frames() -> colorFrame, depthFrame
        capture() -> base.ImageRGBD with .color and .depth

    Unlike the RealSense wrapper, the depth images are not automatically aligned
    to the color camera.
    '''

    ## @brief constructor
    # @param[in] yamlInitFilePath path of yaml file to use for camera initialization
    def __init__(self, yamlInitFilePath: str = None):
        '''Kinect class to capture color images and depth.

        Args:
            yamlInitFilePath (str):
                path of yaml file to use for camera initialization
        '''
        super().__init__(yamlInitFilePath)

    ## @brief Gets the next frames
    # @param[in] normalization if True, normalize depth frame [0,255]. If False, depth frame is in meters
    # @return out colorFrame, depthFrame
    def get_frames(self, normalization: bool = False):
        '''Gets the next frame.

        Args:
            normalization (bool):
                - if True, the depth frame will be normalized to 0-255 for visualization
                - if False, the depth frame will be returned in approximate meters

        Returns:
            (colorFrame, depthFrame):
                - colorFrame = color frame of camera
                - depthFrame = depth frame of camera
        '''
        if not self.ready:
            return (None, None)

        colorFrame = self._get_color_frame()
        depthRaw = self._get_depth_raw_frame()
        depthFrame = self._convert_depth(depthRaw, normalization=normalization)

        return colorFrame, depthFrame

    ## @brief Gets RGBD frames in ImageRGBD() class format
    # @param[in] normalization if True, normalize depth frame [0,255]. If False, depth frame is in meters
    # @return images ImageRGBD object with .color and .depth images populated
    def capture(self, normalization: bool = True):
        '''Gets RGBD frames in ImageRGBD() class format.

        Args:
            normalization (bool):
                - if True, the depth frame will be normalized to 0-255 for visualization
                - if False, the depth frame will be returned in approximate meters

        Returns:
            images:
                ImageRGBD object with .color and .depth images populated
        '''
        colorImage, depthImage = self.get_frames(normalization=normalization)
        images = base.ImageRGBD()
        images.color = colorImage
        images.depth = depthImage
        return images

    ## @brief Will loop through indefinitely and send the obtained data to the passed function.
    # The raw data can be visualized if set, otherwise the processing function is
    # responsible for handling output of raw, intermediate, or final data.
    # @param[in] theProcessor RGBD stream data processor. Should handle input
    # @param[in] figOut if True, show raw data. if False, do NOT show raw data
    def process_loop(self, theProcessor, figOut: bool = False):
        '''Will loop through indefinitely and send the obtained data to the passed function.

        The raw data can be visualized if set, otherwise the processing function
        is responsible for handling output of raw, intermediate, or final data.

        Args:
            theProcessor:
                RGBD stream data processor. Should handle input
            figOut (bool):
                true -> automatically show raw data
                false -> do NOT automatically show raw data
        '''
        self.start()

        while True:
            images = self.capture()

            theProcessor(images)

            if figOut:
                self.display(images.color, "color")
                self.display(images.depth, "depth")

            opKey = display.wait(1)
            if opKey == ord('q'):
                break

        self.stop()

        if figOut:
            display.close("color")
            display.close("depth")

    ## @brief Process selected live frames.
    # Will loop through the live Kinect stream and send selected frames to the passed function.
    # The raw data can be visualized if set, otherwise the processing function is responsible
    # for handling output of raw, intermediate, or final data.
    # @param[in] theProcessor RGBD stream data processor. Should handle input.
    # @param[in] figOut if True, show raw data. if False, do NOT show raw data
    def process_frames_selected(self, theProcessor, figOut: bool = True):
        '''Process selected live frames.

        Will loop through the live Kinect stream and send selected frames to the
        passed function. Press 's' to run theProcessor(images), and 'q' to quit.

        Args:
            theProcessor:
                RGBD stream data processor. Should handle input.
            figOut (bool):
                true -> automatically show raw data
                false -> do NOT automatically show raw data
        '''
        print("Live Kinect stream. Hit 'q' to quit. Hit 's' to select.")
        print("Selection will trigger additional processing. It might involve")
        print("additional user input from keyboard or mouse.")

        self.start()

        while True:
            images = self.capture()

            if figOut:
                self.display(images.color, "color")
                self.display(images.depth, "depth")

            opKey = display.wait(1)
            if opKey == ord('q'):
                break
            elif opKey == ord('s'):
                theProcessor(images)

        self.stop()

    ## @brief process a single frame
    # @param[in] theProcessor RGBD stream data processor. Should handle input.
    # @param[in] figOut if True, show raw data. if False, do NOT show raw data
    def process_frame(self, theProcessor, figOut=False):
        '''Processes a single frame.

        Args:
            theProcessor:
                RGBD stream data processor. Should handle input.
            figOut (bool):
                true -> automatically show raw data
                false -> do NOT automatically show raw data
        '''
        images = self.capture()

        theProcessor(images)

        if figOut:
            self.display(images.color, "color")
            self.display(images.depth, "depth")

        if figOut:
            display.close("color")
            display.close("depth")