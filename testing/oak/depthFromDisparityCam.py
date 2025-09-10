# notes;
#   - ideal depth range is 30cm to 1m
#   - ^ this is what product description said, might not be fully true


import cv2
import numpy as np
import depthai as dai
import math

# make pipeline
pipeline = dai.Pipeline()

# create camera nodes in pipeline
monoLeft = pipeline.create(dai.node.MonoCamera)
monoRight = pipeline.create(dai.node.MonoCamera)
depth = pipeline.create(dai.node.StereoDepth)

# set each camera
# THE_800_P = resolution of 1280x800. This is resolution of camera.
# THE_480_P = resolution of 640x400
# larger resolutions lead to less accurate depths, so better to use lower resolution
monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setCamera("left")
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setCamera("right")

# setp depth presets based on suggestion on 
# https://docs.luxonis.com/software/depthai-components/nodes/stereo_depth/#StereoDepth-Depth%20Presets 
#depth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.ROBOTICS)
depth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
depth.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
depth.setLeftRightCheck(True)
depth.setExtendedDisparity(False)
depth.setSubpixel(True)
depth.setSubpixelFractionalBits(3)
# set confidence level
confidence_level = 100 # tunable parameter
depth.initialConfig.setConfidenceThreshold(confidence_level)

# output source
xout = pipeline.createXLinkOut()
xout.setStreamName("disparity")

xImageLeftOut = pipeline.createXLinkOut()
xImageLeftOut.setStreamName('imageOut')


# linking
monoLeft.out.link(depth.left)       # output left camera to depth left
monoRight.out.link(depth.right)     # output right camera to depth right
depth.disparity.link(xout.input)    # output disparity to xout

monoLeft.out.link(xImageLeftOut.input)


# calculation of depth from disparity map
# depth = focalLengthInPixels * baseline / disparityInPixels
# -> baseline = the distance between two mono cameras
# -> focalLengthInPixels = numberOfPixels/SensorSize * focalLength
baseline = 2 # cm     found here -> https://shop.luxonis.com/products/oak-d-sr?srsltid=AfmBOooNG-Joh5KCYeqKLSlV7UpRdWrjOuh3ZsvGMrHw5vdsbNGsfQLu
HFOV = 80 # degrees
imageWidthInPixels = 640 # 640 due to THE_400_P selection
focalLengthInPixels = imageWidthInPixels * 0.5 / math.tan(HFOV * 0.5 * math.pi/180)

# apply depth frame to frame values that are non-zero. Zero means that we don't know the value
# if value is 0, we say it is at max distance. Define Max distance = 500 cm
maxDistance = 500 # cm = 5 m

with dai.Device(pipeline) as device:

    queue = device.getOutputQueue(name="disparity", maxSize=4, blocking=False)

    imageQueue = device.getOutputQueue('imageOut')
    imageFrame = None
    

    while True:
        # get the disparity data
        inDisparity = queue.get()
        # get the disparity frame
        frame = inDisparity.getFrame()

        # get the image frame
        imageIn = imageQueue.tryGet()
        if imageIn is not None:
            imageFrame = imageIn.getCvFrame()

        if imageFrame is not None:
            cv2.imshow("left camera image", imageFrame)

        # normalization
        normalized_frame = (frame * (255 / depth.initialConfig.getMaxDisparity())).astype(np.uint8)

        # show disparity frame
        cv2.imshow("disparity", normalized_frame)

        # create frame for depth visualization
        depth_frame = np.empty_like(frame)

        # calculate the depth frame (if frame == 0, we assigne it maxDistance because that value is unknown)
        mask = (frame == 0)
        depth_frame[mask] = maxDistance
        depth_frame[~mask] = focalLengthInPixels * baseline / (frame[~mask] / 100)

        # normalize the depth frame
        normalize_depth_frame = (depth_frame * (255/maxDistance)).astype(np.uint8)

        # show black as far away
        normalize_depth_frame = 255 - normalize_depth_frame

        cv2.imshow("raw depth", normalize_depth_frame)

        if cv2.waitKey(1) == ord('q'):
            break