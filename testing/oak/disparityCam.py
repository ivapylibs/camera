import cv2
import numpy as np
import depthai as dai

# create pipeline
pipeline = dai.Pipeline()

# create left and right monoCamera nodes in pipeline
monoLeft = pipeline.createMonoCamera()
monoRight = pipeline.createMonoCamera()

# set each camera
# THE_800_P = resolution of 1280x800. This is resolution of camera.
# THE_480_P = resolution of 640x400
# larger resolutions lead to less accurate depths, so better to use lower resolution
monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setCamera("left")
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setCamera("right")

# create stere depth node in pipeline
depth = pipeline.create(dai.node.StereoDepth)

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
confidence_level = 150 # tunable parameter
depth.initialConfig.setConfidenceThreshold(confidence_level)

# output source
xout = pipeline.createXLinkOut()
xout.setStreamName("disparity")

# linking
monoLeft.out.link(depth.left)       # output left camera to depth left
monoRight.out.link(depth.right)     # output right camera to depth right
depth.disparity.link(xout.input)    # output disparity to xout

with dai.Device(pipeline) as device:

    queue = device.getOutputQueue(name="disparity", maxSize=4, blocking=False)

    while True:
        # get the disparity data
        inDisparity = queue.get()
        # get the disparity frame
        frame = inDisparity.getFrame()
        # print(frame)
        # print("type of frame is : ", type(frame))
        # print("frame size = ", frame.shape)
        # print("data type = ", frame.dtype)

        # normalization
        normalized_frame = (frame * (255 / depth.initialConfig.getMaxDisparity())).astype(np.uint8)

        # show disparity frame
        cv2.imshow("disparity", normalized_frame)

        if cv2.waitKey(1) == ord('q'):
            break

