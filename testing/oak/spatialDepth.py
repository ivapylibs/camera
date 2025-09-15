import cv2
import numpy as np
import depthai as dai

# create pipeline
pipeline = dai.Pipeline()

# create nodes
monoLeft = pipeline.create(dai.node.MonoCamera)
monoRight = pipeline.create(dai.node.MonoCamera)
stereo = pipeline.create(dai.node.StereoDepth)
spatialLocationCalculator = pipeline.create(dai.node.SpatialLocationCalculator)

# set properties
# Properties
monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setCamera("left")
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setCamera("right")

stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
stereo.setLeftRightCheck(True)
stereo.setSubpixel(True)

# Config
topLeft = dai.Point2f(0.4, 0.4)
bottomRight = dai.Point2f(0.6, 0.6)
config = dai.SpatialLocationCalculatorConfigData()
config.depthThresholds.lowerThreshold = 100
config.depthThresholds.upperThreshold = 1000
calculationAlgorithm = dai.SpatialLocationCalculatorAlgorithm.MEDIAN
config.roi = dai.Rect(topLeft, bottomRight)
spatialLocationCalculator.inputConfig.setWaitForMessage(False)
spatialLocationCalculator.initialConfig.addROI(config)

# output sources
xoutDepth = pipeline.create(dai.node.XLinkOut)
xoutDepth.setStreamName("depth")
xoutSpatialData = pipeline.create(dai.node.XLinkOut)
xoutSpatialData.setStreamName('spatialData')
xinSpatialCalcConfig = pipeline.create(dai.node.XLinkIn)
xinSpatialCalcConfig.setStreamName("spatialCalcConfig")

# linking
monoLeft.out.link(stereo.left)       # output left camera to depth left
monoRight.out.link(stereo.right)     # output right camera to depth right

spatialLocationCalculator.passthroughDepth.link(xoutDepth.input)
stereo.depth.link(spatialLocationCalculator.inputDepth)
spatialLocationCalculator.out.link(xoutSpatialData.input)
xinSpatialCalcConfig.out.link(spatialLocationCalculator.inputConfig)

with dai.Device(pipeline) as device:
    # Output queue will be used to get the depth frames from the outputs defined above
    depthQueue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
    spatialCalcQueue = device.getOutputQueue(name="spatialData", maxSize=4, blocking=False)
    spatialCalcConfigInQueue = device.getInputQueue("spatialCalcConfig")

    color = (255, 255, 255)

    while True:
        inDepth = depthQueue.get()
        depthFrame = inDepth.getFrame() # in millimeters

        # normalizeDepth = (depthFrame * (255/1000)).astype(np.uint8)

        depthFrameColor = np.interp(depthFrame, (0, 1000), (0, 255)).astype(np.uint8)

        bigDepthFrameColor = np.repeat(depthFrameColor, 3, axis=1)
        bigDepthFrameColor = np.repeat(bigDepthFrameColor, 3, axis=0)

        cv2.imshow("depth", 255-depthFrameColor)

        cv2.imshow("expanded depth", 255-bigDepthFrameColor)

        # spatialData = spatialCalcQueue.get().getSpatialLocations()
        # for depthData in spatialData:
        #     roi = depthData.config.roi
        #     roi = roi.denormalize(width=depthFrameColor.shape[1], height=depthFrameColor.shape[0])
        #     xmin = int(roi.topLeft().x)
        #     ymin = int(roi.topLeft().y)
        #     xmax = int(roi.bottomRight().x)
        #     ymax = int(roi.bottomRight().y)

        #     depthMin = depthData.depthMin
        #     depthMax = depthData.depthMax

        #     fontType = cv2.FONT_HERSHEY_TRIPLEX

        #     cv2.rectangle(depthFrameColor, (xmin, ymin), (xmax, ymax), color, 1)
        #     cv2.putText(depthFrameColor, f"X: {int(depthData.spatialCoordinates.x)} mm", (xmin + 10, ymin + 20), fontType, 0.5, color)
        #     cv2.putText(depthFrameColor, f"Y: {int(depthData.spatialCoordinates.y)} mm", (xmin + 10, ymin + 35), fontType, 0.5, color)
        #     cv2.putText(depthFrameColor, f"Z: {int(depthData.spatialCoordinates.z)} mm", (xmin + 10, ymin + 50), fontType, 0.5, color)

        # Show the frame
        # cv2.imshow("depth", depthFrameColor)

        key = cv2.waitKey(1)
        if key == ord('q'):
            break