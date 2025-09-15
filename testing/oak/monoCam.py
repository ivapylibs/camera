
import cv2
import depthai as dai

# make a pipeline to the device
pipeline = dai.Pipeline()

# setup pipeline with a monoCamera (we don't have color for this one)
cam_rgb = pipeline.create(dai.node.MonoCamera)
cam_right = pipeline.createMonoCamera()

# output from the device
xout_rgb = pipeline.create(dai.node.XLinkOut)
xright_out = pipeline.createXLinkOut()
# set the stream name for the output stream
xout_rgb.setStreamName('rgb')
xright_out.setStreamName('right')

# link the camera preview with the xlink input
cam_rgb.out.link(xout_rgb.input)
cam_right.out.link(xright_out.input)

# connect to the device and run the pipeline
with dai.Device(pipeline) as device:
    # print connected cameras
    print('Connected cameras: ', device.getConnectedCameras())

    # to consume results, we neeed an output que
    q_rgb = device.getOutputQueue('rgb')
    q_right = device.getOutputQueue('right')

    frame = None
    rframe = None

    # loop through continuously and process frames
    while True:
        # fetch data form teh queue
        in_rgb = q_rgb.tryGet()
        in_right = q_right.tryGet()

        # check to make sure a frame was available
        if in_rgb is not None:
            # retrieve the frame in OpenCV format
            frame = in_rgb.getCvFrame()

        # process frame
        if frame is not None:
            # show the frame on the screen
            cv2.imshow("preview", frame)

        if in_right is not None:
            rframe = in_right.getCvFrame()
        
        if rframe is not None:
            cv2.imshow("right", rframe)

        # exit condition
        if cv2.waitKey(1) == ord('q'):
            break