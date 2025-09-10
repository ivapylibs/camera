
import cv2
import depthai as dai

# make a pipeline to the device
pipeline = dai.Pipeline()

# setup pipeline with a monoCamera (we don't have color for this one)
cam_rgb = pipeline.createMonoCamera()

# set output window size
#cam_rgb.setPreviewSize(300, 300)

#cam_rgb.setInterleaved(False)


# output from the device
xout_rgb = pipeline.createXLinkOut()
# set the stream name for the output stream
xout_rgb.setStreamName('rgb')

# link the camera preview with the xlink input
cam_rgb.out.link(xout_rgb.input)

# connect to the device and run the pipeline
with dai.Device(pipeline) as device:
    # print connected cameras
    print('Connected cameras: ', device.getConnectedCameras())

    # to consume results, we neeed an output que
    q_rgb = device.getOutputQueue('rgb')
    frame = None

    # loop through continuously and process frames
    while True:
        # fetch data form teh queue
        in_rgb = q_rgb.tryGet()

        # check to make sure a frame was available
        if in_rgb is not None:
            # retrieve the fram in OpenCV format
            frame = in_rgb.getCvFrame()

        # process frame
        if frame is not None:
            # show the frame on the screen
            cv2.imshow("preview", frame)

        # exit condition
        if cv2.waitKey(1) == ord('q'):
            break