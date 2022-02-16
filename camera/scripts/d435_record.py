import cv2

import camera.d435.d435_runner as d435
import camera.utils.display as display
from camera.utils.writer import vidWriter 

# settings
vidname = "puzzle_play"        # save video name
W = 1920
H = 1080

# prepare
d435_configs = d435.D435_Configs(
    W_dep=848,
    H_dep=480,
    W_color=W,
    H_color=H,
    exposure = 100,
    gain = 50
)

d435_starter = d435.D435_Runner(d435_configs)

vid_writer = vidWriter(
    dirname = "./",
    vidname= vidname,  
    W = W,
    H = H,
    activate=True,
    save_depth=True 
)

# get started
while(True):
    rgb, dep, success = d435_starter.get_frames()
    #print("The camera gain: {}. The camera exposure: {}".format(d435_starter.get("gain"), d435_starter.get("exposure")))
    if not success:
        print("Cannot get the camera signals. Exiting...")
        exit()

    display.display_rgb_dep_cv(rgb, dep, ratio=0.5, window_name="THe camera signals. (color-scaled depth). Press \'q\' to exit")
    vid_writer.save_frame(rgb,  dep)

    opKey = cv2.waitKey(1)
    if opKey == ord('q'):
        break

    #display.display_rgb_dep_plt(rgb, dep, suptitle=None, fh=fh)
    #plt.draw()
    #plt.pause(1)

vid_writer.finish()
