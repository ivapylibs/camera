# ======================================= display =========================================
"""
        @brief: The display-related utility functions
        
        @Author: Yiye Chen                  yychen2019@gatech.edu
        @Date: 10/07/2021[created, Surveillance repository]
               10/19/2021[moved to the camera repository]

"""
# ======================================= display =========================================

from typing import Callable, List
from dataclasses import dataclass
import matplotlib.pyplot as plt
import numpy as np
import cv2

import improcessor.basic as improcessor

@dataclass
class plotfig:
  num   : any
  axes  : any

def display_rgb_dep_plt(rgb, depth, suptitle=None, figsize=(10,5), fh=None):
    """Display the rgb and depth image in a same figure at two different axes
    The depth will be displayed with a colorbar beside.

    NOTE: This function use the matplotlib for visualization, which is SLOW.
    SO it is not suitable for real-time visualization (e.g. Visualize the camera feed)

    Args:
        rgb (np.ndarray, (H, W, 3)): The rgb image
        depth (np.ndarray, (H, W)): The depth map
        suptitle (str, optional): The suptitle for display. Defaults to None, which will display no suptitle
        figsize (tuple, optional): The figure size following the matplotlib style. Defaults to (10,5).
        fh (matplotlib.Figure): The figure handle. Defaults to None, which means a new figure handle will be created.
    """

    if fh is None:
        fh = plt.figure(figsize=figsize)
    fh.clf()

    if suptitle is not None:
        fh.suptitle(suptitle)
    
    ax0 = fh.add_subplot(121)
    ax1 = fh.add_subplot(122)

    # rgb
    ax0.imshow(rgb)
    ax0.set_title("The color frame")
    # depth
    dep_show = ax1.imshow(depth)
    ax1.set_title("The depth frame")
    # depth colorbar. Give it its own axis. See: https://stackoverflow.com/questions/18195758/set-matplotlib-colorbar-size-to-match-graph
    cax = fh.add_axes([ax1.get_position().x1+0.01,ax1.get_position().y0,0.02,ax1.get_position().height])
    plt.colorbar(dep_show, cax=cax) 


#
#-------------------------------------------------------------------------
#======================== OpenCV Display Interface =======================
#-------------------------------------------------------------------------
#

#===================== OpenCV Single Image Interfaces ====================
#-------------------------------------------------------------------------


#=============================== wait_cv ===============================

def wait_cv(dur=0):
  opKey = cv2.waitKey(dur)
  return opKey

#=============================== depth_cv ==============================
#
def depth_cv(depth, depth_clip=0.08, ratio=None, window_name="OpenCV Display"):

    '''!
    @brief  Display depth image using OpenCV window.

    The depth frame will be scaled to have the range 0-255.
    There will be no color bar for the depth

    Args:
        depth (np.ndarray, (H, W)): The depth map
        depth_clip (float in [0, 1]): The depth value clipping percentage. The top and bottom extreme depth value to remove. Default to 0.0 (no clipping)
        ratio (float, Optional): Allow resizing the images before display.  Defaults to None, which means will perform no resizing
        window_name (sting, Optional): The window name for display. Defaults to \"OpenCV display\"
    '''
    # Apply depth clipping if requested.
    # Convert depth to cv2 image format: scale to 255, convert to 3-channel
    if depth_clip > 0:
        assert depth_clip < 0.5
        improc = improcessor.basic(improcessor.basic.clipTails,(depth_clip,))
        depth_show = improc.apply(depth)
        depth_color = cv2.applyColorMap(cv2.convertScaleAbs(depth_show, alpha=255./depth_show.max(), beta=0), cv2.COLORMAP_JET)
    else:
        depth_color = cv2.applyColorMap(cv2.convertScaleAbs(depth, alpha=255./depth.max(), beta=0), cv2.COLORMAP_JET)

    # Rescale if requested.
    if ratio is not None: 
        H, W = depth.shape[:2]
        H_vis = int(ratio * H)
        W_vis = int(ratio * W)
        depth_color = cv2.resize(depth_color, (W_vis, H_vis))

    # Display colorized depth image.
    cv2.imshow(window_name, depth_color[:,:,::-1])

#================================ rgb_cv ===============================
#
def rgb_cv(rgb, ratio=None, window_name="Image"):
    '''!
    @brief  Display rgb image using the OpenCV

    The rgb frame will be resized to a visualization size prior to visualization.
    Args:
        rgb (np.ndarray, (H, W, 3)): The rgb image
        ratio (float, Optional): Allow resizing the images before display.  Defaults to None, which means will perform no resizing
        window_name (sting, Optional): The window name for display. Defaults to \"OpenCV display\"
    '''
    if ratio is not None:                                   # Resize if requested.
        H, W = rgb.shape[:2]
        H_vis = int(ratio * H)
        W_vis = int(ratio * W)
        rgb_vis = cv2.resize(rgb, (W_vis, H_vis))
        cv2.imshow(window_name, rgb_vis[:,:,::-1])
    else:
        cv2.imshow(window_name, rgb[:,:,::-1])

#================================ bgr_cv ===============================
#
def bgr_cv(bgr, ratio=None, window_name="Image"):
    '''!
    @brief  Display rgb image using the OpenCV

    The rgb frame will be resized to a visualization size prior to visualization.
    Args:
        rgb (np.ndarray, (H, W, 3)): The rgb image
        ratio (float, Optional): Allow resizing the images before display.  Defaults to None, which means will perform no resizing
        window_name (sting, Optional): The window name for display. Defaults to \"OpenCV display\"
    '''
    if ratio is not None:                                   # Resize if requested.
        H, W = bgr.shape[:2]
        H_vis = int(ratio * H)
        W_vis = int(ratio * W)
        bgr_vis = cv2.resize(bgr, (W_vis, H_vis))
        cv2.imshow(window_name, bgr_vis)
    else:
        cv2.imshow(window_name, bgr)

def gray_cv(gim, ratio=None, window_name="Image"):

    '''!
    @brief  Display grayscale image using the OpenCV

    The rgb frame will be resized to a visualization size prior to visualization.
    Args:
        rgb (np.ndarray, (H, W, 3)): The rgb image
        ratio (float, Optional): Allow resizing the images before display.  Defaults to None, which means will perform no resizing
        window_name (sting, Optional): The window name for display. Defaults to \"OpenCV display\"
    '''
    if ratio is not None:                                   # Resize if requested.
        H, W = gim.shape[:2]
        H_vis = int(ratio * H)
        W_vis = int(ratio * W)
        gim_vis = cv2.resize(gim, (W_vis, H_vis))
        cv2.imshow(window_name, gim_vis)
    else:
        cv2.imshow(window_name, gim)

#============================== binary_cv ==============================
#
def binary_cv(bIm, ratio=None, window_name="Binary"):
    '''!
    @brief  Display binary image using OpenCV display routines.

    The binary frame will be resized as indicated prior to visualization.

    @param[in]  bIm     Binary image as a numpy H x W ndarray.
    @param[in]  ratio   Resize ratio of image to display (float, optional).
                        Default is None = no resizing.
    @param[in]  window_name     Window display name (Default = "Binary").    
    '''

    cIm = cv2.cvtColor(bIm.astype(np.uint8)*255, cv2.COLOR_GRAY2BGR)

    if ratio is not None:                                   # Resize if requested.
        H, W = bIm.shape[:2]
        H_vis = int(ratio * H)
        W_vis = int(ratio * W)
        cIm = cv2.resize(cIm, (W_vis, H_vis))
        cv2.imshow(window_name, cIm)
    else:
        cv2.imshow(window_name, cIm)

#============================ trackpoint_cv ============================
#
def trackpoint_cv(rgb, p, ratio=None, window_name="Image"):
    '''!
    @brief  Display rgb image using the OpenCV

    The rgb frame will be resized to a visualization size prior to visualization.
    Args:
        rgb (np.ndarray, (H, W, 3)): The rgb image
        ratio (float, Optional): Allow resizing the images before display.  Defaults to None, which means will perform no resizing
        window_name (sting, Optional): The window name for display. Defaults to \"OpenCV display\"
    '''
    if ratio is not None:                                   # Resize if requested.
        H, W = rgb.shape[:2]
        H_vis = int(ratio * H)
        W_vis = int(ratio * W)
        rgb_vis = cv2.resize(rgb, (W_vis, H_vis))
        p = ratio * p
    else:
        rgb_vis = rgb

    p_vis = np.fix(p)

    cv2.drawMarker(rgb_vis, (int(p_vis[0,0]),int(p_vis[1,0])), (255, 0, 0), cv2.MARKER_CROSS, 10, 2)
    cv2.imshow(window_name, rgb_vis[:,:,::-1])

#============================ trackpoints_cv ===========================
#
def trackpoints_cv(rgb, p, ratio=None, window_name="Image"):
    '''!
    @brief  Display rgb image using the OpenCV

    The rgb frame will be resized to a visualization size prior to visualization.
    Args:
        rgb (np.ndarray, (H, W, 3)): The rgb image
        ratio (float, Optional): Allow resizing the images before display.  Defaults to None, which means will perform no resizing
        window_name (sting, Optional): The window name for display. Defaults to \"OpenCV display\"
    '''
    if ratio is not None:                                   # Resize if requested.
        H, W = rgb.shape[:2]
        H_vis = int(ratio * H)
        W_vis = int(ratio * W)
        rgb_vis = cv2.resize(rgb, (W_vis, H_vis))
        p = ratio * p
    else:
        rgb_vis = rgb

    p_vis = np.fix(p)

    nTPs = np.shape(p_vis)[1]
    for ii in range(nTPs):
      cv2.drawMarker(rgb_vis, (int(p_vis[0,ii]),int(p_vis[1,ii])), (255, 0, 0), cv2.MARKER_CROSS, 10, 2)
    
    cv2.imshow(window_name, rgb_vis[:,:,::-1])


#============================ trackpoint_cv ============================
#
def trackpoint_binary_cv(bIm, p, ratio=None, window_name="Image"):
    '''!
    @brief  Display rgb image using the OpenCV

    The rgb frame will be resized to a visualization size prior to visualization.
    Args:
        rgb (np.ndarray, (H, W, 3)): The rgb image
        ratio (float, Optional): Allow resizing the images before display.  Defaults to None, which means will perform no resizing
        window_name (sting, Optional): The window name for display. Defaults to \"OpenCV display\"
    '''
    cIm = cv2.cvtColor(bIm.astype(np.uint8)*255, cv2.COLOR_GRAY2BGR)
    trackpoint_cv(cIm, p, ratio, window_name)

def trackpoint_gray_cv(gIm, p, ratio=None, window_name="Image"):

    '''!
    @brief  Display grayscale image using the OpenCV

    The rgb frame will be resized to a visualization size prior to visualization.
    Args:
        rgb (np.ndarray, (H, W, 3)): The rgb image
        ratio (float, Optional): Allow resizing the images before display.  Defaults to None, which means will perform no resizing
        window_name (sting, Optional): The window name for display. Defaults to \"OpenCV display\"
    '''
    cIm = cv2.cvtColor(gIm, cv2.COLOR_GRAY2BGR)
    trackpoint_cv(cIm, p, ratio, window_name)


#================== OpenCV Side-by-Side Image Interfaces =================
#-------------------------------------------------------------------------

#============================== images_cv ==============================
#
# @note Used to be called display_images_cv but display is redundant.
#
def images_cv(images:list, ratio=None, window_name="OpenCV Display"):
    """
    @brief  Display a set of images horizontally concatenated (side-by-side).

    Args:
        images (list): A list of the OpenCV images (bgr) of the same size
        window_name (str, Optional): The window name for display. Defaults to \"OpenCV display\"
    """
    # Resized image dimensions. 
    H, W = images[0].shape[:2]
    if ratio is not None:
        H_vis = int(ratio * H)
        W_vis = int(ratio * W)
    else:
        H_vis = H
        W_vis = W

    # Resize if requested. Stack images horizontally. Display w/imshow.
    images_vis = [cv2.resize(img, (W_vis, H_vis)) for img in images]
    image_display = np.hstack(tuple(images_vis))
    cv2.imshow(window_name, image_display)


#============================= rgb_depth_cv ============================
#
def rgb_depth_cv(rgb, depth, depth_clip=0.08, ratio=None, window_name="OpenCV Display"):

    """
    @brief  Display rgb and depth images side-by-side using OpenCV.

    Depth frame will be scaled to range [0,255].
    The rgb and the depth images will be resized to visualization size, then concatenated horizontally.
    The concatenated image will be displayed in a window.
    There will be no color bar for the depth

    Args:
        rgb (np.ndarray, (H, W, 3)): The rgb image
        depth (np.ndarray, (H, W)): The depth map
        depth_clip (float in [0, 1]): The depth value clipping percentage. The top and bottom extreme depth value to remove. Default to 0.0 (no clipping)
        ratio (float, Optional): Allow resizing the images before display.  Defaults to None, which means will perform no resizing
        window_name (sting, Optional): The window name for display. Defaults to \"OpenCV display\"
    """
    # depth clip
    if depth_clip > 0:
        assert depth_clip < 0.5
        improc = improcessor.basic(improcessor.basic.clipTails,(depth_clip,))
        depth_show = improc.apply(depth)
    else:
        depth_show = depth

    # convert the depth to cv2 image format: scale to 255, convert to 3-channel
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_show, alpha=255./depth_show.max(), beta=0), cv2.COLORMAP_JET)

    # diplay
    images_cv((rgb[:,:,::-1], depth_colormap), ratio=ratio, window_name=window_name)


# @todo Should delete this function.
def display_rgb_dep_cv(rgb, depth, depth_clip=0.08, ratio=None, window_name="OpenCV Display"):

    """Display the rgb and depth image using the OpenCV

    The depth frame will be scaled to have the range 0-255.
    The rgb and the depth frame will be resized to a visualization size and then concatenate together horizontally 
    Then the concatenated image will be displayed in a same window.

    There will be no color bar for the depth

    Args:
        rgb (np.ndarray, (H, W, 3)): The rgb image
        depth (np.ndarray, (H, W)): The depth map
        depth_clip (float in [0, 1]): The depth value clipping percentage. The top and bottom extreme depth value to remove. Default to 0.0 (no clipping)
        ratio (float, Optional): Allow resizing the images before display.  Defaults to None, which means will perform no resizing
        window_name (sting, Optional): The window name for display. Defaults to \"OpenCV display\"
    """
    # depth clip
    if depth_clip > 0:
        assert depth_clip < 0.5
        improc = improcessor.basic(improcessor.basic.clipTails,(depth_clip,))
        depth_show = improc.apply(depth)
    else:
        depth_show = depth

    # convert the depth to cv2 image format: scale to 255, convert to 3-channel
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_show, alpha=255./depth_show.max(), beta=0), cv2.COLORMAP_JET)

    # diplay
    images_cv((rgb[:,:,::-1], depth_colormap), ratio=ratio, window_name=window_name)

# @todo Should delete this function.
def display_dep_cv(depth, depth_clip=0.08, ratio=None, window_name="OpenCV Display"):

    '''!
    @brief  Display depth image using OpenCV window.

    The depth frame will be scaled to have the range 0-255.
    There will be no color bar for the depth

    Args:
        depth (np.ndarray, (H, W)): The depth map
        depth_clip (float in [0, 1]): The depth value clipping percentage. The top and bottom extreme depth value to remove. Default to 0.0 (no clipping)
        ratio (float, Optional): Allow resizing the images before display.  Defaults to None, which means will perform no resizing
        window_name (sting, Optional): The window name for display. Defaults to \"OpenCV display\"
    '''
    # Apply depth clipping if requested.
    # Convert depth to cv2 image format: scale to 255, convert to 3-channel
    if depth_clip > 0:
        assert depth_clip < 0.5
        improc = improcessor.basic(improcessor.basic.clipTails,(depth_clip,))
        depth_show = improc.apply(depth)
        depth_color = cv2.applyColorMap(cv2.convertScaleAbs(depth_show, alpha=255./depth_show.max(), beta=0), cv2.COLORMAP_JET)
    else:
        depth_color = cv2.applyColorMap(cv2.convertScaleAbs(depth, alpha=255./depth.max(), beta=0), cv2.COLORMAP_JET)

    # Rescale if requested.
    if ratio is not None: 
        H, W = depth.shape[:2]
        H_vis = int(ratio * H)
        W_vis = int(ratio * W)
        depth_color = cv2.resize(depth_color, (W_vis, H_vis))

    # Display colorized depth image.
    cv2.imshow(window_name, depth_color[:,:,::-1])

#============================ rgb_binary_cv ============================
#
def rgb_binary_cv(cIm, bIm, ratio=None, window_name="Color+Binary"):
    """!
    @brief  Display an RGB and binar image side-by-side using OpenCV API.

    The images will be resized as indicated by the ratio and concatenated
    horizontally. The concatenated image is what gets displayed in the window.

    @param[in]  cIm             Color image (RGB).
    @param[in]  bIm             Binary image (0/1 or logical)
    @param[in]  ratio           Resizing ratio.
    @param[in]  window_name     Window name for display.
    """

    cBm = cv2.cvtColor(bIm.astype(np.uint8)*255, cv2.COLOR_GRAY2BGR)

    images_cv((cIm[:,:,::-1], cBm), ratio=ratio, window_name=window_name)

#=========================== wait_for_confirm ==========================
#
#
def wait_for_confirm(rgb_dep_getter:Callable, color_type="rgb", window_name = "display",
        instruction="Press \'c\' key to select the frames. Press \'q\' to return None", 
        capture_click = False,
        ratio=None):
    """An interface function for letting the user select the desired frame \
        from the given sensor source. The function will display the color and the depth \
        information received from the source, and then wait for the user to confirm via keyboard. 
    
    NOTE: can't distinguish different keys for now. So only support "press any key to confirm"

    Args:
        color_dep_getter (Callable): The color and depth source. \
            Expect to get the sensor information in the np.ndarray format via: \
                        rgb, depth = color_dep_getter() \
            When there is no more info, expected to return None
        color_type (str): The color type. RGB or BGR. Will be used for visualization
        window_name (str):
        instruction ([type], optional): The instruction text printed to the user for selection. Defaults to None.
        ratio (float, Optional): Allow resizing the images before display.  Defaults to None, which means will perform no resizing

    Returns:
        rgb [np.ndarray]: The rgb image confirmed by the user
        depth [np.ndarray]: The depth frame confirmed by the user
    """
    # get the next stream of data
    rgb, dep = rgb_dep_getter()

    # Display instruction
    print(instruction)

    # get started
    while((rgb is not None) and (dep is not None)):

        # visualization 
        display_rgb_dep_cv(rgb, dep, window_name=window_name, ratio=ratio)

        # wait for confirm
        opKey = cv2.waitKey(1)
        if opKey == ord('c'):
            break
        elif opKey == ord('q'):
            return None, None
        
        # if not confirm, then go to the next stream of data
        rgb, dep = rgb_dep_getter()

    cv2.destroyWindow(window_name)

    return rgb, dep

if __name__ == "__main__":
    imgSource = lambda : (
        (np.random.rand(100,100,3) * 255).astype(np.uint8), \
        np.random.rand(100,100)
    )
    color, dep = wait_for_confirm(imgSource, color_type="rgb", instruction="This is just a dummy example. Press the \'c\' key to confirm", \
        ratio=2)
    # visualize
    display_rgb_dep_plt(color, dep, suptitle="The selected sensor info from the  wait_for_confirm example. Use the Matplotlib for display")
    display_rgb_dep_cv(color, dep, window_name="The selected sensor info from the  wait_for_confirm example. Use the OpenCv for display")
    
    cv2.waitKey(1)
    plt.show()
    
#=============================== wait_cv ===============================
#
# @brief    Wait for user input.
#
#def wait_cv():
#  cv2.waitKey()

#=============================== close_cv ==============================
#
# @brief    Close the displayed window.
#
def close_cv(window_name):
  cv2.destroyWindow(window_name)

