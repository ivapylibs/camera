"""

    @brief              The frame writer and video writer.

    @author             Yiye Chen.          yychen2019@gatech.edu
    @date               08/07/2021[created]
"""
import os
import shutil
import cv2
import numpy as np
from tqdm import tqdm

class frameWriter(object):
    """
    The Frame Saver saves a single frame out. Given a [frame_name], the saved out file:
    
    1. [frame_name]_(idx).png             The rgb frame
    2. [frame_name]_(idx).npz             Other info, indexed by sting, including
        2.1 "depth_frame":          The raw depth frame
        2.2 others...

    where (idx) will be appended if in the path_idx_mode. 
    The npz file can also save other information provided to the save_frame API

    Args:
        dirname (str): The directory for storing data
        frame_name (str): The file name by which the frame is saved. 
        path_idx_mode (bool, optional): When calling the save_frame multiple times, will append an idx number at the end of the frame_name \
            to distinguish different savings. If not enabled, will overwrite the previous save. Defaults to True.
    """

    def __init__(self, dirname, frame_name, path_idx_mode=True):
        
        self.dirname = dirname
        self.frame_name = frame_name
        self.path_idx_mode = path_idx_mode
        self.id = 0

        # Todo: Need double-check the path
        if not os.path.exists(os.path.dirname(dirname)):
            os.mkdir(os.path.dirname(dirname))
        if not os.path.exists(dirname):
            os.mkdir(dirname)

        # paths to save data
        self.rgb_path = None
        self.data_path = None

    def save_frame(self, img_rgb, dep, **kwargs):
        # update data paths to save
        self.update_paths()        

        # save
        print("Saving the current frame to: {} and {}".format(self.rgb_path, self.data_path))
        cv2.imwrite(self.rgb_path, img_rgb[:,:,::-1])

        # @note Yunzhi: We do not need it for now
        # np.savez(self.data_path,
        #     depth_frame=dep,
        #     **kwargs
        # )

        # update id number
        self.id+=1

    def update_paths(self):
        if self.path_idx_mode:
            self.rgb_path = os.path.join(
                self.dirname,
                self.frame_name + '_{}.png'.format(str(self.id).zfill(3))
            )
            self.data_path = os.path.join(
                self.dirname, 
                self.frame_name + '_{}.npz'.format(str(self.id).zfill(3))
            )
        else:
            self.rgb_path = os.path.join(
                self.dirname,
                self.frame_name + '.png'.format(str(self.id).zfill(3))
            )
            self.data_path = os.path.join(
                self.dirname, 
                self.frame_name + '.npz'.format(str(self.id).zfill(3))
            )


class vidWriter(object):
    """
    For the depth, the saver will save each depth frame out to a cache file
    Then when the process is terminated - where the self.finish() should be called - the saver will
    collect all the depth frames, assemble them, and then stored into the destination file
    """

    def __init__(self, dirname, vidname, W, H, activate=False, save_depth=False, FPS=20.0):
        self.activate = activate
        self.save_depth = save_depth

        if not self.activate:
            return

        # determine paths
        self.dirname = dirname
        self.vidname = vidname
        self.path_color = os.path.join(
            self.dirname,
            self.vidname + ".avi"
        )
        self.path_dep = os.path.join(
            self.dirname,
            self.vidname + ".npz"
        )

        # store sizes
        self.W = W
        self.H = H

        # prepare video writer
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.out_color = cv2.VideoWriter(self.path_color, fourcc, FPS, 
                    (self.W, self.H))
        self.cache_depths = np.empty(shape=[0, self.H, self.W]) # the depth shape would be aligned to the rgb shape

        # cache
        self.cache_folder = os.path.join(
            os.path.dirname(os.path.realpath(__file__)),
            "cache_vidWriter"
        )
        if not os.path.exists(self.cache_folder):
            os.mkdir(self.cache_folder)
        self.cache_id = 0

    def save_frame(self, img_rgb, dep):
        if not self.activate:
            return

        # ====== color
        self.out_color.write(img_rgb[:,:,::-1])

        # ====== depth
        if self.save_depth:
            #self.out_depth = np.append(self.cache_depths, dep[None,:,:], axis=0)
            # cache the depth out
            cache_file = os.path.join(
                self.cache_folder,
                "cache_{}.npz".format(self.cache_id)
            )
            np.savez(cache_file, depth_frame=dep)
            self.cache_id+=1


    def finish(self, **kwargs):
        """
        If intrinsics is not None, then save it together with the depth frames in a npz

        @param[in]  kwargs              allow saving other information via dictionary-like indexing
        """
        print("\n\nFinishing the saving")
        if not self.activate:
            print("Nothing to save. Finished")
            return

        # ======= color
        self.out_color.release()

        # ======= depth
        if self.save_depth:
            #np.savez(self.path_dep, depth_frames = self.out_depth)
            # collect 
            files = os.listdir(self.cache_folder)
            # sort 
            file_name = [os.path.splitext(f)[0] for f in files]
            file_ids = [int(fn.split("_")[1]) for fn in file_name]
            files_sorted = [f for _, f in sorted(zip(file_ids, files), key=lambda pair: pair[0])]
            # save out to destination
            depth_frames = np.zeros(shape=[len(files_sorted), self.H, self.W])
            for idx, f in tqdm(enumerate(files_sorted), total=len(files_sorted)):
                data = np.load(
                    os.path.join(self.cache_folder, f),
                    allow_pickle=True
                )
                depth_frames[idx, :, :] = data['depth_frame']

            if len(kwargs)==0:
                np.savez(self.path_dep, depth_frames=depth_frames)
            else:
                np.savez(self.path_dep, depth_frames=depth_frames, **kwargs)

        # delete caches
        self._rm_cache()
        print("Finished")

    def _rm_cache(self):
        shutil.rmtree(self.cache_folder)
