"""

    @ brief:        The base D435 recorder

    @ author:       Yiye Chen
    @ date:         02/22/2022

"""

import cv2
import os

import camera.d435.runner as d435
import camera.utils.display as display
from camera.utils.writer_ros import vidWriter_ROS

class D435RecBase():
    def __init__(self, d435_configs:d435.D435_Configs=d435.D435_Configs(exposure=100, gain=50), ext=".bag"):
        """The base d435 recorder.
        The base recorder will create the d435 runner, store the depth scaling parameters,
        and obtain the rgb image and the depth before scaling from the camera.

        Children classes only need to :
        1. Specify the file extension
        2. overwrite what video writer to init 
        3. overwrite how to save out the rgb and depth information

        Args:
            d435_configs (d435.D435_Configs, optional): The D435 camera configurations. Defaults to d435.D435_Configs(exposure=100, gain=50).
            ext (str): The file extension. Defaults to ".bag"
        """
        # The saving directory
        # TODO: for now only support save in the same directory as the script
        self.save_dir = "./"
        self.ext = ext 

        # the camera runner
        self.d435_configs = d435_configs
        self.d435_starter = d435.D435_Runner(self.d435_configs)
        self.depth_scale = self.d435_starter.get("depth_scale")

        # to be created
        self.vid_writer = None
        self.save_path = None
     
    def get_save_path_from_input(self):
        file_name = input("\n Please provide the recording file name (file extension will be appended if not provided): ")
        if not file_name.endswith(self.ext):
            file_name = file_name + self.ext

        save_path = os.path.join(self.save_dir, file_name)

        save_path = self.postprocess_save_path(save_path)

        print("The file will be saved as the following path: {}".format(save_path))

        return save_path

    def postprocess_save_path(self, save_path):
        """Check for duplicates. If any, will append a number at the end of the file name"""
        # get the path and names
        file_dir, file_name = os.path.split(save_path)
        file_name_noext = os.path.splitext(file_name)[0]

        # get all files in the directory
        files = [f for f in os.listdir(file_dir) if os.path.isfile(os.path.join(file_dir, f))]
        duplicate_num = 0
        for exist_file in files:
            if self.check_duplicate(file_name, exist_file):
                duplicate_num = duplicate_num + 1
        
        # append index if necessary
        if duplicate_num > 0:
            new_save_path = os.path.join(
                file_dir,
                file_name_noext + "_{}{}".format(duplicate_num + 1, self.ext)
            )
        else:
            new_save_path = save_path

        return new_save_path 
    
    def check_duplicate(self, save_file, exist_file):
        save_file_name, save_file_ext = os.path.splitext(save_file)
        exist_file_name, exist_file_ext = os.path.splitext(exist_file)
        if save_file_ext != exist_file_ext:
            duplicate = False
        elif save_file_name == exist_file_name: 
            duplicate = True
        elif not '_' in exist_file_name:
            duplicate = False
        else:
            exist_file_name_1, exist_file_name_2 = exist_file_name.rsplit('_', 1)
            if not exist_file_name_2.isdigit():
                duplicate = False
            else:
                duplicate = (save_file_name == exist_file_name_1)
            
        return duplicate
    
    def init_vidWriter(self):
        raise NotImplementedError
    
    def save_frame(self, rgb, depth_before_scale):
        raise NotImplementedError 
    
    def run(self):
        # get the save path and the recorder
        self.save_path = self.get_save_path_from_input()
        self.vid_writer = self.init_vidWriter()

        # get started
        while(True):
            rgb, dep, success = self.d435_starter.get_frames(before_scale=True)
            if not success:
                print("Cannot get the camera signals. Exiting...")
                exit()

            display.display_rgb_dep_cv(rgb, dep*self.depth_scale, depth_clip=0.08, ratio=0.5, window_name="The camera signals. (color-scaled depth). Press \'q\' to exit")
            self.vid_writer.save_frame(rgb,  dep)

            opKey = cv2.waitKey(1)
            if opKey == ord('q'):
                break
        cv2.destroyAllWindows()
        self.vid_writer.finish()

        file_size = os.path.getsize(self.save_path)
        compress_option = input("\n The current file size is: {} GB. Do you want to compress the recording? Compression will take some time. \n Input 1 for compressing, others for not: \n".format(file_size*1e-9))
        if compress_option == "1":
            self.vid_writer.compress()

        file_size = os.path.getsize(self.save_path)
        print("Compression complete. The file size after compression: {} GB".format(file_size*1e-9))

class D435RecRosbag(D435RecBase):
    def __init__(self, d435_configs:d435.D435_Configs=d435.D435_Configs(exposure=100, gain=50),     \
            color_topic_name="color", depth_topic_name="depth", depth_scale_topic_name="depth_scale", frame_rate=None):

        super().__init__(d435_configs, ext=".bag")

        self.color_topic_name = color_topic_name
        self.depth_topic_name = depth_topic_name
        self.depth_scale_topic_name = depth_scale_topic_name
        self.frame_rate = frame_rate
    
    def init_vidWriter(self):
        vid_writer = vidWriter_ROS(
            save_file_path=self.save_path,
            depth_scale=self.depth_scale,
            rgb_topic=self.color_topic_name,
            depth_topic=self.depth_topic_name,
            depth_scale_topic=self.depth_scale_topic_name,
            frame_rate=self.frame_rate,
            auto_compress = False
        )
        self.vid_writer = vid_writer
        return vid_writer

    def save_frame(self, rgb, depth_before_scale):
        self.vid_writer.save_frame(rgb, depth_before_scale)
        