# Camera/scripts

Some temporary scripts that can potentially evolve into utility classes.



## Function Description and Usage

1. ```d435_record_color.py```

   **Description:**Record and save the d435 color streams into a video. 

   **Usage**:  set the save video name and resolution at the beginning of the script, and run the file directly via ```python d435_record_color.py```

2. ```vid_compress.py```

   **Description**: Compress a stored avi video. 

   **Usage**: Set the compress setting (the new FPS and the resolution ratio) and the video name to be compressed at the beginning of the script, then run the file directory via ```python vid_compress.py```.
   
3. ```d435_record_rosbag.py```

   **Description: **Record and save the d435 color and depth streams into a rosbag. 

   **Usage**:  Specify the save path name:

   ```bash
   python d435_record_rosbag.py --save_file_path YOUR/FILE/PATH
   ```

   If do not specify the argument (i.e. directly run ```python d435_record_rosbag.py```), will save to the ```d435_record.bag``` in this script folder.

4. ```vid_playback.py```

   **Description: **Load the rosbag recording of the camera streams and show them.

   **Usage**:  Specify the target path name:

   ```bash 
   python vid_playback.py --target_file_path YOUR/FILE/PATH
   ```

   If do not specify the argument (i.e. directly run ```python vid_playback.py```), will look for the ```d435_record.bag``` in this script folder.





