import cv2

# settings
fps_new = 5             # The new FPS after compression
size_ratio = 0.5        # The new-resolution-to-old-resolution ratio. If None, will keep the original size

# video name
vid_name = "puzzle_play_sort"
vid_compress_name = vid_name + "_compress"
vid_file = vid_name + ".avi"
vid_compress_file = vid_compress_name + ".avi"

# prepare the video reader and get information
cap = cv2.VideoCapture(vid_file)
width = cap.get(3)
height = cap.get(4)
fps = cap.get(5)

# prepare the video writer
fourcc = cv2.VideoWriter_fourcc(*'XVID')
if size_ratio is None:
    size_new = (int(width), int(height))
else:
    size_new = (int(width*size_ratio), int(height*size_ratio))
out = cv2.VideoWriter(vid_compress_file, fourcc, fps_new, size_new)

# frame skip rate
frame_skip_rate = fps / fps_new

# read and write
count = 0
while (cap.isOpened()):
    ret, frame = cap.read()
    if ret is True:
        #cv2.imshow("Frame", frame)
        #if cv2.waitKey(20) == ord('q'):
        #    break
        if count < frame_skip_rate:
            count += 1 
        else:
            frame = cv2.resize(frame, size_new)
            out.write(frame)
            count = 0
    else:
        break

cap.release()
out.release()
cv2.destroyAllWindows()
