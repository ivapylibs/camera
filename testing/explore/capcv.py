#!/usr/bin/python

# FROM: https://docs.opencv.org/3.4/dd/d43/tutorial_py_video_display.html


import cv2
import time

toGray = False


camera = cv2.VideoCapture(0)
while True:
  return_value,image = camera.read()
  if toGray:
    gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    cv2.imshow('image',gray)
  else:
    cv2.imshow('image',image)

  ok = cv2.waitKey(1)

  if (ok == ord('q')):
    break
    
camera.release()
cv2.destroyAllWindows()
