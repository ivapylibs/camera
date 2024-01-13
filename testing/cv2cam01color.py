#!/usr/bin/python
#================================ cv2cam01color ================================
"""!
@brief  Tet script to confim that cv2cam interface for a Color camera works.

@author     Patricio A. Vela,       pvela@gatech.edu
@date       2023/12/21              [created]
"""
#
#================================ cv2cam01color ================================


import camera.cv2cam as cam
import cv2


camera = cam.Color()
camera.start()

while True:
  image, isGood = camera.capture()

  if isGood:
    camera.display(image, window_name = 'image')

  ok = cv2.waitKey(1)

  if (ok == ord('q')):
    break

camera.stop()
cv2.destroyAllWindows()

#
#================================ cv2cam01color ================================
