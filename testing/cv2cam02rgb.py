#!/usr/bin/python
#================================= cv2cam02rgb ================================
"""!
@brief  Tet script to confim that cv2cam interface for a Color camera works.

@author     Patricio A. Vela,       pvela@gatech.edu
@date       2023/12/21              [created]
"""
#
# NOTE: columns = 90, indent = 2, wrap margin = 8.
#
#================================= cv2cam02rgb ================================


import camera.cv2cam as cam
import cv2


camfigs = cam.CfgColor()
camfigs.camera.color.toRGB = True

camera = cam.Color(camfigs)
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
#================================= cv2cam02rgb ================================
