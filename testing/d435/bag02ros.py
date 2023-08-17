#!/usr/bin/python

import pprint

import subprocess, yaml
import rosbag
import cv2
#from cv_bridge import CvBridge

testInputBagFile = 'bagsource.bag'
readTopic = '/device_0/sensor_1/Color_0/image/data'

#
# @note Works for othe rbag files, but craps out for chosen one due to use
#       of lz4 compression.
#

if __name__ == '__main__':
	bag = rosbag.Bag(testInputBagFile)

	print(bag)

	genBag = bag.read_messages(readTopic)

	for k,b in enumerate(genBag):
		print("OK, %d / %d" % (k, info_dict['messages']))

		#cv2.imshow('topic', cv_image)
		#key = cv2.waitKey(0)

		if 113 == key:
			print("q pressed. Abort.")
			break

	#cv2.destroyAllWindows()

	bag.close()
