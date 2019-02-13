# -*- coding: utf-8 -*-
"""
Created on Tue Aug  7 17:53:04 2018

@author: David
quiero que envie  8 frames por segundo
"""
import time
import cv2
cap = cv2.VideoCapture(0)
end =2
start =0
start2 = 1
while True:
	dl = end -start
	fps = 1/dl
	start=time.time()
	_, im_np = cap.read()

	print("FPS: {}   Time/frame: {}".format(fps,dl))
	cv2.imshow('Hola', im_np)
	end =time.time()
	if cv2.waitKey(1) & 0xFF == ord(' '):
		cap.release()
		cv2.destroyAllWindows()
		break
