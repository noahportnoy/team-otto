import cv2
import numpy as np
import serial
import time
from collections import deque
from datetime import datetime



def stopRecording():
	out.release()
	video_capture.release()
	#cv2.destroyWindow("Video")

def startRecording():
	frameCount = 0
	startTime = time.time()
	
	while True:
		ret, frame = video_capture.read()   														# frame is single frame, ignore ret
		frame = cv2.flip(frame, 1)
		
		global out
		out.write(frame)
			 
		#cv2.imshow("Video", frame)
		
		if(frameCount % 750 == 0 and frameCount != 0):
			out.release()
			out = cv2.VideoWriter('/home/pi/video/output_' + str(datetime.now()) + '.avi', cv2.cv.CV_FOURCC('M','J','P','G'), 7, (640, 480))
			print "New video saved"
		
		frameCount = frameCount + 1
		if(frameCount % 50 == 0):
			stopTime = time.time()
			print "average fps: ", (50 / (stopTime - startTime))
			startTime = time.time()

		if cv2.waitKey(1) & 0xFF == ord('q'):														# if q is pressed, quit
			stopRecording()
			break

video_capture = cv2.VideoCapture(0)
out = cv2.VideoWriter('/home/pi/video/output_' + str(datetime.now()) + '.avi', cv2.cv.CV_FOURCC('M','J','P','G'), 7, (640, 480))
scale_down = 4
dilation_amount = 15

x_queue = deque()
y_queue = deque()

pixelToServoScale = 20

x_ctr = 320
y_ctr = 240

x_offset = 100
y_offset = 100

startRecording()
