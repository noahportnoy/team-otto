import cv2
import numpy as np
import serial
from collections import deque

imgCount = 0

def stopRecording():
	# When everything is done, release the capture
	video_capture.release()
	cv2.destroyWindow("Video")

def startRecordingNoMyo():
	while True:
		# Capture frame-by-frame
		ret, frame = video_capture.read()   														# frame is single frame, ignore ret
		frame = cv2.flip(frame, 1)
			 
		cv2.imshow("Video", frame)

		if cv2.waitKey(1) & 0xFF == ord('q'):														# if q is pressed, quit
			stopRecording()
			break

video_capture = cv2.VideoCapture(0)
scale_down = 1
dilation_amount = 15

x_queue = deque()
y_queue = deque()

pixelToServoScale = 20

x_ctr = 320
y_ctr = 240

x_offset = 100
y_offset = 100

startRecordingNoMyo()
