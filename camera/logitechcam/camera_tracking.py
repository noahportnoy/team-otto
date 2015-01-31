import cv2
import numpy as np
import serial
import time
from collections import deque

def stopRecording():
	video_capture.release()
	cv2.destroyWindow("Video")

def startRecording():
	frameCount = 0
	startTime = time.time()
	
	while True:
		ret, frame = video_capture.read()   														# frame is single frame, ignore ret
		frame = cv2.flip(frame, 1)

		# img = cv2.GaussianBlur(frame, (5,5), 0)													# blur frame
		img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

		img = cv2.resize(img, (len(frame[0]) / scale_down, len(frame) / scale_down))				# scaling
		
		red_lower = np.array([0, 150, 0], np.uint8)													# red color range in hsv
		red_upper = np.array([5, 255, 255], np.uint8)
		
		green_lower = np.array([35, 100, 125], np.uint8)											# green shirt color range in hsv
		green_upper = np.array([80, 255, 255], np.uint8)
			
		mask = cv2.inRange(img, red_lower, red_upper)												# mask = pixels that fall within the red color range

		dilation = np.ones((dilation_amount, dilation_amount), "uint8")
		mask = cv2.dilate(mask, dilation)															# dilate the mask
		
		contours, hierarchy = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)		# find contours in the mask
		
		max_area = 0
		largest_contour = None

		for idx, contour in enumerate(contours):													# find the largest contour
			area = cv2.contourArea(contour)
			if area > max_area:
				max_area = area
				largest_contour = contour
		
		if not largest_contour == None:																# if there is a largest contour
			moment = cv2.moments(largest_contour)

			if moment["m00"] > 1000 / scale_down:													# if the first moment of the contour is greater than 1000
				coordinates = cv2.boundingRect(largest_contour)										# create rectange bounding the largest contour
				
				x = coordinates[0] * scale_down
				y = coordinates[1] * scale_down
				w = coordinates[2] * scale_down
				h = coordinates[3] * scale_down

				cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)							# draw the bounding box

				x_object = x+w/2
				y_object = y+h/2

				x_queue.append(x_object)
				if( len(x_queue) > 10 ):
					x_queue.popleft()

				y_queue.append(y_object)
				if( len(y_queue) > 10 ):
					y_queue.popleft()

				x_total = 0
				y_total = 0

				for x_value in x_queue:
					x_total = x_total + x_value

				for y_value in y_queue:
					y_total = y_total + y_value

				x_avg = x_total/len(x_queue)
				y_avg = y_total/len(y_queue)

				if( abs(x_ctr - x_object) > x_offset ):												# detect leaving frame along x direction
					cv2.rectangle(frame, (x_ctr-x_offset, y_ctr-y_offset), (x_ctr+x_offset, y_ctr+y_offset), (0, 0, 255), 2)
					servoRotateDeg = ((x_object - x_ctr))/pixelToServoScale;
					# print servoRotateDeg

					if(servoRotateDeg > 0):
						# servoRotateString = '+' + str(servoRotateDeg);
						servoRotateString = '-1'
					elif(servoRotateDeg < 0):
						# servoRotateString = '-' + str(servoRotateDeg);
						servoRotateString = '+1'
					else:
						servoRotateString = '+0'
			 
		cv2.imshow("Video", frame)
		
		frameCount = frameCount + 1
		if(frameCount % 50 == 0):
			stopTime = time.time()
			print "average fps: ", (50 / (stopTime - startTime))
			startTime = time.time()

		if cv2.waitKey(1) & 0xFF == ord('q'):														# if q is pressed, quit
			stopRecording()
			break

video_capture = cv2.VideoCapture(0)
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
