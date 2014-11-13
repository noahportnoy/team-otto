import cv2
import sys
import serial
from collections import deque

imgCount = 0

def stopRecording():
	# When everything is done, release the capture
	video_capture.release()
	#out.release()
	cv2.destroyAllWindows()

def startRecordingNoMyo():
	while True:
		# Capture frame-by-frame
		ret, frame = video_capture.read()   # frame is single frame, ignore ret

		# print ser.readline()

		# Write frame to video file
		#out.write(frame)

		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

		faces = faceCascade.detectMultiScale(
			gray,
			scaleFactor=1.1,
			minNeighbors=5,
			minSize=(125, 125),
			flags=cv2.cv.CV_HAAR_SCALE_IMAGE		# CV_HAAR_SCALE_IMAGE
		)

		# Draw a rectangle around the faces
		for (x, y, w, h) in faces:
			cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
			x_face = x+w/2
			y_face = y+h/2


			x_queue.append(x_face)
			if( len(x_queue) > 10 ):
				x_queue.popleft()

			y_queue.append(y_face)
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

			# print x_avg, ", ", y_avg

			# detect leaving frame along x direction
			if( abs(x_ctr - x_face) > x_offset ):
				cv2.rectangle(frame, (x_ctr-x_offset, y_ctr-y_offset), (x_ctr+x_offset, y_ctr+y_offset), (0, 0, 255), 2)
				servoRotateDeg = ((x_face - x_ctr))/pixelToServoScale;
				# print servoRotateDeg

				if(servoRotateDeg > 0):
					# servoRotateString = '+' + str(servoRotateDeg);
					servoRotateString = '+1'
					ser.write(servoRotateString)
				elif(servoRotateDeg < 0):
					# servoRotateString = '-' + str(servoRotateDeg);
					servoRotateString = '-1'
					ser.write(servoRotateString)
				else:
					servoRotateString = '+0'
					ser.write(servoRotateString)

		# Display the resulting frame
		cv2.imshow('Video', frame)

		if cv2.waitKey(1) & 0xFF == ord('q'):   # if q is pressed
			ser.close()
			stopRecording()
			break

cascPath = sys.argv[1]
faceCascade = cv2.CascadeClassifier(cascPath)

video_capture = cv2.VideoCapture(0)     # may want to change paramater number to get proper webcam
#out = cv2.VideoWriter('output.avi',-1, 20, (640,480))

ser = serial.Serial(15, 9600)
x_queue = deque()
y_queue = deque()

pixelToServoScale = 20

x_ctr = 320
y_ctr = 240

x_offset = 100
y_offset = 100

startRecordingNoMyo()