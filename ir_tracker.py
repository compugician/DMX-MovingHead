import cv2
import numpy as np
import argparse
import time
import os
import math
import serial


ARDUINO_SERIAL_PORT_PI = '/dev/ttyACM0'
ARDUINO_SERIAL_PORT_MAC = '/dev/tty.usbmodem1411'
ARDUINO_BAUD = 115200

CAMERA_TO_USE = 1 # For Mac only! 0 = built-in; 1 = USB

TRACKBAR_WINDOW_NAME = "Trackbars"
GAUSSIAN_BLUR_TRACKBAR_NAME = "Gaussian blur radius"
GAUSSIAN_BLUR_MAX_RADIUS = 199
GAUSSIAN_BLUR_INITIAL_RADIUS = 41

OVERLAY_COLOR = (0,0,255)


def get_arguments():
    ap = argparse.ArgumentParser()
    ap.add_argument('-c', '--camera', type = int, required=False, help='Which camera to use')
    ap.add_argument('-p', '--port', required=False, help='Serial port used by Arduino')
    args = vars(ap.parse_args())

    return args


def callback(value):
    pass


def setup_trackbars():
    cv2.namedWindow(TRACKBAR_WINDOW_NAME, 0)
    cv2.createTrackbar(GAUSSIAN_BLUR_TRACKBAR_NAME, TRACKBAR_WINDOW_NAME, GAUSSIAN_BLUR_INITIAL_RADIUS, GAUSSIAN_BLUR_MAX_RADIUS, callback)


def get_trackbar_value():
    value = cv2.getTrackbarPos(GAUSSIAN_BLUR_TRACKBAR_NAME, TRACKBAR_WINDOW_NAME)

    if value % 2 == 0:
    	value += 1

    return value


def calculate_dimensions(frame):
	isDimensionsCalculated = True

	global width, height, widthDiv2, heightDiv2
	width = np.size(frame, 1)
	height = np.size(frame, 0)
	widthDiv2 = math.floor(width/2)
	heightDiv2 = math.floor(height/2)


def add_overlay(frame, radius, maxLoc, maxLocConverted, frameCount):
	# Gaussian radius
	cv2.putText(frame, str(radius), (10,50), cv2.FONT_HERSHEY_SIMPLEX, 2, OVERLAY_COLOR)

	# Mark the brightest section of the frame
	cv2.circle(frame, maxLoc, radius, (255, 0, 0), 2)

	# Crosshair
	cv2.line(frame, (widthDiv2, 0),(widthDiv2, height), (255,0,0), 2)
	cv2.line(frame, (0, heightDiv2),(width, heightDiv2), (255,0,0), 2)

	# Brightest section coordinates
	cv2.putText(frame, str(maxLoc), (10, height-10), cv2.FONT_HERSHEY_SIMPLEX, 2, OVERLAY_COLOR)
	cv2.putText(frame, str(maxLocConverted), (width-550, height-10), cv2.FONT_HERSHEY_SIMPLEX, 2, OVERLAY_COLOR)

	# Frame Count
	cv2.putText(frame, str(frameCount), (width-200, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, OVERLAY_COLOR)


def convert_coordinates(maxLoc):
	return [maxLoc[0]-widthDiv2, -1*(maxLoc[1]-heightDiv2)]


def is_pi():
    return 'arm' == os.uname()[4][:3]


def init_camera():
	if is_pi():
		camera = PiCamera()
		camera.resolution = (640, 480)
		camera.framerate = 32
		rawCapture = PiRGBArray(camera, size=(640, 480))

		return camera
	else:
		camera = cv2.VideoCapture(CAMERA_TO_USE)
		camera.set(cv2.CAP_PROP_GAIN, 0.1)
		camera.set(cv2.CAP_PROP_EXPOSURE, 0.1)
		return camera


def main():

	# workaround so we don't calculate the dimensions every iteration
    global isDimensionsCalculated
    isDimensionsCalculated = False
    frameCount = 0

    cv2.namedWindow("output", cv2.WINDOW_NORMAL)
    cv2.moveWindow("output", 0, 0)

    args = get_arguments()
    
    # Video stream object
    camera = init_camera()
    arduino = serial.Serial(args["port"], ARDUINO_BAUD, timeout=.1)

    # allow the camera to warmup
    time.sleep(4)

    setup_trackbars()

    while(True):
        ret, frame = camera.read()
        frameCount += 1
    
        orig = frame.copy()

        if isDimensionsCalculated == False:
        	calculate_dimensions(frame)

        radius = get_trackbar_value()

        # apply a Gaussian blur to the image then find the brightest region
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (radius, radius), 0)

        # Locate the pixel with the max / min intensity
        (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(gray)

        maxLocConverted = convert_coordinates(maxLoc)

        # Send the brightest pixel to the Arduino
        if (frameCount % 25) == 0:
        	frameCount = 0
        	# arduino.write(str.encode(str(-2 * maxLocConverted[0])))
        	# arduino.write(str.encode('640'))
        	arduino.write(str.encode('x'))
        	arduino.write(str(-1.4 * maxLocConverted[0]).encode())
        	arduino.write(str.encode('\n'))
#        	arduino.write(str.encode('y'))
#        	arduino.write(str(2 * maxLocConverted[1]).encode())
#       	arduino.write(str.encode('\n'))

        add_overlay(frame, radius, maxLoc, maxLocConverted, frameCount)

        #cv2.resize(frame, (800, 800))
        #cv2.imshow('frame',frame)

        
        imS = cv2.resize(frame, (960, 540))
        cv2.imshow("output", imS)  



        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

if __name__ == '__main__':
    main()

