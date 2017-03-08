import cv2
import numpy as np
import argparse
import time
import os
import math
import serial
import traceback
from pidController import PID


ARDUINO_SERIAL_PORT_PI = '/dev/ttyACM0'
ARDUINO_SERIAL_PORT_MAC = '/dev/tty.usbmodem1411'
ARDUINO_BAUD = 115200

CAMERA_TO_USE = 1  # For Mac only! 0 = built-in; 1 = USB

TRACKBAR_WINDOW_NAME = "Trackbars"
GAUSSIAN_BLUR_TRACKBAR_NAME = "Gaussian blur radius"
GAUSSIAN_BLUR_MAX_RADIUS = 199
GAUSSIAN_BLUR_INITIAL_RADIUS = 32
FEEDRATE_BASE_TRACKBAR_NAME = "Feedrate base"
FEEDRATE_BASE_MAX_VALUE = 4000
FEEDRATE_BASE_INITIAL_VALUE = 2000
FEEDRATE_DISTANCE_COEFFICIENT_TRACKBAR_NAME = "Feedrate distance coefficient"
FEEDRATE_DISTANCE_COEFFICIENT_MAX_VALUE = 3000
FEEDRATE_DISTANCE_COEFFICIENT_INITIAL_VALUE = 167

THRESH_VAL = 64

PIXELS_PER_X = 9.25
PIXELS_PER_Y = 25.6

OVERLAY_COLOR = (255, 255, 255)

GRBL_JOG_OVERRIDE = 0x85

def get_arguments():
    ap = argparse.ArgumentParser()
    ap.add_argument('-c', '--camera', type=int,
                    required=False, help='Which camera to use')
    ap.add_argument('-p', '--port', required=False,
                    help='Serial port used by Arduino')
    args = vars(ap.parse_args())

    return args


def callback(value):
    pass


def setup_trackbars():
    cv2.namedWindow(TRACKBAR_WINDOW_NAME, 0)
    cv2.createTrackbar(GAUSSIAN_BLUR_TRACKBAR_NAME, TRACKBAR_WINDOW_NAME,
                       GAUSSIAN_BLUR_INITIAL_RADIUS, GAUSSIAN_BLUR_MAX_RADIUS, callback)
    cv2.createTrackbar(FEEDRATE_BASE_TRACKBAR_NAME, TRACKBAR_WINDOW_NAME,
                       FEEDRATE_BASE_INITIAL_VALUE, FEEDRATE_BASE_MAX_VALUE, callback)
    cv2.createTrackbar(FEEDRATE_DISTANCE_COEFFICIENT_TRACKBAR_NAME, TRACKBAR_WINDOW_NAME,
                       FEEDRATE_DISTANCE_COEFFICIENT_INITIAL_VALUE, FEEDRATE_DISTANCE_COEFFICIENT_MAX_VALUE, callback)


def get_trackbar_value(trackbar_name):
    value = cv2.getTrackbarPos(
        trackbar_name, TRACKBAR_WINDOW_NAME)

    if trackbar_name == GAUSSIAN_BLUR_TRACKBAR_NAME and (value % 2 == 0):
        value += 1

    return value


def calculate_dimensions(frame):
    isDimensionsCalculated = True

    global width, height, widthDiv2, heightDiv2
    width = np.size(frame, 1)
    height = np.size(frame, 0)
    widthDiv2 = int(math.floor(width / 2))
    heightDiv2 = int(math.floor(height / 2))


def add_overlay(frame, radius, maxLoc, maxLocConverted, frameCount, feedrateBase, feedrateDistanceCoefficient, distance, prevDistance):
    # Gaussian radius
    cv2.putText(frame, str(radius), (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, OVERLAY_COLOR)

    # Feedrate base
    cv2.putText(frame, str(feedrateBase), (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 1, OVERLAY_COLOR)

    # Feedrate distance coefficient
    cv2.putText(frame, str(feedrateDistanceCoefficient), (10, 90),
                cv2.FONT_HERSHEY_SIMPLEX, 1, OVERLAY_COLOR)

    # Mark the brightest section of the frame
    cv2.circle(frame, maxLoc, radius, (255, 0, 0), 1)

    # Crosshair
    cv2.line(frame, (widthDiv2, 0), (widthDiv2, height), (255, 255, 255), 1)
    cv2.line(frame, (0, heightDiv2), (width, heightDiv2), (255, 255, 255), 1)

    # Brightest section coordinates
    #cv2.putText(frame, str(maxLoc), (10, height - 10),
    #            cv2.FONT_HERSHEY_SIMPLEX, 2, OVERLAY_COLOR)
    cv2.putText(frame, str(maxLocConverted), (10, height - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 1, OVERLAY_COLOR)

    # Frame Count
    cv2.putText(frame, str(frameCount), (width - 50, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, OVERLAY_COLOR)

    # prev and current distance
    if prevDistance is not None:
    	distanceString = "%.2f"%distance+" / "+"%.2f"%prevDistance
    else:
   		distanceString = "%.2f"%distance+" / "+str(prevDistance)
    cv2.putText(frame, distanceString, (width - 350, height - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 1, OVERLAY_COLOR)


def convert_coordinates(maxLoc):
    return [-1 * (maxLoc[0] - widthDiv2), (maxLoc[1] - heightDiv2)]


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
#        camera.set(cv2.CAP_PROP_FRAME_WIDTH, 960)
#        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 540)
        return camera

def processFrame():
    ret, frame = camera.read()
    global frameCount 
    frameCount +=1 
    global lastFrameMaxLocConverted
    prevDistance = None

    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    #frame_gray = cv2.cvtColor( frame, CV_COLOR_RGB2GRAY)
#        b,g,r = cv2.split(frame)

    ret, frame = cv2.threshold(frame,THRESH_VAL,255,cv2.THRESH_TOZERO)

    #        frame_gray = cv2.threshold( frame_gray, 30, 255,3 )

    if isDimensionsCalculated == False:
        calculate_dimensions(frame)

    radius = get_trackbar_value(GAUSSIAN_BLUR_TRACKBAR_NAME)

    # apply a Gaussian blur to the image then find the brightest region
    #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    frame = cv2.GaussianBlur(frame, (radius, radius), 0)

    # Locate the pixel with the max / min intensity
    (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(frame)

    if not (maxVal>THRESH_VAL):
        maxLoc = (widthDiv2 , heightDiv2)

    maxLocConverted = convert_coordinates(maxLoc)

    # Send the brightest pixel to the Arduino
    if (frameCount % 1) == 0:
        frameCount = 0

        X_FACTOR = 0.5 # //TAL_QUICK_HACK
        # current frame calculations
        xMove = maxLocConverted[0]/PIXELS_PER_X * X_FACTOR
        yMove = maxLocConverted[1]/PIXELS_PER_Y

        #TODO: normalize x and y distance such that feedrate makes sense. simply 'steps' for both here isn't really what makes sense because the velocity on eahc is different
        distance = (xMove*xMove + yMove*yMove)**(0.5)

        if lastFrameMaxLocConverted is not None:
	        # last frame calculations
	        prev_xMove = lastFrameMaxLocConverted[0]/PIXELS_PER_X
	        prev_yMove = lastFrameMaxLocConverted[1]/PIXELS_PER_Y

	        #TODO: normalize x and y distance such that feedrate makes sense. simply 'steps' for both here isn't really what makes sense because the velocity on eahc is different
	        prevDistance = (prev_xMove*prev_xMove + prev_yMove*prev_yMove)**(0.5)

        shouldMove = False
#        command = '! \x18 G0 G91 '

        # feedrate = 6000
        # if (abs(xMove)<3 or abs(yMove)<2):
        #     feedrate = 2000

        #such that at 30 it's 6000
        feedrateBase = 5000
        # feedrateDistanceCoefficient = get_trackbar_value(FEEDRATE_DISTANCE_COEFFICIENT_TRACKBAR_NAME)
        if prevDistance is not None:
        	speed = abs(prevDistance - distance)
        else:
        	speed = 1

        speedFactor = (min(speed, 30)/30)
        distanceFactor = 1/2*distance if distance > 0 else 0

        feedrate =  feedrateBase - ((0.5*speedFactor + 0.5*distanceFactor)*feedrateBase)
        print("Amnon," + str(prevDistance) + "," + str(distance))

        feedrateDistanceCoefficient = 0


        #print("***************************************** " + str((1 / (feedrateSpeedCoefficient * speed))))
        # feedrate = feedrateBase + distance*feedrateDistanceCoefficient



		# feedrateBase = get_trackbar_value(FEEDRATE_BASE_TRACKBAR_NAME)
  #       feedrateDistanceCoefficient = get_trackbar_value(FEEDRATE_DISTANCE_COEFFICIENT_TRACKBAR_NAME)
  #       if prevDistance is not None:
  #       	speed = prevDistance - distance
  #       else:
  #       	speed = 1
  #       proportional = abs((feedrateBase + distance*feedrateDistanceCoefficient))

  #       speedFactor = (speed/30)
  #       distanceFactor = 1/distance if distance > 0 else 0

  #       feedrate =  proportional - speedFactor*distanceFactor*proportional





        command ='$J=G91 F'+str(feedrate)+' '
        if (abs(xMove)>0.3):
            if (abs(xMove)<3):
                xMove = xMove * 0.25
                print("shrinking x")
            command = command +  'X' +str(round(xMove, 2)) + ' '
            shouldMove = True
        if (abs(yMove)>0.35):
            if (abs(yMove<1)):
                yMove = yMove * 0.25
                print("shrinking y")
            command = command + ' Y' +str(round(yMove, 2))
            shouldMove = True

        if (shouldMove):
            print(command)
            arduino.write(chr(GRBL_JOG_OVERRIDE))
            arduino.write(command.encode('latin_1')+ '\n')

        # arduino.write(str.encode(str(-2 * maxLocConverted[0])))
        # arduino.write(str.encode('640'))
#             arduino.write(str.encode('x'))
#             arduino.write(str(-1.4 * maxLocConverted[0]).encode())
#             arduino.write(str.encode('\n'))
# #         arduino.write(str.encode('y'))
#           arduino.write(str(2 * maxLocConverted[1]).encode())
#           arduino.write(str.encode('\n'))


    add_overlay(frame, radius, maxLoc, maxLocConverted, frameCount, feedrateBase, feedrateDistanceCoefficient, distance, prevDistance)
    #add_overlay(frame, 5, (0,0), 0, frameCount)
    lastFrameMaxLocConverted = maxLocConverted

    #cv2.resize(frame, (800, 800))
    # cv2.imshow('frame',frame)

    #imS = cv2.resize(frame, (960, 540))
    cv2.imshow("output", frame)

# workaround so we don't calculate the dimensions every iteration
global isDimensionsCalculated
isDimensionsCalculated = False
global frameCount 
frameCount = 0

def main():


    cv2.namedWindow("output", cv2.WINDOW_NORMAL)
    cv2.moveWindow("output", 0, 150)

    args = get_arguments()

    global arduino
    arduino = serial.Serial(args["port"], ARDUINO_BAUD, timeout=.05)

    # Video stream object
    global camera 
    camera = init_camera()

    # allow the camera to warmup
    time.sleep(2)

    setup_trackbars()

    # Move lastFrameMaxLocConverted to some program init
    global lastFrameMaxLocConverted
    lastFrameMaxLocConverted = None

    while(True):
        try:
            processFrame()
            while arduino.in_waiting:  # Or: while ser.inWaiting():
                print("grbl: "+arduino.readline())
        except Exception as e: 
            print("Exception: " + str(e))
            traceback.print_exc()

        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("Goodbye!")
            arduino.close()
            break

if __name__ == '__main__':
    main()
