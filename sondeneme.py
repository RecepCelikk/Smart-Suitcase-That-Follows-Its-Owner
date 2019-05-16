from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import time
import numpy as np
import imutils

defaultSpeed = 100
windowCenter = 320
centerBuffer = 100
leftBound = int(windowCenter - centerBuffer)
rightBound = int(windowCenter + centerBuffer)
ballPixel = 0


#GPIO
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
#Pin definitions
rightFwd = 18
rightRev = 22
leftFwd = 21
leftRev = 19

GPIO_TRIGGER = 12      #Front ultrasonic sensor
GPIO_ECHO = 11

GPIO.setup(GPIO_TRIGGER,GPIO.OUT)  # Trigger
GPIO.setup(GPIO_ECHO,GPIO.IN)

# Set trigger to False (Low)
GPIO.output(GPIO_TRIGGER, False)

#GPIO initialization
GPIO.setup(leftFwd, GPIO.OUT)
GPIO.setup(leftRev, GPIO.OUT)
GPIO.setup(rightFwd, GPIO.OUT)
GPIO.setup(rightRev, GPIO.OUT)

#Disable movement at startup
GPIO.output(leftFwd, False)
GPIO.output(leftRev, False)
GPIO.output(rightFwd, False)
GPIO.output(rightRev, False)

#PWM Initialization

rightMotorFwd = GPIO.PWM(rightFwd, 50)
leftMotorFwd = GPIO.PWM(leftFwd, 50)
rightMotorRev = GPIO.PWM(rightRev, 50)
leftMotorRev = GPIO.PWM(leftRev, 50)
rightMotorFwd.start(defaultSpeed)
leftMotorFwd.start(defaultSpeed)
leftMotorRev.start(defaultSpeed)
rightMotorRev.start(defaultSpeed)
def updatePwm(rightPwm, leftPwm):
	rightMotorFwd.ChangeDutyCycle(rightPwm)
	leftMotorFwd.ChangeDutyCycle(leftPwm)

def pwmStop():
	rightMotorFwd.ChangeDutyCycle(0)
	rightMotorRev.ChangeDutyCycle(0)
	leftMotorFwd.ChangeDutyCycle(0)
	leftMotorRev.ChangeDutyCycle(0)
	
def distance():
  
    GPIO.output(GPIO_TRIGGER, True)
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
    startTime = time.time()
    stopTime = startTime
    while GPIO.input(GPIO_ECHO) == 0:
        startTime = time.time()

    while GPIO.input(GPIO_ECHO) == 1:
        stopTime = time.time()

    timeElapsed = stopTime - startTime
    distance = (timeElapsed * 34300) / 2
 
    return distance


camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size = (640, 480))

time.sleep(0.1)

lower_blue = np.array([99,115,150])
upper_blue = np.array([110,255,255])

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	
	image = frame.array
	output = image.copy()
	hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	
	mask = cv2.inRange(hsv, lower_blue, upper_blue)
	mask = cv2.erode(mask, None, iterations=2)
	mask = cv2.dilate(mask, None, iterations=2)
	output = cv2.bitwise_and(output, output, mask=mask)
	gray = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
	circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 3, 500, minRadius = 10, maxRadius = 200, param1 = 100,  param2 = 60)
	Curdistance = distance()
	ballPixel = 0
	
	if circles is not None:
		circles = np.round(circles[0, :]).astype("int")
		for (x, y, radius) in circles:

			cv2.circle(output, (x, y), radius, (0, 255, 0), 4)
			
		
			if radius > 10:	
				ballPixel = x
			else:
				ballPixel = 0
	
	cv2.imshow("output", output)
	key = cv2.waitKey(1) & 0xFF
	rawCapture.truncate(0)
	
	#Proportional controller
	if ballPixel == 0:
		print "no ball"
		pwmStop()
		
	elif (ballPixel < leftBound) or (ballPixel > rightBound):
		
            if Curdistance > 40:
		       if  ballPixel < (leftBound):
			       print "left side"
                   if radius > 10 and ballPixel < 220:
			             print ballPixel
			             updatePwm(20,defaultSpeed)
			   
		        elif ballPixel > (rightBound):
			         print "right side"
                     if radius > 10 and ballPixel > 420:
			             print ballPixel
			             updatePwm(defaultSpeed,20)
		    else:
                pwmStop()	  
	else:
        if Curdistance > 40:   
            if (radius < 60): 	
		            print "forward"
		            updatePwm(defaultSpeed, defaultSpeed)
	   else:
		   pwmStop()
	
	if key == ord('q'):
		break

cv2.destroyAllWindows()
camera.close()
pwmStop()
GPIO.cleanup()
