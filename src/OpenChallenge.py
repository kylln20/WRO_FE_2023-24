#import necessary libraries
import sys
sys.path.append('/home/pi/TurboPi/')
import cv2
from picamera2 import Picamera2
#import serial
from time import sleep
import RPi.GPIO as GPIO
import numpy as np
import threading
import HiwonderSDK.Board as Board
import time

#function used to send signals to arduino to control speeds of the dc motor and the angle of the servo motor
def write(motor, value):
    
    if(motor == "servo"):
        pulseWidth = int(11.1*value+500)
        Board.setPWMServoPulse(1, pulseWidth, 1)
    
    elif(motor == "dc"):
        Board.setPWMServoPulse(5, value, 100)
        #print("no motor")
        

def buzz():

    Board.setBuzzer(1)
    time.sleep(0.5)
    Board.setBuzzer(0)
    
def LED1(r, g, b):
    Board.RGB.setPixelColor(0, Board.PixelColor(r, g, b))
    Board.RGB.show()
    
def LED2(r, g, b):
    Board.RGB.setPixelColor(1, Board.PixelColor(r, g, b))
    Board.RGB.show()
        
        
#function to bring the car to a stop
def stopCar():
    write("servo", 87)
    write("dc", 1500)
    
    if debug:
        LED1(0, 0, 0)
        LED2(0, 0, 0)

#function which displays the regions of interest on the image
def displayROI():
    image = cv2.line(img, (ROI1[0], ROI1[1]), (ROI1[2], ROI1[1]), (0, 255, 255), 4)
    image = cv2.line(img, (ROI1[0], ROI1[1]), (ROI1[0], ROI1[3]), (0, 255, 255), 4)
    image = cv2.line(img, (ROI1[2], ROI1[3]), (ROI1[2], ROI1[1]), (0, 255, 255), 4)
    image = cv2.line(img, (ROI1[2], ROI1[3]), (ROI1[0], ROI1[3]), (0, 255, 255), 4)
    
    image = cv2.line(img, (ROI2[0], ROI2[1]), (ROI2[2], ROI2[1]), (0, 255, 255), 4)
    image = cv2.line(img, (ROI2[0], ROI2[1]), (ROI2[0], ROI2[3]), (0, 255, 255), 4)
    image = cv2.line(img, (ROI2[2], ROI2[3]), (ROI2[2], ROI2[1]), (0, 255, 255), 4)
    image = cv2.line(img, (ROI2[2], ROI2[3]), (ROI2[0], ROI2[3]), (0, 255, 255), 4)

    image = cv2.line(img, (ROI3[0], ROI3[1]), (ROI3[2], ROI3[1]), (0, 255, 255), 4)
    image = cv2.line(img, (ROI3[0], ROI3[1]), (ROI3[0], ROI3[3]), (0, 255, 255), 4)
    image = cv2.line(img, (ROI3[2], ROI3[3]), (ROI3[2], ROI3[1]), (0, 255, 255), 4)
    image = cv2.line(img, (ROI3[2], ROI3[3]), (ROI3[0], ROI3[3]), (0, 255, 255), 4)

if __name__ == '__main__':

    #initialize camera
    picam2 = Picamera2()
    picam2.preview_configuration.main.size =(640,480)
    picam2.preview_configuration.main.format = "RGB888"
    picam2.preview_configuration.controls.FrameRate = 30
    picam2.preview_configuration.align()
    picam2.configure("preview")
    picam2.start()

    #lists storing coordinates for the regions of interest to find contours of the lanes and the orange line 
    # order: x1, y1, x2, y2
    ROI1 = [20, 170, 240, 220]
    ROI2 = [400, 170, 620, 220]
    ROI3 = [200, 300, 440, 350]

    #booleans for tracking whether car is in a left or right turn
    lTurn = False
    rTurn = False
  
    t = 0 #number of turns car has completed
    
    kp = 0.006 #value of proportional for proportional steering
    kd = 0.006 #value of derivative for proportional and derivative sterring
    
    straightConst = 87 #angle in which car goes straight

    turnThresh = 150 #if area of a lane is under this threshold car goes into a turn
    exitThresh = 1500 #if area of both lanes is over this threshold car exits a turn
  
    angle = 87 #variable for the current angle of the car
    prevAngle = angle #variable tracking the angle of the previous iteration
    tDeviation = 25 #value used to calculate the how far left and right the car turns during a turn
    sharpRight = straightConst - tDeviation #the default angle sent to the car during a right turn
    sharpLeft = straightConst + tDeviation #the default angle sent to the car during a left turn
    
    speed = 1650 #variable for the speed of the car
    
    aDiff = 0 #value storing the difference of area between contours
    prevDiff = 0 #value storing the previous difference of contours for derivative steering
    
    write("dc", 1500)
    LED1(255, 0, 0)

    sleep(8) #delay 8 seconds for the servo to be ready

    #boolean tracking whether the orange line on the mat is detected
    lDetected = False
    
    debug = False
    key2_pin = 16
    
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(key2_pin, GPIO.IN, pull_up_down = GPIO.PUD_UP)
    
    
    LED1(0, 255, 0)
    
    if len(sys.argv) > 1 and sys.argv[1] == "Debug":
        debug = True
    else:
        buzz()
        while GPIO.input(key2_pin) == GPIO.HIGH:
            pass
        
    if debug: 
        LED2(0, 0, 255)
    
    #write initial values to car
    write("dc", speed) 
    write("servo", angle)

    #main loop
    while True:
        
        #time.sleep(1)
          
        #declare variables for the areas of the left and right contours
        rightArea, leftArea = 0, 0

        #get an image from pi camera
        img = picam2.capture_array()
        
        # convert from BGR to HSV
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        # black mask
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 50])
        
        imgThresh = cv2.inRange(img_hsv, lower_black, upper_black)
        
        # orange mask
        lower_orange = np.array([0, 100, 20])
        upper_orange = np.array([25, 255, 255])

        o_mask = cv2.inRange(img_hsv, lower_orange, upper_orange)

        #find contours to detect orange line
        contours_orange = cv2.findContours(o_mask[ROI3[1]:ROI3[3], ROI3[0]:ROI3[2]], cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)[-2]
        
        #find left and right contours of the lanes
        contours_left, hierarchy = cv2.findContours(imgThresh[ROI1[1]:ROI1[3], ROI1[0]:ROI1[2]], 
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    
        contours_right, hierarchy = cv2.findContours(imgThresh[ROI2[1]:ROI2[3], ROI2[0]:ROI2[2]], 
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
      
        #find all contours in image for debugging
        contours, hierarchy = cv2.findContours(imgThresh, 
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        
        #iterate through every contour in both the left and right region of interest and take the largest one in each
        for cnt in contours_left:
            area = cv2.contourArea(cnt)
            
            leftArea = max(area, leftArea) 

        for cnt in contours_right:
            area = cv2.contourArea(cnt)

            rightArea = max(area, rightArea)

        #iterate through the contours in the centre region of interest to find the orange line
        for i in range(len(contours_orange)):
            cnt = contours_orange[i]
            area = cv2.contourArea(cnt)
            
            #if contour is detected set lDetected to true
            if area > 100:
                lDetected = True
                
        #draw all contours in full image
        for i in range(len(contours)):
            cnt = contours[i]
            area = cv2.contourArea(cnt)
            
            cv2.drawContours(img, contours, i, (0, 255, 0), 2)
            approx=cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt,True),True)
            x,y,w,h=cv2.boundingRect(approx)
        
        #calculate difference of areas between the areas of the lanes
        aDiff = rightArea - leftArea

        #calculate angle using PD steering
        angle = int(max(straightConst + aDiff * kp + (aDiff - prevDiff) * kd, 0)) 

        #if the area of either lane is less than or equal to turnThresh and the car is not in a turn going the other direction, set the boolean of the respective direction turn to true
        if leftArea <= turnThresh and not rTurn:
            lTurn = True
            
            if debug: 
                LED1(255, 0, 0)

        elif rightArea <= turnThresh and not lTurn:
            #print("turned right")
            rTurn = True
            if debug: 
                LED1(255, 0, 0)

        #if angle is different from previous angle
        if angle != prevAngle:
            #if car is in a left or right turn
            if lTurn or rTurn: 

              #if the area of the lane the car is turning towards is greater than or equal to exitThresh, the turn is completed and the booleans are set to false and the number of turns is increased by 1
              if (rightArea > exitThresh and rTurn) or (leftArea > exitThresh and lTurn): 
                  #set turn variables to false as turn is over
                  lTurn = False 
                  rTurn = False
                  
                  if debug: 
                      LED1(0, 255, 0)

                  #reset prevDiff
                  prevDiff = 0 
                  
                  #increase number of turns by 1 only if the orange line has been detected 
                  if lDetected: 
                      t += 1
                      
                      if debug: 
                          if t == 4:
                              LED2(255, 255, 0)
                          elif t == 8:
                              LED2(255, 255, 255)
                        
                      
                      lDetected = False

              #if car is still in a left turn set the angle to the maximum of angle and sharpLeft
              elif lTurn:
                  angle = max(angle, sharpLeft)
              #if car is still in a right turn set the angle to the minimum of angle and sharpRight
              elif rTurn: 
                  angle = min(angle, sharpRight)

              #write angle to arduino to change servo
              write("servo", angle)
              time.sleep(0.01)
            #if not in a turn write the angle and if the angle is over sharpLeft or sharpRight values it will be rounded down to those values
            else:
                write("servo", max(min(angle, sharpLeft), sharpRight))
                time.sleep(0.01)
          
        #update previous area difference
        prevDiff = aDiff
        
        prevAngle = angle #update previous angle
        
        if t == 12 and abs(angle - straightConst) <= 10:
            sleep(2)
            stopCar() 
            break
    
        if debug: 
            
            #stop the car and end the program if either q is pressed or the car has done 3 laps (12 turns) and is mostly straight (within 15 degrees)
            if cv2.waitKey(1)==ord('q'):
                stopCar() 
                break
          
            #display regions of interest
            displayROI()

            #show image
            cv2.imshow("finalColor", img)
              
    if debug: 
        #close all image windows
        cv2.destroyAllWindows()
