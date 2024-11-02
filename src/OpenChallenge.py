#import necessary libraries
import sys
sys.path.append('/home/pi/TurboPi/')
import cv2
from picamera2 import Picamera2
from time import sleep
import RPi.GPIO as GPIO
import numpy as np
import threading
import HiwonderSDK.Board as Board
import time
from functions import *
from masks import rMagenta, rRed, rGreen, rBlue, rOrange, rBlack, lotType
    
if __name__ == '__main__':
    
    time.sleep(3)

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
    ROI2 = [400, 170, 620, 220] # 380, 600 | 400, 620
    ROI3 = [200, 300, 440, 350]

    #booleans for tracking whether car is in a left or right turn
    lTurn = False
    rTurn = False
  
    t = 0 #number of turns car has completed
    
    kp = 0.02 #value of proportional for proportional steering
    kd = 0.006 #value of derivative for proportional and derivative sterring
    
    straightConst = 87 #angle in which car goes straight

    turnThresh = 150 #if area of a lane is under this threshold car goes into a turn
    exitThresh = 1500 #if area of both lanes is over this threshold car exits a turn
  
    angle = 87 #variable for the current angle of the car
    prevAngle = angle #variable tracking the angle of the previous iteration
    tDeviation = 25 #value used to calculate the how far left and right the car turns during a turn
    sharpRight = straightConst - tDeviation #the default angle sent to the car during a right turn
    sharpLeft = straightConst + tDeviation #the default angle sent to the car during a left turn
    
    #maximum limit for angles
    maxRight = straightConst - 50
    maxLeft = straightConst + 50
    
    speed = 1665 #variable for the speed of the car, 1660
    
    aDiff = 0 #value storing the difference of area between contours
    prevDiff = 0 #value storing the previous difference of contours for derivative steering
    
    write(1500)
    LED1(255, 0, 0)

    sleep(8) #delay 8 seconds for the servo to be ready

    #boolean tracking whether the orange line on the mat is detected
    lDetected = False
    
    debug = False
    
    #set up button 
    key2_pin = 16
    
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(key2_pin, GPIO.IN, pull_up_down = GPIO.PUD_UP)
    
    LED1(0, 255, 0)
    
    #change debugging mode based on program arguments
    if len(sys.argv) > 1 and sys.argv[1] == "Debug":
        debug = True
    else:
        buzz()
        #wait until button press
        while GPIO.input(key2_pin) == GPIO.HIGH:
            pass
        time.sleep(3)
        
    LED1(0, 0, 0)
    LED2(0, 0, 255)
    
    #write intial values to car
    multi_write([angle, 0.5, 1670, 0.1, speed])

    #main loop
    while True:
        
        #time.sleep(1)
          
        #declare variables for the areas of the left and right contours
        rightArea, leftArea = 0, 0

        #get an image from pi camera
        img = picam2.capture_array()
        
        cListLeft = find_contours(img, rBlack, ROI1)
        cListRight = find_contours(img, rBlack, ROI2)
        cListOrange = find_contours(img, rOrange, ROI3)
        
        leftArea = max_contour(cListLeft, ROI1)[0]
        rightArea = max_contour(cListRight, ROI2)[0]
        
        if max_contour(cListOrange, ROI3)[0] > 100: 
            lDetected = True
        
        #draw all contours in full image
            
        cv2.drawContours(img[ROI3[1]:ROI3[3], ROI3[0]:ROI3[2]], cListOrange, -1, (0, 255, 0), 2)
        
        #calculate difference of areas between the areas of the lanes
        aDiff = rightArea - leftArea

        #calculate angle using PD steering
        angle = int(max(straightConst + aDiff * kp + (aDiff - prevDiff) * kd, 0)) 

        #if the area of either lane is less than or equal to turnThresh and the car is not in a turn going the other direction, set the boolean of the respective direction turn to true
        if leftArea <= turnThresh and not rTurn:

            lTurn = True
            LED1(255, 0, 0)

        elif rightArea <= turnThresh and not lTurn:

            rTurn = True
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
                  
                  LED1(0, 255, 0)

                  #reset prevDiff
                  prevDiff = 0 
                  
                  #increase number of turns by 1 only if the orange line has been detected 
                  if lDetected: 
                      t += 1

                      if t == 4:
                          LED2(255, 255, 0)
                      elif t == 8:
                          LED2(255, 255, 255)
                        
                      lDetected = False

              #if car is still in a left turn set the angle to the maximum of angle and sharpLeft
              elif lTurn:
                  angle = min(max(angle, sharpLeft), maxLeft)
              #if car is still in a right turn set the angle to the minimum of angle and sharpRight
              elif rTurn: 
                  angle = max(min(angle, sharpRight), maxRight)

              #write angle to arduino to change servo
              
              write(angle)
              time.sleep(0.01)
            #if not in a turn write the angle and if the angle is over sharpLeft or sharpRight values it will be rounded down to those values
            else:
                write(max(min(angle, sharpLeft), sharpRight))
                time.sleep(0.01)
          
        #update previous area difference
        prevDiff = aDiff
        
        prevAngle = angle #update previous angle
        
        if t == 12 and abs(angle - straightConst) <= 10:
            sleep(1)
            stop_car() 
            break
    
        if debug: 
            
            #stop the car and end the program if either q is pressed or the car has done 3 laps (12 turns) and is mostly straight (within 15 degrees)
            if cv2.waitKey(1)==ord('q'):
                stop_car() 
                break
          
            #display regions of interest
            img = display_roi(img, [ROI1, ROI2, ROI3], (255, 204, 0))

            #show image
            cv2.imshow("finalColor", img)
            
            #cv2.imshow("walls", imgThresh)
            
            variables = {
                "left wall area": leftArea,
                "right wall area": rightArea,
                "left turn": lTurn,
                "right turn": rTurn,
                "# turns": t,
                "lDetected": lDetected

            }

            display_variables(variables)
              
    if debug: 
        #close all image windows
        cv2.destroyAllWindows()
