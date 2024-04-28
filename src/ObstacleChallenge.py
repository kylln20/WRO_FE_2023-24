#import necessary libraries
import cv2
import sys
sys.path.append('/home/pi/TurboPi/')
from picamera2 import Picamera2
import serial
from time import sleep
import time
import RPi.GPIO as GPIO
import numpy as np
import HiwonderSDK.Board as Board

#function used to send signals to arduino to control speeds of the dc motor and the angle of the servo motor
#function used to send signals to arduino to control speeds of the dc motor and the angle of the servo motor
def write(motor, value):
    
    if(motor == "servo"):
        pulseWidth = int(11.1*value+500)
        Board.setPWMServoPulse(1, pulseWidth, 100)
    
    elif(motor == "dc"):
        Board.setPWMServoPulse(5, value, 100)
        #print("no motor")

#function which displays the Regions of Interest on the image
def displayROI(ROIs):
    for ROI in ROIs: 
        image = cv2.line(img, (ROI[0], ROI[1]), (ROI[2], ROI[1]), (0, 255, 255), 4)
        image = cv2.line(img, (ROI[0], ROI[1]), (ROI[0], ROI[3]), (0, 255, 255), 4)
        image = cv2.line(img, (ROI[2], ROI[3]), (ROI[2], ROI[1]), (0, 255, 255), 4)
        image = cv2.line(img, (ROI[2], ROI[3]), (ROI[0], ROI[3]), (0, 255, 255), 4)

#function to bring the car to a stop
def stopCar():
    #write(1438)
    #sleep(1)
    write("servo", 87)
    write("dc", 1500)
    cv2.destroyAllWindows()

if __name__ == '__main__':

    #initialize camera
    picam2 = Picamera2()
    picam2.preview_configuration.main.size = (640,480)
    picam2.preview_configuration.main.format = "RGB888"
    picam2.preview_configuration.controls.FrameRate = 30
    picam2.preview_configuration.align()
    picam2.configure("preview")
    picam2.start()

    #set the target x coordinates for each red and green pillar
    redTarget = 160
    greenTarget = 480

    #variable that keeps track of the target of the last pillar the car has passed
    lastTarget = 0

    #boolean that is used for when the car has to turn around to the opposite direction so the car can complete the last lap the other way around
    reverse = False

    #boolean storing the only direction the car is turning during the run
    turnDir = "none" 
    
    #lists storing coordinates for the regions of interest to find contours of different areas
    #ROI1: for finding left lane
    #ROI2: for finding right lane
    #ROI3: for finding signal pillars
    #ROI4: for detecting blue and orange lines on mat
    # order: x1, y1, x2, y2
    #ROI1 = [0, 165, 330, 285]
    #ROI2 = [330, 165, 640, 285]
    
    parkingR = False
    parkingL = False
    
    ROI1 = [0, 165, 330, 255]
    ROI2 = [330, 165, 640, 255]
    #ROI1 = [20, 170, 240, 220]
    #ROI2 = [400, 170, 620, 220]
    ROI3 = [redTarget - 50, 130, greenTarget + 50, 350]
    ROI4 = [200, 250, 440, 300]
    #ROI5 = [220, 130, 270, 200]
    #ROI6 = [370, 130, 410, 200]
    
    ROIs = [ROI1, ROI2, ROI3, ROI4]

    #booleans for tracking whether car is in a left or right turn
    lTurn = False
    rTurn = False
  
    kp = 0.005 #value of proportional for proportional steering
    kd = 0.005  #value of derivative for proportional and derivative sterring

    cKp = 0.17 #value of proportional for proportional steering for avoiding signal pillars
    cKd = 0.17 #value of derivative for proportional and derivative sterring for avoiding signal pillars
    cy = 0.125 #value used to affect pd steering based on how close the pillar is based on its y coordinate
  
    straightConst = 87 #angle in which car goes straight
    exitThresh = 4000 #if area of both lanes is over this threshold car exits a turn
  
    angle = 87#variable for the current angle of the car
    prevAngle = angle #variable tracking the angle of the previous iteration
    tDeviation = 40 #value used to calculate the how far left and right the car turns during a turn
    sharpRight = straightConst - tDeviation #the default angle sent to the car during a right turn
    sharpLeft = straightConst + tDeviation #the default angle sent to the car during a left turn
    
    speed = 1640 #variable for initial speed of the car
    #tSpeed = 1434 #variable for speed of the car during turn to opposite direction
    #reverseSpeed = 1615 #variable for speed of the car going backwards
    
    stopTime = 0 #stores the time of when the car begins its stopping progress
    s = 0 #stores for how many seconds the car runs after stopTime
    
    aDiff = 0 #value storing the difference of area between contours
    prevDiff = 0 #value storing the previous difference of contours for derivative steering

    error = 0 #value containing the difference between a pillar's x coordinate and its target x coordinate
    prevError = 0 #stores previous error

    cTarget = 0 #stores the target x coordinate for the current signal pillar, stays 0 if there is no signal pillar
    contX = 0 #stores x value of the current signal pillar
    contY = 0 #stores y value of the current signal pillar
    
    t = 0 #tracks number of turns
    
    tSignal = False #boolean that makes sure that a pillar doesn't affect a turn too early
    
    write("dc", 1500) 

    sleep(8) #delay 8 seconds for the servo to be ready
    
    #write initial values to car
    write("dc", speed) 
    write("servo", angle)

    #main loop
    while True:

        #if s seconds has passed after the car began the stopping process, end the program and stop the car
        '''
        if stopTime != 0:
            if time.time() - stopTime > s:
                stopCar()
                break
        '''
            
        #reset rightArea, and leftArea variables
        rightArea, leftArea = 0, 0

        #get an image from pi camera
        img = picam2.capture_array()

        #convert to grayscale
        imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        #threshold image
        ret, imgThresh = cv2.threshold(imgGray, 55, 255, cv2.THRESH_BINARY_INV)

        #find left and right contours of the lanes
        contours_left, hierarchy = cv2.findContours(imgThresh[ROI1[1]:ROI1[3], ROI1[0]:ROI1[2]], 
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    
        contours_right, hierarchy = cv2.findContours(imgThresh[ROI2[1]:ROI2[3], ROI2[0]:ROI2[2]], 
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        
        contours_parking, hierarchy = cv2.findContours(imgThresh[ROI4[1]:ROI4[3], ROI4[0]:ROI4[2]], 
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        #iterate through every contour in both the left and right region of interest and take the largest one in each
        for cnt in contours_left:
            area = cv2.contourArea(cnt)
            
            leftArea = max(area, leftArea) 

        for cnt in contours_right:
            area = cv2.contourArea(cnt)

            rightArea = max(area, rightArea)

        # convert from BGR to HSV
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        #create red mask
        lower_red = np.array([173, 175, 50])
        upper_red = np.array([180, 255, 255])
      
        r_mask = cv2.inRange(img_hsv, lower_red, upper_red)

        #find red contours of signal pillars
        contours_red = cv2.findContours(r_mask[ROI3[1]:ROI3[3], ROI3[0]:ROI3[2]], cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)[-2]

        #create green mask
        lower_green = np.array([60, 80, 40])
        upper_green = np.array([95, 255, 255])

        g_mask = cv2.inRange(img_hsv, lower_green, upper_green)

        #find green contours of signal pillars
        contours_green = cv2.findContours(g_mask[ROI3[1]:ROI3[3], ROI3[0]:ROI3[2]], cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)[-2]
        
        #create blue mask
        lower_blue = np.array([100, 100, 100])
        upper_blue = np.array([135, 255, 255])

        b_mask = cv2.inRange(img_hsv, lower_blue, upper_blue)

        #find blue contours to detect the lines on the mat
        contours_blue = cv2.findContours(b_mask[ROI4[1]:ROI4[3], ROI4[0]:ROI4[2]], cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)[-2]
        
        #create orange mask
        lower_orange = np.array([0, 100, 175])
        upper_orange = np.array([25, 255, 255])

        o_mask = cv2.inRange(img_hsv, lower_orange, upper_orange)

        #find orange contours to detect the lines on the mat
        contours_orange = cv2.findContours(o_mask[ROI4[1]:ROI4[3], ROI4[0]:ROI4[2]], cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)[-2]
        
        #create magenta mask
        lm = np.array([168, 175, 50])
        um = np.array([172, 255, 255])

        m_mask = cv2.inRange(img_hsv, lm, um)

        #find magenta contours to detect the parking lot
        contours_magenta_l = cv2.findContours(m_mask[ROI1[1]:ROI1[3], ROI1[0]:ROI1[2]], cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)[-2]
        
        contours_magenta_r = cv2.findContours(m_mask[ROI2[1]:ROI2[3], ROI2[0]:ROI2[2]], cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)[-2]
        
        num_pillars = 0

        #iterate through green contours
        for i in range(len(contours_green)):
          cnt = contours_green[i]
          area = cv2.contourArea(cnt)

          if area > 100:
              num_pillars += 1
            
              #get width, height, and x and y coordinates by bounding rect
              approx=cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt,True),True)
              x,y,w,h=cv2.boundingRect(approx)

              #since the x and y coordinates are the coordinates just in the ROI, add to the x and y values to make it the proper coordinates on the overall image
              x += ROI3[0]
              y += ROI3[1] + h

              #draw rectangle around signal pillar
              cv2.rectangle(img,(x,y - h),(x+w,y),(0,0,255),2)

              #if the y value is bigger than the previous contY value update contY, contX, and cTarget since this means the current pillar is closer than the previous one
              if y > contY:
                contY = y
                contX = x + w // 2
                cTarget = greenTarget

        #iterate through red contours
        for i in range(len(contours_red)):
          cnt = contours_red[i]
          area = cv2.contourArea(cnt)

          if area > 100:
              num_pillars += 1
            
              #get width, height, and x and y coordinates by bounding rect
              approx=cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt,True),True)
              x,y,w,h=cv2.boundingRect(approx)

              #since the x and y coordinates are the coordinates just in the ROI, add to the x and y values to make it the proper coordinates on the overall image
              x += ROI3[0]
              y += ROI3[1] + h

              #draw rectangle around signal pillar
              cv2.rectangle(img,(x,y - h),(x+w,y),(0,0,255),2)

              #if the y value is bigger than the previous contY value update contY, contX, and cTarget since this means the current pillar is closer than the previous one
              if y > contY:
                contY = y
                contX = x + w // 2
                cTarget = redTarget
        
        #print("num pillars:", num_pillars, end = " ")
        
        if num_pillars >= 2:
            cy = 0.05
            
            kp = 0.007 #value of proportional for proportional steering
            kd = 0.007  #value of derivative for proportional and derivative sterring
            
            cKp = 0.15 #value of proportional for proportional steering for avoiding signal pillars
            cKd = 0.15 #value of derivative for proportional and derivative sterring for avoiding signal pillars
            
            
        elif num_pillars == 1:
            
            kp = 0.007 #value of proportional for proportional steering
            kd = 0.007  #value of derivative for proportional and derivative sterring
            
            cKp = 0.20 #value of proportional for proportional steering for avoiding signal pillars
            cKd = 0.20 #value of derivative for proportional and derivative sterring for avoiding signal pillars
            
            cy = 0.15
        
        else:
            
            kp = 0.007 #value of proportional for proportional steering
            kd = 0.007  #value of derivative for proportional and derivative sterring
            
            cKp = 0.17 #value of proportional for proportional steering for avoiding signal pillars
            cKd = 0.17 #value of derivative for proportional and derivative sterring for avoiding signal pillars
            
            cy = 0.15

        #iterate through orange contours
        for i in range(len(contours_orange)):
          cnt = contours_orange[i]
          area = cv2.contourArea(cnt)
          
          if area > 100:

              #if the turn direction hasn't been changed yet change the turn direction to right
              if turnDir == "none":
                  turnDir = "right"

              #if the turn direction is right
              if turnDir == "right":

                  #if the last pillar we passed is red and we have already completed 7 turns meaning this is the 8th turn 
                  #if lastTarget == redTarget and t == 7:

#                       reverse = True #set reverse to true so the car turns and reverses its direction
                      #turnDir = "left" #change the turn direction as we are heading in the opposite direction
                  #else:

                      #set tTurn and tSignal to true to indicate a right turn
                      rTurn = True
                      tSignal = True                  

        #iterate through blue contours
        for i in range(len(contours_blue)):
          cnt = contours_blue[i]
          area = cv2.contourArea(cnt)

          if area > 100:
              #if the turn direction hasn't been changed yet change the turn direction to left
              if turnDir == "none":
                  turnDir = "left" 
            
              #if the turn direction is left
              if turnDir == "left":

                  #if the last pillar we passed is red and we have already completed 7 turns meaning this is the 8th turn 
                  #if lastTarget == redTarget and t == 7:
                    
                      #reverse = True #set reverse to true so the car turns and reverses its direction
                      #turnDir = "right" #change the turn direction as we are heading in the opposite direction
                  #else:

                      #set tTurn and tSignal to true to indicate a left turn
                      lTurn = True
                      tSignal = True
                      
        
        if t >= 0:
            if turnDir == "right":
                #cy = 0.175
                redTarget = greenTarget
            elif turnDir == "left": 
                greenTarget = redTarget
                      
        maxAreaL = 0
        maxAreaR = 0
        areaFront = 0
        
        for i in range(len(contours_magenta_l)):
            cnt = contours_magenta_l[i]
            maxAreaL = max(cv2.contourArea(cnt), maxAreaL)
        
        for i in range(len(contours_magenta_r)):
            cnt = contours_magenta_r[i]
            maxAreaR = max(cv2.contourArea(cnt), maxAreaR)
            
        for i in range(len(contours_parking)):
            cnt = contours_parking[i]
            areaFront = max(cv2.contourArea(cnt), areaFront)
            
        mDiff = maxAreaR- maxAreaL
        print(mDiff)
        
        if areaFront > 7000:
            time.sleep(0.3)
            stopCar()
            break
        
        if ((maxAreaL > 700 and num_pillars == 0) or (maxAreaL > 2000 and num_pillars == 1) or (rTurn and maxAreaL > 300)) and t >= 0:
            
            if not parkingL and not parkingR:
                parkingL = True
                
            
            time.sleep(0.5)
            
            if parkingL: 
                angle = sharpLeft
            
        
        elif (maxAreaR > 3200 or (lTurn and maxAreaR > 100)) and t >= 0:
            if not parkingL and not parkingR:
                parkingR = True
            
            if parkingR: 
                angle = sharpRight
                  
        #if cTarget is 0 meaning no pillar is detected
        if cTarget == 0 and not parkingL and not parkingR:
            '''
            
            
            elif not parkingL and not parkingR:
            '''

            #calculate the difference in the left and right lane areas
            aDiff = rightArea - leftArea
            #calculate angle using PD steering
            angle = max(0, int(straightConst + aDiff * kp + (aDiff - prevDiff) * kd))

            #update the previous difference
            prevDiff = aDiff

            #if the areas of the two walls are above a threshold meaning we are facing the wall and are also close to the wall
            if leftArea > 7000 and rightArea > 7000:

                #if the last pillar the car passed was green take a hard right turn and if the last pillar was red take a hard left turn
                if lastTarget == greenTarget:
                    angle = sharpRight
                    #print("green")
                elif lastTarget == redTarget:
                    angle = sharpLeft
                    #print("red")
            

        #if pillar is detected
        elif not parkingR and not parkingL:
          
            #if car is in a turn and tSignal is false meaning no orange or blue line is detected currently, end the turn
            if (lTurn or rTurn) and not tSignal:

                #reset prevError and prevDiff 
                prevError = 0
                prevDiff = 0

                #reset lTurn and rTurn booleans to indicate the turn is over
                lTurn = False
                rTurn = False

                #add a turn
                t += 1

                #if car is done 3 laps begin the stopping process by setting stopTime to the current time and s to 3
                '''
                if t == 12:
                    stopCar()
                    break
                '''
            
            #calculate error based on the difference between the target x coordinate and the pillar's current x coordinate
            error = cTarget - contX

            #calculate new angle using PD steering
            angle = int(straightConst + error * cKp + (error - prevError) * cKd)

            #adjust the angle further based on cy value, if error is 
            if error <= 0:
                angle -= int(cy * (contY - ROI3[1]))  
            else:
                angle += int(cy * (contY - ROI3[1]))

            #if the cTarget is equal to greenTarget or redTarget meaning we have a green pillar or a redPillar and the pillar has already exceeded its x target and is also close to the bottom of the ROI, set LastTarget to the respective target as we have basically passed the pillar. 
            if cTarget == greenTarget and contX > greenTarget and contY > ROI3[1] + 50:
                lastTarget = greenTarget
            elif cTarget == redTarget and contX < redTarget and contY > ROI3[1] + 50:
                lastTarget = redTarget

            #make sure angle value is over 2000 
            angle = max(0, angle)

        #if the car needs to turn around to the opposite direction
        '''
        if reverse:

            #code to implement a three point turn
            write(tSpeed)
            write(2098 + 50)
            sleep(1.5)
            write(1500)
            sleep(1)
            write(reverseSpeed)
            write(2098 - 60)
            sleep(1.5)
            write(1500)
            sleep(1)
            write(speed)
           
            #set reverse to false as the turn is over and add 2 to t to make up for the missing turns
            reverse = False
            t += 2
        '''
        #if angle is different from previous angle
        if angle != prevAngle:
          
            #if the area of the lane the car is turning towards is greater than or equal to exitThresh and tSignal is false meaning the blue or orange line is not currently detected
            if ((rightArea >= exitThresh and rTurn) or (leftArea >= exitThresh and lTurn)) and not tSignal: 
              
                  #set turn variables to false as turn is over
                  lTurn = False 
                  rTurn = False
              
                  #increase number of turns by 1
                  t += 1

                  #if car is done 3 laps begin the stopping process by setting stopTime to the current time and s to 2
                  '''
                  if t == 12:
                      stopCar()
                      break
                      
                      
                      stopTime = time.time()
                      if s == 0:
                          write(tSpeed) 
                          s = 1
                      
                  '''

            #if in a right turn and no pillar is detected set the angle to sharpRight
            if rTurn and cTarget == 0:
                angle = sharpRight

            #if in a left turn and no pillar is detected set the angle to sharpLeft
            elif lTurn and cTarget == 0:
                angle = sharpLeft
                
            #write the angle which is kept in the bounds of sharpLeft and sharpRight
            write("servo", max(min(angle, sharpLeft), sharpRight))
                
        #if q is pressed break out of main loop and stop the car
        if cv2.waitKey(1)==ord('q'):
            stopCar() 
            break

        prevAngle = angle #update previous angle
        tSignal = False #reset tSignal
        
        #reset variables for next iteration 
        prevError = error
        contY = 0
        contX = 0
        cTarget = 0
        
        #display regions of interest
        displayROI(ROIs)
        
        #time.sleep(0.1)

        #show image
        #print("turns", t)
        cv2.imshow("finalColor", img) 

