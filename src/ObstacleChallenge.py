#import necessary libraries
import cv2
import sys
sys.path.append('/home/pi/TurboPi/')
from picamera2 import Picamera2
import time
import RPi.GPIO as GPIO
import numpy as np
import HiwonderSDK.Board as Board
import math

#function used to send signals to arduino to control speeds of the dc motor and the angle of the servo motor
def write(motor, value):
    
    if(motor == "servo"):
        pulseWidth = int(11.1*value+500)
        Board.setPWMServoPulse(1, pulseWidth, 1)
    
    elif(motor == "dc"):
        Board.setPWMServoPulse(5, value, 100)

#function which displays the Regions of Interest on the image
def displayROI(ROIs):
    for ROI in ROIs: 
        image = cv2.line(img, (ROI[0], ROI[1]), (ROI[2], ROI[1]), (0, 255, 255), 4)
        image = cv2.line(img, (ROI[0], ROI[1]), (ROI[0], ROI[3]), (0, 255, 255), 4)
        image = cv2.line(img, (ROI[2], ROI[3]), (ROI[2], ROI[1]), (0, 255, 255), 4)
        image = cv2.line(img, (ROI[2], ROI[3]), (ROI[0], ROI[3]), (0, 255, 255), 4)

#controls buzzer
def buzz():

    Board.setBuzzer(1)
    time.sleep(0.5)
    Board.setBuzzer(0)

#functions to control each on board LED
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
    
    LED1(0, 0, 0)
    LED2(0, 0, 0)
    
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
    redTarget = 120 #160
    greenTarget = 520 #480

    #variable that keeps track of the target of the last pillar the car has passed
    lastTarget = 0

    #boolean that is used for when the car has to turn around to the opposite direction so the car can complete the last lap the other way around
    reverse = False

    tempR = ""

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
    
    #variables to indicate when the car should park and whether it parks on the right or left side
    parkingR = False
    parkingL = False

    #makes sure the car begins to park when no pillar is detected
    tempParking = False
    
    #counts the previous number of red and green pillars
    prevPillarCountR = 0
    prevPillarCountG = 0
    
    #variable for a state when 2 pillars were previously seen and only 1 pillar is currently seen
    state = False
    
    #determines the offset from the bottom of the ROI when the car should stop seeing a pillar
    endConst = 30
    
    #distance from a pillar to the car
    pDist = 0
    
    #regions of interest
    ROI1 = [0, 165, 330, 255]
    ROI2 = [330, 165, 640, 255]
    #ROI1 = [20, 170, 240, 220]
    #ROI2 = [400, 170, 620, 220]
    ROI3 = [redTarget - 50, 100, greenTarget + 50, 300]
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
    
    speed = 1643 #variable for initial speed of the car
    #tSpeed = 1434 #variable for speed of the car during turn to opposite direction
    reverseSpeed = 1340 #variable for speed of the car going backwards
    
    stopTime = 0 #stores the time of when the car begins its stopping progress
    s = 0 #stores for how many seconds the car runs after stopTime
    
    aDiff = 0 #value storing the difference of area between contours
    prevDiff = 0 #value storing the previous difference of contours for derivative steering

    error = 0 #value containing the difference between a pillar's x coordinate and its target x coordinate
    prevError = 0 #stores previous error

    cTarget = 0 #stores the target x coordinate for the current signal pillar, stays 0 if there is no signal pillar
    contX = 0 #stores x value of the current signal pillar
    contY = 0 #stores y value of the current signal pillar
    pArea = 0 #stores the area of a signal pillar
    
    close = False #boolean indicating whether a signal pillar is within a certain proximity of the camera
    
    t = 0 #tracks number of turns
    
    tSignal = False #boolean that makes sure that a pillar doesn't affect a turn too early
    
    LED1(255, 0, 0)
    
    write("dc", 1500) 

    time.sleep(8) #delay 8 seconds for the servo to be ready
    
    debug = False #boolean for debug mode
    pl = False #variable for left parking mode, a debug mode where it parks immedietly for testing purposes
    pr = False #variable for right parking mode, a debug mode where it parks immedietly for testing purposes
    mReverse = False #variable for reverse mode, a debug mode where it goes directly into a 3 point turn

    #code for starting button
    key2_pin = 16
    
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(key2_pin, GPIO.IN, pull_up_down = GPIO.PUD_UP)
    
    
    LED1(0, 255, 0)
    
    #set mode based on program arguments
    if len(sys.argv) > 1:
        debug = True
        
        if sys.argv[1].lower() == "parkingl":
            pl = True
        elif sys.argv[1].lower() == "parkingr":
            pr = True
        elif sys.argv[1].lower() == "turn":
            mReverse = True
        elif sys.argv[1].lower() == "steer":
            speed = 1500
        
    #if no mode is specified, assume the regular program and wait for button put
    else:
        buzz()
        while GPIO.input(key2_pin) == GPIO.HIGH:
            pass
        
    LED1(0, 0, 0)
    time.sleep(0.5)
    #write initial values to car
    write("dc", speed) 
    write("servo", angle)

    #main loop
    while True:
            
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

        #find black contours in the parking region of interest to determine when to stop during parking
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
        lower_green = np.array([58, 62, 55])
        upper_green = np.array([96, 255, 255])

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

        if tempParking: 
            lm = np.array([168, 175, 50])
            um = np.array([172, 255, 255])

            m_mask = cv2.inRange(img_hsv, lm, um)

            #find magenta contours to detect the parking lot
            contours_magenta_l = cv2.findContours(m_mask[ROI1[1]:ROI1[3], ROI1[0]:ROI1[2]], cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)[-2]
            
            contours_magenta_r = cv2.findContours(m_mask[ROI2[1]:ROI2[3], ROI2[0]:ROI2[2]], cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)[-2]
        
        #count number of red and green pillars
        num_pillars_g = 0
        num_pillars_r = 0
        
        #stores distance between the y coordinates of 2 signal pillars
        yDiff = 480
        pDist = 100000

        #iterate through green contours
        for i in range(len(contours_green)):
          cnt = contours_green[i]
          area = cv2.contourArea(cnt)

          if area > 100:
              num_pillars_g += 1
              
              #get width, height, and x and y coordinates by bounding rect
              approx=cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt,True),True)
              x,y,w,h=cv2.boundingRect(approx)

              #since the x and y coordinates are the coordinates just in the ROI, add to the x and y values to make it the proper coordinates on the overall image
              x += ROI3[0]
              y += ROI3[1] + h
              
              print("green", area, y)
              
              temp_dist = math.dist([x, y], [320, 480])
              
              print(temp_dist, "pixels away")
              
              if y > ROI3[3] - endConst or temp_dist > 390:
                  continue

              #draw rectangle around signal pillar
              cv2.rectangle(img,(x,y - h),(x+w,y),(0,0,255),2)
        
              
              #update the y difference of the pillars
              yDiff = min(yDiff, abs(contY - y))

              #if the y value is bigger than the previous contY value or within a range and has a bigger area update the data as this pillar is now the closest one
              if temp_dist < pDist:
                contY = y
                contX = x + w // 2
                cTarget = greenTarget
                pDist = temp_dist
                
        #iterate through red contours
        for i in range(len(contours_red)):
          cnt = contours_red[i]
          area = cv2.contourArea(cnt)

          if area > 100:
              
              num_pillars_r += 1
              
              #get width, height, and x and y coordinates by bounding rect
              approx=cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt,True),True)
              x,y,w,h=cv2.boundingRect(approx)

              #since the x and y coordinates are the coordinates just in the ROI, add to the x and y values to make it the proper coordinates on the overall image
              x += ROI3[0]
              y += ROI3[1] + h
              
              print("red", area, y)
              
              temp_dist = math.dist([x, y], [320, 480])
              
              print(temp_dist, "pixels away")
              
              if y > ROI3[3] - endConst or temp_dist > 390:
                  continue

              #draw rectangle around signal pillar
              cv2.rectangle(img,(x,y - h),(x+w,y),(0,0,255),2)
            
              #update the y difference of the pillars
              yDiff = min(yDiff, abs(contY - y))

              #if the y value is bigger than the previous contY value or within a range and has a bigger area update the data as this pillar is now the closest one
              if temp_dist < pDist:
                contY = y
                contX = x + w // 2
                cTarget = redTarget
                pDist = temp_dist
        
        #print("num pillars:", num_pillars, end = " ")
        
        #if the difference between the current pillar and another pillar is below 100 set close to true
        if yDiff <= 100:
            close = True
        
        #state is a variable
        if not state:
                if (num_pillars_r == 1 and prevPillarCountR == 2 and close) or (num_pillars_g == 1 and prevPillarCountG == 2 and close):
                    close = False
                    state = True
                        
        if state:
            if num_pillars_r == 0 and num_pillars_g == 0:
                state = False
                
        state = False
                
        #change control variables if there are more than 2 pillars of the same colour, most likely meaning we are turning along an inside corner. Make the control variables less strong
        if (num_pillars_r >= 2 or num_pillars_g >= 2):
            
            '''
            
            cy = 0.1
            
            kp = 0.003 #value of proportional for proportional steering
            kd = 0.003  #value of derivative for proportional and derivative sterring
            
            cKp = 0.20 #0.15 value of proportional for proportional steering for avoiding signal pillars
            cKd = 0.20 #0.15 value of derivative for proportional and derivative sterring for avoiding signal pillars
            
            redTarget = 130
            greenTarget = 510
            
            ROI3 = [redTarget - 50, 125, greenTarget + 50, 350]
            
            LED2(0, 0, 0)
            '''
            
            LED2(0, 0, 0)
            
            cy = 0.15
            
            kp = 0.01 #value of proportional for proportional steering
            kd = 0.01  #value of derivative for proportional and derivative sterring
            
            cKp = 0.3 #0.15 value of proportional for proportional steering for avoiding signal pillars
            cKd = 0.3 #0.15 value of derivative for proportional and derivative sterring for avoiding signal pillars
            
            redTarget = 120
            greenTarget = 520
            
            ROI3 = [redTarget - 40, 120, greenTarget + 40, 350]
            
            endConst = 70
            
            if rightArea > 1000 and leftArea > 1000:
                endConst = 0
        
        #state is a state where there was previously 2 pillars of the same colour and now there is one meaning we are in the second half of an inside corner turn, if state is true, change control variables
        elif state:
            
            LED2(255, 255, 255)
            
            cy = 0.1
            
            kp = 0.003 #value of proportional for proportional steering
            kd = 0.003  #value of derivative for proportional and derivative sterring
            
            cKp = 0.15 #0.15 value of proportional for proportional steering for avoiding signal pillars
            cKd = 0.15 #0.15 value of derivative for proportional and derivative sterring for avoiding signal pillars
            
            redTarget = 130
            greenTarget = 510
            
            ROI3 = [redTarget - 20, 125, greenTarget + 20, 350]

        #any other combination of number of pillars
        else:
            
            LED2(0, 0, 0)
            
            cy = 0.15
            
            kp = 0.01 #value of proportional for proportional steering
            kd = 0.01  #value of derivative for proportional and derivative sterring
            
            cKp = 0.2 #0.15 value of proportional for proportional steering for avoiding signal pillars
            cKd = 0.2 #0.15 value of derivative for proportional and derivative sterring for avoiding signal pillars
            
            redTarget = 120
            greenTarget = 520
            
            ROI3 = [redTarget - 40, 120, greenTarget + 40, 350]
            
            endConst = 30
        

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

                  #set tTurn and tSignal to true to indicate a left turn
                  lTurn = True
                  tSignal = True
                      
        
        #if no pillar is detected (tempParking) and turns are greater than or equal to 12 change it so the car will always stay on the outside of signal pillars
        #do the same if pl and pr are true (debugging mode)

        if (t >= 12 and tempParking) or pl or pr:
            if turnDir == "right" or pl:
                
                #cy = 0.175
                redTarget = greenTarget
                ROI3 = [redTarget - 50, 180, greenTarget + 50, 350]
            elif turnDir == "left" or pr: 
                greenTarget = redTarget
        
        areaFront = 0
                
        for i in range(len(contours_parking)):
            cnt = contours_parking[i]
            areaFront = max(cv2.contourArea(cnt), areaFront)

        #if the area of the wall in front is above a limit stop as we are very close to the wall
        if areaFront > 7000:
            time.sleep(0.3)
            stopCar()
            break
        
        if tempParking or pr or pl: 
            maxAreaL = 0 #biggest magenta contour on left ROI
            maxAreaR = 0 #biggest magenta contour on right ROI
            areaFront = 0 #area of black contour on main ROI
            
            for i in range(len(contours_magenta_l)):
                cnt = contours_magenta_l[i]
                maxAreaL = max(cv2.contourArea(cnt), maxAreaL)
            
            for i in range(len(contours_magenta_r)):
                cnt = contours_magenta_r[i]
                maxAreaR = max(cv2.contourArea(cnt), maxAreaR)

            #conditions for initiating parking on the left side
            if ((maxAreaL > 500 and num_pillars_r == 0 and num_pillars_g == 0) or (maxAreaL > 2000 and num_pillars_g + num_pillars_r == 1) or (rTurn and maxAreaL > 230)) and (t >= 12 or pl):
                
                if not parkingL and not parkingR:
                    parkingL = True
                    if debug: 
                        LED1(255, 0, 255)
                    
                time.sleep(0.5)
                
                if parkingL: 
                    angle = sharpLeft

            #conditions for initiating parking on the right side
            elif (maxAreaR > 3800 or (lTurn and maxAreaR > 100)) and (t >= 12 or pr):
                if not parkingL and not parkingR:
                    parkingR = True
                    
                    if debug: 
                        LED1(220, 255, 125)
                
                if parkingR: 
                    angle = sharpRight
                  
        #if cTarget is 0 meaning no pillar is detected
        if cTarget == 0 and not parkingL and not parkingR:

            #once a pillar is no longer detected after 2 laps (8 turns) have been completed begin the three point turn by changing the cars turn direction and setting reverse to true to start the turn
            #setting tempR to "d" allows for us to alter the three-point turn process slightly for this special case
            if tempR == "w":
                if turnDir == "right":
                    turnDir == "left"
                else:
                    turnDir = "right"
                    
                reverse = True
                tempR = "d"


            #set tempParking to true after 12 turns to indicate pillars should be passed on the outside
            if not tempParking and t == 12:
                tempParking = True
            
            LED1(0, 0, 0)
            
            #since no pillar is detected none can be close so set close to false
            close = False

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
            
            if debug:
                if cTarget == redTarget:
                    LED1(255, 0, 0)
                elif cTarget == greenTarget:
                    LED1(0, 255, 0)
          
            #if car is in a turn and tSignal is false meaning no orange or blue line is detected currently, end the turn
            #in this case the turn is finished while still seeing a pillar
            if (lTurn or rTurn) and not tSignal:

                #reset prevError and prevDiff 
                prevError = 0
                prevDiff = 0

                #reset lTurn and rTurn booleans to indicate the turn is over
                lTurn = False
                rTurn = False
                
                #add a turn
                t += 1

                #if a turn is added and we still see a pillar, set tempR to "w" and wait for no pillars to be detected in future iterations
                #do this only if 2 laps is completed and pillar is red 
                #also do this if mReverse is true (debugging purposes)
                if (t == 8 or mReverse) and cTarget == redTarget:
                    tempR = "w"
            
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

        #sequence for three-point turn
        if reverse:

            LED1(0, 0, 255)
            
            if turnDir == "left": 


                #special case when a pillar had to be passed first before the three-point turn could start
                if tempR == "d":
                    write("dc", 1660) 
                    write("servo", sharpLeft)
                    time.sleep(5)
                else:

                    write("dc", 1660) 
                    write("servo", straightConst)
                    time.sleep(1)
                    write("servo", sharpLeft)
                    time.sleep(2.5)
                    
                write("dc", 1500)
                time.sleep(0.5)
                write("servo", sharpRight)
                write("dc", reverseSpeed)
                time.sleep(4)
                write("dc", 1500)
                time.sleep(0.5)
                write("dc", speed)
                #write("servo", sharpLeft)
                #write("dc", speed)
                #time.sleep(4)
            
            else: #turn sequence if turn direction is right
                
                '''
                if tempR == "d":
                    write("dc", 1660) 
                    write("servo", sharpRight)
                    time.sleep(5)
                else:

                    write("dc", 1660) 
                    write("servo", straightConst)
                    time.sleep(1)
                    write("servo", sharpRight)
                    time.sleep(2.5)
                '''
                
                write("dc", 1500)
                time.sleep(0.5)
                write("servo", sharpLeft)
                write("dc", reverseSpeed)
                time.sleep(5)
                write("dc", 1500)
                time.sleep(0.5)
                write("dc", speed)
                write("servo", sharpRight)
                write("dc", speed)
                time.sleep(1)
                
            reverse = False
            
            #end the program if we just wanted to see the three-point turn
            if mReverse:
                stopCar()
                break
            
        
        #if angle is different from previous angle
        if angle != prevAngle:
          
            #if the area of the lane the car is turning towards is greater than or equal to exitThresh and tSignal is false meaning the blue or orange line is not currently detected
            if ((rightArea >= exitThresh and rTurn) or (leftArea >= exitThresh and lTurn)) and not tSignal: 
              
                  #set turn variables to false as turn is over
                  lTurn = False 
                  rTurn = False
              
                  #increase number of turns by 1
                  #in this case the turn is ended without seeing a pillar
                  t += 1
                  
                  #if 2 laps have been completed or mReverse is true (debugging mode) and the last pillar is red initiate a regular three point turn
                  if (t == 8 or mReverse) and lastTarget == redTarget:
                      reverse = True
                      
                      if turnDir == "right":
                        turnDir == "left"
                      else:
                        turnDir = "right"

            #if in a right turn and no pillar is detected set the angle to sharpRight
            if rTurn and cTarget == 0:
                angle = sharpRight

            #if in a left turn and no pillar is detected set the angle to sharpLeft
            elif lTurn and cTarget == 0:
                angle = sharpLeft
                
            #write the angle which is kept in the bounds of sharpLeft and sharpRight
            write("servo", max(min(angle, sharpLeft), sharpRight))
                
        prevAngle = angle #update previous angle
        tSignal = False #reset tSignal
        
        #reset variables for next iteration 
        prevError = error
        contY = 0
        contX = 0
        cTarget = 0
        pArea = 0
        prevPillarCountR = num_pillars_r
        prevPillarCountG = num_pillars_g
        
        #additional debugging information in debugging mode
        if debug: 
        
            #if q is pressed break out of main loop and stop the car
            if cv2.waitKey(1)==ord('q'):
                stopCar() 
                break
            
            #display regions of interest
            displayROI(ROIs)
            
            #time.sleep(0.1)

            #show image
            print("turns", t)

            cv2.imshow("finalColor", img) 

cv2.destroyAllWindows()

