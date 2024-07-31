# ------------------------------------------------------------{ libraries }-------------------------------------------------------------------------
#command to calibrate gyro "python3 -m gyro.calibrateBerryIMU.py"
import cv2
import sys
sys.path.append('/home/pi/TurboPi/')
from picamera2 import Picamera2
import time
import RPi.GPIO as GPIO
import numpy as np
import HiwonderSDK.Board as Board
import math
from gyro import berryIMU, IMU, LSM9DS0, LSM9DS1, LSM6DSL, LIS3MDL


# ------------------------------------------------------------{ function declarations }-------------------------------------------------------------------------

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
    
def initGyro(hList, h, val):
    closest = 360
    hIndex = 0
    
    for i in range(len(hList)):
        if abs(h - hList[i]) <= closest:
            #print(h, hList[i])
            closest = abs(h - hList[i])
            hIndex = i
    
    if val == "index": 
        return hIndex
    else:
        return closest
    

if __name__ == '__main__':
    
# ------------------------------------------------------------{ initialization of variables }-------------------------------------------------------------------------

    #initialize camera
    picam2 = Picamera2()
    picam2.preview_configuration.main.size = (640,480)
    picam2.preview_configuration.main.format = "RGB888"
    picam2.preview_configuration.controls.FrameRate = 30
    picam2.preview_configuration.align()
    picam2.configure("preview")
    picam2.start()

    #set the target x coordinates for each red and green pillar
    redTarget = 120 
    greenTarget = 520

    #variable that keeps track of the target of the last pillar the car has passed
    lastTarget = 0

    #boolean that is used for when the car has to turn around to the opposite direction so the car can complete the last lap the other way around
    reverse = False

    #variable helping to delay a three point turn until the current pillar is passed
    tempR = False

    #boolean storing the only direction the car is turning during the run
    turnDir = "none" 
    
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
    #ROI1: for finding left lane
    #ROI2: for finding right lane
    #ROI3: for finding signal pillars
    #ROI4: for detecting blue and orange lines on mat
    # order: x1, y1, x2, y2
    ROI1 = [0, 165, 330, 255]
    ROI2 = [330, 165, 640, 255]
    ROI3 = [redTarget - 40, 120, greenTarget + 40, 350]
    ROI4 = [200, 250, 440, 300]
    
    ROIs = [ROI1, ROI2, ROI3, ROI4]

    #booleans for tracking whether car is in a left or right turn
    lTurn = False
    rTurn = False
  
    kp = 0.02 #proportional value for PD steering 
    kd = 0.01 #derivative value for PD steering

    cKp = 0.2 #value of proportional for proportional steering for avoiding signal pillars
    cKd = 0.2 #value of derivative for proportional and derivative sterring for avoiding signal pillars
    cy = 0.15 #value used to affect pd steering based on how close the pillar is based on its y coordinate
  
    straightConst = 87 #angle in which car goes straight
    exitThresh = 4000 #if area of both lanes is over this threshold car exits a turn
  
    angle = 87#variable for the current angle of the car
    prevAngle = angle #variable tracking the angle of the previous iteration
    tDeviation = 40 #value used to calculate the how far left and right the car turns during a turn
    sharpRight = straightConst - tDeviation #the default angle sent to the car during a right turn
    sharpLeft = straightConst + tDeviation #the default angle sent to the car during a left turn
    
    speed = 1650 #variable for initial speed of the car
    reverseSpeed = 1340 #variable for speed of the car going backwards
    
    aDiff = 0 #value storing the difference of area between contours
    prevDiff = 0 #value storing the previous difference of contours for derivative steering

    error = 0 #value containing the difference between a pillar's x coordinate and its target x coordinate
    prevError = 0 #stores previous error

    cTarget = 0 #stores the target x coordinate for the current signal pillar, stays 0 if there is no signal pillar
    contX = 0 #stores x value of the current signal pillar
    contY = 0 #stores y value of the current signal pillar
    pArea = 0 #stores the area of a signal pillar
    
    t = 0 #tracks number of turns
    
    tSignal = False #boolean that makes sure that a pillar doesn't affect a turn too early
    
    LED1(255, 0, 0)
    
    write("dc", 1500) 

    time.sleep(8) #delay 8 seconds for the servo to be ready
    
    debug = False #boolean for debug mode
    pl = False #variable for left parking mode, a debug mode where it parks immedietly for testing purposes
    pr = False #variable for right parking mode, a debug mode where it parks immedietly for testing purposes
    mReverse = False #variable for reverse mode, a debug mode where it goes directly into a 3 point turn
    
    #variables used in parking
    pKp = 0.1
    pKd = 0.1
    targetHeading = 0
    heading = 0
    stage = 1
    pGyroError = 0
    
    headingIndex = 0
    
    #North, West, South, East
    targetHeadings = [173, 94, 0, 271]
    #targetHeadings = [180, 90, 0, 270]
    
    heading, tHeading = berryIMU.compute_heading()
    print("initial", heading, tHeading)
    
    if heading > 340:
        heading -= 360
    
    headingIndex = initGyro(targetHeadings, heading, "index")
            
    timeStraight = 100000
    
    startTime = 0
    curTime = 0
    
    lotAreas = [0, 0, 0, 0]
    lotLocation = 0
    
    
    print(headingIndex)
            

    #code for starting button
    key2_pin = 16
    
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(key2_pin, GPIO.IN, pull_up_down = GPIO.PUD_UP)
    
    LED1(0, 255, 0)
    
    #set mode based on program arguments
    if len(sys.argv) > 1:
        debug = True
        
        if sys.argv[1].lower() == "parkingl":
            redTarget = greenTarget
            tempParking = True
            pl = True
        elif sys.argv[1].lower() == "parkingr":
            greenTarget = redTarget
            tempParking = True
            pr = True
        elif sys.argv[1].lower() == "turn":
            mReverse = True
        elif sys.argv[1].lower() == "steer":
            pr = True
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

# ------------------------------------------------------------{ main loop }-------------------------------------------------------------------------
    while True:

# ------------------------------------------------------------{ contour detection of walls, pillars, and orange and blue lines }-------------------------------------------------------------------------
            
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
        lower_red = np.array([174, 175, 50])
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
        if tempParking or pl or pr or (5 > t > 0): 
            lm = np.array([168, 175, 50])
            um = np.array([174, 255, 255])

            m_mask = cv2.inRange(img_hsv, lm, um)

            #find magenta contours to detect the parking lot
            contours_magenta_l = cv2.findContours(m_mask[ROI1[1]:ROI1[3], ROI1[0]:ROI1[2]], cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)[-2]
            
            contours_magenta_r = cv2.findContours(m_mask[ROI2[1]:ROI2[3], ROI2[0]:ROI2[2]], cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)[-2]

# ------------------------------------------------------------{ processing contours }-------------------------------------------------------------------------
        
        #count number of red and green pillars
        num_pillars_g = 0
        num_pillars_r = 0
        
        #distance of the pillar from the bottom middle of the screen
        pDist = 100000
        pArea = 0

# -----------------{ green signal pillar contour processing }--------------

        for i in range(len(contours_green)):
          cnt = contours_green[i]
          area = cv2.contourArea(cnt)

          if area > 100:
              
              #get width, height, and x and y coordinates by bounding rect
              approx=cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt,True),True)
              x,y,w,h=cv2.boundingRect(approx)

              #since the x and y coordinates are the coordinates just in the ROI, add to the x and y values to make it the proper coordinates on the overall image
              x += ROI3[0]
              y += ROI3[1] + h
              
              #print("green", area, y)
              
              #calculates the distance between the pillar and the bottom middle of the screen
              temp_dist = math.dist([x + w // 2, y], [320, 480])
              
              #print(temp_dist, "pixels away")
              
              #if the pillar is close enough add it to the number of pillars
              if temp_dist < 395:
                  num_pillars_g += 1
                  
                #if the pillar is too close, stop the car and reverse to give it enough space
              if area > 6500 and x + w // 2 <= 340 and not pr and not pl:
                  LED2(255, 255, 0)
                  write("servo", 87)
                  write("dc", 1500)
                  write("dc", reverseSpeed)
                  time.sleep(1)
                  write("dc", speed)
              else:
                  LED2(0, 0, 0)
              
              #deselects current pillar if it is past a certain limit on the bottom of the screen meaning its too close, if the pillars distance is too far, or the car is too close to the wall
              if y > ROI3[3] - endConst or temp_dist > 370:
                  continue
                
              #if the area of the right wall is too big stop worrying about the pillar so the regular lane control algorithm can move away from the wall
              if leftArea > 13000 and (turnDir == "none" or turnDir == "left"):
                  continue
              
              #draw rectangle around signal pillar
              cv2.rectangle(img,(x,y - h),(x+w,y),(0,0,255),2)

              #if the y value is bigger than the previous contY value or within a range and has a bigger area update the data as this pillar is now the closest one
              if temp_dist < pDist:
                pArea = area
                contY = y
                contX = x + w // 2
                cTarget = greenTarget
                pDist = temp_dist
                
# -----------------{ red signal pillar contour processing }--------------

        for i in range(len(contours_red)):
          cnt = contours_red[i]
          area = cv2.contourArea(cnt)

          if area > 100:
              
              #get width, height, and x and y coordinates by bounding rect
              approx=cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt,True),True)
              x,y,w,h=cv2.boundingRect(approx)

              #since the x and y coordinates are the coordinates just in the ROI, add to the x and y values to make it the proper coordinates on the overall image
              x += ROI3[0]
              y += ROI3[1] + h
              
              #print("red", area, y)
              
              #calculates the distance between the pillar and the bottom middle of the screen
              temp_dist = math.dist([x + w // 2, y], [320, 480])
              
              #print(temp_dist, "pixels away")
              
              #if the pillar is close enough add it to the number of pillars
              if temp_dist < 395:
                  num_pillars_r += 1
              
              #if the pillar is too close, stop the car and reverse to give it enough space
              if area > 6500 and x + w // 2 >= 300 and not pr and not pl:
                  LED2(255, 255, 0)
                  
                  write("servo", 87)
                  write("dc", 1500)
                  write("dc", reverseSpeed)
                  time.sleep(1)
                  write("dc", speed)
                  
              else:
                  LED2(0, 0, 0)
                
              #deselects current pillar if it is past a certain limit on the bottom of the screen meaning its too close, if the pillars distance is too far, or the car is too close to the wall
              if y > ROI3[3] - endConst or temp_dist > 370:
                  continue
            
              #if the area of the right wall is too big stop worrying about the pillar so the regular lane control algorithm can move away from the wall
              if rightArea > 13000 and (turnDir == "none" or turnDir == "right"):
                  continue
            
              #draw rectangle around signal pillar
              cv2.rectangle(img,(x,y - h),(x+w,y),(0,0,255),2)

              #if the y value is bigger than the previous contY value or within a range and has a bigger area update the data as this pillar is now the closest one
              if temp_dist < pDist:
                pArea = area
                contY = y
                contX = x + w // 2
                cTarget = redTarget
                pDist = temp_dist

# -----------------{ control variable manipulation based on number of pillars }--------------

        #change control variables if there are more than 2 pillars of the same colour, most likely meaning we are turning along an inside corner. Make the control variables less strong
        if (num_pillars_r >= 2 or num_pillars_g >= 2):
            
            LED2(255, 255, 255)
            
            endConst = 50
            
            cKp = 0.2 #value of proportional for proportional steering for avoiding signal pillars
            cKd = 0.2 #value of derivative for proportional and derivative sterring for avoiding signal pillars
            cy = 0.05 #value used to affect pd steering based on how close the pillar is based on its y coordinate
            
        #any other combination of number of pillars
        else:
            
            LED2(0, 0, 0)
            
            endConst = 30
            
            cKp = 0.25 #value of proportional for proportional steering for avoiding signal pillars
            cKd = 0.25 #value of derivative for proportional and derivative sterring for avoiding signal pillars
            cy = 0.08 #value used to affect pd steering based on how close the pillar is based on its y coordinate, 0.15

# -----------------{ orange line contour processing }---------------

        #used in the three point turn, as in our turn we check if we can see a pillar after seeing the orange or blue line we use temp to ensure it only checks for the iteration one time
        temp = False

        #number of orange contours detected
        oCount = 0

        for i in range(len(contours_orange)):
          cnt = contours_orange[i]
          area = cv2.contourArea(cnt)
          
          if area > 100:
              oCount += 1

              #if the turn direction hasn't been changed yet change the turn direction to right
              if turnDir == "none":
                  turnDir = "right"

              #if the turn direction is right
              if turnDir == "right" and not parkingR and not parkingL:

                  #set tTurn and tSignal to true to indicate a right turn
                  rTurn = True
                  tSignal = True
            
              #check for three point turn
              elif not temp:
                  
                  temp = True
                  
                  #this indicates there is a pillar at the very end of the second lap
                  if (t == 8 or mReverse) and cTarget == redTarget and reverse == False:
                      tempR = True
                
                  #no pillar after turn, so if previous pillar was red perform a three point turn
                  elif (t == 8 or mReverse) and cTarget == 0 and reverse == False and lastTarget == redTarget:
                      reverse = True
                    
                      #add a pause so the car can fully complete three laps
                      time.sleep(1)
                    
                      turnDir = "right"
        
        #if orange line isnt detected anymore reset temp
        if oCount == 0:
            temp = False

# -----------------{ blue line contour processing }--------------

        #number of blue contours detected
        bCount = 0

        #iterate through blue contours
        for i in range(len(contours_blue)):
          cnt = contours_blue[i]
          area = cv2.contourArea(cnt)

          if area > 100:
              bCount += 1
              #if the turn direction hasn't been changed yet change the turn direction to left
              if turnDir == "none":
                  turnDir = "left" 
            
              #if the turn direction is left
              if turnDir == "left" and not parkingL and not parkingR:

                  #set tTurn and tSignal to true to indicate a left turn
                  lTurn = True
                  tSignal = True
                  
              #check for three point turn
              elif not temp:
                  
                  temp = True
                  
                  #this indicates there is a pillar at the very end of the second lap
                  if (t == 8 or mReverse) and cTarget == redTarget and reverse == False:
                      tempR = True

                  #no pillar after turn, so if previous pillar was red perform a three point turn
                  elif (t == 8 or mReverse) and cTarget == 0 and reverse == False and lastTarget == redTarget:
                      reverse = True
                    
                      #add a pause so the car can fully complete three laps
                      time.sleep(1)
                    
                      turnDir = "left"
                      
                      if lotLocation == 1:
                          lotLocation = 3
                      elif lotLocation == 3:
                          lotLocation = 1
                
        #if blue line isnt detected anymore reset temp
        if bCount == 0:
            temp = False
        
        print(turnDir)

# ------------------------------------------------------------{ final parking algorithm }-------------------------------------------------------------------------
        heading, tHeading = berryIMU.compute_heading()
        if headingIndex == 2 and heading > 320:
            heading -= 360
            
        print("heading and target heading:", heading, tHeading, targetHeadings[headingIndex])
        
        if t > 4 and lotLocation == 0:
            for i in range(4):
                if lotAreas[i] > lotAreas[lotLocation]:
                    lotLocation = i
            
            lotLocation += 1
        
        print("location of lot:", lotLocation)
                
        pKp = 1
        pKd = 0.1
        #if no pillar is detected (tempParking) and turns are greater than or equal to 12 change it so the car will always stay on the outside of signal pillars
        #do the same if pl and pr are true (debugging mode)
        if ((t >= 12 and t - 11 >= lotLocation and tempParking) or pl or pr):
            
            error = heading - targetHeadings[headingIndex]
            
            angle = int(straightConst + pKp * error + pKd * (error - pGyroError))
            
            print(angle)
            
            pGyroError = error
            
            #angle = 87
        
        areaFront = 0
                
        for i in range(len(contours_parking)):
            cnt = contours_parking[i]
            areaFront = max(cv2.contourArea(cnt), areaFront)

        #if the area of the wall in front is above a limit stop as we are very close to the wall
        if areaFront > 7500:
            time.sleep(0.3)
            write("servo", straightConst)
            time.sleep(0.5)
            stopCar()
            break
        
        if tempParking or pr or pl or (5 > t > 0): 
            maxAreaL = 0 #biggest magenta contour on left ROI
            leftY = 0
            maxAreaR = 0 #biggest magenta contour on right ROI
            rightY = 0
            areaFront = 0 #area of black contour on main ROI
            
            for i in range(len(contours_magenta_l)):
                cnt = contours_magenta_l[i]
                area = cv2.contourArea(cnt)
                
                if area > 100:
                
                    #get width, height, and x and y coordinates by bounding rect
                    approx=cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt,True),True)
                    x,y,w,h=cv2.boundingRect(approx)

                    #since the x and y coordinates are the coordinates just in the ROI, add to the x and y values to make it the proper coordinates on the overall image
                    x += ROI1[0]
                    y += ROI1[1] + h
                
                    if area > maxAreaL:
                        maxAreaL = area
                        leftY = y
            
            for i in range(len(contours_magenta_r)):
                cnt = contours_magenta_r[i]
                area = cv2.contourArea(cnt)
                
                if area > 100:
                
                    #get width, height, and x and y coordinates by bounding rect
                    approx=cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt,True),True)
                    x,y,w,h=cv2.boundingRect(approx)

                    #since the x and y coordinates are the coordinates just in the ROI, add to the x and y values to make it the proper coordinates on the overall image
                    x += ROI2[0]
                    y += ROI2[1] + h
                
                    if area > maxAreaR:
                        maxAreaR = area
                        rightY = y
             
            print("right parking lane:", maxAreaR, rightY)
            
            if 5 > t > 0: 
                if turnDir == "right":
                    lotAreas[t-1] = max(maxAreaL, lotAreas[t-1])
                elif turnDir == "left":
                    lotAreas[t-1] = max(maxAreaR, lotAreas[t-1])
                
            #conditions for initiating parking on the left side
            if leftY >= 240 and (t >= 12 or pl):
                
                if not parkingL and not parkingR:
                    write("dc", 1640)
                    parkingL = True
                    
                    timeStraight = 3
                    startTime = time.time()
                    
                    if pl or debug: 
                        LED1(255, 0, 255)

            #conditions for initiating parking on the right side
            if rightY >= 240 and (t >= 12 or pr):
                
                if not parkingL and not parkingR:
                    write("dc", 1640)
                    parkingR = True
                    
                    timeStraight = 3
                    startTime = time.time()
                    
                    if pr or debug: 
                        LED1(255, 0, 255)
                        
            if curTime != -1: 
                curTime = time.time()
                
            if curTime - startTime >= timeStraight and curTime != -1 and (parkingR or parkingL):
                
                if parkingR:
                    '''
                    write("dc", 1500)
                    time.sleep(0.1)
                    write("dc", reverseSpeed)
                    time.sleep(0.1)
                    
                    
                    write("servo", sharpLeft)
                    
                    time.sleep(1.5)
                    write("dc", 1640)
                    '''
                    write("dc", 1500)
                    time.sleep(0.1)
                    write("dc", 1360)
                    write("servo", 127)
                    time.sleep(4)
                    write("dc", 1500)
                    time.sleep(0.1)
                    write("dc", 1640)
                    write("servo", 47)
                    time.sleep(3)
                    #write("dc", 1500)
                    write("servo", 87)
                    
                    headingIndex -= 1
                    
                    if headingIndex < 0:
                        headingIndex = 3
                
                elif parkingL:
                    write("dc", 1500)
                    time.sleep(0.1)
                    write("dc", 1360)
                    write("servo", 47)
                    time.sleep(4)
                    write("dc", 1500)
                    time.sleep(0.1)
                    write("dc", 1640)
                    write("servo", 127)
                    time.sleep(3)
                    #write("dc", 1500)
                    write("servo", 87)
                    
                    headingIndex += 1
                    
                    if headingIndex > 3:
                        headingIndex = 0
                        
                    '''
                    write("dc", 1500)
                    time.sleep(0.1)
                    write("dc", reverseSpeed)
                    time.sleep(0.1)
                    write("servo", sharpRight)
                    
                    time.sleep(2.5)
                    
                    #break
                    write("dc", 1640)
                    write("servo", straightConst)
                    
                    headingIndex += 1
                    
                    if headingIndex > 3:
                        headingIndex = 0
                    '''
                    
                curTime = -1
            '''
            if curTime == -1:
                if parkingR: 
                    angle = sharpRight
            '''

# ------------------------------------------------------------{ servo motor calculations based on pillars and walls}-------------------------------------------------------------------------

# -----------------{ no pillar detected }--------------
        if cTarget == 0 and t < 11 + lotLocation and not pr and not pl:

            #once a pillar is no longer detected after 2 laps (8 turns) have been completed begin the three point turn by changing the cars turn direction and setting reverse to true to start the turn
            if tempR:
                if turnDir == "right":
                    turnDir = "left"
                else:
                    turnDir = "right"
                
                #flip location as we are heading in the opposite direction
                if lotLocation == 1:
                    lotLocation = 3
                elif lotLocation == 3:
                    lotLocation = 1
                    
                reverse = True
                tempR = False


            #set tempParking to indicate the car should be parking and change pillars to be passed on the outside
            if t == 12:
                if turnDir == "right": 
                    redTarget = greenTarget
                else: 
                    greenTarget = redTarget

                tempParking = True

            
            LED1(0, 0, 0)

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
            
        
# -----------------{ pillar detected }--------------
#and not pl and not pr

        elif t < 12 + lotLocation and (cTarget == redTarget or cTarget == greenTarget) and not parkingR and not parkingL and not (pr and t == 1):
            
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

                #add a turn
                t += 1
                
                if lTurn:
                    headingIndex += 1
                      
                    if headingIndex > 3:
                        headingIndex = 0
                          
                elif rTurn:
                    headingIndex -= 1
                  
                    if headingIndex < 0:
                        headingIndex = 3
                
                #reset lTurn and rTurn booleans to indicate the turn is over
                lTurn = False
                rTurn = False
            
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
        else:
            LED1(0, 0, 0)

# ------------------------------------------------------------{ three point turn logic }-------------------------------------------------------------------------

        if reverse:

            LED1(0, 0, 255)
            
            if turnDir == "left": #car is turning right before the change in direction
                
                write("servo", sharpLeft)
                
                #stop turning once right in front of wall
                if areaFront > 2000:
                    write("dc", 1500)
                    write("servo", sharpRight)
                    write("dc", reverseSpeed)
                    time.sleep(1.5)
                    write("dc", 1500)
                    time.sleep(0.1)
                    write("dc", 1650)
                else:
                    continue

            else: #turn sequence if turn direction is left before change in direction
                
                write("servo", sharpLeft)
                
                #stop turning once right in front of wall
                if areaFront > 2000:
                    write("dc", 1500)
                    write("servo", sharpRight)
                    write("dc", reverseSpeed)
                    time.sleep(2)
                    write("dc", 1500)
                    time.sleep(0.1)
                    write("dc", 1650)
                else:
                    continue
                
            reverse = False
            
            #end the program if we just wanted to see the three-point turn
            if mReverse:
                stopCar()
                break
            
# ------------------------------------------------------------{ final processing before writing angle to servo motor }-------------------------------------------------------------------------
        
        if angle != prevAngle or rTurn or lTurn:
          
            diff = initGyro(targetHeadings, heading, "diff")
            index = initGyro(targetHeadings, heading, "diff")
            
            #if area of wall is large enough and turning line is not detected end turn
            if ((rightArea >= exitThresh and rTurn) or (leftArea >= exitThresh and lTurn) or (tempParking and (lTurn or rTurn) and diff <= 15 and headingIndex != index)) and not tSignal:
                
                  if lTurn:
                      headingIndex += 1
                      
                      if headingIndex > 3:
                          headingIndex = 0
                          
                  elif rTurn:
                      headingIndex -= 1
                  
                      if headingIndex < 0:
                          headingIndex = 3
                        
              
                  #set turn variables to false as turn is over
                  lTurn = False 
                  rTurn = False
              
                  #increase number of turns by 1
                  #in this case the turn is ended without seeing a pillar
                  t += 1
                  
                  #if 2 laps have been completed or mReverse is true (debugging mode) and the last pillar is red initiate a regular three point turn
                  if (t == 8 or mReverse) and lastTarget == redTarget:
                      reverse = True
                      
                      #flip lot location as direction changed
                      if lotLocation == 1:
                          lotLocation = 3
                      elif lotLocation == 3: 
                          lotLocation = 1
                      
                      if turnDir == "right":
                         turnDir = "left"
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
        

# ------------------------------------------------------------{ reset variables }------------------------------------------------------------------------

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

# ------------------------------------------------------------{ debugging information }-------------------------------------------------------------------------

        if debug: 
        
            #if q is pressed break out of main loop and stop the car
            if cv2.waitKey(1)==ord('q'):
                stopCar() 
                break
            
            #display regions of interest
            displayROI(ROIs)

            #show image
            '''
            print("turns", t)
            print(turnDir)
            print("areas:", leftArea, rightArea)
            '''

            cv2.imshow("finalColor", img) 

cv2.destroyAllWindows()

