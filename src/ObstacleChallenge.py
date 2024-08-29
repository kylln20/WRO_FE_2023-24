# ------------------------------------------------------------{ libraries }-------------------------------------------------------------------------

import cv2
import sys
sys.path.append('/home/pi/TurboPi/')
from picamera2 import Picamera2
import libcamera
import time
import RPi.GPIO as GPIO
import numpy as np
import HiwonderSDK.Board as Board
import math

# ------------------------------------------------------------{ function declarations }-------------------------------------------------------------------------

#function used to send signals to arduino to control speeds of the dc motor and the angle of the servo motor
def write(value):
    
    if 180 > value > 0: 
        pulseWidth = int(11.1*value+500)
        Board.setPWMServoPulse(1, pulseWidth, 1)
    
    else: 
        Board.setPWMServoPulse(5, value, 100)

#function which displays the Regions of Interest on the image
def display_roi(color):
    for ROI in ROIs: 
        image = cv2.line(img, (ROI[0], ROI[1]), (ROI[2], ROI[1]), color, 4)
        image = cv2.line(img, (ROI[0], ROI[1]), (ROI[0], ROI[3]), color, 4)
        image = cv2.line(img, (ROI[2], ROI[3]), (ROI[2], ROI[1]), color, 4)
        image = cv2.line(img, (ROI[2], ROI[3]), (ROI[0], ROI[3]), color, 4)

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
def stop_car():

    write(87)
    
    write(1500)
    
    LED1(0, 0, 0)
    LED2(0, 0, 0)
    
    cv2.destroyAllWindows()

#returns contours of a specific hsv range of a specific region of interest
def contours(hsvRange, ROI): 
    lower_mask = np.array(hsvRange[0])
    upper_mask = np.array(hsvRange[1])

    mask = cv2.inRange(img_hsv, lower_mask, upper_mask)
   
    kernel = np.ones((7, 7), np.uint8)
    
    eMask = cv2.erode(mask, kernel, iterations=1)
    dMask = cv2.dilate(eMask, kernel, iterations=1)
    
    #cv2.imshow("mask", dMask)

    contours = cv2.findContours(dMask[ROI[1]:ROI[3], ROI[0]:ROI[2]], cv2.RETR_EXTERNAL,
    cv2.CHAIN_APPROX_SIMPLE)[-2]

    return contours

#returns the area of the largest contour in a group of contours
def max_contour(contours): 
    maxArea = 0

    for cnt in contours:
        area = cv2.contourArea(cnt)
        
        maxArea = max(area, maxArea)

    return maxArea

def return_contour(contours): 
    maxArea = 0
    cont = 0

    for cnt in contours:
        area = cv2.contourArea(cnt)
        
        if area > maxArea:
            cont = cnt
            maxArea = area

    return cont

def contour_location(contours):
    
    maxArea = 0
    maxX = 0
    maxY = 0

    for cnt in contours:
        area = cv2.contourArea(cnt)
        
        #get width, height, and x and y coordinates by bounding rect
        approx=cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt,True),True)
        x,y,w,h=cv2.boundingRect(approx)

        #since the x and y coordinates are the coordinates just in the ROI, add to the x and y values to make it the proper coordinates on the overall image
        x += ROI3[0] + w // 2
        y += ROI3[1] + h 

        
        
        if area > maxArea:
            maxArea = area
            maxX = x
            maxY = y

    return maxArea, maxX, maxY

#takes in an array of commands (dc, servo, sleep) and executes each
def multi_write(sequence):

    for action in sequence: 
        
        #delay commands
        if action < sharpRight: 
            time.sleep(action)
        else: 
            write(action)

#takes in a dictionary of values to print for debugging
def display_variables(variables): 

    names = list(variables.keys())

    for i in range(len(names)):
        name = names[i]
        value = variables[name]
        # Print each item on a new line
        print(f"{name}: {value}", end="\r\n")
    
    # Move the cursor up to overwrite the previous lines
    print("\033[F" * len(names), end="")
    
    time.sleep(0.1)

if __name__ == '__main__':
    
# ------------------------------------------------------------{ initialization of variables }-------------------------------------------------------------------------

    #initialize camera
    
    picam2 = Picamera2()
    
    picam2.set_controls({"AeEnable": True})
    picam2.preview_configuration.main.size = (640,480)
    picam2.preview_configuration.main.format = "RGB888"
    picam2.preview_configuration.controls.FrameRate = 30
    picam2.preview_configuration.align()
    picam2.configure("preview")
    
    picam2.start()
    
    eTurnMethod = ""
    lapsComplete = False

    #set the target x coordinates for each red and green pillar
    redTarget = 110 
    greenTarget = 530

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
    
    sTime = 0
    sec = 0
    
    noPillar = False

    #makes sure the car begins to park when no pillar is detected
    tempParking = False
    
    #counts the previous number of red and green pillars
    prevPillarCountR = 0
    prevPillarCountG = 0
    
    #determines the offset from the bottom of the ROI when the car should stop seeing a pillar
    endConst = 30
    
    #distance from a pillar to the car
    pDist = 0
    
    ignore = False
    prevIgnore = False
    
    #regions of interest
    #ROI1: for finding left lane
    #ROI2: for finding right lane
    #ROI3: for finding signal pillars
    #ROI4: for detecting blue and orange lines on mat
    # order: x1, y1, x2, y2
    ROI1 = [0, 165, 330, 255]
    ROI2 = [330, 165, 640, 255]
    ROI3 = [redTarget - 40, 110, greenTarget + 40, 335] #+- 40
    ROI4 = [200, 250, 440, 300]
    
    
    ROIs = [ROI1, ROI2, ROI3, ROI4]
    
    
    
    #use in yellow lighting
    '''
    rBlack = [[0, 0, 0], [180, 255, 50]]
    rBlue = [[100, 100, 100], [135, 255, 255]]
    rRed = [[166, 185, 50], [180, 255, 255]]
    rGreen = [[58, 62, 55], [96, 255, 255]]
    rMagenta = [[150, 180, 50], [166, 255, 255]]
    rOrange = [[0, 100, 175], [25, 255, 255]]
    rWhite = [[0, 0, 150], [180, 28, 255]]
    '''
    
    #mask header
    rBlack = [[0, 0, 0], [180, 255, 50]]
    rBlue = [[100, 100, 100], [135, 255, 255]]
    rRed = [[171, 175, 50], [180, 255, 255]]
    rGreen = [[58, 62, 55], [96, 255, 255]]
    rMagenta = [[160, 140, 50], [170, 255, 255]]
    rOrange = [[0, 100, 175], [25, 255, 255]]
    rWhite = [[0, 0, 150], [180, 28, 255]]
    
    
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
  
    angle = 87 #variable for the current angle of the car
    prevAngle = angle #variable tracking the angle of the previous iteration
    tDeviation = 50 #value used to calculate the how far left and right the car turns during a turn, could actually be 50
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

    #temp is used to make sure a three-point turn is only checked at one specific point during a turn
    temp = False
    
    t = 7 #tracks number of turns
    
    t2 = 0
    prevT2 = 0
    
    maxR = 0
    
    tSignal = False #boolean that makes sure that a pillar doesn't affect a turn too early
    
    LED1(255, 0, 0)
    
    time.sleep(0.5)
    
    write(1500) 

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
            turnDir = "right"
            
        if sys.argv[1].lower() == "parkingl": #for testing parking on the left side
            redTarget = greenTarget
            tempParking = True
            pl = True
            ##sharpRight = straightConst - 50
            #sharpLeft = straightConst + 50
        elif sys.argv[1].lower() == "parkingr": #for testing parking on the right side
            greenTarget = redTarget
            tempParking = True
            pr = True
            #sharpRight = straightConst - 50
            #sharpLeft = straightConst + 50
        elif sys.argv[1].lower() == "turn": #for testing three-point turns
            mReverse = True
        elif sys.argv[1].lower() == "steer": #for stationary testing
            #tempParking = True
            #redTarget = greenTarget
            speed = 1500
        
    #if no mode is specified, assume the regular program and wait for button put
    else:
        buzz()
        while GPIO.input(key2_pin) == GPIO.HIGH:
            pass
        
    LED1(0, 0, 0)
    time.sleep(0.5)

    #write initial values to car
    write(angle)
    time.sleep(0.1)
    write(1680)
    time.sleep(0.1)
    write(speed) 
    

# ------------------------------------------------------------{ main loop }-------------------------------------------------------------------------
    while True:

# ------------------------------------------------------------{ contour detection of walls, pillars, and orange and blue lines }-------------------------------------------------------------------------
        
        if sTime != 0 and not lapsComplete:
            if time.time() - sTime > s:
                multi_write([1500, 5, 1680, 0.1, 1650])
                lapsComplete = True
                
        #reset rightArea, and leftArea variables
        rightArea, leftArea, areaFront, areaFrontMagenta = 0, 0, 0, 0

        #get an image from pi camera
        img = picam2.capture_array()
        
        # convert from BGR to HSV
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        contours_left = contours(rBlack, ROI1)
        contours_right = contours(rBlack, ROI2)
        
        #convert to grayscale
        imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        #threshold image
        ret, imgThresh = cv2.threshold(imgGray, 55, 255, cv2.THRESH_BINARY_INV)
        
        #find black contours in the parking region of interest to determine when to stop during parking
        contours_parking, hierarchy = cv2.findContours(imgThresh[ROI4[1]:ROI4[3], ROI4[0]:ROI4[2]], 
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        #iterate through every contour in both the left and right region of interest and take the largest one in each

        leftArea = max_contour(contours_left)
        rightArea = max_contour(contours_right)
        areaFront = max_contour(contours_parking)
            
        #find contours 
        contours_red = contours(rRed, ROI3)
        contours_green = contours(rGreen, ROI3)
        contours_blue = contours(rBlue, ROI4)
        contours_orange = contours(rOrange, ROI4)
        #contours_black = contours(rBlack, ROI3)
        #contours_white = contours(rWhite, ROI3)
        ''' 
        max_white = return_contour(contours_white)
        '''


# ------------------------------------------------------------{ processing contours }-------------------------------------------------------------------------
        
        #count number of red and green pillars
        num_pillars_g = 0
        num_pillars_r = 0
        
        #distance of the pillar from the bottom middle of the screen
        pDist = 100000
        pArea = 0

# -----------------{ green / red signal pillar contour processing }--------------

        arr1 = [contours_green, contours_red]
        targets = [greenTarget, redTarget]
        
        ignore = False

        #iterate through both lists of contours
        for i in range(len(arr1)): 
            #process each contour in each list
            for x in range(len(arr1[i])):
                cnt = arr1[i][x]
                area = cv2.contourArea(cnt)

                if area > 100:
                    
                    #get width, height, and x and y coordinates by bounding rect
                    approx=cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt,True),True)
                    x,y,w,h=cv2.boundingRect(approx)

                    #since the x and y coordinates are the coordinates just in the ROI, add to the x and y values to make it the proper coordinates on the overall image
                    x += ROI3[0] + w // 2 
                    y += ROI3[1] + h
                    
                    
                    '''
                    if ((y < by or x < bx) and leftArea > 10000) or ((y < by or x > bx) and rightArea > 10000):
                        continue
                    '''
                    '''
                    points_inside = 0
                    closest_dist = -1000000
                    
                    for point in cnt:
                        point = tuple(point[0])
                        for wCont in contours_white:
                            distance = cv2.pointPolygonTest(wCont, point, True)
                            
                            closest_dist = max(closest_dist, distance)
                    
                    
                    if closest_dist < -10:
                        continue
                    '''
                    '''
                    points_inside = 0
                    closest_dist = -1000000
                    
                    for point in cnt:
                        point = tuple(point[0])
                        for wCont in contours_white:
                            distance = cv2.pointPolygonTest(wCont, point, True)
                            
                            if distance > -10:
                                points_inside += 1
                    ##print(points_inside)
                    
                    if points_inside < 1:
                        continue
                    '''
                    
                    
                    
                    #calculates the distance between the pillar and the bottom middle of the screen
                    temp_dist = round(math.dist([x, y], [320, 480]), 0)
                    if t == 8: 
                        print(temp_dist, area)
                    '''
                    if (t == 7 or (mReverse and t2 == t)) and area > 800:
                        ###print("within processing:", lastTarget)
                        lastTarget = targets[i]
                        ###print("within processing:", lastTarget)
                    '''
                    #if the pillar is close enough add it to the number of pillars
                    if temp_dist < 380 and not mReverse and t2 != 7 and not tempR: #395
                        if i == 0: 
                            num_pillars_g += 1
                        else: 
                            num_pillars_r += 1
                    
                    #if the pillar is too close, stop the car and reverse to give it enough space
                    if ((area > 6500 and ((x <= 420 and i == 0) or (x >= 220 and i == 1)))) and not pr and not pl:
                        LED2(255, 255, 0)
                        multi_write([straightConst, 0.1, 1500, 0.1, reverseSpeed, 0.5, speed])
                        ignore = True
                       # ##print("ignore set to true -------------------------------------", ignore)
                      ##  write(1500)
                    else:
                        LED2(0, 0, 0)
                    
                    #deselects current pillar if it comes too close to bottom of the screen
                    if y > ROI3[3] - endConst or (temp_dist > 370 and not mReverse and t2 != 7 and not tempR) or ((t2 == 7 or mReverse or tempR) and temp_dist > 380): #370
                        ###print("too far")
                        
                        continue
                        
                    #if the area of either wall is too big deselect the pillar to let the car come closer to the middle
                    ##if (leftArea > 13000 and (turnDir == "none" or turnDir == "left")) or (rightArea > 13000 and (turnDir == "none" or turnDir == "right")):
                        #break
                    
                    if leftArea > 13000 or rightArea > 13000:
                        continue

                    #draw rectangle around signal pillar
                    cv2.rectangle(img,(x - w // 2, y - h),(x+ w // 2,y),(0,0,255),2)

                    #if the y value is bigger than the previous contY value or within a range and has a bigger area update the data as this pillar is now the closest one
                    if temp_dist < pDist:
                        pArea = area
                        contY = y
                        contX = x
                        cTarget = targets[i]
                        pDist = temp_dist

            #draw contours of pillars for debugging
            if i == 0: 
                cv2.drawContours(img[ROI3[1]:ROI3[3], ROI3[0]:ROI3[2]], arr1[i], -1, (144, 238, 144), 3)
            else:
                cv2.drawContours(img[ROI3[1]:ROI3[3], ROI3[0]:ROI3[2]], arr1[i], -1, (144, 144, 238), 3)
                
# -----------------{ control variable manipulation based on number of pillars }--------------

        #change control variables if there are more than 2 pillars of the same colour, most likely meaning we are turning along an inside corner. Make the control variables less strong
        if (num_pillars_r >= 2 or num_pillars_g >= 2):
            
            if debug: LED2(255, 255, 255)
        
            endConst = 60
            
            cKp = 0.2 #value of proportional for proportional steering for avoiding signal pillars
            cKd = 0.2 #value of derivative for proportional and derivative sterring for avoiding signal pillars
            cy = 0.05 #value used to affect pd steering based on how close the pillar is based on its y coordinate
            
        #any other combination of number of pillars
        else: 
            
            if debug: LED2(0, 0, 0)     
            
            endConst = 30
            
            cKp = 0.25 #value of proportional for proportional steering for avoiding signal pillars
            cKd = 0.25 #value of derivative for proportional and derivative sterring for avoiding signal pillars
            cy = 0.08 #value used to affect pd steering based on how close the pillar is based on its y coordinate, 0.15

# -----------------{ orange / blue line contour processing }---------------
       # if ignore:
            ###print("beginning of orange / blue processing")
            
        arr2 = [contours_orange, contours_blue]
        directions = ["right", "left"]
        
        #iterate through both lists of contours
        for i in range(len(arr2)): 
            #number of contours detected
            count = 0
            
           # if ignore:
               ## ##print(i)

            #process each contour in each list
            for x in range(len(arr2[i])):
                
                ###print(ignore)

                    
                cnt = arr2[i][x]
                area = cv2.contourArea(cnt)
                
                if area > 100:
                    print(i)
                    count += 1

                    #set the turn direction
                    if turnDir == "none":
                        turnDir = directions[i]
                        ###print("setting direction:", turnDir)

                    #if the turn direction corresponds to the correct line colour (orange line means right turn, blue means left)
                    if turnDir == directions[i] and not parkingR and not parkingL:
                        
                        if ignore == False and prevIgnore == True:
                            ###print("turn documented")
                            #LED1(133, 45, 77)
                            t -= 1
                            ignore = False
                            
                        
                        t2 = t
                        
                        if t2 == 7 or mReverse:
                            ROI3 = [redTarget - 40, 90, greenTarget + 40, 335]
                            ROIs = [ROI1, ROI2, ROI3, ROI4]

                        if i == 0: 
                            rTurn = True
                        else:
                            lTurn = True

                        tSignal = True
                    
                    #check for three point turn, check at blue line when turning right and check at orange line when turning left
                    elif not temp and reverse != "done" and ((turnDir == "right" and i == 1) or (turnDir == "left" and i == 0)):
                        
                        ##print("--------------------------------------------------------------------------------")
                        
                        #write(1500)
                        #time.sleep(5)
                        temp = True
                        '''
                        #this indicates there is a pillar at the very end of the second lap
                        if (t2 == 7 or mReverse) and cTarget == redTarget and reverse == False:
                            #write(1500)
                            
                            
                            LED1(255, 0, 0)
                            
                            tempR = True
                        elif (t2 == 7 or mReverse) and cTarget == greenTarget and reverse == False:
                            reverse = "done"
                        '''
                        '''
                        #no pillar after turn, so if previous pillar was red perform a three point turn
                        elif (t2 == 7 or mReverse) and (cTarget == 0) and reverse == False and lastTarget == redTarget and tempR == False:
                            
                            t = 8 
                            
                            reverse = True
                            
                            ROI3 = [redTarget - 40, 110, greenTarget + 40, 335]
                            ROIs = [ROI1, ROI2, ROI3, ROI4]
                            
                            #add a pause so the car can fully complete the second lap
                            time.sleep(1)
                            write(straightConst)
                            time.sleep(0.25)
                            
                            ###print("turn 7:", turnDir)
                            turnDir = directions[i]
                            ###print("turn 7:", turnDir)
                        '''
                        
                        #write(1500)
            ###print("count:", count)
            #if colored line isnt detected anymore reset temp
            if count == 0:
                temp = False
                
        ###print("after loop")
# ------------------------------------------------------------{ final parking algorithm }------------------------------------------------------------------------      
            
        maxAreaL = 0
        leftY = 0
        maxAreaR = 0
        rightY = 0
        centerY = 0
        
        # detect magenta contours directly in front of car
        if reverse or pl or pr or tempParking:
            contours_magenta_c = contours(rMagenta, ROI4)
            
            areaFrontMagenta = max_contour(contours_magenta_c)
        
        if tempParking or pr or pl:
            
            #find magenta contours in left and right regions of interest
            contours_magenta_l = contours(rMagenta, ROI1)
            contours_magenta_r = contours(rMagenta, ROI2)
            contours_magenta_c = contours(rMagenta, ROI4)
                                    
            #for each array representing a set of contours: area, y-coordinate, Region of Interest
            info = [[0, 0, ROI1], [0, 0, ROI2], [0, 0, ROI4]]
            conts = [contours_magenta_l, contours_magenta_r, contours_magenta_c]

            #finds the largest magenta contour in the left and right regions of interest and the y-coordinates
            for i in range(len(conts)): 
                for x in range(len(conts[i])):
                    cnt = conts[i][x]
                    area = cv2.contourArea(cnt)
                    
                    if area > 100:
                    
                        #get width, height, and x and y coordinates by bounding rect
                        approx=cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt,True),True)
                        x,y,w,h=cv2.boundingRect(approx)

                        #since the x and y coordinates are the coordinates just in the ROI, add to the x and y values to make it the proper coordinates on the overall image
                        x += info[i][2][0]
                        y += info[i][2][1] + h
                        
                        #replace largest contour 
                        if area > info[i][0]:
                            info[i][0] = area
                            info[i][1] = y
            
            maxAreaL = info[0][0] #biggest magenta contour on left ROI
            leftY = info[0][1]
            maxAreaR = info[1][0] #biggest magenta contour on right ROI
            rightY = info[1][1]
            centerY = info[2][1]
            
            maxR = max(maxR, maxAreaR)
                
            #conditions for initiating parking on the left side
            if leftY >= 220 and maxAreaL > 100 and (t >= 13 or pl or pr):
                if not parkingL and not parkingR:
                    write(1640)
                    parkingL = True
                    
                    ###print(maxAreaL)
                    
                    if maxAreaL > 4500:
                        buzz() if debug else time.sleep(0.5)
                    elif maxAreaL < 1500 and t == 12:
                        write(1650)
                        write(straightConst)
                        time.sleep(1)
                        write(1640)
                    
                    
                ROI4 = [250, 250, 390, 300]
                ROIs = [ROI1, ROI2, ROI3, ROI4]
                    
            #conditions for initiating parking on the right side
            if rightY >= 240 and maxAreaR > 100 and (t >= 13 or pr or pl):
                if not parkingL and not parkingR:
                    
                    
                
                    if maxAreaR > 6000 and t == 12:
                        multi_write([straightConst, 1.5, 1500, 0.1, reverseSpeed, 0.1, sharpLeft, 0.5])
                        continue
                    
                    parkingR = True
                        
                    write(1640)
                
                ROI4 = [250, 250, 390, 300]
                ROIs = [ROI1, ROI2, ROI3, ROI4]
                
            if parkingR:
                
                
                
                #readjust if the parking lot is in front
                if centerY > 290:
                    if debug: LED1(255, 0, 0) 
                    multi_write([1500, 0.1, 1352, sharpLeft, 0.5, 1500])
                #turn right into parking lot
                else:
                    #if debug: LED1(255, 0, 255) 
                    multi_write([1640, sharpRight])
            
            elif parkingL:
                
                #readjust if the parking lot is in front
                if rightArea > 8000 and maxAreaR > 2000:
                    multi_write([1640, sharpRight, 1])
                elif centerY > 290 and areaFront < 3000:
                    if debug: LED1(255, 0, 0) 
                    multi_write([1500, 0.1, 1352, sharpRight, 0.5, 1500])
                #turn left into parking lot
                else:
                    #if debug: LED1(255, 0, 255)
                        
                    multi_write([1640, sharpLeft])
                        
            
            #if the area of the wall in front is above a limit stop as we are very close to the wall
            if areaFront > 3500:
                '''
                if maxR < 200:
                    reverse = "parking"
                    parkingL = False
                else:
                '''
                multi_write([straightConst, 1])
                stop_car()
                break
                    
# ------------------------------------------------------------{ servo motor calculations based on pillars and walls}-------------------------------------------------------------------------

# -----------------{ no pillar detected }--------------
        if cTarget == 0 and not parkingL and not parkingR:
            
            

            #start a three-point turn 
            if tempR:
                
                if reverse == "done":
                    tempR = False
                    continue
                #swap directions
                
                ###print("tempR:", turnDir)
                
                ROI3 = [redTarget - 40, 110, greenTarget + 40, 335]
                ROIs = [ROI1, ROI2, ROI3, ROI4]
                
                #if turnDir == "right": 
                    #write(straightConst)
                    #time.sleep(0.25)
                '''
                write(sharpLeft)
                time.sleep(1)
                '''
                turnDir = "left" if turnDir == "right" else "right"
                
                ###print("tempR:", turnDir)
            
                reverse = True
                tempR = False

            #change pillar targets so all are passed on the outside 
            if t == 12:

                redTarget, greenTarget = (greenTarget, greenTarget) if turnDir == "right" else (redTarget, redTarget)

                #used as an indication of when it is ok to park 
                tempParking = True
                
                #sharpRight = straightConst - 40
                #sharpLeft = straightConst + 40

            LED1(0, 0, 0)

            #calculate the difference in the left and right lane areas
            aDiff = rightArea - leftArea
            #calculate angle using PD steering
            angle = max(0, int(straightConst + aDiff * kp + (aDiff - prevDiff) * kd))
            print("wall", angle)

            #update the previous difference
            prevDiff = aDiff
            
            if leftArea > 2000 and rightArea > 2000 and abs(aDiff) < 500:
                noPillar = True
            
            if leftArea < 1000 and rightArea < 1000 and abs(aDiff) < 500 and not lTurn and not rTurn:
                angle = sharpRight if lastTarget == greenTarget else sharpLeft
                
        
            #if the areas of the two walls are above a threshold meaning we are facing the wall and are also close to the wall
            if leftArea > 6000 and rightArea > 6000:
                angle = sharpRight if lastTarget == greenTarget else sharpLeft
                
                #if the last pillar the car passed was green take a hard right turn and if the last pillar was red take a hard left turn
                if t == 8 and reverse == "done":
                    reverse = True
            
            
                
                
                
                
            
        
# -----------------{ pillar detected }--------------

        elif not parkingR and not parkingL:
            
            
            
            if tempR == "temp":
                tempR = True
            
            if debug:
                LED1(255, 0, 0) if cTarget == redTarget else LED1(0, 255, 0)
          
            #if car is in a turn and tSignal is false meaning no orange or blue line is detected currently, end the turn
            #in this case the turn is finished while still seeing a pillar
            if (lTurn or rTurn) and not tSignal:

                #reset prevError and prevDiff 
                prevError = 0
                prevDiff = 0

                #add a turn
                if reverse == False or reverse == "done":
                    eTurnMethod = "pillar"
                    #write(1500)
                     
                    t += 1
                    
                    if t == 8 or mReverse:
                        
                        ##print("check three point")
                            
                        '''
                        if cTarget == redTarget: 
                            tempR = "temp"
                        else:
                            reverse = True
                            ROI3 = [redTarget - 40, 110, greenTarget + 40, 335]
                            ROIs = [ROI1, ROI2, ROI3, ROI4]
                            
                            turnDir = "right" if turnDir == "left" else "left"
                        '''
                        if turnDir == "left":
                            
                            if noPillar:
                                if area > 200 and cTarget == redTarget:
                                    tempR = True
                                

                            elif lastTarget == redTarget:
                                if cTarget == redTarget:
                                    tempR = True
                                
                            elif lastTarget == greenTarget:
                                if cTarget == redTarget and area > 800:
                                    tempR = True
                        else:
                            if noPillar:
                                if area > 200 and cTarget == redTarget:
                                    tempR = True
                                    
                            elif lastTarget == redTarget:
                                if not (cTarget == greenTarget and area > 800):
                                    tempR = True
                                
                            elif lastTarget == greenTarget:
                                if cTarget == redTarget:
                                    tempR = True
                                
                            
                            
                    
                    if t == 12:
                        s = 4
                        sTime = time.time()
                
                #reset lTurn and rTurn booleans to indicate the turn is over
                lTurn = False
                rTurn = False
                
                noPillar = False
            
            #calculate error based on the difference between the target x coordinate and the pillar's current x coordinate
            error = cTarget - contX

            #calculate new angle using PD steering
            angle = int(straightConst + error * cKp + (error - prevError) * cKd)

            #adjust the angle further based on cy value, if error is 
            angle -= int(cy * (contY - ROI3[1])) if error <= 0 else -int(cy * (contY - ROI3[1]))

            #if the cTarget is equal to greenTarget or redTarget meaning we have a green pillar or a redPillar and the pillar has already exceeded its x target and is also close to the bottom of the ROI, set LastTarget to the respective target as we have basically passed the pillar. 
            if cTarget == greenTarget and contX > 320 and area > 1000:
                ##print("changed to green")
                lastTarget = greenTarget

            elif cTarget == redTarget and contX < 320 and area > 1000:
                lastTarget = redTarget


            #make sure angle value is over 2000 
            angle = max(0, angle)
            
        elif not parkingR and not parkingL:
            LED1(0, 0, 0)

# ------------------------------------------------------------{ three point turn logic }-------------------------------------------------------------------------

        if reverse != "done" and (reverse == True or reverse == "parking"):
            
            rTurn = False
            lTurn = False
            '''
            if reverse == "parking":
                multi_write([1500, 0.1, sharpRight, 0.1, reverseSpeed, 2, 1500, 0.1, 1650])
                reverse = "done"
                redTarget = 120
                greenTarget = 120
                continue
            '''
            #sharpRight = straightConst - 40 #the default angle sent to the car during a right turn
            #sharpLeft = straightConst + 40 #the default angle sent to the car during a left turn
            
           # LED1(0, 0, 255)
            ##print("parking", cTarget)
            if cTarget == 0 or cTarget == greenTarget:
                
                angle = sharpLeft
            
            # turnDir == "left": car is turning right before the change in direction
            #stop turning once right in front of wall
            if areaFront > 2000 or areaFrontMagenta > 1000:
                
                multi_write([1500, 0.1, sharpRight, 0.1, reverseSpeed, 1, 1500, 0.1, 1680, 0.1, 1650]) if turnDir == "left" else multi_write([1500, 0.1, sharpRight, 0.1, reverseSpeed, 1.5, 1500, 0.1, 1680, 0.1, 1650])
                reverse = "done"
            
                #end the program if we just wanted to see the three-point turn
                if mReverse:
                    stop_car()
                    break


            
            
# ------------------------------------------------------------{ final processing before writing angle to servo motor }-------------------------------------------------------------------------
        
        if angle != prevAngle or rTurn or lTurn:
            
            print("3", angle)
            
            #if area of wall is large enough and turning line is not detected end turn
            if ((rightArea >= exitThresh and rTurn) or (leftArea >= exitThresh and lTurn)) and not tSignal:
                        
                  #set turn variables to false as turn is over
                  lTurn = False 
                  rTurn = False
              
                  #increase number of turns by 1
                  #in this case the turn is ended without seeing a pillar
                  if reverse == False or reverse == "done":
                      eTurnMethod = "wall"
                      #write(1500)
                      t += 1
                      if t == 12: 
                          s = 2
                          sTime = time.time()
                  
                  #if 2 laps have been completed or mReverse is true (debugging mode) and the last pillar is red initiate a regular three point turn
                  
                  if (t == 8 or mReverse):
                      if turnDir == "left" and lastTarget == redTarget:
                            reverse = True
                            ROI3 = [redTarget - 40, 110, greenTarget + 40, 335]
                            ROIs = [ROI1, ROI2, ROI3, ROI4]
                            
                            turnDir = "right" if turnDir == "left" else "left"
                      
                  
                  
            # if a car is parking or performing a three-point turn
            if not parkingR and not parkingL:
                print("2", angle)

                #change angles for a right and left turn 
                
                if rTurn and cTarget == 0 and rightArea < 10000:
                    angle = straightConst - 25 if tempParking else sharpRight

                elif lTurn and cTarget == 0 and leftArea < 10000:
                    angle = sharpLeft
                
                angle = max(min(angle, sharpLeft), sharpRight)
                '''
                if t == 12 and not lapsComplete:
                    if leftArea > 2000 and rightArea > 2000:
                        lapsComplete = True
                        #multi_write([1500, 5, 1680, 0.1, 1650])
                '''
                #write the angle which is kept in the bounds of sharpLeft and sharpRight
                print(angle)
                write(angle)
        


# ------------------------------------------------------------{ debugging information }-------------------------------------------------------------------------

        if debug: 
        
            #if q is pressed break out of main loop and stop the car
            if cv2.waitKey(1)==ord('q'):
                stop_car() 
                break
            
            #display regions of interest
            display_roi((255, 204, 0))
            
            cv2.imshow("finalColor", img)
            
            ##gMask = cv2.inRange(img_hsv, np.array(rGreen[0]), np.array(rGreen[1]))
            #cv2.imshow("Green Mask", gMask)
            
            #rMask = cv2.inRange(img_hsv, np.array(rRed[0]), np.array(rRed[1]))
            #cv2.imshow("Red Mask", rMask)
            
            pColour = "r" if lastTarget == redTarget else "g"
            if cTarget == redTarget: 
                cpColour = "r"
            elif cTarget == greenTarget:
                cpColour = "g"
            else:
                cpColour = "n"
            
            if rTurn:
                turn_status = "r"
            elif lTurn:
                turn_status = "l"
            else:
                turn_status = "n"

            variables = {
                "left wall area": leftArea,
                "right wall area": rightArea,
                "left parking lot area": maxAreaL,
                "right parking lot area": maxAreaR,
                "front parking area": areaFrontMagenta,
                "three-point turn status": reverse,
                "last pillar": pColour,
                "current pillar": cpColour,
                "pillar area": pArea, 
                "turn direction": turnDir[0],
                "pillar distance": pDist, 
                "# turns": t,
                "t2": t2,
                "turn status": turn_status
                
            }

            #display_variables(variables)
        print(eTurnMethod)
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
        
        
        prevIgnore = ignore

cv2.destroyAllWindows()




