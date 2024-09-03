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
    
    elif 2000 > value > 1000:
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
    
    #time.sleep(0.1)

if __name__ == '__main__':
    
# ------------------------------------------------------------{ initialization of variables }-------------------------------------------------------------------------
    
    time.sleep(3)
    
    #initialize camera
    picam2 = Picamera2()
    
    picam2.set_controls({"AeEnable": True})
    picam2.preview_configuration.main.size = (640,480)
    picam2.preview_configuration.main.format = "RGB888"
    picam2.preview_configuration.controls.FrameRate = 30
    picam2.preview_configuration.align()
    picam2.configure("preview")
    
    picam2.start()
    
    #stores whether a turn was counted by seeing a wall or a pillar
    eTurnMethod = ""
    
    #boolean used to indicate whether the car has stopped after 3 laps
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
    
    #stores a stop time 
    sTime = 0
    
    #determines how many number of seconds after sTime are needed to stop
    s = 0

    #makes sure the car begins to park when no pillar is detected
    tempParking = False
    
    #determines the offset from the bottom of the ROI when the car should stop seeing a pillar
    endConst = 30
    
    #distance from a pillar to the car
    pDist = 0
    
    #stores whether a pillar or no pillar was seen, True for if a pillar was seen, False for if no pillar was seen. Is used to ensure no small interuptions have effect
    tList = []
    
    #used to make sure that when the car moves back, it doesn't count an extra turn
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
    
    #define colour masks
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
    
    t = 0 #tracks number of turns
    
    t2 = 0 #tracks number of turns but updates only when seeing a coloured line at a corner
    
    tSignal = False #boolean detecting whether a coloured line is currently seen, makes sure that a pillar doesn't affect a turn too early
    
    LED1(255, 0, 0)
    
    write(1500) # send initial value to esc for calibration

    time.sleep(8) #delay 8 seconds for the servo to be ready
    
    debug = False #boolean for debug mode
   
    #code for starting button
    key2_pin = 16
    
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(key2_pin, GPIO.IN, pull_up_down = GPIO.PUD_UP)
    
    LED1(0, 255, 0)
    
    #set mode based on program arguments
    if len(sys.argv) > 1:
        debug = True
        
        if sys.argv[1].lower() == "parkingl": #for testing parking on the left side
            t = 12
            turnDir = "right"
        elif sys.argv[1].lower() == "parkingr": #for testing parking on the left side
            t = 12
            turnDir = "left"
        elif sys.argv[1].lower() == "stop": #for testing parking on the left side
            t = 11
        elif sys.argv[1].lower() == "turn": #for testing three-point turns
            t = 7
        elif sys.argv[1].lower() == "steer": #for stationary testing
            speed = 1500
        
        
    #if no mode is specified, assume the regular program and wait for button input
    else:
        buzz()
        while GPIO.input(key2_pin) == GPIO.HIGH:
            pass
        
        time.sleep(3)
        
    LED1(0, 0, 0)
    
    #write initial valuess
    time.sleep(0.5)
    multi_write([angle, 0.5, 1670, 0.1, speed])

# ------------------------------------------------------------{ main loop }-------------------------------------------------------------------------
    while True:

# ------------------------------------------------------------{ contour detection of walls, pillars, and orange and blue lines }-------------------------------------------------------------------------
        
        #stop the car after the third lap
        if sTime != 0 and not lapsComplete:
            if time.time() - sTime > s:
                multi_write([1500, 2.5, 1680, 0.1, 1650])
                lapsComplete = True
                
        #reset rightArea, and leftArea variables
        rightArea, leftArea, areaFront, areaFrontMagenta = 0, 0, 0, 0

        #get an image from pi camera
        img = picam2.capture_array()
        
        # convert from BGR to HSV
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        #get contours of left and right walls
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
                    
                    #calculates the distance between the pillar and the bottom middle of the screen
                    temp_dist = round(math.dist([x, y], [320, 480]), 0)

                    #if the pillar is close enough add it to the number of pillars
                    if 160 < temp_dist < 380 : #395
                        if i == 0: 
                            num_pillars_g += 1
                        else: 
                            num_pillars_r += 1
                    
                    #if the pillar is too close, stop the car and reverse to give it enough space
                    if ((area > 6500 and ((x <= 420 and i == 0) or (x >= 220 and i == 1)))) and t < 13:
                        LED2(255, 255, 0)
                        
                        #move back 
                        multi_write([straightConst, 0.1, 1500, 0.1, reverseSpeed, 0.5, speed])
                        
                        #if car is supposed to stop increase the timer as we moved back
                        if s != 0:
                            s += 1.5
                        
                        #indicate that a turn should be ignored if an extra turn is counted
                        ignore = True
                        
                        #in the case of the car turning for a three point turn and seeing a pillar first instead of the wall, complete it as we have already switched directions
                        if reverse == "turning" and turnDir == "right":
                            reverse = "done"
                            turnDir = "left"
                            t += 1
                    
                    else:
                        LED2(0, 0, 0)
                    
                    #deselects current pillar if it comes too close to bottom of the screen
                    if y > ROI3[3] - endConst or (temp_dist > 370 and t2 != 7) or (t2 == 7 and temp_dist > 380): #370
                        continue
                    
                    #means we are to close to the wall so we need to move back to the middle
                    if leftArea > 13000 or rightArea > 13000:
                        continue

                    #draw rectangle around signal pillar
                    if debug: cv2.rectangle(img,(x - w // 2, y - h),(x+ w // 2,y),(0,0,255),2)
                    
                    #cv2.line(img, (x, 0), (x, 480), (0, 0, 255), 4)
                    #cv2.line(img, (targets[i], 0), (targets[i], 480), (0, 255, 0), 4)

                    #if the y value is bigger than the previous contY value or within a range and has a bigger area update the data as this pillar is now the closest one
                    if temp_dist < pDist:
                        pArea = area
                        contY = y
                        contX = x
                        cTarget = targets[i]
                        pDist = temp_dist

            #draw contours of pillars for debugging
                        
            if i == 0: 
                if debug: cv2.drawContours(img[ROI3[1]:ROI3[3], ROI3[0]:ROI3[2]], arr1[i], -1, (144, 238, 144), 3)
            else:
                if debug: cv2.drawContours(img[ROI3[1]:ROI3[3], ROI3[0]:ROI3[2]], arr1[i], -1, (144, 144, 238), 3)
                
                
# -----------------{ control variable manipulation based on number of pillars }--------------

        #lower control variables if there are more than 2 pillars of the same colour, most likely meaning we are turning along an inside corner.
        if (num_pillars_r >= 2 or num_pillars_g >= 2):
            
            LED2(255, 255, 255)
        
            endConst = 60
            
            cKp = 0.2 
            cKd = 0.2 
            cy = 0.05 
            
        #any other combination of number of pillars
        else: 
            
            LED2(0, 0, 0)     
            
            endConst = 30
            
            cKp = 0.25
            cKd = 0.25 
            cy = 0.08 

# -----------------{ orange / blue line contour processing }---------------

        arr2 = [contours_orange, contours_blue]
        directions = ["right", "left"]
        
        #iterate through both lists of contours
        for i in range(len(arr2)): 
            for x in range(len(arr2[i])):
                    
                cnt = arr2[i][x]
                area = cv2.contourArea(cnt)
                
                if area > 100:

                    #set the turn direction
                    if turnDir == "none":
                        turnDir = directions[i]

                    #if the turn direction corresponds to the correct line colour (orange line means right turn, blue means left)
                    if turnDir == directions[i] and not parkingR and not parkingL:
                        
                        #if the car just backed up, remove a turn because we already counted a turn
                        if ignore == False and prevIgnore == True:
                            t -= 1
                            ignore = False
                        
                        #update t2
                        t2 = t
                        
                        #for three-point turn, update region of interest to see farther
                        if t2 == 7:
                            ROI3 = [redTarget - 40, 90, greenTarget + 40, 335]
                            ROIs = [ROI1, ROI2, ROI3, ROI4]

                        #set either the right or left turn to true based on turn direction
                        if i == 0:
                            rTurn = True
                        else:
                            lTurn = True
                        
                        #indicate that the coloured line is seen
                        tSignal = True
                                   
# ------------------------------------------------------------{ final parking algorithm }------------------------------------------------------------------------      
            
        maxAreaL = 0 #area of left parking wall
        leftY = 0 #y-coordinate of left parking wall
        maxAreaR = 0 #area of right parking wall
        rightY = 0 #y-coordinate of right parking wall
        centerY = 0 #y-coordinate of any magenta contour in front of car
        
        # detect magenta contours directly in front of car
        if t == 8 or tempParking:
            
            #find largest magenta contour
            contours_magenta_c = contours(rMagenta, ROI4)
            areaFrontMagenta = max_contour(contours_magenta_c)
            
            #if area of magenta in front is too large right after the three-point turn, avoid it by turning left
            if areaFrontMagenta > 500 and t == 8:
                angle = sharpLeft
            
        if tempParking:
            
            #find magenta contours in left and right regions of interest
            contours_magenta_l = contours(rMagenta, ROI1)
            contours_magenta_r = contours(rMagenta, ROI2)
                                    
            #for each array representing a set of contours: area, y-coordinate, Region of Interest
            info = [[0, 0, ROI1], [0, 0, ROI2], [0, 0, ROI4]]
            conts = [contours_magenta_l, contours_magenta_r, contours_magenta_c]

            #finds the largest magenta contour in the left and right and center regions of interest and the y-coordinates
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
            
            maxAreaL = info[0][0] 
            leftY = info[0][1]
            maxAreaR = info[1][0]
            rightY = info[1][1]
            centerY = info[2][1]
                
            #conditions for initiating parking on the left side
            if leftY >= 220 and maxAreaL > 100 and t2 >= 12:
                if not parkingL and not parkingR:
                    write(1640)
                    parkingL = True
                    
                    #delay the turn if the area of the parking wall is too large
                    if maxAreaL > 4500:
                        buzz() if debug else time.sleep(0.5)
                
                #readjust region of interest for parking
                ROI4 = [230, 250, 370, 300]
                ROIs = [ROI1, ROI2, ROI3, ROI4]
                    
            #conditions for initiating parking on the right side
            if rightY >= 240 and maxAreaR > 100 and t2 >= 12:
                if not parkingL and not parkingR:
                    
                    parkingR = True
                    write(1640)
                    
                    #delay the turn if the area of the parking wall is too large
                    if maxAreaR > 6500:
                        buzz() if debug else time.sleep(0.5)
                
                #readjust region of interest for parking
                ROI4 = [250, 250, 390, 300]
                ROIs = [ROI1, ROI2, ROI3, ROI4]
                
            if parkingR:
                
                #readjust by backing up if the parking lot is in front
                if centerY > 290:
                    LED1(255, 0, 0) 
                    multi_write([1500, 0.1, 1352, sharpLeft, 0.5, 1500])
                #turn right into parking lot
                else:
                    multi_write([1640, sharpRight])
            
            elif parkingL:
                
                #if car is too for left turn back to the right
                if rightArea > 8000 and maxAreaR > 2000:
                    multi_write([1640, sharpRight, 1])
                #readjust by backing up if the parking lot is in front
                elif centerY > 290 and areaFront < 3000:
                    LED1(255, 0, 0) 
                    multi_write([1500, 0.1, 1352, sharpRight, 0.5, 1500])
                #turn left into parking lot
                else:
                    multi_write([1640, sharpLeft])
                    
            #if the area of the wall in front is above a limit stop as we are very close to the wall
            if areaFront > 3500:
                multi_write([straightConst, 0.2, 1640, 1])
                stop_car()
                break
                    
# ------------------------------------------------------------{ servo motor calculations based on pillars and walls}-------------------------------------------------------------------------
        
        #if the initial three-point turn wasn't enough to turn fully to the other direction
        if t == 8 and areaFront > 4000 and reverse == "done":
            turnDir = "right" if turnDir == "left" else "left"
            reverse = True
            
# -----------------{ no pillar detected }--------------
        if cTarget == 0 and not parkingL and not parkingR:

            #change pillar targets so all are passed on the outside 
            if t == 12 and not tempParking and all(target == False for target in tList[:2]):
                
                redTarget, greenTarget = (greenTarget, greenTarget) if turnDir == "right" else (redTarget, redTarget)

                #used as an indication of when it is ok to park 
                tempParking = True

            LED1(0, 0, 0)

            #calculate the difference in the left and right lane areas
            aDiff = rightArea - leftArea
            #calculate angle using PD steering
            angle = max(0, int(straightConst + aDiff * kp + (aDiff - prevDiff) * kd))

            #update the previous difference
            prevDiff = aDiff
            
            #if both areas are small enough change angle based on last pillar to help make it easier to pass pillars
            if leftArea < 1000 and rightArea < 1000 and abs(aDiff) < 500 and not lTurn and not rTurn:
                angle = sharpRight if lastTarget == greenTarget else sharpLeft

            
# -----------------{ pillar detected }--------------

        elif not parkingR and not parkingL:     
            
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
                     
                    t += 1
                    
                    #check for three-point turn by checking the last pillar, turn direction, and area 
                    if t == 8:
                        
                        print(contX, pDist, pArea, lastTarget, cTarget)
                        
                        if turnDir == "left":
                            
                            #means we are taking a wider turn into the corner
                            if lastTarget == redTarget or lastTarget == 0:
                                if not (cTarget == greenTarget and area > 250):
                                    reverse = True
                            
                            #means weare taking a tighter turn into the corne 
                            elif lastTarget == greenTarget or lastTarget == 0:
                                if cTarget == redTarget and pArea > 400:
                                    
                                    #if pillar is far away turn right away, else wait to pass the current pillar
                                    if pArea < 400: 
                                        reverse = "turning"
                                    else:
                                        reverse = True
                                    
                        else:
                            
                            #means we are taking a tighter turn into the corner
                            if lastTarget == redTarget or lastTarget == 0:
                                if not (cTarget == greenTarget and (pArea > 600 or contX > 550)):
                                    
                                    #if pillar is far away turn right away, else wait to pass the current pillar
                                    if pArea < 400 and contX < 550:
                                        reverse = "turning"
                                    else: 
                                        reverse = True
                            
                            #means we are taking a wider turn into the corner
                            elif lastTarget == greenTarget or lastTarget == 0:
                                if cTarget == redTarget:
                                    reverse = True
                        
                        #reset ROI3
                        if reverse == False:
                            ROI3 = [redTarget - 40, 110, greenTarget + 40, 335]
                            ROIs = [ROI1, ROI2, ROI3, ROI4]
                    
                    #after three laps set a timer for 3.75 in order to stop in middle of starting section
                    if t == 12:
                        s = 3.75
                        sTime = time.time()
                
                #set turns to false as the turn ended
                lTurn = False
                rTurn = False
            
            #calculate error based on the difference between the target x coordinate and the pillar's current x coordinate
            error = cTarget - contX

            #calculate new angle using PD steering
            angle = int(straightConst + error * cKp + (error - prevError) * cKd)

            #adjust the angle further based on cy value, if error is 
            angle -= int(cy * (contY - ROI3[1])) if error <= 0 else -int(cy * (contY - ROI3[1]))

            #if the pillar is large enough and the x-coordinate is close enough to its target set lastTarget to the current pillars target 
            if cTarget == greenTarget and contX > 320 and area > 1000:
                lastTarget = greenTarget

            elif cTarget == redTarget and contX < 320 and area > 1000:
                lastTarget = redTarget

            #make sure angle value is over 2000 
            angle = max(0, angle)
            
        elif not parkingR and not parkingL:
            LED1(0, 0, 0)
            
        #keep track of whether a pillar is seen for the last 10 frames
        if len(tList) == 10:
            tList.append(True if cTarget != 0 else False)
            tList.pop(0)
        else:
            tList.append(True if cTarget != 0 else False)
# ------------------------------------------------------------{ three point turn logic }-------------------------------------------------------------------------

        if reverse != "done" and (reverse == True or reverse == "turning"):
            
            LED2(0, 0, 255)
            
            #make sure turns are false
            rTurn = False
            lTurn = False
            
            #reset ROI3
            ROI3 = [redTarget - 40, 110, greenTarget + 40, 335]
            ROIs = [ROI1, ROI2, ROI3, ROI4]
            
            #if anything but a red pillar is seen turn left
            if cTarget == 0 or cTarget == greenTarget:
                angle = sharpLeft
            
            #if a pillar wasn't seen for the last 10 frames keep the car turning left until it sees the parking lot or wall in front
            if all(target == False for target in tList) and reverse != "turning":
                reverse = "turning"
                
            if reverse == "turning":
                angle = sharpLeft
            
            #stop turning once right in front of wall or parking lot
            if areaFront > 2000 or areaFrontMagenta > 750:
                
                #reverse car
                delay = 1.5 if areaFront > 2000 else 2
                multi_write([1500, 0.1, sharpRight, 0.1, reverseSpeed, delay, 1500, 0.1, 1680, 0.1, 1650])
                
                #change direction and end turn
                turnDir = "right" if turnDir == "left" else "left"
                reverse = "done"
                
# ------------------------------------------------------------{ final processing before writing angle to servo motor }-------------------------------------------------------------------------
        
        if angle != prevAngle or rTurn or lTurn:
            
            #if area of wall is large enough and turning line is not detected end turn
            if ((rightArea >= exitThresh and rTurn) or (leftArea >= exitThresh and lTurn)) and not tSignal:
                        
                  #set turn variables to false as turn is over
                  lTurn = False 
                  rTurn = False
              
                  #increase number of turns by 1
                  #in this case the turn is ended without seeing a pillar
                  if reverse == False or reverse == "done":
                      eTurnMethod = "wall"
                      t += 1
                      
                      #set 2.5 second timer to stop at the end of third lap
                      if t == 12: 
                          s = 2.5
                          sTime = time.time()
                  
                  #if 2 laps have been completed and the last pillar is red initiate a regular three point turn
                  if t == 8:
                      if lastTarget == redTarget:
                          multi_write([straightConst, 0.25])    
                          reverse = "turning"
                          
                      #reset ROI3
                      if reverse == False:
                          ROI3 = [redTarget - 40, 110, greenTarget + 40, 335]
                          ROIs = [ROI1, ROI2, ROI3, ROI4]
                  
            # if a car is parking or performing a three-point turn
            if not parkingR and not parkingL:

                #change angles for a right and left turn 
                if rTurn and cTarget == 0 and rightArea < 5000:
                    angle = straightConst - 25 if tempParking else sharpRight

                elif lTurn and cTarget == 0 and leftArea < 5000:
                    angle = sharpLeft
                
                #keep angle within bounds
                angle = max(min(angle, sharpLeft), sharpRight)
            
                write(angle)
        
# ------------------------------------------------------------{ debugging information }-------------------------------------------------------------------------

        if debug: 
        
            #if q is pressed break out of main loop and stop the car
            if cv2.waitKey(1)==ord('q'):
                stop_car() 
                break
            
            #display regions of interest
            display_roi((255, 204, 0))
            
            #display image
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
                "front wall area": areaFront,
                "three-point turn status": reverse,
                "last pillar": pColour,
                "current pillar": cpColour,
                "pillar area": pArea, 
                "turn direction": turnDir[0],
                "pillar distance": pDist, 
                "# turns": t,
                "t2": t2,
                "turn status": turn_status,
                "end turn method": eTurnMethod if eTurnMethod == "pillar" or eTurnMethod == "wall" else " ",  
            }
            
            display_variables(variables)

# ------------------------------------------------------------{ reset variables }------------------------------------------------------------------------

        prevAngle = angle #update previous angle
        tSignal = False #reset tSignal
        
        #reset variables for next iteration 
        prevError = error
        contY = 0
        contX = 0
        cTarget = 0
        pArea = 0
        prevIgnore = ignore
        
        #just in case the car doesn't stop on turn 12, set tempParking to true so the car can park
        if t == 13 and not tempParking:
            redTarget, greenTarget = (greenTarget, greenTarget) if turnDir == "right" else (redTarget, redTarget)
            tempParking = True
        
        time.sleep(0.1)

picam2.stop()
cv2.destroyAllWindows()




