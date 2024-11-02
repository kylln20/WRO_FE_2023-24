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
from shapely.geometry import Polygon
from masks import rMagenta, rRed, rGreen, rBlue, rOrange, rBlack, lotType
from functions import *

# ------------------------------------------------------------{ function declarations }-------------------------------------------------------------------------

class Pillar:
    def __init__(self, area, dist, x, y, target): 
        self.area = area
        self.dist = dist
        self.x = x
        self.y = y
        self.w = 0
        self.h = 0
        self.target = target
    
    def setDimensions(self, w, h):
        self.w = w
        self.h = h

def find_pillar(contours, target, p): 
    
    global t, s, reverse, leftArea, rightArea, turnDir, maxDist
    num_p = 0
    
    for cnt in contours: 
    
        area = cv2.contourArea(cnt)
        
        if (area > 100 and target == redTarget) or (area > 200 and target == greenTarget):
            
            #get width, height, and x and y coordinates by bounding rect
            approx=cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt,True),True)
            x,y,w,h=cv2.boundingRect(approx)

            #since the x and y coordinates are the coordinates just in the ROI, add to the x and y values to make it the proper coordinates on the overall image
            x += ROI3[0] + w // 2 
            y += ROI3[1] + h
            
            #calculates the distance between the pillar and the bottom middle of the screen
            temp_dist = round(math.dist([x, y], [320, 480]), 0)

            #if the pillar is close enough add it to the number of pillars
            if 180 < temp_dist < 390: #160, 394
                num_p += 1
            
            #print(x, area)
                #if the pillar is too close, stop the car and reverse to give it enough space
            if ((area > 6500 and target == redTarget) or (area > 9000 and target == greenTarget)) and ((x <= 440 and target == greenTarget) or (x >= 200 and target == redTarget)) and not tempParking and speed != 1500: #420, 220, 370, 270, 6500
                LED2(255, 255, 0)
                
                #move back 
                multi_write([straightConst, 0.1, 1500, 0.1, reverseSpeed, 1, speed])
                
                #if car is supposed to stop increase the timer as we moved back
                if s != 0:
                    s += 1.5
                
                #in the case of the car turning for a three point turn and seeing a pillar first instead of the wall, complete it as we have already switched directions
                if reverse == "turning" and turnDir == "right":
                    reverse = "done"
                    turnDir = "left"
                    t += 1
                
            else:
                LED2(0, 0, 0)
            
            #deselects current pillar if it comes too close to bottom of the screen
            if y > ROI3[3] - endConst or temp_dist > maxDist or (target == greenTarget and (leftArea > 11250 or rightArea > 11250)) or (target == redTarget and (leftArea > 12000 or rightArea > 12000)): #370, 12500
                continue
            
            if (tempParking or reverse == True or t2 == 7 and reverse != "done") and temp_dist > 370:
                continue

            
            #if the y value is bigger than the previous contY value or within a range and has a bigger area update the data as this pillar is now the closest one
            if temp_dist < p.dist:
                p.area = area
                p.dist = temp_dist
                p.y = y
                p.x = x
                p.target = target
                p.setDimensions(w, h)

    return p, num_p

if __name__ == '__main__':
    
# ------------------------------------------------------------{ initialization of variables }-------------------------------------------------------------------------
    
    time.sleep(3)
    
    #initialize camera
    picam2 = Picamera2()
    
    picam2.preview_configuration.main.size = (640,480)
    picam2.set_controls({"AeEnable": True})
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
    
    pillarAtStart = -1

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
    ROI1 = [0, 175, 330, 265] # 165, 255
    ROI2 = [330, 175, 640, 265]
    ROI3 = [redTarget - 50, 120, greenTarget + 50, 345] #+- 40 [redTarget - 40, 110, greenTarget + 40, 335]
    ROI4 = [200, 260, 440, 310] #250, 300
    ROI5 = [270, 120, 370, 140]
    
    ROIs = [ROI1, ROI2, ROI3, ROI4, ROI5]
    
    #booleans for tracking whether car is in a left or right turn
    lTurn = False
    rTurn = False
  
    kp = 0.015 #proportional value for PD steering 0.015
    kd = 0.01 #derivative value for PD steering 0.01

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
    
    speed = 1660 #variable for initial speed of the car
    reverseSpeed = 1340 #variable for speed of the car going backwards
    
    aDiff = 0 #value storing the difference of area between contours
    prevDiff = 0 #value storing the previous difference of contours for derivative steering

    error = 0 #value containing the difference between a pillar's x coordinate and its target x coordinate
    prevError = 0 #stores previous error

    cTarget = 0 #stores the target x coordinate for the current signal pillar, stays 0 if there is no signal pillar
    contX = 0 #stores x value of the current signal pillar
    contY = 0 #stores y value of the current signal pillar
    pArea = 0 #stores the area of a signal pillar
    
    maxDist = 370

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
            speed = 1650
        elif sys.argv[1].lower() == "parkingr": #for testing parking on the left side
            t = 12
            turnDir = "left"
            speed = 1650
        elif sys.argv[1].lower() == "psteer": #for testing parking on the left side
            tempParking = True
            t = 12
            t2 = 12
            speed = 1500
        elif sys.argv[1].lower() == "stop": #for testing parking on the left side
            t = 11
        elif sys.argv[1].lower() == "turnt": #for testing three-point turns
            t = 7
            speed = 1660
            pillarAtStart = True
        elif sys.argv[1].lower() == "turnf":
            t = 7
            speed = 1660
            pillarAtStart = False
        elif sys.argv[1].lower() == "steer": #for stationary testing
            speed = 1500
        elif sys.argv[1].lower() == "fast": #for stationary testing
            speed = 1660
        
        
    #if no mode is specified, assume the regular program and wait for button input
    else:
        buzz()
        while GPIO.input(key2_pin) == GPIO.HIGH:
            pass
        
        time.sleep(3)
        
    LED1(0, 0, 0)
    
    frames = 0
    
    pTimer = time.time()
    
    #write initial valuess
    time.sleep(0.5)
    multi_write([angle, 0.5, speed, 0.1, speed])
    
    startArea = 0 

# ------------------------------------------------------------{ main loop }-------------------------------------------------------------------------
    while True:
        
        frames += 1
        fps_start = time.time()

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

        #get contours of left and right walls
        if tempParking:
            contours_left = pOverlap(img, ROI1)
            contours_right = pOverlap(img, ROI2)
        else:
            contours_left = pOverlap(img, ROI1, True)
            contours_right = pOverlap(img, ROI2, True)

        
        if ROI5[0] != 0: 
            contours_turn = find_contours(img, rBlack, ROI5)
            contours_turn_m = find_contours(img, rMagenta, ROI5)
            tArea = max_contour(contours_turn, ROI5)[0] + max_contour(contours_turn_m, ROI5)[0]
        
        contours_parking = find_contours(img, rBlack, ROI4)

        #iterate through every contour in both the left and right region of interest and take the largest one in each
        left_region = max_contour(contours_left, ROI1)
        right_region = max_contour(contours_right, ROI2)
        
        leftArea = left_region[0]
        lWallY = left_region[2]
        
        rightArea = right_region[0]
        rWallY = right_region[2]
        
        areaFront = max_contour(contours_parking, ROI4)[0]
        
        #find contours 
        contours_red = find_contours(img, rRed, ROI3)
        contours_green = find_contours(img, rGreen, ROI3)
        contours_blue = find_contours(img, rBlue, ROI4)
        contours_orange = find_contours(img, rOrange, ROI4)
        
# ------------------------------------------------------------{ processing contours }-------------------------------------------------------------------------
        
        #count number of red and green pillars
        num_pillars_g = 0
        num_pillars_r = 0

# -----------------{ green / red signal pillar contour processing }--------------

        temp = Pillar(0, 1000000, 0, 0, greenTarget)

        cPillar, num_pillars_g = find_pillar(contours_green, greenTarget, temp)
        cPillar, num_pillars_r = find_pillar(contours_red, redTarget, cPillar)
            
        if t == 0:
            startArea = max(startArea, cPillar.area)
        '''
        if time.time() - pTimer > 2 and pillarAtStart == -1 and t != 7:
            if cPillar.area != 0:
                pillarAtStart = True
            else:
                pillarAtStart = False
        '''   
        
                
        if cPillar.area == 0:
            cPillar.target = 0
        
        #draw contours of pillars for debugging
        if debug: cv2.drawContours(img[ROI3[1]:ROI3[3], ROI3[0]:ROI3[2]], contours_green, -1, (144, 238, 144), 2)
        if debug: cv2.drawContours(img[ROI3[1]:ROI3[3], ROI3[0]:ROI3[2]], contours_red, -1, (144, 144, 238), 2)
                
# -----------------{ control variable manipulation based on number of pillars }--------------

        #lower control variables if there are more than 2 pillars of the same colour, most likely meaning we are turning along an inside corner.
        
        if (num_pillars_r >= 2 or num_pillars_g >= 2):
            
            LED2(255, 255, 255)
        
            endConst = 60 #60
            
            cKp = 0.2 
            cKd = 0.2 
            cy = 0.05 
            
        #any other combination of number of pillars
        else: 
            
            LED2(0, 0, 0)     
            
            endConst = 20 #30
            
            cKp = 0.25 #0.25
            cKd = 0.25 #0.25
            cy = 0.08 #0.08

# -----------------{ orange / blue line contour processing }---------------

        maxO = max_contour(contours_orange, ROI3)[0]
        maxB = max_contour(contours_blue, ROI3)[0]

        if turnDir == "none":
            if maxO > 100: 
                turnDir = "right"
            elif maxB > 100: 
                turnDir = "left"
        
        if (turnDir == "right" and maxO > 100) or (turnDir == "left" and maxB > 100): 
            
            #update t2
            t2 = t
            
            #for three-point turn, update region of interest to see farther
            if t2 == 7:
                maxDist = 370
                #ROI3[1] = 110
                #ROI3 = [redTarget - 40, 90, greenTarget + 40, 335]
                #ROIs = [ROI1, ROI2, ROI3, ROI4]
                
            if tempParking:
                ROI5 = [270, 120, 370, 140]
                

            #set either the right or left turn to true based on turn direction
            if turnDir == "right":
                rTurn = True
            else:
                lTurn = True
            
            '''
            if time.time() - pTimer < 3 and t != 7:
                pillarAtStart = False
            '''
            
            if t == 0 and pillarAtStart == -1:
                if startArea > 3000:
                    pillarAtStart = True
                else:
                    pillarAtStart = False
            #indicate that the coloured line is seen
            tSignal = True
                                           
# ------------------------------------------------------------{ final parking algorithm }------------------------------------------------------------------------      
        maxAreaL = 0
        leftY = 0
        maxAreaR = 0
        rightY = 0
        centerY = 0
        
        # detect magenta contours directly in front of car
        if t == 8 or tempParking:
            
            #find largest magenta contour
            contours_magenta_c = find_contours(img, rMagenta, ROI4)
            areaFrontMagenta = max_contour(contours_magenta_c, ROI4)[0]
            
            #if area of magenta in front is too large right after the three-point turn, avoid it by turning left
            if areaFrontMagenta > 500 and t == 8:
                angle = sharpLeft
            
        if tempParking:
            
            #find magenta contours in left and right regions of interest
            contours_magenta_l = find_contours(img, rMagenta, ROI1)
            contours_magenta_r = find_contours(img, rMagenta, ROI2)

            leftLot = max_contour(contours_magenta_l, ROI1)
            rightLot = max_contour(contours_magenta_r, ROI2)
            centerLot = max_contour(contours_magenta_c, ROI4)
            
            maxAreaL = leftLot[0]
            leftY = leftLot[2]
            maxAreaR = rightLot[0]
            rightY = rightLot[2]
            rightX = rightLot[1]
            centerY = centerLot[2]
            
                
            #conditions for initiating parking on the left side
            if leftY >= 220 and maxAreaL > 650 and t2 >= 12:
                if not parkingL and not parkingR:
                    write(1640)
                    parkingL = True
                    
                    if debug: print("area at start:", maxAreaL)
                    
                    #delay the turn if the area of the parking wall is too large
                    
                    
                    if(maxAreaL > 1000 and lotType == "dark") or (maxAreaL > 1800 and lotType == "light"): #4500
                        
                        buzz()
                        
                        if lotType == "light":
                            delay = maxAreaL / 2500 - 1
                        else:
                            delay = maxAreaL / 2500 - 0.25
                            
                        time.sleep(max(delay, 0))
                        
                
                    #readjust region of interest for parking
                    ROI4 = [230, 250, 370, 300]
                    
            #conditions for initiating parking on the right side
            if rightY >= 240 and maxAreaR > 600 and t2 >= 12:
                if not parkingL and not parkingR:
                    write(1640)
                    parkingR = True
                    
                    #delay the turn if the area of the parking wall is too large
                    if debug: print("area at start:", maxAreaR, "x:", rightX)
                    
                    if maxAreaR > 3000 and maxAreaR < 6000:
                        write(straightConst)
                        buzz()
                        #write(straightConst)
                        time.sleep(max(maxAreaR / 4000 - 0.5, 0))
                    elif maxAreaR > 6000:
                        write(straightConst)
                        buzz()
                        #write(straightConst)
                        time.sleep(1)
                    
                    #readjust region of interest for parking
                    #ROI4 = [250, 250, 390, 300]
                    ROI4 = [270, 250, 450, 300]
                
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
                if rightArea > 13000 and maxAreaR > 2000: #15000
                    pass
                    multi_write([1640, sharpRight, 1])
                #readjust by backing up if the parking lot is in front
                if centerY > 280 and areaFront < 3500:
                    LED1(255, 0, 0) 
                    multi_write([1500, 0.1, 1352, sharpRight, 0.5, 1500])
                #turn left into parking lot
                else:
                    pass
                    multi_write([1640, sharpLeft])
                    
            #if the area of the wall in front is above a limit stop as we are very close to the wall
            if areaFront > 3500:
                multi_write([straightConst, 0.2, 1640, 1.5])
                stop_car()
                break
                    
# ------------------------------------------------------------{ servo motor calculations based on pillars and walls}-------------------------------------------------------------------------
        
        #if the initial three-point turn wasn't enough to turn fully to the other direction
        if t == 8 and areaFront > 4000 and reverse == "done":
            turnDir = "right" if turnDir == "left" else "left"
            reverse = True
        
        #change pillar targets so all are passed on the outside 
        if t == 12 and not tempParking and ((leftArea > 2000 and rightArea > 2000 and cPillar.area < 1000) or (cPillar.area < 400)):
            
            redTarget, greenTarget = (greenTarget, greenTarget) if turnDir == "right" else (redTarget, redTarget)
            
            #used as an indication of when it is ok to park
            if turnDir == "left":
                ROI3[1] = 140 #130
            
            tempParking = True
            maxDist = 370
            
# -----------------{ no pillar detected }--------------
        if cPillar.target == 0 and not parkingL and not parkingR:

            #change pillar targets so all are passed on the outside 
            if t == 12 and not tempParking and all(target == False for target in tList[:1]):
                
                redTarget, greenTarget = (greenTarget, greenTarget) if turnDir == "right" else (redTarget, redTarget)
                
                #used as an indication of when it is ok to park
                if turnDir == "left":
                    ROI3[1] = 140 #130
                
                tempParking = True
                maxDist = 370

            
            LED2(0, 0, 0)

            #calculate the difference in the left and right lane areas
            aDiff = rightArea - leftArea
            #calculate angle using PD steering
            angle = max(0, int(straightConst + aDiff * kp + (aDiff - prevDiff) * kd))
            #update the previous difference
            prevDiff = aDiff
            
            #if both areas are small enough change angle based on last pillar to help make it easier to pass pillars
            #print(lWallY, rWallY, lTurn, rTurn, speed)
            '''
            if 0 < lWallY < 215 and 0 < rWallY < 215 and not tempParking and not lTurn and not rTurn:
                
                angle = sharpRight if turnDir == "right" else sharpLeft
            '''
            '''
            if leftArea < 2000 and rightArea < 2000 and abs(leftArea - rightArea) < 500 and not lTurn and not rTurn:
                angle = sharpRight if lastTarget == greenTarget else sharpLeft
            '''
            
            
# -----------------{ pillar detected }--------------

        elif not parkingR and not parkingL:
            
            LED2(255, 0, 0) if cPillar.target == redTarget else LED2(0, 255, 0)
          
            #if car is in a turn and tSignal is false meaning no orange or blue line is detected currently, end the turn
            #in this case the turn is finished while still seeing a pillar
            if (lTurn or rTurn) and not tSignal:

                #reset prevError and prevDiff 
                prevError = 0
                prevDiff = 0

                #add a turn
                if reverse == False or reverse == "done":
                    eTurnMethod = "pillar"
                    
                    ROI5 = [270, 120, 370, 140]
                    t += 1
                    
                    #check for three-point turn by checking the last pillar, turn direction, and area 
                    if t == 8:
                        
                        if debug: print(f"pillar area: {cPillar.area}, wall areas: {leftArea} {rightArea}, targets: {lastTarget} {cPillar.target}")
                        
                        if pillarAtStart:
                            
                           if cPillar.area > 500 and cPillar.target == redTarget:
                                reverse = True
                           elif cPillar.area < 700 and cPillar.target == greenTarget and lastTarget == redTarget:
                                reverse = True 
                        
                        else:
                           if cPillar.target == redTarget and cPillar.area > 100:
                                reverse = True
                        
                        print("turn should start")
                        ROI3[1] = 120
                        maxDist = 370
                        
                    #after three laps set a timer for 3.75 in order to stop in middle of starting section
                    if t == 12:
                        print(cPillar.area)
                        s = 2.75 if cPillar.area > 400 else 1
                        sTime = time.time()
                
                #set turns to false as the turn ended
                lTurn = False
                rTurn = False
            
            #calculate error based on the difference between the target x coordinate and the pillar's current x coordinate
            error = cPillar.target - cPillar.x

            #calculate new angle using PD steering
            angle = int(straightConst + error * cKp + (error - prevError) * cKd)

            #adjust the angle further based on cy value, if error is 
            angle -= int(cy * (contY - ROI3[1])) if error <= 0 else -int(cy * (contY - ROI3[1]))

            #if the pillar is large enough and the x-coordinate is close enough to its target set lastTarget to the current pillars target 
            if cPillar.target == greenTarget and cPillar.x > 320 and cPillar.area > 1000:
                lastTarget = greenTarget

            elif cPillar.target == redTarget and cPillar.x < 320 and cPillar.area > 1000:
                lastTarget = redTarget

            #make sure angle value is over 2000 
            angle = max(0, angle)
            
        elif not parkingR and not parkingL:
            LED1(0, 0, 0)
            
        if ((tArea > 1000 and turnDir == "left") or (tArea > 1250 and turnDir == "right")): # and not lTurn and not rTurn
            rTurn = False
            lTurn = False
            if cPillar.area > 5000 or (tempParking and cPillar.area > 2000) or (turnDir == "right" and cPillar.area > 3500): #4000
                angle = straightConst
            else:
                angle = sharpRight if turnDir == "right" else sharpLeft

        '''
        if tArea > 1500 and turnDir == "left" and tempParking:
            if cPillar.area > 5000: #4000
                angle = straightConst
            else:
                angle = sharpLeft
        '''
        '''
        if cPillar.target != 0 and cPillar.area < 500 and cPillar.y < 200 and tArea > 500 and not rTurn and not lTurn:
            print("removed")
            tArea = 0
            ROI5 = [0, 0, 0, 0]
        '''
        
        if (cPillar.area == 0 and tArea < 100) or abs(leftArea - rightArea) > 5000 and tArea > 1000 and not tempParking:
            tArea = 0
            ROI5 = [0, 0, 0, 0]
        '''    
        if leftArea < 6000 or rightArea < 6000 and tArea < 100 and not tempParking:
            tArea = 0
            ROI5 = [0, 0, 0, 0]
        '''
        
        
        
        #keep track of whether a pillar is seen for the last 10 frames
        if len(tList) == 10:
            tList.append(True if cPillar.target != 0 else False)
            tList.pop(0)
        else:
            tList.append(True if cPillar.target != 0 else False)
# ------------------------------------------------------------{ three point turn logic }-------------------------------------------------------------------------

        if reverse != "done" and (reverse == True or reverse == "turning"):
            
            LED2(0, 0, 255)
            
            #make sure turns are false
            rTurn = False
            lTurn = False
            
            
            #if anything but a red pillar is seen turn left
            if cPillar.target == 0 or cPillar.target == greenTarget:
                angle = sharpLeft
            
            #if a pillar wasn't seen for the last 10 frames keep the car turning left until it sees the parking lot or wall in front
            if all(target == False for target in tList) or cPillar.area < 400 and reverse != "turning":
                reverse = "turning"
                
            if reverse == "turning":
                angle = sharpLeft
            
            #stop turning once right in front of wall or parking lot
            if areaFront > 2000 or areaFrontMagenta > 750:
                
                #reverse car
                delay = 1.5 if areaFront > 2000 else 2
                multi_write([1500, 0.1, sharpRight, 0.1, reverseSpeed, delay, 1500, 0.1, 1680, 0.1, speed])
                
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
                          s = 2.5 if speed == 1650 else 1.5
                          sTime = time.time()
                  
                  #if 2 laps have been completed and the last pillar is red initiate a regular three point turn
                  if t == 8:
                      if lastTarget == redTarget:
                          multi_write([straightConst, 0.25])    
                          reverse = "turning"
                    
                      ROI3[1] = 120
                      maxDist = 370
                  
            # if a car is parking or performing a three-point turn
            if not parkingR and not parkingL:

                #change angles for a right and left turn 
                if rTurn and cPillar.target == 0 and rightArea < 5000:
                    angle = straightConst - 25 if tempParking else sharpRight

                elif lTurn and cPillar.target == 0 and leftArea < 5000:
                    angle = sharpLeft
                
                #keep angle within bounds
                angle = max(min(angle, sharpLeft), sharpRight)
                #print(angle)
                write(angle)
        
        while int(1 / (time.time() - fps_start)) > 30:
                pass
        
# ------------------------------------------------------------{ debugging information }-------------------------------------------------------------------------

        if debug: 
        
            #if q is pressed break out of main loop and stop the car
            if cv2.waitKey(1)==ord('q'):
                stop_car() 
                break
        
            fps = "fps: " + str(int(1 / (time.time() - fps_start)))
            
            elapsed = "time elapsed: " + str(int(time.time() - pTimer)) + "s"
            
            ROIs = [ROI1, ROI2, ROI3, ROI4, ROI5]
            
            #display regions of interest
            display_roi(img, ROIs, (255, 204, 0))
            '''
            back = np.zeros((1080, 1920, 3), dtype=np.uint8)
            back[:] = 0
            
            
            
            
            
            img = cv2.resize(img, (960, 720))

            back[0:720, 0:960] = img
            
            cv2.putText(back, fps, (410, 800), cv2.FONT_HERSHEY_DUPLEX, 1, (255, 255, 255), 2)
            cv2.putText(back, elapsed, (10, 800), cv2.FONT_HERSHEY_DUPLEX, 1, (255, 255, 255), 2)
            '''
            
            
            if cPillar.area > 0:
                
                
                color = (0, 0, 0)
                
                cv2.putText(img, str(round(cPillar.area)), (int(cPillar.x + cPillar.w//2 + 5), int(cPillar.y - cPillar.h//2)), cv2.FONT_HERSHEY_DUPLEX, 1, color, 2)
                
                color = (144, 238, 144) if cPillar.target == greenTarget else (144, 144, 238)
                
                cv2.putText(img, str(round(cPillar.area)), (int(cPillar.x + cPillar.w//2 + 5), int(cPillar.y - cPillar.h//2)), cv2.FONT_HERSHEY_DUPLEX, 1, color, 1)
                
            cv2.putText(img, fps, (500, 30), cv2.FONT_HERSHEY_DUPLEX, 1, (0, 0, 0), 4)
            cv2.putText(img, elapsed, (10, 30), cv2.FONT_HERSHEY_DUPLEX, 1, (0, 0, 0), 4)
            cv2.putText(img, fps, (500, 30), cv2.FONT_HERSHEY_DUPLEX, 1, (255, 255, 255), 1)
            cv2.putText(img, elapsed, (10, 30), cv2.FONT_HERSHEY_DUPLEX, 1, (255, 255, 255), 1)
            

            
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
                #"left wall y": lWallY,
                #"right wall y": rWallY,
                #"left parking lot area": maxAreaL if tempParking else 0,
                "right parking lot area": maxAreaR if tempParking else 0,
                #"front parking area": areaFrontMagenta,
                #"front wall area": areaFront,
                #"three-point turn status": reverse,
                #"last pillar": pColour,
                #"current pillar": cpColour,
                #"pillar area": cPillar.area, 
                "turn direction": turnDir[0],
                #"pillar distance": cPillar.dist, 
                "# turns": t,
                "t2": t2,
                "green pillars": num_pillars_g,
                "red pillars:": num_pillars_r,
                "turn status": turn_status,
                #"end turn method": eTurnMethod if eTurnMethod == "pillar" or eTurnMethod == "wall" else " ",
                "tArea": tArea,
                "angle:": angle,
                #"pillar y": cPillar.y,
                "pillar at start": pillarAtStart,
                #"frames": frames
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
        
        #time.sleep(0.1)

picam2.stop()
cv2.destroyAllWindows()





