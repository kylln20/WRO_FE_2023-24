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

from masks import rMagenta, rBlack, lotType

# ------------------------------------------------------------{ function declarations }-------------------------------------------------------------------------

#function used to send signals to arduino to control speeds of the dc motor and the angle of the servo motor
def write(value):
    
    if 180 > value > 0:
        pulseWidth = int(11.1*value+500)
        Board.setPWMServoPulse(1, pulseWidth, 1)
    
    elif 2000 > value > 1000:
        Board.setPWMServoPulse(5, value, 100)
        
#takes in an array of commands (dc, servo, sleep) and executes each
def multi_write(sequence):

    for action in sequence: 
        
        #delay commands
        if action < 87 - 50: 
            time.sleep(action)
        else: 
            write(action)

#function which displays the Regions of Interest on the image
def display_roi(img, ROIs, color):
    for ROI in ROIs: 
        img = cv2.line(img, (ROI[0], ROI[1]), (ROI[2], ROI[1]), color, 4)
        img = cv2.line(img, (ROI[0], ROI[1]), (ROI[0], ROI[3]), color, 4)
        img = cv2.line(img, (ROI[2], ROI[3]), (ROI[2], ROI[1]), color, 4)
        img = cv2.line(img, (ROI[2], ROI[3]), (ROI[0], ROI[3]), color, 4)
    
    return img

#controls buzzer
def buzz():
    Board.setBuzzer(1)
    time.sleep(0.35)
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
def find_contours(img_lab, lab_range, ROI):
    
    img_segmented = img_lab[ROI[1]:ROI[3], ROI[0]:ROI[2]]
    
    #img_blur = cv2.GaussianBlur(img_segmented, (7, 7), 0)

    #cv2.imshow("cam", img_lab)
    
    lower_mask = np.array(lab_range[0])
    upper_mask = np.array(lab_range[1])

    mask = cv2.inRange(img_segmented, lower_mask, upper_mask)
    
    kernel = np.ones((5, 5), np.uint8)
    
    eMask = cv2.erode(mask, kernel, iterations=1)
    dMask = cv2.dilate(eMask, kernel, iterations=1)
    
    contours = cv2.findContours(dMask, cv2.RETR_EXTERNAL,
    cv2.CHAIN_APPROX_SIMPLE)[-2]
    
    return contours

#returns the area of the largest contour in a group of contours
def max_contour(contours, ROI): 
    maxArea = 0
    maxY = 0
    maxX = 0
    mCnt = 0
    
    for cnt in contours:
        
        area = cv2.contourArea(cnt)
        
        if area > 100: 
            #get width, height, and x and y coordinates by bounding rect
            approx=cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt,True),True)
            x,y,w,h=cv2.boundingRect(approx)

            #since the x and y coordinates are the coordinates just in the ROI, add to the x and y values to make it the proper coordinates on the overall image
            x += ROI[0] + w // 2
            y += ROI[1] + h
            
            if area > maxArea:
                maxArea = area
                maxY = y
                maxX = x
                mCnt = cnt

    return [maxArea, maxX, maxY, mCnt]

def pOverlap(img_lab, ROI, add=False):
    
        lower_mask = np.array(rBlack[0])
        upper_mask = np.array(rBlack[1])
        
        mask = cv2.inRange(img_lab[ROI[1]:ROI[3], ROI[0]:ROI[2]], lower_mask, upper_mask)
        #cv2.imshow("o", mask)
        lower_mask2 = np.array(rMagenta[0])
        upper_mask2 = np.array(rMagenta[1])
        
        mask2 = cv2.inRange(img_lab[ROI[1]:ROI[3], ROI[0]:ROI[2]], lower_mask2, upper_mask2)
        
        if not add: 
            mask = cv2.subtract(mask, cv2.bitwise_and(mask, mask2))
        else:
            mask = cv2.add(mask, mask2)
        
        kernel = np.ones((5, 5), np.uint8)
        
        eMask = cv2.erode(mask, kernel, iterations=1)
        #cv2.imshow("ko", mask)
        
        #cv2.imshow("k", mask2)
        
        contours = cv2.findContours(eMask, cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)[-2]
        
        return contours

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