#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/TurboPi/')
import time
import signal
import threading
import HiwonderSDK.Board as Board
from gyro import berryIMU, IMU, LSM9DS0, LSM9DS1, LSM6DSL, LIS3MDL

def write(motor, value):
    
    if(motor == "servo"):
        pulseWidth = int(11.1*value+500)
        Board.setPWMServoPulse(1, pulseWidth, 1)
    
    elif(motor == "dc"):
        print("value:", value)
        Board.setPWMServoPulse(5, value, 100)

headings = []
tHeadings = []

endHeading = 0
endTHeading = 0

lh = 360
hh = 0

lth = 360
hth = 0

hDeviation = 0
thDeviation = 0

write("dc", 1500)
time.sleep(8)
write("dc", 1650)
write("servo", 87)


while len(tHeadings) < 1000:
    
    heading, tHeading = berryIMU.compute_heading()
    
    print(heading, tHeading)
    
    if heading > 320:
        heading -= 360
    
    if tHeading > 320:
        tHeading -= 360
    
    lh = min(lh, heading)
    hh = max(hh, heading)
    
    hDeviation = hh - lh
    
    lth = min(lth, tHeading)
    hth = max(hth, tHeading)
    
    thDeviation = hth - lth
    
    headings.append(heading)
    tHeadings.append(tHeading)
    
    endHeading = sum(headings) / len(headings)
    endTHeading = sum(tHeadings) / len(headings)
    

print(f"heading: {endHeading} | compensated heading: {endTHeading}\nmax heading deviation: {hDeviation} | max compensated heading devviation: {thDeviation}")
write("dc", 1500)
    
        
    

