&nbsp;

🛠️ Engineering Documentation 🛠️
======

> This repository details Team Asparagus’ building and programming process in our second participating year of the 2024 WRO Future Engineers Competition.
&nbsp; 

---

## Content of Repository 
* `models` - 3D CAD files
* `others` - Other essential files
* `schemes` - Electrical schematics
* `src` - Main and other programs to run/control software
* `t-photos` - Team photos
* `v-photos` - Robot photos
* `video` - Video demonstration
&nbsp; 

Team Members
---

* Kayla Lin, 17, kylln2027@gmail.com
* Eric Rao, 16, ericrao08@gmail.com
* Brian Yin, 16, brianyin256@gmail.com

<img src="https://github.com/kylln20/WRO_FE_2023-24/blob/main/t-photos/Team_Official.jpg" width="40%" height="40%"> <img src="https://github.com/kylln20/WRO_FE_2023-24/blob/main/t-photos/Team_Funny.jpg" width="40%" height="40%">

***Kayla Lin (Left), Eric Rao (Middle), Brian Yin (Right)***

## 📖 Content of README 📖
* ### Hardware
  * `Components`
  * `Mobility Management`
    *  `Chassis`
    *  `Design`
    *  `Motors`
  * `Power and Sense Management`
    *  `Sensors`
    *  `Power and Wiring`
  * `Schematic`
    
* ### Software
  * `Initialization and Connection Process`
  * `Object Management`
    *  `Object Detection`
    *  `Wall Detection/Management`
    *  `Signal Pillar Detection/Management`
    *  `Turning`
    *  `Parking Lot Detection/Management`
    *  `Three-Point Turn`
    *  `Possible Improvements`
&nbsp;

---

&nbsp;

🤖 Hardware 🤖
===


### ⚙️ Components ⚙️

| Name | Product | Price |
| ----------- | ----------- | ----------- |
| RC Car | [`Carisma GT24`](https://www.canadahobbies.ca/product/hobby-brands/carisma-rc/gt24-124th-4wd-toyota-celica-gt-four-st185-wrc/) | $263 |
| RC Car Battery | [`Gens Ace 1300mAh Battery`](https://www.adrenalinehobby.com/products/gens-ace-g-tech-1300mah-2s-7-4v-25c-lipo-deans-plug) | $35 | 
| Drive Motor & ESC | [`Furitek Komodo Motor + Lizard Pro ESC`](https://www.xtremerc.ca/products/furitek-scx24-stinger-brushless-power-system-w-1212-3450kv-brushless-motor?_pos=1&amp;_sid=cf7c35a05&amp;_ss=r) | $226 |
| Turning Motor | [`Hitec HS-5055MG Servo Motor`](https://ca.robotshop.com/products/hs-5055mg-metal-gear-micro-servo-motor?srsltid=AfmBOopv8Z7LoCVOEqe16w05ZV-R78dNmy7dappldIxZiQzCJroxcssFc2Y) | $38 |
| Motor Mount | [`GT24 High Performance Alloy Motor Mount`](https://carisma-shop.com/products/gt24-alum-motor-mount) | $17 |
| CSI Camera | [`SainSmart Wide Angle Fish-Eye Camera`](https://www.amazon.ca/SainSmart-Fish-Eye-Camera-Raspberry-Arduino/dp/B00N1YJKFS/ref=sr_1_5) | $19 |
| Raspberry Pi HAT | [`HiWonder TurboPi HAT`](https://www.hiwonder.com/collections/raspberrypi-bionic-robot/products/turbopi?variant=40112905388119) | $61 [^1] |
| Raspberry Pi | [`Vemico Raspberry Pi 4`](https://www.amazon.ca/Vemico-Raspberry-Kit-Heatsinks-Screwdriver/dp/B09WXRCYL4/ref=sr_1_3) | $176 |
| Raspberry Pi Fan | [`GeeekPi Fan`](https://www.amazon.ca/dp/B07C9H9LJN?psc=1&ref=ppx_yo2ov_dt_b_product_details) | $19 |
| ON OFF Switch | [`DaierTek Switch`](https://a.co/d/05vnrpJD) | $13 |

**Total Car Cost: $867 CAD**

[^1]:This link is for the `Hiwonder TurboPi Car`, which uses the `HiWonder TurboPi HAT`. We contacted the manufacturer to purchase the HAT directly.

<sup>*3D printed parts were made with the [`BambuLab P1P`](https://ca.store.bambulab.com/products/p1p)</sup>

&nbsp;

🚗 Mobility Management 🚗
---
#### Chassis
We use the chassis of the `Carisma GT24`, a pre-built 1/24 scale RC car (15 cm in length)to accommodate the addition of the magenta parking lot in the obstacle challenge. It allows us to simply park head-on.

#### Design 
Our car consists of the chassis of the `Carisma GT24` with 3D-printed components placed on top to hold extra electronic components

<img src="https://github.com/kylln20/WRO_FE_2023-24/blob/main/other/extra%20images/Labeled.jpg" height="300px"> <img src="https://github.com/kylln20/WRO_FE_2023-24/blob/main/other/extra%20images/Labeled2.jpg" height="300px">

1. **Base Nut Front / Back**: used to secure the 3D-printed base onto the chassis
2. **Cable Barrier**: is attached to the chassis to prevent cables from interfering with tires
3. **Camera Extension**: a piece attaching to the car’s base to give the camera extra height and tilt
4. **Camera Mount**: the piece the camera is attached to, adds height and tilt
5. **Camera Support**: used to help stabilize the overall camera structure
6. **Car Base**: the main platform of the car, is attached to the chassis through 4 posts, components of the base includes:
    - Holder for Raspberry Pi
    - Holder for cooling fan 
    - Holder for battery
    - Vertical post to route battery cable
    - Cable ladder organizer on the right side
    - Hook on the left to hold the power cable in place
7. **Servo Attachment**: attaches to the servo motor and controls steering
   
#### Motors
Our car uses a `Furitek Micro Komodo Brushless Motor`. Brushless motors refer to the lack of small "brushes" in the motor that a brushed motor would have. This design reduces motor friction, improving lifespan, torque, efficiency, and acceleration.

The `Furitek Micro Komodo` is also very small compared to other RC car motors, making it fit well with our small chassis. It receives power and signal via PWM from the `Furitek Lizard Pro Electronic Speed Controller (ESC)`.

To control steering, we use a `Hitec HS-5055MG Servo Motor`, which is a three-wire connection (signal, voltage, ground) metal gear servo motor. We control and power from the `Raspberry Pi Hardware Attached on Top (HAT)`. It is connected to the wheel axis with a 3D-printed adapter piece that is screwed onto the rotational part of the servo.

&nbsp;

⚡ Sense and Power Management ⚡
---
#### Sensors
We use a `SainSmart Wide-Angle Camera`, which carries pixel data to the HAT via a Camera Serial Interface (CSI) cable. Based on said pixel data, we can identify those objects based on their size and colour. From such information, our program will calculate the desired speed and turning angle which it will send through the HAT to the DC and servo motors respectively with pulse-width modulation (PWM) signals. 
&nbsp;

Our camera is sensitive to different lighting conditions meaning colors may look different in different scenarios. This could have been improved by changing camera settings or attaching a lamp to the car to make sure the lighting conditions are always consistent. 

#### Power and Wiring
Our car gets power from a single `Gens Ace 1300mAh Battery` which powers the Raspberry Pi and ESC circuit. This battery was chosen mainly due to its high 45 C rating allowing for a higher discharge of electricity while still being lightweight and compact. 

Although the `Raspberry Pi 4B` runs off 5V, our Pi HAT contains a voltage regulator allowing the 7.4V output of the battery to be limited to 5V to power the Raspberry Pi. 

The wires of the battery are connected to the circuits of the Raspberry Pi and ESC in parallel with a switch controlling the passage of electricity at the beginning of the circuit. This design eliminated the need for two separate batteries, saving space, simplifying our circuit, and reducing the car’s overall weight.

The battery wires, Raspberry Pi, ESC, and switch are all soldered together, making the wiring very durable. 

The wiring is placed underneath the car base. The switch is secured near the back of the car with a zip tie, the battery cable comes out of the right, the Raspberry Pi Power Adapter comes out from the left, and the ESC is placed in the center. 

One area that could be improved about our wiring is the switch. 

Our switch is large and along with the fact that the wires connecting to the switch are too long, it ends up extending the length of our car by a couple of centimeters. Our design could be improved by using a smaller switch with a shorter length of wire, making the car more compact. 

🔌 Schematic 🔌
---
<img src="https://github.com/kylln20/WRO_FE_2023-24/blob/main/schemes/schematic.png" height="400px"> <img src="https://github.com/kylln20/WRO_FE_2023-24/blob/main/other/extra%20images/wiring2.jpg" height="400px"> 

&nbsp;

💻 Software 💻
===
We use a `Raspberry Pi 4 Board` as our single board computer (SBC). It is connected to the HAT, from which it takes and processes sensor input to return turning-angle and speed values back to the HAT, which then get sent to the servo and DC motors respectively. 

The program running on the Raspberry Pi is written in Python.     

🛜 Initialization and Connection Process 🛜
---
We used the Raspberry Pi imager to write the Raspberry Pi operating system onto an SD card that allowed us to use our Pi HAT. Once the operating system is downloaded onto the SD card, when the Raspberry Pi is running AP (Access Point) mode, we can connect to the Raspberry Pi through a wifi connection. Once we have selected the Access Point in the wifi tab, we use VNC Viewer to connect remotely to and interact with the Raspberry Pi using a set IP address. 

Object Management
---

### Object Detection <sub> (Open Challenge / Obstacle Challenge) </sub>

The camera captures an image which the program then converts from OpenCV’s default pixel data format of BGR (blue, green, red) to HSV (hue, saturation, value). Then, binary thresholding is applied, which changes the pixel data of the inputted image such that areas of interest are white, and everything else is black. This is done with the OpenCV library inRange() function, where we specify the input image and a colour mask (a range of HSV values). The input image can be modified with array splicing so we only search in a specific region of interest. The colour mask[^2] allows us to account for how lighting causes colour variation for the target object.

The bounded white areas can then be extracted as a list of contours within a specified region of interest. By measuring the size of each contour by using the OpenCV library function contourArea(), we can predict which contour(s) is the target object by checking if the contour is within a certain size range. The target objects may be the signal pillars, the coloured lines on the game mat, the magenta parking lot, or even the black walls around the track.

[^2]: The colour masks used for each object still depend on the environment the car is in. We had difficulties getting the program to perform well in a room with yellow-tinted lights instead of white LEDs. This required changing the colour masks when running the car in that environment. An existing code that was useful for finding/adjusting colour masks was from an [OpenCV tutorial](https://docs.opencv.org/3.4/da/d97/tutorial_threshold_inRange.html), which was modified to be the ColourTester.py code.

### Wall Detection/Management <sub> (Open Challenge / Obstacle Challenge) </sub>

The wall contours are detected with the colour mask ([0, 0, 0] to [180, 255, 50]), and one region of interest for each wall. 

<img src="https://github.com/kylln20/WRO_FE_2023-24/blob/main/other/extra%20images/wallcontour.png" height="300px">

To stay centered when driving straight, we use a proportional derivitave (PD) algorithm, which involves calculating the difference between the areas of the two contours and calculating a turning angle based on:

* A difference in contour areas (error)
* A straight constant (straightConst)- the number that would be sent for the car to go straight
* A proportional constant (cKp) - the constant that gets multiplied by error 
* A derivative constant (cKd) - the constant that gets multiplied by the difference in the error and the previous error (prevError)

The resulting calculation is:
> angle = int(straightConst + error * cKp + (error - prevError) * cKd)
<br/><br/>
### Signal Pillar Detection/Management <sub> (Obstacle Challenge) </sub>

In the Obstacle Challenge, if a signal pillar is detected, the program will switch from calculating the angle based on the walls to the signal pillar instead. 

Signal pillars are found with green and red colour masks, and by searching in a region of interest specific to the locations of the obstacles. 

<img src="https://github.com/kylln20/WRO_FE_2023-24/blob/main/other/extra%20images/pillarcontour.png" height="300px">

A function called boundingRect() approximates a rectangle around the selected contour. boundingRect() also returns the x and y coordinates of the rectangle’s top left corner. When applied to the contour of the signal pillar, this can be used to determine its location.

Then, a PD calculation is applied based on the difference between the x-coordinate of the pillar, and the target x-coordinate. The target x-coordinate for the green pillars is near the right side, as the car needs to pass it on the left side. The opposite is true for the red pillars. The calculation also includes a value changing the angle based on how close the pillar is by using the pillar's y-value. 

<img src="https://github.com/kylln20/WRO_FE_2023-24/blob/main/other/extra%20images/pillarcoord.png" height="300px">

In the event there are 2 or more pillars seen, we determine the one to focus on by calculating the distance between the bottom middle of the screen, to the bottom middle of the pillar. We use the closest pillar to calculate the servo angle. 

Additionally, we change the control variables to be weaker when 2 or more pillars of the same color are seen, this is done so the car can successfully navigate tight corner cases. 

While we detect a pillar, if the area of the left or right walls becomes too large, we deselect the current pillar so the wall areas determine the angle instead. This allows the car to turn towards the middle and avoid hitting the wall

After twelve turns, once the car has completed three laps and is searching for the parking lot, the target x-coordinate is such that the car drives toward the outside of the red and green pillars.
<br/><br/>

### Turning <sub> (Open Challenge) </sub>
The car turns when one side of the camera shows a wall, and the other doesn't. In the camera feed, the area of one wall contour will be significantly less than the area of the other wall contour. The turning programs ends once both contours become similar again.

During a turn, we set the servo to a default turning angle of 25 degrees. However, if the angle calculated through the difference of wall areas is greater than 25 degrees, the car will use this angle instead of the default 25 degrees. This ensures we don’t hit the wall during tighter turns. 

Additionally, to count the number of corners the car has passed, the program counts the orange lines on the mat. The line is searched for using binary thresholding, with an orange colour mask on a centred region of interest. Once the area of the line contour has passed a certain value, the program knows the car has passed a corner.
<br/><br/>

### Turning <sub> (Obstacle Challenge) </sub>
Unlike the open challenge, in the obstacle challenge, we can't detect turns by the areas of walls due to the need to avoid pillars. To compensate, the obstacle challenge code instead enters a turning mode once the nearest mat line (blue if travelling clockwise, orange if travelling counter-clockwise) has been detected and is of a certain area to ensure we are at a corner. 

The car is automatically set to the maximum 50-degree turning angle to ensure it can see pillars as quickly as possible after a turn. 

If it detects a signal pillar, the turn ends immediately, and the angle calculation is based on the signal pillar. Otherwise, like in the open challenge, the program will stop turning when the difference between the two wall contours has decreased to a certain threshold, and the angle calculation will be based on the wall area difference.

If a pillar is detected during a turn, we found the car may struggle in certain cases to successfully turn around the pillar when the angle of approach is very narrow. To counteract this, we added a 6th region of interest for detecting the wall in front to turn earlier. If a turn is ended by seeing a pillar, we check if this region of interest is filled and turn to the maximum angle based on the color of the pillar (left if red, right if green) even if we still detect the pillar. If the pillar's area becomes too large while turning, the car turned too early and is on course to hit. In this case, we straighten the car's turning angle until the area is lower than a certain threshold to avoid collision. As we don't want this region of interest to interfere with our regular driving algorithms, the region of interest is only present when a turn is ended by seeing a pillar and is hidden once it passes the pillar and the region of interest no longer detects the wall in front. This approach makes the car turn earlier making the turns around pillars much more consistent, allowing our car to control better at faster speeds. 


<br/><br/>
### Parking Lot Detection/Management <sub> (Obstacle Challenge) </sub>

The parking walls are found with magenta colour masks and by searching in three regions of interest that when combined cover the vertical middle of the captured image, again with binary thresholding. This starts after twelve turns. If it is the case that after thirteen turns, the magenta parking lot isn’t detected, then parking mode will start after a magenta contour of the right size reaches a specific Y-coordinate.

<img src="https://github.com/kylln20/WRO_FE_2023-24/blob/main/other/extra%20images/parkingdetection.png" height="300px">

Once the magenta parking lot has been found in the left or right region of interest, the car turns in that direction. If the program detects a magenta contour in the central region of interest, it backs up, to allow more distance to adjust and park between the walls without touching them. Additionally, while parking into the lot on the left, if the right region of interest is found to have a large enough area in both magenta and black, the car is too far left, meaning we have to turn right.

The car stops once the area of the wall detected in the middle is large enough, using the same ROI that detects the orange/blue lines.
<br/><br/>
### Three-Point Turn <sub> (Obstacle Challenge) </sub>

After the eighth turn has been counted by seeing a wall or a pillar, we check whether a three-point turn is required. This is done by checking whether there was a pillar directly in front of the car and the area of the pillar detected during the 8th turn. 

We determine if there was a pillar in front of the car in the starting section by seeing if the maximum pillar area we have detected before the first turn, is larger than a certain threshold. 

If we know there was a pillar directly in front of the car in the starting section. We can assume that any pillar seen during the 8th turn is the last pillar of the second lap. 

If we know there was no pillar directly in front of the car. We know that if there is another pillar in the starting section it would be close to the edge as it's impossible to have a pillar in the middle. This means that the area of the pillar that is seen during the turn must be large. Therefore, we check if the area of the pillar is above a threshold. 

If no pillar is detected during the 8th turn, we just use the last pillar we had detected to determine whether to do a three-point turn. 

if turn is ended by seeing the wall: 
     if last pillar seen is red: 
          perform three point turn
          
if turn is ended by seeing the pillar: 
     if there was a pillar directly in front: 
          if area of current pillar is larger than a threshold and is red: 
                 perform turn
     if there was no pillar directly in front and current pillar is red: 
          perform turn

This approach to determining the need to perform a three-point turn proved much more consistent than our last approach, which relied heavily on specific wall area and pillar area thresholds vulnerable to lighting and color. 

Once we know a three-point turn must be performed, the car will immediately turn to the left unless it detects a red pillar, in which case it will turn after passing the red pillar by waiting until no pillar is detected for 10 iterations of the main loop. The car will turn left until it detects the wall or parking lot in front. It will then back up while turning to the right for a certain period. Then the program will resume.

After the initial three-point turn, if the car detects a large black area in front again, the initial three-point turn isn't sharp enough, so another three-point turn is performed.

If the car doesn't detect the wall in front and instead comes too close to a pillar, it means the car has already turned around, so we count an extra turn.

### Possible Improvements <sub> (Open Challenge / Obstacle Challenge) </sub>

One possible improvement that could be made is the placement of our regions of interest. Our regions of interest on the screen are placed perfectly symmetrically on both sides of the screen. This means we didn’t account for the fact that our camera is not perfectly aligned. 

This causes the car to control slightly differently when the car is running clockwise or counter-clockwise. The car could be made a lot more consistent in the open challenge and obstacle challenge with better adjustment of our regions of interest tailored to the view of the camera. 

<br/><br/>
