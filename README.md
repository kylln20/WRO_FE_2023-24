&nbsp;

🛠️ Engineering Documentation 🛠️
======

> This repository details Team Asparagus’ building and programming process in our second participating year of the 2024 WRO Future Engineers Competition.

&nbsp; 
  
---

## Content of Repository 
* `models` - 3D CAD files
* `others` - other essential files
* `schemes` - electrical schematics
* `src` - main and other programs to run/control software
* `t-photos` - team photos
* `v-photos` - robot photos
* `video` - video demonstration

&nbsp; 

---
### Team Members:

<img src="https://github.com/kylln20/WRO_FE_2023-24/blob/main/t-photos/Team_Official.jpg" width="40%" height="40%"> <img src="https://github.com/kylln20/WRO_FE_2023-24/blob/main/t-photos/Team_Funny.jpg" width="40%" height="40%">

***Kayla Lin (Left), Eric Rao (Middle), Brian Yin (Right)***

---
## 📖 Content of README 📖
* ### Hardware
  * [`Components`](#components) - list of components 
  * [`Mobility`](#mobility) - hardware for robot movement
  * [`Sensors`](#sensors) - sensors used
  * [`Electricity`](#electricity) - electrical wiring and circuit boards
  * [`Power`](#power) - power supply
    
* ### Software
  * `Initialization and Connection Process` - 
  * `Object Management` - 
    
* ### Design Process

&nbsp;

---

## 🤖 Hardware 🤖

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

### 🚗 Mobility 🚗

#### Chassis
We use the `Carisma GT24`, a pre-built 1/24 scale RC car (15 cm in length), as opposed to the 1/18 scale car (26.5 cm in length) from the previous year. This is to accomodate the addition of the magenta parking lot in the obstacle challenge, and it allows us to simply park head-on, which is a more efficient procedure compared to parallel parking. 

#### Motors
Our car uses a Furitek Micro Komodo Brushless Motor. Brushless motors refer to the lack of small "brushes" in the motor that a brushed motor would have. This design reduces motor friction, improving lifespan, torque, efficiency, and acceleration.

The Furitek Micro Komodo is also very small compared to other RC car motors, making it fit well with our small chassis. The motor is mounted on a sturdy metal mount instead of the standard plastic mount, increasing durability and passive cooling. 

To control steering, we use a Hitec HS-5055MG Servo Motor, which is a metal gear servo motor. The servo motor that originally came with the RC chassis had a weaker plastic gear and a five-wire connection. We replaced it with a new servo with a three-wire connection (signal, voltage, ground), which we can control and power with the Raspberry Pi Hardware Attached on Top (HAT). It is connected to the wheel axis with a 3D-printed adapter piece that is screwed onto the rotational part of the servo. However, it is to be noted that the 3D printed piece is not perfectly secured to the axis and will, on the rare occasion, pop out.

To send signals to both components, we use a Furitek Lizard Pro Electronic Speed Controller (ESC), which comes with the brushless motor. The signals sent are PWM (pulse-width modulation) signals from the Pi HAT's motor ports. 
&nbsp;

### 👀 Camera Vision 👀
We use a `SainSmart Wide-Angle Camera`, which carries pixel data to the HAT via a Camera Serial Interface (CSI) cable. Based on said pixel data, we can determine the size (and distance) of surrounding objects, such as the red and green obstacles, the blue and orange lines on the game mat, and the black walls.
&nbsp;

### ⚡ Electricity & Power ⚡

Our car gets power from a single `Gens Ace 1300mAh Battery` which powers the Raspberry Pi and ESC circuit. This battery was chosen mainly due to its high 45 C rating allowing for a higher discharge of electricity while still being lightweight and compact. 

 Although the Raspberry Pi 4B runs off 5V, our Pi HAT contains a voltage regulator allowing the 7.4V output of the battery to be limited to 5V to power the Raspberry Pi. 

The wires of the battery are connected to the circuits of the Raspberry Pi and ESC in parallel with a switch controlling the passage of electricity at the beginning of the circuit. This design eliminated the need for two separate batteries, saving space, simplifying our circuit, and reducing the car’s overall weight.


---
## 💻 Software 💻

We use a Raspberry Pi 4 Board as our single board computer (SBC). It is connected to the HAT, from which it takes and processes sensor input to return turning-angle and speed values back to the HAT, which then get sent to the servo and DC motors respectively. 

The program running on the Raspberry Pi is written in Python.     
&nbsp;

### Initialization and Connection Process
 
We used the Raspberry Pi imager to write a custom Raspberry Pi operating system onto an SD card that allowed us to use our Pi hat. Once the operating system is downloaded onto the SD card, when the Raspberry Pi is running AP (Access Point) mode, we can connect to the Raspberry Pi through a wifi connection. Once we have selected the Access Point in the wifi tab, we use VNC Viewer to connect remotely to and interact with the Raspberry Pi using a set IP address. 

&nbsp;

### Object Detection
The camera captures an image which the program then converts from OpenCV’s default pixel data format of BGR (blue, green, red) to HSV (hue, saturation, value). Then, binary thresholding is applied, which changes the pixel data of the inputted image such that areas of interest are white, and everything else is black. This is done with the OpenCV library inRange() function, where we specify the input image and a colour mask (a range of HSV values). The input image can be modified with array splicing so we only search in a specific region of interest. The colour mask[^2] allows us to account for how lighting causes colour variation for the target object.
&nbsp;
The bounded white areas can then be extracted as a list of contours within a specified region of interest. By measuring the size of each contour by using the OpenCV library function contourArea(), we can predict which contour(s) is the target object by checking if the contour is within a certain size range. The target objects may be the signal pillars, the coloured lines on the game mat, the magenta parking lot, or even the black walls around the track.

[^2]: The colour masks used for each object still depend on the environment the car is in. We had difficulties getting the program to perform well in a room with yellow-tinted lights instead of white LEDs. This required changing the colour masks when running the car in that environment. An existing code that was useful for finding/adjusting colour masks was from an OpenCV tutorial, which was modified to be the ColourTester.py code.

&nbsp;

#### Wall Detection/Management
The wall contours are detected with the colour mask ([0, 0, 0] to [180, 255, 50]), and two regions of interest—one for the left wall, and one for the right wall. To stay centred when driving straight, we use a PD algorithm. P stands for proportional, and D stands for derivative. The algorithm involves calculating the difference between the areas of the two contours and calculating a turning angle based on:
&nbsp;
* A difference in contour areas (error)
* A straight constant (straightConst)- the number that would be sent for the car to go straight
* A proportional constant (cKp) - the constant that gets multiplied by error 
* A derivative constant (cKd) - the constant that gets multiplied by the difference in the error and the previous error (prevError)
&nbsp;
The resulting calculation is:
> angle = int(straightConst + error * cKp + (error - prevError) * cKd)
&nbsp;

PID is the standard algorithm (with a demonstration in this video here). “I” stands for integral, which represents a different constant that would be multiplied against an average error. However, while testing the cKp and cKd, we found our program was consistent enough without the added calculations, or the arduous trial and error that would be required to find a cKi value
&nbsp;


#### Signal Pillar and Parking Wall Detection
Signal pillars are found with green and/or red colour masks, and by searching in a region of interest specific to the locations of the obstacles. The parking walls are found with magenta colour masks and by searching in a region of interest near the walls.
&nbsp;

#### Corner Detection

&nbsp;

