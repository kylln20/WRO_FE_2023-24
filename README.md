&nbsp;

🛠️ Engineering Documentation 🛠️
======

> This repository details Team <>’s building and programming process in our second participating year of the 2024 WRO Future Engineers Competition.

&nbsp; 

---

## 💻 Content of Repository 💻
* `models` - 3D CAD files
* `others` - other essential files
* `schemes` - electrical schematics
* `src` - main and other programs to run/control software
* `t-photos` - team photos
* `v-photos` - robot photos
* `video` - video demonstration

&nbsp; 

---

## 📖 Content of README 📖

* ### Hardware
  * [`Components`](#components) - list of components 
  * [`Mobility`](#mobility) - hardware for robot movement
  * [`Sensors`](#sensors) - sensors used
  * [`Electricity`](#electricity) - electrical wiring and circuit boards
  * [`Power`](#power) - power supply
    
* ### Software
  * `Object Detection` - 
  * `Movement Logic` - 
  * `lorem` - 
  * `ipsum` - 
    
* ### Design Process

&nbsp;

---

## 🤖 Hardware 🤖

&nbsp;

### ⚙️ Components ⚙️

| Name | Product | Price |
| ----------- | ----------- | ----------- |
| Drive motor | [`motor name`](https://www.examplelink.com) | $1000 |
| Drive motor | [`motor name`](https://www.examplelink.com) | $1000 |
| Drive motor | [`motor name`](https://www.examplelink.com) | $1000 |
| Drive motor | [`motor name`](https://www.examplelink.com) | $1000 |
| Drive motor | [`motor name`](https://www.examplelink.com) | $1000 |
| Drive motor | [`motor name`](https://www.examplelink.com) | $1000 |
| Drive motor | [`motor name`](https://www.examplelink.com) | $1000 |
| Drive motor | [`motor name`](https://www.examplelink.com) | $1000 |


> * Brushless Motor and ESC: [Furitek Komodo Motor + Lizard Pro ESC](https://www.xtremerc.ca/products/furitek-scx24-stinger-brushless-power-system-w-1212-3450kv-brushless-motor?_pos=1&amp;_sid=cf7c35a05&amp;_ss=r)
> * Raspberry Pi Fan: 
> * CSI Camera: [DORHEA 160° Camera](https://www.amazon.com/Raspberry-Camera-Module-160FOV-Fisheye/dp/B083XMGSVP/)
> * Servo Motor: [Hitech HS-5055MG](https://ca.robotshop.com/products/hs-5055mg-metal-gear-micro-servo-motor?srsltid=AfmBOopv8Z7LoCVOEqe16w05ZV-R78dNmy7dappldIxZiQzCJroxcssFc2Y)
> * Raspberry Pi HAT Extension:
> * RC Car Lipo Battery: 
> * RC Car: [Carisma GT24](https://www.canadahobbies.ca/product/hobby-brands/carisma-rc/gt24-124th-4wd-toyota-celica-gt-four-st185-wrc/)

&nbsp;

### 🚗 Mobility 🚗

<img src="v-photos/rock.webp" width="700" height=auto>

#### Chassis
Last year, we used a pre-built 1/18 scale RC car (26.5 cm in length). However, the addition of the magenta parking lot in the obstacle challenge required a change in strategy. There are three ways to park – parallel parking, head-in parking, and reverse parking. Out of these three procedures, head-in parking requires the least number of steps, and would be the quickest to perform. Since the length of the parking lot is always 20 cm, by choosing a car of shorter length then 20 cm, we give ourselves the capability of head-in parking. As such, we chose a 1/24 scale RC car (15 cm in length).
#### Drive Motor + Turning Motor
Our car uses a Furitek Micro Komodo Brushless Motor. A brushless motor was chosen over a brushed motor since the latter is powered by direct contact, and can wear down over time. A brushless motor in comparison, has a longer lifespan. Additionally a brushless motor has greater torque, efficiency, and acceleration than a brushed motor. The Furitek Micro Komodo specifically is also very small compared to other RC car motors on the market, making it fit well with our small chassis.

To control steering, we have a metal gear servo motor. The servo motor that originally came with the RC chassis had a weaker plastic gear and a five-wire connection. Our new servo has a three-wire connection, which we can control and power with the HAT. 

To send signals to both these components, we use a Furitek Lizard Pro Electronic Speed Controller (ESC).

#### Mounting
For the mounting of the motor and servo, they were slotted into the original compartments of the chassis. For everything else, they are mounted on a 3D printed platform. The platform has recesses for the Raspberry Pi board, the fan, and the battery. Additionally, there is a stand that the camera is screwed into.

&nbsp;

### 👀 Sensors 👀

#### Camera
 - information

&nbsp;

### Electricty

&nbsp;

### Power

&nbsp;
