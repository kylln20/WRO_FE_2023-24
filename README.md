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
We use a pre-built 1/24 scale RC car (15 cm in length), as opposed to the 1/18 scale car (26.5 cm in length) from the previous year. This is to accomodate the addition of the magenta parking lot in the obstacle challenge, and it allows us to simply park head-on, instead of actually parallel parking. 

#### Drive Motor
Our car uses a Furitek Micro Komodo brushless motor. Brushless motors refer to the lack of small "brushes" in the motor that a brushed motor would have. This design reduces friction within the motor, and provides a longer lifespan, greater torque, efficiency, and acceleration. 

The Furitek Micro Komodo specifically is also very small compared to other RC car motors on the market, making it fit well with our small chassis.

#### Turning Motor
To control steering, we have a metal gear servo motor. The servo motor that originally came with the RC chassis had a weaker plastic gear and a five-wire connection. We replaced it with a new servo that has a three-wire connection, which we can control and power with the Raspberry Pi HAT. 

#### ESC
To send signals to both these components, we use a Furitek Lizard Pro Electronic Speed Controller (ESC), which came with the brushless motor.

&nbsp;

### 👀 Sensors 👀

#### Camera
 - information

&nbsp;

### Electricty

&nbsp;

### Power

&nbsp;
