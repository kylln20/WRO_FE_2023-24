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
| RC Car | [`Carisma GT24`](https://www.canadahobbies.ca/product/hobby-brands/carisma-rc/gt24-124th-4wd-toyota-celica-gt-four-st185-wrc/) | $263 |
| RC Car Battery | [`Gens Ace 1300mAh Battery`](https://www.examplelink.com) | $1000 | *link to be found*
| Drive Motor & ESC | [`Furitek Komodo Motor + Lizard Pro ESC`](https://www.xtremerc.ca/products/furitek-scx24-stinger-brushless-power-system-w-1212-3450kv-brushless-motor?_pos=1&amp;_sid=cf7c35a05&amp;_ss=r) | $226 |
| Turning Motor | [`Hitec HS-5055MG Servo Motor`](https://ca.robotshop.com/products/hs-5055mg-metal-gear-micro-servo-motor?srsltid=AfmBOopv8Z7LoCVOEqe16w05ZV-R78dNmy7dappldIxZiQzCJroxcssFc2Y) | $38 |
| CSI Camera | [`SainSmart Wide Angle Fish-Eye Camera`](https://www.amazon.ca/SainSmart-Fish-Eye-Camera-Raspberry-Arduino/dp/B00N1YJKFS/ref=sr_1_5) | $19 |
| Raspberry Pi HAT | [`HiWonder TurboPi HAT`](https://www.hiwonder.com/collections/raspberrypi-bionic-robot/products/turbopi?variant=40112905388119) | $61 [^1] |
| Raspberry Pi | [`Vemico Raspberry Pi 4`](https://www.amazon.ca/Vemico-Raspberry-Kit-Heatsinks-Screwdriver/dp/B09WXRCYL4/ref=sr_1_3) | $176 |
| Raspberry Pi Fan | [`GeeekPi Fan`](https://www.amazon.ca/dp/B07C9H9LJN?psc=1&ref=ppx_yo2ov_dt_b_product_details) | $19 |
| ON OFF Switch | [`DaierTek Switch`](https://a.co/d/05vnrpJD) | $13 |

[^1]:This link is for the `Hiwonder TurboPi Car`, which uses the `HiWonder TurboPi HAT`. We contacted the manufacturer to purchase the HAT directly.

<sup>*3D printed parts were made with the [`BambuLab P1P`](https://ca.store.bambulab.com/products/p1p)</sup>

&nbsp;

### 🚗 Mobility 🚗

<img src="v-photos/rock.webp" width="700" height=auto>

#### Chassis
We use the `Carisma GT24`, a pre-built 1/24 scale RC car (15 cm in length), as opposed to the 1/18 scale car (26.5 cm in length) from the previous year. This is to accomodate the addition of the magenta parking lot in the obstacle challenge, and it allows us to simply park head-on, which is a more efficient procedure compared to parallel parking. 

#### Motors
Our car uses a `Furitek Micro Komodo Brushless Motor`. Brushless motors refer to the lack of small "brushes" in the motor that a brushed motor would have. The design reduces friction within the motor, and provides a longer lifespan, greater torque, efficiency, and acceleration. 

The Furitek Micro Komodo specifically is also very small compared to other RC car motors on the market, making it fit well with our small chassis.

To control steering, we have use a `Hitec HS-5055MG Servo Motor`, which is a metal gear servo motor. The servo motor that originally came with the RC chassis had a weaker plastic gear and a five-wire connection. We replaced it with a new servo that has a three-wire connection, which we can control and power with the `Raspberry Pi Hardware Attached on Top` (HAT). 

To send signals to both these components, we use a `Furitek Lizard Pro Electronic Speed Controller (ESC)`, which came with the brushless motor.

&nbsp;

### 👀 Camera Vision 👀
We use a `SainSmart Wide-Angle Camera`, which carries pixel data to the HAT via a Camera Serial Interface (CSI) cable. Based on said pixel data, we can determine the size (and distance) of surrounding objects, such as the red and green obstacles, the blue and orange lines on the game mat, and the black walls.
&nbsp;

### ⚡ Electricity & Power ⚡

&nbsp;

