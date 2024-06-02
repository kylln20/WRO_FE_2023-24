Engineering Documentation
======

This repository details Team <>’s building and programming process in our 2nd participating year of the 2024 WRO Future Engineers Competition.



## Content of Repository
* `models` - 3D CAD files
* `others` - other essential files
* `schemes` - electrical schematics
* `src` - main and other programs to run/control software
* `t-photos` - team photos
* `v-photos` - robot photos
* `video` - video demonstration

---

## Content of README

* #### Hardware
  * [`Mobility`](#mobility) - hardware for robot movement
  * `Sensors` - sensors used
  * `Electricity` - electrical wiring and circuit boards
  * `Power` - power supply
    
* #### Software
  * `Object Detection` - 
  * `Movement Logic` - 
  * `lorem` - 
  * `ipsum` - 
    
* #### Design Process

---

## Hardware

### Mobility

<img src="v-photos/rock.webp" width="700" height=auto>

  Last year, we used a pre-built 1/18 scale RC car (26.5 cm in length). However, with the addition of the magenta parking lot in the obstacle challenge, we decided that we wanted a smaller chassis. Since the length of the parking lot is always 20 cm, by choosing a car of shorter length then 20 cm, we avoid having to parallel park. As such, we chose a 1/24 scale RC car (15 cm in length).

(insert info about HAT)
  
  Our car uses a Furitek Micro Komodo Brushless Motor with the Furitek Lizard Pro Electronic Speed Controller (ESC). A brushless motor was chosen over a brushed motor since the latter is powered by direct contact, and can wear down over time. A brushless motor in comparison, has a longer lifespan. Additionally a brushless motor has greater torque, efficiency, and acceleration than a brushed motor. 
    
  To control steering, we have a metal gear servo motor. The servo motor that originally came with the RC chassis had a weaker plastic gear and a five-wire connection. Specifically, our new servo has a three-wire connection, which we can connect and power with the HAT.

## Components
* Brushless Motor and ESC: [Furitek Komodo Motor + Lizard Pro ESC](https://www.xtremerc.ca/products/furitek-scx24-stinger-brushless-power-system-w-1212-3450kv-brushless-motor?_pos=1&amp;_sid=cf7c35a05&amp;_ss=r)
* Raspberry Pi Fan: 
* CSI Camera: [DORHEA 160° Camera](https://www.amazon.com/Raspberry-Camera-Module-160FOV-Fisheye/dp/B083XMGSVP/)
* Servo Motor: [Hitech HS-5055MG](https://ca.robotshop.com/products/hs-5055mg-metal-gear-micro-servo-motor?srsltid=AfmBOopv8Z7LoCVOEqe16w05ZV-R78dNmy7dappldIxZiQzCJroxcssFc2Y)
* Raspberry Pi HAT Extension:
* RC Car Lipo Battery: 
* RC Car: [Carisma GT24](https://www.canadahobbies.ca/product/hobby-brands/carisma-rc/gt24-124th-4wd-toyota-celica-gt-four-st185-wrc/)

