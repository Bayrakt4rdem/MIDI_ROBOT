# MIDI ROBOT

###### *Built by:*
- Ege Sarp Dengizmen
- Erdem Bayraktar
- Muhammed Ömer Yiğit
- Omar Habib
- Seçkin Eren Yetim

---
# Table of Contents
- [Introduction](#introduction)
    - [What is MIDI](#what-is-midi)
    - [Why MIDI is Necessary](#why-midi-is-necessary)
    - [Motivation Behind the Project](#motivation-behind-the-project)
    - [Acknowledgement and Credits](#acknowledgement-and-credits)
- [Features](#features)
- [Building the MIDI](#building-the-midi)
    - [Hardware Components](#hardware-components)
        - [Components that Must be Purchased](#components-that-must-be-purchased)
        - [Components that Must be Printed](#components-that-must-be-printed)
    - [Electronic Circuitry](#electronic-circuitry)
        - [Electronic Diagrams](#electronic-diagrams)
        - [PCB Design](#pcb-design)
    - [Connecting the Elements](#connecting-the-elements)
- [Software Documentation](#software-documentation)
    - [Software and Versions](#software-and-versions)
    - [Installing Guide](#installing-guide)
    - [Docker Setup Instructions](#docker-setup-instructions)
    - [Rasberry Pi Pico/ESP](#rasberry-pi-pico/esp)
    - [API](#api)
    
- [Testing for Connections](#testing-for-connections)
    - [Confirming the Connections](#confirming-the-connections)
    - [Troubleshooting](#troubleshooting)
    - [Demo](#demo)

- [Detailed Usage](#detailed-usage)
    - [Detailed Instruction](#confirming-the-connections)
    - [Configuration and Customization](#configuration-and-customization)
    - [Command Line Interface](#command-line-interface)

## Introduction
---

### What is MIDI 
MIDI is a robotic project that focus on improvised programmability and easy integration of additional sensors. It is made to be long-lasting and versatile with a design that conveniences assembly and manufacturing processes.

Its modular design is key to its ease of integration of new sensors and tools as mission needs change. This flexibility increases the rover's capacity to carry out a wide range of scientific studies for indoor environments with uninterruptable power supply.

Essential component of MIDI is its programmability, which gives researchers the ability to alter and improve robotic operations in real time with built in sensors and expandability. With comprehensive array of sensors and convenience to control them, it can be utilized by wide range of users that includes anyone from a beginner to a PhD student.

### Why is MIDI is Necessary
Understanding the basic of robotics requires wide range of information from diverse professions combined. In fact, learning all discreate subject is exhausting and discouraging. MIDI robot is specifically designed to programming in ease by user. Furthermore, building and assembling this robot may teach the user diverse information about power electronics, mechanics and programming that includes pi pico and python with considerably less effort.

For expert users, MIDI may be considered as a strong candidate for testing their algorithms, making scientific researches or just for fun. With the expandable sensor design with proper communication between them, one can implement diverse tasks to MIDI without any worries about power-loss issues or lose data.

### Motivation Behind the Project
Learning the theoretical part of the robotic is always enjoyable for us. Still, without proper application, this knowledge is likely to be forgotten or become meaningless. For this project, we have aimed to apply our theoretical skills to build a robot that makes us feel satisfied. Still, building the robot was challenging. We learned lots of things while solving the problems that we faced. In fact, one can say that theory feeds the application but applicational issues creates the theory.

The main idea behind building the MIDI is helping the people to make their own robots to learn more about robotics and testing their codes and theoretical implementations. All the code implementations and mechanical designs are specifically made for learning the robotics in more convenient way. In fact, From high-school student to graduate students, anyone that really interested in robotics can use this robot to make some additions, to make some experiments or daily use that includes lots of fun.

### Acknowledgement and Credits
While doing this robot, we have faced diverse problems that is about the facility problems and guiding. In fact we want thank the Ahmet Buğra Koku for guiding as, and Micromanufacturing Laboratory with Romer to allow us use their resources.

## Features
---
### Mechanical-Features
In mechanical manner, MIDI can hold about 20 kilogram without facing any problems. It is specifically designed for indoor use in terms of elevated platforms, sensor integration and enviromental effects. Still, one can change the sensors and some necessary components to adapt MIDI for outdoor use.

With its modular design, one can add many other stuff to the platform in there. Furthermore, it is very easy to add new floors in case of lack of area or some other reasons.

There are 4 bumpers on the edges to realize the collusion of midi with sufficient damping. Furthermore, 4 ultrasonic sensors are implemented to perceive the environment in case of upper colusions and falling down. Lidar, camera and IMU is also implemented to perveive the world in enhanced manner to achieve diverse tasks.

For power supply, 18V Makita battery is used with UPS Module. Electromechanical part is specifically designed to avoid  battery-loss problems.


## Building the MIDI
---

### Hardware Components

#### Components that Must be Purchased
To build the model that provided, one can need diverse sensors and components. In fact, necessary components must be listed. One can found the necessary equipments with some links below. 

- [MPU6050 IMU x 1](https://www.robotistan.com/mpu6050-6-eksen-ivme-ve-gyro-sensoru-6-dof-3-axis-accelerometer-and-gyros)
- [YL99 Impact Switch Module x 4](https://www.robolinkmarket.com/yl-99-darbe-svic-sensor-modulu)  
- [HC-SR04 Ultrasonic Distance Sensor x 4](https://www.robotistan.com/hc-sr04-ultrasonic-distance-sensor)
- [A4988 Stepper Motor Driver x2](https://www.robotistan.com/a4988-step-motor-surucu-karti-kirmizi)
- [YDLidar G2 x1](https://www.robotistan.com/ydlidar-g2-lidar-mesafe-sensoru)
- [NEMA17 Stepper Motor x 2](https://www.robolinkmarket.com/17hs4401-nema17-step-motor) 
- [Makita 18V Battery x1](https://www.powertoolworld.co.uk/brands/makita/makita-batteries-chargers/makita-batteries-18v-lxt)

Note, other NEMA17 models or other power supply models may be selected. Still, battery must be adjusted as 18 Volts.

- [UPS Module 3S x 1](https://www.amazon.com.tr/waveshare-Kesintisiz-UPS-Destekler-3S/dp/B0BQC2WNR8)
- [Rasberry Pi Pico x 1](https://www.robotistan.com/raspberry-pi-pico?gad_source=1&gclid=EAIaIQobChMI8vb6z6zyhgMV0ZeDBx1ifA3kEAAYASAAEgIh4PD_BwE)
- [Rasbeery Pi 4 x 1](https://www.robotistan.com/raspberry-pi-4-4gb)
- [lm2096 Voltage Regulator x 2](https://www.robotistan.com/mini-ayarlanabilir-3a-voltaj-regulator-karti-lm2596-adj) 
- [XL4016E1 DC-DC Step Down x 1](https://www.az-delivery.de/en/products/xl4016e-yh11060d)
- [Megapixel Wide Angle Fish Eye Camera x 1](https://www.amazon.com/Hilitand-Fisheye-Professional-Adjustable-Raspberry/dp/B0877439FF)
- [30x30 Fan x 1](https://www.rhino3dprinter.com/urun/30x30-fan-12v-ve-24v-secenekleriyle)
- [8 Strip Led x 2](https://www.robolinkmarket.com/neopixel-8li-serit)

For electronic integration lots of jumper may be needed. In fact, given quantities below may not be sufficient.
-[Male to Male Jumper x 40](https://www.robotistan.com/20cm-40-pin-m-m-jumper-wires)
-[Female to Male Jumper x 40](https://www.robotistan.com/20cm-40-pin-m-f-jumper-wires)
-[Female to Female Jumper x 40](https://www.robotistan.com/20cm-40-pin-f-f-jumper-wires)
-[100 μf Capacitor x 1 ](https://www.robotistan.com/25v-100uf-capacitor)
-[10 μf Capacitor x 2 ](https://www.robotistan.com/10-uf-16-v-elektrolit-kondansator)

For mechanical components,
-[20x20 Sigma Profile 30cm x4](https://www.robolinkmarket.com/20x20-v-slot-sigma-profil-kanal-6-320mm)
-[20x20 Sigma Profile 21cm x4](https://www.robolinkmarket.com/20x20-v-slot-sigma-profil-kanal-6-320mm)
-[20x20 Sigma Profile 8cm x4](https://www.robolinkmarket.com/20x20-v-slot-sigma-profil-kanal-6-320mm)
-[20x20 Sigma Profile 6cm x4](https://www.robolinkmarket.com/20x20-v-slot-sigma-profil-kanal-6-320mm)

Buying one of them and cutting is sufficient. For assembly:
-[Sigma Connection Component x 12](https://www.robolinkmarket.com/20x20-genis-kose-baglanti?_sgm_campaign=scn_c0a573c60e000&_sgm_source=6893&_sgm_action=click)
-[M3 16mm Screw x 30](https://www.amazon.com/Phillips-Countersunk-Machine-Screws-100pcs/dp/B018RSUVFU)
-[M4 16mm Screw x 30](https://www.amazon.com/Phillips-Machine-Screws-Metric-Thread/dp/B01HF7BLIA)
-[M3 Nuts x 30](https://www.robotistan.com/m3-somun-50-adet)
-[M4 Nuts x 30](https://www.robotistan.com/m4-somun)

#### Components that Must be Printed
For stabilizing the items some parts are 3D printed. Detailed model can be seen at:

[Assembly Design](https://a360.co/3XIiJEC)

All parts link can be reached from below:

[Airless Tire x 2](https://a360.co/45ATHcI)
[Wheel Caster x 4](https://a360.co/45ATHcI)
[Wheel Connector x 3](https://a360.co/4bb6Udq)
[Side Tire Part x 2](https://a360.co/3RF8P2N)
[Sigma Holder x 8](https://a360.co/3RIavc6)
[Back Tire Part x 1](https://a360.co/3z8LiAY)
[Bumpers x 4](https://a360.co/3XACzlq)
[Fron Tires x 2](https://a360.co/4biU4dd)
[Plexi Holders x 16](https://a360.co/45EZhdV)
[Sigma Plugging Element x 8](https://a360.co/3VFlunN)
[Ultrasonic Holder Element x 4](https://a360.co/3z8LKPG)
[Lidar Platform x 3](https://a360.co/3KY62yg)
[Lidar Cage Holder x 4](https://a360.co/3xCmzEI)

Ground for all floors are designed as plexi. Still, one can change the material.
[Plexi Ground x 2](https://a360.co/3L1k55O)

Casings for PCBs, UPS, Rasberry Pi and Makita is given as a single file given below:
[Casings x 1](https://a360.co/3xvUAqk)

Note, soldering equipment will also be needed.

### Electronic Circuitry
There are some voltage regulating processes available in the power assembly. In very basic manner:
- 18 Volt Power is used to feed UPS
- UPS feeds the Lidar
- 18 Volt power is transformed into 12 Volt to:
    - Feed the Steppers
- 12 Volt power is transformed into 5 Volt 
    - Feed the Strip Leds
- 5 Volt Power is transformed into 3.3 Volt to:
    - Feed the Drivers
    - Feed the Bumpers 
    - Feed the IMU
    - Feed the Ultrasonic Sensors


#### Electronic Diagrams

![Detailed Circuit Diagram for Pico](image_power_pico) 
#### PCB Design

### Connecting the Elements

## Software Documentation
---
### Software and Versions
### Installing Guide
### Docker Setup Instructions)
### Rasberry Pi Pico/ESP
### API




## Conclusion


