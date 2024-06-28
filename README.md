# MIDI ROBOT

###### *Built for:*
- METU ME Department ME462 Capstone Project
###### *Built by:*
- Ege Sarp Dengizmen
- Erdem Bayraktar
- Muhammed Ömer Yiğit
- Omar Habib
- Seçkin Eren Yetim


## Motivation of the Project

### What is MIDI Robot?
MIDI is a robotic project that focuses on improvised programmability and easy integration of additional sensors. It is made to be long-lasting and versatile with a design that conveniences assembly and manufacturing processes.

Its modular design is key to its ease of integration of new sensors and tools as mission needs change. This flexibility increases the robot's capacity to carry out a wide range of scientific studies for indoor environments with uninterruptable power supply.

Essential component of MIDI is its programmability, which gives researchers the ability to alter and improve robotic operations in real time with built in sensors and expandability. With comprehensive array of sensors and convenience to control them, it can be utilized by wide range of users that includes anyone from a beginner to a PhD student.

### Why is MIDI Robot Necessary?
Understanding the basic of robotics requires wide range of information from diverse professions combined. In fact, learning all discreate subject is exhausting and discouraging. MIDI robot is specifically designed to programming in ease by user. Furthermore, building and assembling this robot may teach the user diverse information about power electronics, mechanics and programming that includes pi pico and python with considerably less effort.

For expert users, MIDI may be considered as a strong candidate for testing their algorithms, making scientific researches or just for fun. With the expandable sensor design with proper communication between them, one can implement diverse tasks to MIDI without any worries about power-loss issues or lose data.

### Motivation Behind the Project
Learning the theoretical part of the robotic is always enjoyable for us. Still, without proper application, this knowledge is likely to be forgotten or become meaningless. For this project, we have aimed to apply our theoretical skills to build a robot that makes us feel satisfied. Still, building the robot was challenging. We learned lots of things while solving the problems that we faced. In fact, one can say that theory feeds the application but applicational issues creates the theory.

The main idea behind building the MIDI is helping the people to make their own robots to learn more about robotics and testing their codes and theoretical implementations. All the code implementations and mechanical designs are specifically made for learning the robotics in more convenient way. In fact, From high-school student to graduate students, anyone that really interested in robotics can use this robot to make some additions, to make some experiments or daily use that includes lots of fun.

### Acknowledgement and Credits

While doing this robot, we have faced diverse problems that is about the facility problems and guiding.   
We would like to thank the Ahmet Buğra Koku for guiding us, ROMER and METU Micromanufacturing Laboratory for allowing us to use their manufacturing infrastructure and resources.


## Main features

Structrually MIDI is expected to carry 15 kilograms without facing any problems. It is specifically designed for indoor use. Still, one can change some of the mechanical components of the MIDI to adapt MIDI for outdoor use.

With its modular design, one can add different sensors and payloads to the platform. Due to its modular design, it is very easy to add extra floors to the MIDI.

There are 4 bumpers on the corners of the robot to sense the collision of MIDI. These bumpers both act as a sensor and a mechanical spring.

4 ultrasonic sensors are implemented to sense the distance of the robot with respect to outer obstacles. This sensors can be used to prevent crashes and fall downs. 

A lidar, a wide-angle camera and an IMU is also implemented to perceive the environment in an enhanced manner to achieve diverse tasks.

As a power source an 18V Makita battery is used. There is also an extra UPS module on the robot, in order to prevent direct power cuts of the Raspberry Pi. 

## Shared Documentation and Files

  1. Bill of materials to make replication of the device easier
  2. PCB circuitry design and electronic schematic files   
  1. CAD design files to make further development available   
  1. STL design files for 3D printing availability
  1. Assembly manual for an easier assembly
  2. Software explanations and guides
  3. Example use cases

## Bill of Materials

### Off-the-shelf Materials Which Must be Purchased

To build the model that provided, one will need diverse sensors and components. One can found the necessary materials with some sample links below. 

#### Electronic components
- [MPU6050 IMU x 1](https://www.robotistan.com/mpu6050-6-eksen-ivme-ve-gyro-sensoru-6-dof-3-axis-accelerometer-and-gyros)
- [YL99 Impact Switch Module x 4](https://www.robolinkmarket.com/yl-99-darbe-svic-sensor-modulu)  
- [HC-SR04 Ultrasonic Distance Sensor x 4](https://www.robotistan.com/hc-sr04-ultrasonic-distance-sensor)
- [A4988 Stepper Motor Driver x2](https://www.robotistan.com/a4988-step-motor-surucu-karti-kirmizi)
- [YDLidar G2 x1](https://www.robotistan.com/ydlidar-g2-lidar-mesafe-sensoru)
- [NEMA17 Stepper Motor x 2](https://www.robolinkmarket.com/17hs4401-nema17-step-motor) 
- [Makita 18V Battery x1](https://www.powertoolworld.co.uk/brands/makita/makita-batteries-chargers/makita-batteries-18v-lxt)
- [UPS Module 3S x 1](https://www.amazon.com.tr/waveshare-Kesintisiz-UPS-Destekler-3S/dp/B0BQC2WNR8)
- [Rasberry Pi Pico x 1](https://www.robotistan.com/raspberry-pi-pico?gad_source=1&gclid=EAIaIQobChMI8vb6z6zyhgMV0ZeDBx1ifA3kEAAYASAAEgIh4PD_BwE)
- [Rasbeery Pi 4 x 1](https://www.robotistan.com/raspberry-pi-4-4gb)
- [lm2096 Voltage Regulator x 2](https://www.robotistan.com/mini-ayarlanabilir-3a-voltaj-regulator-karti-lm2596-adj) 
- [XL4016E1 DC-DC Step Down x 1](https://www.az-delivery.de/en/products/xl4016e-yh11060d)
- [Megapixel Wide Angle Fish Eye Camera x 1](https://www.amazon.com/Hilitand-Fisheye-Professional-Adjustable-Raspberry/dp/B0877439FF)
- [30x30 Fan x 1](https://www.rhino3dprinter.com/urun/30x30-fan-12v-ve-24v-secenekleriyle)
- [8 Strip Led x 2](https://www.robolinkmarket.com/neopixel-8li-serit)
- [100 μf Capacitor x 1 ](https://www.robotistan.com/25v-100uf-capacitor)
- [10 μf Capacitor x 2 ](https://www.robotistan.com/10-uf-16-v-elektrolit-kondansator)

  Note that other NEMA17 models or other power supply models may be selected. Still, battery must be adjusted as 18 Volts.

#### Structural components

- [20x20 Sigma Profile 30cm x4](https://www.robolinkmarket.com/20x20-v-slot-sigma-profil-kanal-6-320mm)   
- [20x20 Sigma Profile 21cm x4](https://www.robolinkmarket.com/20x20-v-slot-sigma-profil-kanal-6-320mm)   
- [20x20 Sigma Profile 15cm x4](https://www.robolinkmarket.com/20x20-v-slot-sigma-profil-kanal-6-320mm)   
- [Sigma Connection Component x 12](https://www.robolinkmarket.com/20x20-genis-kose-baglanti?_sgm_campaign=scn_c0a573c60e000&_sgm_source=6893&_sgm_action=click)   
- [M3 16mm Screw x 30](https://www.amazon.com/Phillips-Countersunk-Machine-Screws-100pcs/dp/B018RSUVFU)   
- [M4 16mm Screw x 30](https://www.amazon.com/Phillips-Machine-Screws-Metric-Thread/dp/B01HF7BLIA)   
- [M3 Nuts x 30](https://www.robotistan.com/m3-somun-50-adet)   
- [M4 Nuts x 30](https://www.robotistan.com/m4-somun)   

  Note that sigma profiles are sold as whole 4-6 meters. Buying one and cutting in proper dimensions is necessary.

### Printed Parts

To make the MIDI robot more accessible and easy to replicate, most of the parts are suitable for 3D printing. All parts can be reached from below links:

- [Main Assembly](https://a360.co/3XIiJEC)
- [Airless Tire x 2](https://a360.co/45ATHcI)
- [Wheel Caster x 4](https://a360.co/45ATHcI)
- [Wheel Connector x 3](https://a360.co/4bb6Udq)
- [Side Tire Part x 2](https://a360.co/3RF8P2N)
- [Sigma Holder x 8](https://a360.co/3RIavc6)
- [Back Tire Part x 1](https://a360.co/3z8LiAY)
- [Bumpers x 4](https://a360.co/3XACzlq)
- [Front Tires x 2](https://a360.co/4biU4dd)
- [Plexy Holders x 16](https://a360.co/45EZhdV)
- [Sigma Plugging Element x 8](https://a360.co/3VFlunN)
- [Ultrasonic Holder Element x 4](https://a360.co/3z8LKPG)
- [Lidar Platform x 3](https://a360.co/3KY62yg)
- [Lidar Cage Holder x 4](https://a360.co/3xCmzEI)
- [Plexy Base x 2](https://a360.co/3L1k55O) Bases for all floors are designed for plexy material and to be cut from laser - cutter. Still, one can change the material.
- [Casings for the components x 1](https://a360.co/3xvUAqk) Casings for the PCBs, UPS, Rasberry Pi and Makita are given as a single.

Note that soldering equipment will also be needed.

## Printing Settings for 3D Printed Parts

There are 2 main materials and settings used for 3D printed parts.   
For the elastic components such as springs and tires, TPU material is used.   
For the remaining parts, PLA material is used.   

[Link for the TPU filament](https://www.porima3d.com/porima-tpu-flex-esnek-95a-filament-175mm-1kg)   
[Link for the PLA filament](https://www.porima3d.com/porima-pla-filament-175mm-1kg)     

The production files can be found under the /CAD_files directory.   
The STEP files are for further development and editing the model in any CAD environment. <br>
The STL files are for manufacturing the necessary parts with a 3D printer. <br>
 <br>
 AŞAĞISI YANLIŞ BURASI DÜZENLENECEK AYARLARDAN BAKARAK
 
The recommended settings for slicing the STL files for PLA material are as follows: <br>
Material: PLA filament <br>
Nozzle / build plate temperature: 210 / 65 degrees <br>
Layer height: 0.2 mm <br>
Initial layer height: 0.3 mm <br>
Line width: 0.4 mm <br>
Wall line count: 3 <br>
Infill density: 40.0% <br>
Infill pattern: Lines <br>
Print speed: 70.0 mm/s <br>
Infill speed: 100.0 mm/s <br>
Wall speed: 35.0 mm/s <br>
Initial layer speed: 35.0 mm/s <br>
Fan: ON <br>
Supports: ON / Normal 45 degree overhang <br>
Adhesion type: Brim <br>

The recommended settings for slicing the STL files for TPU material are as follows: <br>
Material: TPU filament <br>
Nozzle / build plate temperature: 210 / 65 degrees <br>
Layer height: 0.2 mm <br>
Initial layer height: 0.3 mm <br>
Line width: 0.4 mm <br>
Wall line count: 3 <br>
Infill density: 40.0% <br>
Infill pattern: Lines <br>
Print speed: 70.0 mm/s <br>
Infill speed: 100.0 mm/s <br>
Wall speed: 35.0 mm/s <br>
Initial layer speed: 35.0 mm/s <br>
Fan: ON <br>
Supports: ON / Normal 45 degree overhang <br>
Adhesion type: Brim <br>

## Power Circuitry

Basically in our power circuitry we have 18V, 12.8V, 5V, 3.3V power lines.

- 18 Volts Power is used to feed UPS and the UPS feeds the Raspberry Pi
- 18 Volts power is transformed into 12 Volts to feed the steppers
- 12 Volts power is transformed into 5 Volts to feed the led strips
- 12 Volts Power is also transformed into 3.3 Volts to feed the Raspberry Pi Pico and rest of the sensors.

### Electronic Diagrams & PCB Designs

The PCB Files and Electronic Schematics can be found under the PCB&Schematics/ directory.  
The Fusion 360 software is used while designing the PCB circuit. <br>
Below is the link to the up to date files: <br>
[LİNK EKLENECEK]d <br>
The PCB is designed as single layer, in order to make fabrication process easier. <br>
But due to size limitations of the PCB, not all the wiring fit into one layer. <br>
As a result there is two options: <br>
Easier option: Fabricate only the top layer as a single layer PCB and make additional air wiring afterwards.<br>
(less than 10 additional airwires are  necessary). <br>
Second option: Fabricate the PCB as a two layer one. <br>

### Assembly Guide

## Software Documentation

### Software and Versions
### Installing Guide
### Docker Setup Instructions
To connect to Raspberry Pi 4(RPi4), one should connect RPi4 and the PC to a network with a name **midiwifi**. Password for this network should be **midi1234**. Now, to **SSH** into RPi4, open up a terminal on the PC and use the command `ssh midibot@local.bash`. The password for RPi4 is **midi1234**. Current OS in RPi4 is **Raspberry Pi OS**. However, the docker is employed in MIDI. <br>
To understand and install docker, one can follow: [Docker for Robotics by Articulated Robotics](https://www.youtube.com/watch?v=XcJzOYe3E6M&list=PLunhqkrRNRhaqt0UfFxxC_oj7jscss2qe) <br>
For just installing the docker: [Starting from 0:39](https://www.youtube.com/watch?v=SAMPOK_lazw&list=PLunhqkrRNRhaqt0UfFxxC_oj7jscss2qe&index=2) <br>
MIDI's docker file can be reached [here-link değişecek](https://github.com/momeryigit/ME462-MIDI/blob/main/docker_trial/Dockerfile). <br>
As can be seen from MIDI's dockerfile, it is customized. 
Camera and other stuff... <br>
<br>
After installing the docker into Raspberry Pi 4, run a docker container and attach to it (or use `exec -it`). `dev_ws` should be accesible. To check this, go to root directory with `cd /`. Then list all the directories with `ls`. Now, go into developer workspace with `cd dev_ws`.
### Rasberry Pi Pico/ESP
### API




## Conclusion


