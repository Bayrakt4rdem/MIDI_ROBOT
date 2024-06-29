# MIDI ROBOT

###### *Built for:*
- METU ME Department ME462 Capstone Project
###### *Built by:*
- Ege Sarp Dengizmen
- Erdem Bayraktar
- Muhammed Ömer Yiğit
- Omar Habib
- Seçkin Eren Yetim


<p align="center">
<img src="https://github.com/Bayrakt4rdem/MIDI_ROBOT/blob/main/Pictures/combined2.png" width=100% height=100%>
</p>


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
3D Printer Model: Creality CR-6 SE 
Material: PLA filament <br>
Nozzle / build plate temperature: 210 / 65 degrees <br>
Layer height: 0.2 mm <br>
Initial layer height: 0.2 mm <br>
Line width: 0.4 mm <br>
Wall line count: 3 <br>
Infill density: 30.0% <br>
Infill pattern: Lines <br>
Print speed: 70.0 mm/s <br>
Infill speed: 100.0 mm/s <br>
Wall speed: 35.0 mm/s <br>
Initial layer speed: 35.0 mm/s <br>
Fan: ON <br>
Supports: ON / Normal 45 degree overhang <br>
Adhesion type: NO <br>

The recommended settings for slicing the STL files for TPU material are as follows: <br>
3D Printer Model: Ender 3 V2 (modified for direct drive extrusion)
Material: TPU filament <br>
Nozzle / build plate temperature: 215 / 65 degrees <br>
Layer height: 0.2 mm <br>
Initial layer height: 0.2 mm <br>
Line width: 0.8 mm <br>
Wall line count: 3 <br>
Infill density: 100.0% <br>
Infill pattern: Lines <br>
Print speed: 50.0 mm/s <br>
Infill speed: 50.0 mm/s <br>
Wall speed: 25.0 mm/s <br>
Initial layer speed: 35.0 mm/s <br>
Fan: ON <br>
Supports: OFF / Normal 45 degree overhang <br>
Adhesion type: NO <br>

## Power Circuitry

Basically in our power circuitry we have 18V, 12.8V, 5V, 3.3V power lines.

- 18 Volts Power is used to feed UPS and the UPS feeds the Raspberry Pi
- 18 Volts power is transformed into 12 Volts to feed the steppers
- 12 Volts power is transformed into 5 Volts to feed the led strips
- 12 Volts Power is also transformed into 3.3 Volts to feed the Raspberry Pi Pico and rest of the sensors.

### Electronic Diagrams & PCB Designs

The PCB Files and Electronic Schematics can be found under the PCB&Schematics/ directory.  
The Fusion 360 software is used while designing the PCB circuit. <br>

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
This project uses a Raspberry Pi 4b as the main computer of the robot. Flash the latest Raspberry Pi OS 64-bit version onto your sd card. We recommend using the **PiImager**. Make sure to configure your image to your needs (enable SSH, set wlan key and password, setup username and password). Flash the sd card, insert it into your Pi and power it on. Now, to **SSH** into RPi4, open up a terminal on the PC and use the command `ssh pi@<yourusername>.local`. The console will ask for your password, enter it. In order to setup the Pi, you should clone the software files repo into your home adress using `git clone`, after the files are cloned go inside the `Raspi_files`. Inside the `Raspi_Files` folder give execution rights to the setup file using, `chmod +x setup_pi.bash` and then relax :) the script will take care of all the Pi configuration, docker installation, building of the docker container, adding necessesary configs modifications etc. Feel free to inspect the bash script. <br>
**Important Note:** this script adds a function to your `.bashrc` such that everytime you ssh into your Pi the project docker container opens up. You can modify your `.bashrc` file to change this behaviour as needed.  <br>
After the script the Pi will reset itself, you can just go inside `Examples` and try some examples. Provided that you pip installed the projects python package. See below for that. <br>
### Why Use Docker?
We have opted with using a Docker image because it allows for end-users like you to get right into the action with a ready to use image with all required dependencies of our project pre built into it. One more advantage of this is even if you are not running on a RPi 4 you can still use the Dockerfile provided in **Docker_files** folder to build the ready to use image and get started on whatever software you are running. If you want to add functionality just modify the docker file and rebuild it again.
<br>
### Rasberry Pi Pico/ESP
MIDI has Raspberry Pi Pico W microcontroller. MIDI can be used with any microcontroller(ESP32) that can be coded in MicroPython. <br>
To flash MicroPython into Pico W, this [tutorial](https://projects.raspberrypi.org/en/projects/getting-started-with-the-pico/3) could be followed. After flashing MicroPython, the provided scripts should be uploded into Pico W from [here](https://github.com/momeryigit/ME462-MIDI/tree/omar-test/sarp-esp).


### API
Midibot's API is open-source and published on PYPI (see the latest version [here](https://pypi.org/project/romer-midibot/)). It requires Python 3.6 or newer and `pyserial`.

To install `pyserial`, you can use:
```bash
pip install pyserial
```

To install `romer-midibot`, you can use:
```bash
pip install romer-midibot
```

This API is designed to connect to, control, and get readings from Midibot using Python. First, import the `DifferentialDriveRobot` class:
```python
from romer_midibot import DifferentialDriveRobot as Robot
```

To initialize a robot class:
```python
robot = Robot(serial_port="replace_with_your_serial_port")
```

#### The arguments are as follows:
- **`serial_port` (str)**: Serial port on PC or Raspberry Pi for connecting to the robot.
- **`baudrate` (int, optional)**: Baud rate for serial communication. Defaults to 115200.
- **`timeout` (int, optional)**: Timeout in seconds for communication operations. Defaults to 1.
- **`ip` (str, optional)**: IP address of Pico for socket communication. Defaults to "192.168.137.28".
- **`config_file` (str, optional)**: Path to the config.json file. Defaults to None.
- **`socket_port` (int, optional)**: Port for socket communication on PC or Raspberry Pi. Defaults to 8080.
- **`stepper_ids` ([int], optional)**: Integer array of stepper motor IDs. Defaults to [1, 2].
- **`u_ids` ([int], optional)**: Integer array of ultrasonic sensors. Defaults to [1, 2, 3, 4].
- **`b_ids` ([int], optional)**: Integer array of bumper sensors. Defaults to [1, 2, 3, 4].
- **`imu_connected` (bool, optional)**: Whether IMU is connected. Defaults to False.
- **`u_median_filter_len` (int, optional)**: Length of median filter window. Defaults to 3.
- **`default_emergency_behavior` (bool, optional)**: Default behavior of bumper switches coded on Pico. Defaults to True. Change to False if custom behavior is desired. Bumper switches will be polled for data if enabled.

To initialize the robot API, there are two methods:
1. Manually enter parameters into the class initializer.
2. Use a config file.

* **1**: To initialize by manually entering parameters, keep `config_file` as `None` and modify the other default parameters as desired. The API enables/disables robot peripherals as defined by initialized parameters from the user, which will be merged with a default [config_file](https://github.com/momeryigit/ME462-MIDI/blob/main/romer_midibot/romer_midibot/default_configs.py) to send to the Pi Pico.

* **2**: To use a config file, ignore the other parameters, and provide the path to your `config_file.json`. It should be similar to [this](https://github.com/momeryigit/ME462-MIDI/blob/main/romer_midibot/romer_midibot/default_config.json). Example:
```python
robot = Robot(serial_port="COM6", config_file=r"path/to/your/config/file")
```

Once your robot is initialized, you can connect via serial or socket (still in development) as follows:
```python
robot.connect(connection_type="serial")
```
```python
robot.connect(connection_type="socket")
```
Given that the Pi Pico is running and waiting for a connection, handshakes will be exchanged, and the config file will be sent to the Pico where it will initialize from it. If all this is successful, you are ready to send commands to the robot and receive data from it. The API will start a thread just to poll serial or socket ports and manage received data.

#### Robot Commands
* To set a data callback function that will be called whenever data is received:
```python
robot.set_data_callback(callback)
```
  - **`callback`**: Callback function called with raw data, whenever received from serial. Keep it low cost to prevent lag in data polling.

* To manually send a command to the robot:
```python
robot.send_command(command)
```
  - **`command`**: Command string sent to Pico.

* To send a pause command:
```python
robot.send_pause_command()
```
This stops all the robot's movements and data sending until either a continue or re-config command is received.

* To send a continue command:
```python
robot.send_continue_command()
```

* To disconnect from the robot:
```python
robot.disconnect()
```

* To configure the Pico with the sensor and motor configuration:
```python
robot.pico_config()
```

* To reconfigure the Pico with a new configuration file:
```python
robot.pico_reconfig(config_file)
```
  - **`config_file`**: Path to the new configuration file.

* To set the number of ticks and duration for the left and right wheels of the robot:
```python
robot.set_ticks_duration(left_ticks, right_ticks, duration_l, duration_r)
```
  - **`left_ticks`**: Number of ticks for the left wheel.
  - **`right_ticks`**: Number of ticks for the right wheel.
  - **`duration_l`**: Duration for the left wheel to complete the ticks. In s
  - **`duration_r`**: Duration for the right wheel to complete the ticks. In m

* To set the distance for each wheel and the duration to reach the distance:
```python
robot.set_distance_duration(left_distance, right_distance, duration_l, duration_r)
```
  - **`left_distance`**: Distance for the left wheel. In m
  - **`right_distance`**: Distance for the right wheel. In m
  - **`duration_l`**: Duration for the left wheel to reach the distance. In s
  - **`duration_r`**: Duration for the right wheel to reach the distance. In s

* To set the speed of the left and right wheels of the robot:
```python
robot.set_speed(left_speed, right_speed)
```
  - **`left_speed`**: Speed, in frequency, of the left wheel.
  - **`right_speed`**: Speed, in frequency, of the right wheel.

* To move the robot forward for a specified duration:
```python
robot.move_forward(duration=1, speed=500)
```
  - **`duration`**: Duration, in seconds, to move forward. 
  - **`speed`**: Speed, frequency, at which to move forward.

* To rotate the robot at a given speed in a given direction for a set duration:
```python
robot.rotate(duration=5, speed=800, direction="cw")
```
  - **`duration`**: Duration to rotate. In seconds
  - **`speed`**: Speed at which to rotate. In frequency
  - **`direction`**: Direction to rotate ("cw" for clockwise, "ccw" for counterclockwise).

* To convert desired linear and angular speeds to wheel frequencies and set the robot speed:
```python
robot.send_twist(linear_speed, angular_speed)
```
  - **`linear_speed`**: Linear speed. In m/s
  - **`angular_speed`**: Angular speed. In rad/s

* To stop the robot:
```python
robot.stop()
```

* To immediately stop the robot:
```python
robot.emergency_stop()
```

* To turn off the LEDs on the robot:
```python
robot.turn_leds_off()
```

* To fill all LEDs with a given color:
```python
robot.fill_leds(color)
```
  - **`color`**: Color to fill the LEDs with. RGB tuple (100, 20, 20) for example

* To set the color of a single LED pixel:
```python
robot.set_led_pixel(np_id, pixel_index, color)
```
  - **`np_id`**: ID of the neopixel strip.
  - **`pixel_index`**: Index of the LED pixel.
  - **`color`**: Color to set the LED pixel. RGB tuple (100, 20, 20) for example

* To get sensor data from the robot (default is ultrasound sensor data):
```python
robot.get_sensor_data(sensor_type="u")
```
  - **`sensor_type`**: Type of sensor data to get ("u": ultrasonic, "b": bumper, "i", imu).

* To request the current status of the robot:
```python
robot.get_status()
```

* Destructor to stop the robot and disconnect on deletion:
```python
del robot
```

Be sure to send disconnect command before unplugging the pico. Refer to examples or demos for example uses of the API.

## Trying out the ROS nodes
Midibot comes with some essentials to get you starting on using ROS2 on midibot. The provided docker container is ROS2 ready, all you have to do is go inside the `dev_ws` folder and run `colcon build --symlink-install`. This will build the source files and ready up your systen for testing. After build is complete make sure to source the enviorement by using `source install/setup.bash` (.bash/.zsh depending on the terminal you are using). This command essentially includes our ROS2 packages in that current terminal session. The core ROS installation is sourced in every terminal since `source /opt/ros/iron/setup.bash` is included inside the .bashrc file of the docker image. The command simply overlays our package contents over the core ROS2 installation. <br>
After sourcing you can get into testing. Simply run the launch file for RPi using `ros2 launch midi_bringup pi.launch.py`, this runs the node for serial communication and the lidar if given as an argument `ros2 launch your_package pi.launch.py lidar:=true`. Then on your PC again source the ROS2 enviorement and run `ros2 launch midi_bringup drive.launch.py`. This sets up the PC side nodes, opens Rviz and sets up the controller nodes. Then by connecting an xbox controller to your machine you can control the robot using the controller. <br>





## Conclusion
The MIDI Robot is a versatile and accessible platform designed for both beginners and experts in robotics. Its modular design and easy programmability allow users to integrate new sensors and tools as needed, making it adaptable for various scientific and educational applications. With MIDI, users can gain hands-on experience in electronics, mechanics, and programming, bridging the gap between theory and practice. This robot is ideal for learning and experimenting, providing an approachable entry point for newcomers while offering advanced capabilities for seasoned researchers. Overall, the MIDI Robot empowers users to explore and innovate in robotics, making it a valuable tool for education, research, and personal projects. Its design ensures reliability and ease of use, fostering a deeper understanding and appreciation of robotic systems.
