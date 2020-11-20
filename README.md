# robotcar

## Develop and test Advanced Driver Assistance Systems (ADAS) with the robotcar using Python, ROS, OpenCV and TensorFlow

![robotcar](relative/path/to/img.jpg?raw=true "Title")

## 1 Project Information
=====================

Project of the [Faculty of Computer Science](https://www.hs-furtwangen.de/en/faculties/computer-science/) of the [Furtwangen University](https://www.hs-furtwangen.de/en/) inside the master degree [Mobile Systems](https://www.hs-furtwangen.de/en/programmes/mobile-systems-master/).

Part of the master thesis "Entwicklung einer erweiterbaren Simulationsplattform für Fahrerassistenzfunktionen am Beispiel von Python, ROS, OpenCV und TensorFlow"

Developer: Michael Christian Dörflinger

Supervisiors:

- [Prof. Dr. Steffen Thiel](https://www.hs-furtwangen.de/personen/profil/75-steffenthiel/)
- [Ingo Maindorfer](https://www.hs-furtwangen.de/personen/profil/1707-ingomaindorfer/)

The development of driver assistance functions is extensive and expensive. Virtual simulations significantly improve the development process, but do not reflect the reality of vehicle actuators and sensors. Development under physically real conditions is therefore indispensable. The thesis will investigate how such a simulation platform can be implemented using a small robot car, taking into account the expandability of new sensor technology and driver assistance functions.

## 2 Hardware
=====================

(Not the actual used components because you can get it cheaper.)

| Quantity | Modul | Article | Estimated price |
|--------- | ------| --------| ----------------|
| 1 | chassis | [4WD RC Smart Car Chassis](https://www.elecrow.com/4wd-smart-car-robot-chassis-for-arduino-servo-steering.html) | 40,00€ |
| 1 | chassis | [20x20 cm perforated plate](https://smile.amazon.de/dp/B00QAGIXLG/ref=pe_3044161_185740101_TE_item) | 2,50€ |
| 1 | chassis | [mounting kit raspberry pi](https://www.amazon.de/gp/product/B07G5HM9ZT/ref=ppx_yo_dt_b_asin_title_o04_s00?ie=UTF8&psc=1) | 12,00€ |
| 1 | onboard computer | [Raspberry Pi 4 Model B](https://www.raspberrypi.org/products/raspberry-pi-4-model-b/?resellerType=home&variant=raspberry-pi-4-model-b-8gb) | 77,50€ |
| 1 | onboard computer | [Raspberry Pi cooler](https://www.amazon.de/gp/product/B07JGNF5F8/ref=ppx_yo_dt_b_asin_title_o06_s00?ie=UTF8&psc=1) | 8,99€ |
| 1 | onboard computer | [Raspberry Pi stacking header](https://www.amazon.de/gp/product/B07NTH2RZX/ref=ppx_yo_dt_b_asin_title_o08_s00?ie=UTF8&psc=1) | 10,35€ |
| 1 | onboard computer | [Raspberry Pi port doubler](https://www.amazon.de/gp/product/B07GPPZ7TB/ref=ppx_yo_dt_b_asin_title_o05_s00?ie=UTF8&psc=1) | 7,23€ |
| 1 | co-processor | [Coral Edge TPU USB accelerator](https://www.mouser.de/ProductDetail/Coral/G950-06809-01?qs=u16ybLDytRbcxxqFKdbhgQ%3D%3D&vip=1&gclid=CjwKCAiA7939BRBMEiwA-hX5J7SCRvcS0FLpI7EGL1z0iKYdMWKdEDGNVP10fhpuVj-wnjDSk8_xyRoCXKIQAvD_BwE) | 74,50€ |
| 1 | storage medium | [tf card class 10 64gb](https://www.amazon.de/Intenso-Micro-Class-Speicherkarte-SD-Adapter/dp/B00FMB9A30/ref=sr_1_4?__mk_de_DE=%C3%85M%C3%85%C5%BD%C3%95%C3%91&dchild=1&keywords=tf%2Bclass%2B10%2B64gb&qid=1605913236&quartzVehicle=1522-1061&replacementKeywords=class%2B10%2B64gb&sr=8-4&th=1) | 5,99€ |
| 1 | motor controller | [L298N H-Bridge](https://www.amazon.de/Br%C3%BCcke-Treiberplatine-Schrittmotor-Stepper-Controller/dp/B07PRXMH9P/ref=sr_1_2_sspa?__mk_de_DE=%C3%85M%C3%85%C5%BD%C3%95%C3%91&dchild=1&keywords=L298N&qid=1605913516&s=ce-de&sr=1-2-spons&psc=1&spLa=ZW5jcnlwdGVkUXVhbGlmaWVyPUExQzdOVkYyQlJDM1EwJmVuY3J5cHRlZElkPUEwNjUyNTU5MlRVSjJPR0cyQVEwVyZlbmNyeXB0ZWRBZElkPUEwMTkwMzM0NlEwSlQzMFc1QU00JndpZGdldE5hbWU9c3BfYXRmJmFjdGlvbj1jbGlja1JlZGlyZWN0JmRvTm90TG9nQ2xpY2s9dHJ1ZQ==) | 5,89€ |
| 1 | servo controller | [PCA9685 16 Channel Servo Controller](https://www.amazon.de/gp/product/B06XSFFXQY/ref=ppx_yo_dt_b_asin_title_o07_s00?ie=UTF8&psc=1) | 6,89€ |
| 1 | step down converter | [Dual USB 9V/12V/24V/36V to 5V Converter DC-DC 3A Step Down Power Module](https://www.amazon.de/dp/B0768D2NYH?tag=ingmstap-21&linkCode=ogi&th=1&psc=1) | 8,26€ |
| x | battery | [11.1 V LiPo](https://www.amazon.de/Lipo-11-1800mAh-25C-3S-1P/dp/B009H497IG/ref=sr_1_2?__mk_de_DE=%C3%85M%C3%85%C5%BD%C3%95%C3%91&dchild=1&keywords=lipo+11.1v+1800mAh+25c&qid=1605912549&sr=8-2) | 27,19€ |
| 1 | power supply unit | [universal power supply unit](https://www.reichelt.de/de/de/universal-schaltnetzteil-36-w-5-15-v-3000-ma-mw-3h36gs-p89789.html?r=1) | 13,20€ |
| 1 | power supply unit | [Deans T plug](https://www.amazon.de/gp/product/B07FND44XC/ref=ppx_yo_dt_b_asin_title_o07_s01?ie=UTF8&psc=1) | 10,80€ |
| 1 | power supply unit | [dc female terminal block adapter](https://www.amazon.de/gp/product/B00E8CURKO/ref=ppx_yo_dt_b_asin_title_o07_s01?ie=UTF8&psc=1) | 2,00€ |
| 1 | camera | [MakerHawk Raspberry Pi camera IR Fisheye wide angle 150-160 degrees 5MP OV5647](https://www.amazon.de/gp/product/B07DRH5Y5S/ref=ppx_yo_dt_b_asin_title_o07_s00?ie=UTF8&psc=1) | 27,99 |
| 1 | camera | [pan-tilt-bracket](https://www.amazon.de/dp/B079H3WY7T?tag=ingmstap-21&linkCode=ogi&th=1&psc=1) |  |
| 1 | camera | [passive cooler](https://www.amazon.de/gp/product/B06XWFG7Q7/ref=ppx_yo_dt_b_asin_title_o06_s00?ie=UTF8&psc=1) | 3,99€ |
| 1 | camera | [flat ribbon cable](https://www.amazon.de/gp/product/B075PBTQPG/ref=ppx_yo_dt_b_asin_title_o05_s00?ie=UTF8&psc=1) | 5,29€ |
| 1 | imu | [Raspberry Pi Sense HAT](https://www.amazon.de/dp/B014T2IHQ8/ref=dp_prsubs_1) | 37,63€ |
| 2 | time-of-flight | [VL53L1X](https://www.amazon.de/gp/product/B07DM2TKKL/ref=ppx_yo_dt_b_asin_title_o06_s00?ie=UTF8&psc=1) | 17,65€ |
| 2 | ultrasonic | [Parallax Ping)))](https://www.amazon.de/Parallax-Ping-Ultrasonic-Distance-Sensor/dp/B004SRTM0K/ref=sr_1_1?__mk_de_DE=%C3%85M%C3%85%C5%BD%C3%95%C3%91&dchild=1&keywords=parallax+ultrasonic&qid=1605913993&sr=8-1) | 39,00€ |
| 4 | ultrasonic | [HC-SR04](https://www.amazon.de/gp/product/B07KPJNLSS/ref=ppx_yo_dt_b_asin_title_o01_s00?ie=UTF8&psc=1) | 8,00€ |
| 1 | ultrasonic | [330 Ohm resistor](https://www.amazon.de/gp/product/B00YW3E2LO/ref=ppx_yo_dt_b_asin_title_o01_s00?ie=UTF8&psc=1) | 6,37€ |
| 1 | ultrasonic | [470 Ohm resistor](https://www.amazon.de/gp/product/B00YW4DTRG/ref=ppx_yo_dt_b_asin_title_o01_s00?ie=UTF8&psc=1) | 6,49€ |
| 2 | infrared |  |  |
| 1 | infrared |  |  |
| 1 | infrared |  |  |
| 1 | gps |  |  |
| 1 | i2c |  |  |
| 1 | display | [I2C OLED 128 x 64 Pixel 0,96 Zoll](https://www.amazon.de/gp/product/B074N9VLZX/ref=ppx_yo_dt_b_asin_title_o09_s00?ie=UTF8&psc=1) | 11,99€ |
| 1 | wire | jump wires |  |
| 1 | wire | https://www.amazon.de/dp/B01AD62W56?tag=ingmstap-21&linkCode=ogi&th=1&psc=1 |  |
|  |  |  |  |
|  |  |  |  |
|  |  |  |  |


## 3 Software
=====================

At first download and flash [Debian Buster](https://www.raspberrypi.org/downloads/raspberry-pi-os/) on a Micro SD Card.

### 3.1 ROS Melodic
#### 3.1.1 Installation

Find more under

- [ROS Melodic on Raspberry Pi 4 Debian Buster + RPLIDAR A1M8](https://www.instructables.com/id/ROS-Melodic-on-Raspberry-Pi-4-RPLIDAR/)
- [ROSberryPi: Installing ROS Melodic on the Raspberry Pi](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Melodic%20on%20the%20Raspberry%20Pi)
- [ROSberryPi: Installing ROS Kinetic on the Raspberry Pi](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi)

##### 3.1.1.1 Prerequisites

These instructions assume that Raspbian Buster is being used as the OS on the Raspberry Pi 4 Model B.

###### 3.1.1.1.1 Setup ROS Repositories

First install repository key:
```bash
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

Now, make sure your Debian package index is up-to-date:

```bash
$ sudo apt-get update
$ sudo apt-get upgrade
```

###### 3.1.1.1.2 Install Bootstrap Dependencies

```bash
$ sudo apt install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential cmake
```

###### 3.1.1.1.3 Initializing rosdep

```bash
$ sudo rosdep init
$ rosdep update
```

##### 3.1.1.2. Installation

Now, we will download and build ROS Melodic.

###### 3.1.1.2.1 Create a catkin Workspace

In order to build the core packages, you will need a catkin workspace. Create one now:

```bash
$ mkdir -p ~/ros_catkin_ws
$ cd ~/ros_catkin_ws
```

Desktop Install: includes GUI tools, such as rqt, rviz, and robot-generic libraries. Might be better choice for beginners to ROS.

```bash
$ rosinstall_generator desktop --rosdistro melodic --deps --wet-only --tar > melodic-desktop-wet.rosinstall
$ wstool init -j8 src melodic-desktop-wet.rosinstall
```

This will add all of the catkin or wet packages in the given variant and then fetch the sources into the ~/ros_catkin_ws/src directory. The command will take a few minutes to download all of the core ROS packages into the src folder. The -j8 option downloads 8 packages in parallel.

###### 3.1.1.2.2 Resolve Dependencies

Before you can build your catkin workspace, you need to make sure that you have all the required dependencies. We use the rosdep tool for this.

Resolving Dependencies with rosdep
The dependencies should be resolved by running rosdep:

```bash
$ cd ~/ros_catkin_ws
$ rosdep install -y --from-paths src --ignore-src --rosdistro melodic -r --os=debian:buster
```

This will look at all of the packages in the src directory and find all of the dependencies they have. Then it will recursively install the dependencies.

The --from-paths option indicates we want to install the dependencies for an entire directory of packages, in this case src. The --ignore-src option indicates to rosdep that it shouldn't try to install any ROS packages in the src folder from the package manager, we don't need it to since we are building them ourselves. The --rosdistro option is required because we don't have a ROS environment setup yet, so we have to indicate to rosdep what version of ROS we are building for. Finally, the -y option indicates to rosdep that we don't want to be bothered by too many prompts from the package manager.

After a while rosdep will finish installing system dependencies and you can continue.

###### 3.1.1.2.3 Building the catkin Workspace

Once you have completed downloading the packages and have resolved the dependencies, you are ready to build the catkin packages.

Invoke catkin_make_isolated:

```bash
$ sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/melodic -j8
```

Now ROS should be installed! Remember to source the new installation. Source the setup.bash in the ~/.bashrc, so that ROS environment variables are automatically added to your bash session every time a new shell is launched:

```bash
$ echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
```

#### 3.1.2 Update ROS Packages

Find more under http://wiki.ros.org/melodic/Installation/Source

```bash
cd ~/ros_catkin_ws/
```

To update your workspace, first move your existing rosinstall file so that it doesn't get overwritten, and generate an updated version. For simplicity, we will cover the *destop-full* variant. For other variants, update the filenames and rosinstall_generator arguments appropriately.

```bash
$ mv -i melodic-desktop-full.rosinstall melodic-desktop-full.rosinstall.old
$ rosinstall_generator desktop_full --rosdistro melodic --deps --tar > melodic-desktop-full.rosinstall
```

Then, compare the new rosinstall file to the old version to see which packages will be updated:

```bash
$ diff -u melodic-desktop-full.rosinstall melodic-desktop-full.rosinstall.old
```

If you're satisfied with these changes, incorporate the new rosinstall file into the workspace and update your workspace:

```bash
$ vcs import src < melodic-desktop-full.rosinstall
```

Now that the workspace is up to date with the latest sources, rebuild it:

```bash
sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/melodic -j8
```

Once your workspace has been rebuilt, you should source the setup files again:

```bash
$ source /opt/ros/melodic/setup.bash
```

Or just reload the ~/.bashrc file:

```bash
$ source ~/.bashrc
```

#### 3.1.3 Adding released ROS Packages

You may add additional packages to the installed ros workspace that have been released into the ros ecosystem. First, a new rosinstall file must be created including the new packages (Note, this can also be done at the initial install). For example, if we have installed ros_comm, but want to add ros_control and joystick_drivers, the command would be:

```bash
$ cd ~/ros_catkin_ws
$ rosinstall_generator ros_comm ros_control joystick_drivers --rosdistro melodic --deps --wet-only --tar > melodic-custom_ros.rosinstall
```

You may keep listing as many ROS packages as you'd like separated by spaces.


Next, update the workspace with wstool:

```bash
$ wstool merge -t src melodic-custom_ros.rosinstall
$ wstool update -j8 -t src
```

After updating the workspace, you may want to run rosdep to install any new dependencies that are required:

```bash
$ rosdep install --from-paths src --ignore-src --rosdistro melodic -y -r --os=debian:buster
```

Finally, now that the workspace is up to date and dependencies are satisfied, rebuild the workspace:

```bash
$ sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/melodic -j8
```

Once your workspace has been rebuilt, you should source the setup files again:

```bash
$ source /opt/ros/melodic/setup.bash
```

Or just reload the ~/.bashrc file:

```bash
$ source ~/.bashrc
```

#### 3.1.3.1 Adding raspicam_node Package

Don`t follow the [raspicam_node installation guide](https://github.com/UbiquityRobotics/raspicam_node) because the [raspicam_node](https://github.com/UbiquityRobotics/raspicam_node) was built for the kinetic distribution. For the melodic distribution you have to do the following workaround:

```bash
$ cd ~/ros_catkin_ws
$ rosinstall_generator compressed_image_transport --rosdistro melodic --deps --wet-only --tar > melodic-compressed_image_transport-wet.rosinstall
$ rosinstall_generator camera_info_manager --rosdistro melodic --deps --wet-only --tar > melodic-camera_info_manager-wet.rosinstall
$ rosinstall_generator dynamic_reconfigure --rosdistro melodic --deps --wet-only --tar > melodic-dynamic_reconfigure-wet.rosinstall
$ wstool merge -t src melodic-compressed_image_transport-wet.rosinstall
$ wstool merge -t src melodic-camera_info_manager-wet.rosinstall
$ wstool merge -t src melodic-dynamic_reconfigure-wet.rosinstall
$ wstool update -j8 -t src
$ rosdep install --from-paths src --ignore-src --rosdistro melodic -y -r --os=debian:buster
$ sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/melodic -j8
```

After you have installed and updated the ROS packages you can clone and build the [raspicam_node](https://github.com/UbiquityRobotics/raspicam_node).

```bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/UbiquityRobotics/raspicam_node.git

$ cd ~/catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro melodic -y
$ sudo ./src/catkin/bin/catkin_make_isolated --install --install-space /opt/ros/melodic -DCMAKE_BUILD_TYPE=Release -j8
```

Make sure that your user is in the `video` group by running `groups|grep video`. If not you have to do:

```bash
sudo usermod -a -G video pi
```

After that you can check the installation by running

```bash
$ roslaunch raspicam_node camerav2_1280x960.launch
```

cd  ~/catkin_ws/src
git clone https://github.com/ros-perception/image_common.git
cd ~/catkin_ws/
catkin_make

rosrun image_view image_view image:=raspicam_node/image/compressed



## 4 Simulation Platform
=====================

The simulation platform uses four differents software modules which also needs further catkin packages.

|   Software Module  |    System   |      Description       |
|--------------------------------------------- | ------------------------------|
| [robotcar](https://github.com/Michdo93/robotcar) | RobotCar | The robotcar module provides all drivers for actors and sensors. Also it provides all needed Libraries and configurations. Filters like the kalman filter are also provided. Programs for configuring and resetting the sensors are available as well as for testing the motor and controller. Service files for systemd and start-up are also available. This software module implements the functionality of the robot car. |
| [robotcar-pkg](https://github.com/Michdo93/robotcar-pkg) | RobotCar | Via the ROS Publisher-Subscriber pattern, the robotcar-pkg enables the exchange of information. Publishers provide the information to ADAS and via subscriber an ADAS can intervene in the driving of the robot car. This software module therefore enables the necessary communication for the functionality. |
| [robotcar_controller](https://github.com/Michdo93/robotcar_controller) | control computer respectively operating computer | The controller forwards appropriate control commands to the robot car, which should make it possible to move the robot car purposefully. For this purpose, primarily the motor and the steering are controlled. An extension, for example to move the camera or switch ADAS on and off, would be conceivable in the future. |
| [robotcar_msgs](https://github.com/Michdo93/robotcar_msgs) | Robotcar and control computer respectively operating computer | The robotcar_msgs extend the ROS message types with custom message types for complete communication. For new ADAS these may have to be extended. On every computer that wants to participate in the communication, these must be installed additionally. |

Further packages:

* With the [beginner_tutorials](https://github.com/Michdo93/beginner_tutorials) you can learn the basic concepts of the RobotCar.
* With the [robotcar_subscriber](https://github.com/Michdo93/robotcar_subscriber) you can subscribe different informations from the RobotCar. It could be used as blue print for ADAS.
* With the [robotcar_sensorfusion_examples](https://github.com/Michdo93/robotcar_sensorfusion_examples) you can learn how to use as example a simple kalman filter for sensor fusion. It could be used as blue print for ADAS.
* The [std_header_msgs](https://github.com/Michdo93/std_header_msgs) could be used as example for sensor fusion because the sensor fusion needs timestamps which are missing in the [std_msgs](http://docs.ros.org/en/melodic/api/std_msgs/html/index-msg.html) from ROS.
* The [raspicam_node](https://github.com/Michdo93/raspicam_node) is needed to use the raspicam with ROS. So the robot car definitely needs this package.







![](https://cdn-images-1.medium.com/max/800/1*4GhtKM-eyuYqEpZnnUJZ9w@2x.jpeg)

test


