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










![](https://cdn-images-1.medium.com/max/800/1*4GhtKM-eyuYqEpZnnUJZ9w@2x.jpeg)

test


