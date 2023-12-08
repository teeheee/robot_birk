# robot_birk
autonomouse indoor navigation robot platform based on an exisiting lowcost cleaning robot


# START
       * source devel/setup.bash
       * roslaunch birk_upstart base.launch

# BUILD
       * catkin_make install

# MANAGE
       * rosservice ...
       * rostopic ...

# CREATE PACKAGE
       * catkin_create_pkg

# BUILD WITH DOCKER

docker run --mount src="$(pwd)",target=/ws,type=bind -it ros:melodic

cd ws
apt update
apt install ros-melodic-move-base
catkin_make


# setup raspberry pi

raspberry ubuntu image

source:
ubuntu@ubuntu:~/ws$ cat /etc/apt/sources.list.d/ros-latest.list
deb http://packages.ros.org/ros/ubuntu bionic main

install:
Commandline: apt-get remove --yes --purge grub-legacy-ec2
Commandline: apt install ros-melodic-ros-base
Commandline: apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
Commandline: apt-get install ros-melodic-ros-tutorials ros-melodic-geometry-tutorials ros-melodic-rviz ros-melodic-rosbash ros-melodic-rqt-tf-tree
Commandline: apt-get install ros-melodic-robot-upstart
Commandline: apt-get install python-dev python-pip
Commandline: apt-get upgrade
Commandline: apt-get -y install python-rpi.gpio
Commandline: apt install network-manager
Commandline: apt-get install python-cffi
Commandline: apt-get install ros-melodic-openslam-gmapping
Commandline: apt-get install ros-melodic-slam-gmapping
Commandline: apt-get install ros-melodic-hector-slam
Commandline: apt-get install ros-melodic-navigation
Commandline: apt-get install gdb
Commandline: apt-get install keychain
Commandline: apt-get install ros-melodic-rosserial
Commandline: apt-get install ros-melodic-rosserial-arduino
Commandline: apt-get install ros-melodic-controller-manager
Commandline: apt-get install ros-melodic-joint-limits-interface
Commandline: apt-get install ros-melodic-rplidar-ros