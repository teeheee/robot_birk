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
