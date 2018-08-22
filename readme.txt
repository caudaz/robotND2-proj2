$ sudo apt-get install ros-kinetic-navigation
$ sudo apt-get install ros-kinetic-map-server
$ sudo apt-get install ros-kinetic-move-base
$ rospack profile
$ sudo apt-get install ros-kinetic-amcl


mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
git clone https://github.com/caudaz/robotND2-proj2
cd ~/catkin_ws
catkin_make

TERMINAL1
$ cd ~/catkin_ws
$ source devel/setup.bash
$ roslaunch udacity_bot udacity_world.launch

TERMINAL2
$ cd ~/catkin_ws
$ source devel/setup.bash
$ roslaunch udacity_bot amcl.launch
(NOTE: at this point you can still set the 2D navigation goal manually on RVIZ)

TERMINAL3
$ cd ~/catkin_ws
$ source devel/setup.bash
$ rosrun udacity_bot navigation_goal
