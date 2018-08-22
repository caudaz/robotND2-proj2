# Robotics Nanodegree #

## Term2 – Project2: Where Am I? (Particle Filters) ##

![](./media/image1.gif)


UPDATE ROS Packages

```
sudo apt-get install ros-kinetic-navigation

sudo apt-get install ros-kinetic-map-server

sudo apt-get install ros-kinetic-move-base

rospack profile

sudo apt-get install ros-kinetic-amcl

```
CREATE CATKIN WORKSPACE

```
mkdir -p \~/catkin\_ws/src

cd \~/catkin\_ws/src

catkin\_init\_workspace

git clone <https://github.com/caudaz/robotND2-proj2>

cd \~/catkin\_ws

catkin\_make

```
TERMINAL1

```
cd \~/catkin\_ws

source devel/setup.bash

roslaunch udacity\_bot udacity\_world.launch

```
TERMINAL2

```
cd \~/catkin\_ws

source devel/setup.bash

roslaunch udacity\_bot amcl.launch

```
(NOTE: at this point you can still set the 2D navigation goal manually
on RVIZ)

TERMINAL3

```
cd ~/catkin_ws

source devel/setup.bash

rosrun udacity_bot navigation_goal
```


## ABSTRACT ##

This project attempts to solve the problem of local and global
localization of a robot. Such a problem could be encountered on a
factory floor or on a home robot such as the Roomba. The project
utilizes Particle Filters for Localization. The implementation is done
using ROS, Gazebo, and RVIZ.

## INTRO ##

This project involves:

-   building your own simulated mobile robot, added sensors to it, and
    integrated it with Gazebo and RViz by developing a ROS package for
    that robot.

-   integrating ROS packages like the Adaptive Monte Carlo Localization
    (AMCL) package and the Navigation Stack, that allows your robot to
    navigate and localize itself in a particular environment or map.

-   tuning different parameters corresponding to these packages that
    improve results

## BASIC ROBOT MODEL ##

The basic model for GAZEBO is defined in the URDF folder:
udacity\_bot.xacro (XML file) . It defines:

-   geometry (for visual and collision)

-   mass/inertia

-   links (rigid bodies)

-   joints

It consists of a base(blue), two actuators (wheels in green joined by a
differential model), a camera (red), and a laser sensor (gray/black):

![](./media/image2.jpeg)

## GAZEBO PLUGINS ##

Since the URDF file cannot perform actions such as take images during
simulation, plugins (included in the udacity\_bot.gazebo file) are used
for:

-   camera sensor

-   Hokuyo sensor

-   controlling the wheel joints (libgazebo\_ros\_diff\_drive.so)

## RVIZ Integration ##

-   To visualize any type of sensor data being published over a ROS
    topic, RVIZ is used:

-   camera images

-   points cloud from laser

-   maps

## GAZEBO Map ##

The “map” folder contains files: jackal\_race.pgm and jackal\_race.yaml.
These will be visualized in GAZEBO (but not on RVIZ).

## Particle Filter (AMCL Package) ##

A ROS package for Adaptive Monte Carlo Localization (AMCL) is used on
the project. It changes the number of particles over time, being
computationally more efficient than MCL.

## Navigation Stack ##

The move\_base package from ROS will be used, so that a goal position
can be defined in RVIZ and the robot will navigate to it. It uses
costmap, which divides the map into occupied and unoccupied areas. The
package has:

-   built-in correction to navigate around obstacles

-   detect if the robot is stuck

-   rotate until it finds a clear path ahead

The local costmap only displays what the laser sensor captures during
that time period:

![](./media/image3.jpeg)

The global costmap displays all occupied areas plus an inflated radius
(shown in blue):

![](./media/image4.jpeg)

The “Inflated Radius” parameter will influence the blue area on the
maps. Below is a comparison of a R=0.1m vs. R=10.0m parameter:

![](./media/image5.gif)

![](./media/image6.gif)

## PARAMETER TUNING ##

Parameters in the config and launch folders were modified to allow the
robot to reach the goal in an accurate and timely manner. All parameters
were tested/tuned to balance the tradeoff between accuracy and
computational cost.

costmap\_common\_params.yaml

\# delay or latency allowed between transforms

transform\_tolerance: 0.2

\# map update loop

update\_frequency: 10.0

publish\_frequency: 10.0

\# obstacle detected by a laser sensor is within 0.1 meters from the
base of the robot,

\# that obstacle will be added to the costmap.

\# Can help discarding noise, falsely detecting obstacles, and
computational costs.

obstacle\_range: 3.0

\# clear and update the free space in the costmap as the robot moves.

raytrace\_range: 3.0

\# minimum distance between the robot geometry and the obstacles

inflation\_radius: 0.15

local\_costmap\_params.yaml

width: 5.0 \# original 20.0

height: 5.0 \# original 20.0

resolution: 0.05

global\_costmap\_params.yaml

width: 15.0 \# original 40.0

height: 15.0 \# original 40.0

resolution: 0.15 \# original 0.05

amcl.launch (for localization)

&lt;!--delay or latency allowed between transforms--&gt;

&lt;param name="transform\_tolerance" value="0.2"/&gt;

&lt;!-- tuned based on your system specifications--&gt;

&lt;!--A larger range, with a high maximum might be too computationally
extensive for a low-end system--&gt;

&lt;param name="min\_particles" value="20"/&gt;

&lt;param name="max\_particles" value="80"/&gt;

&lt;!--Upon receiving a laser scan, it checks the values for
update\_min\_a and update\_min\_d and compares to how far the robot has
moved--&gt;

&lt;!--Decides whether or not to perform a filter update or to discard
the scan data--&gt;

&lt;!--Discarding data could result in poorer localization results--&gt;

&lt;!--and too many frequent filter updates for a fast moving robot
could also cause computational problems--&gt;

&lt;param name="update\_min\_d" value="0.01"/&gt;

&lt;param name="update\_min\_a" value="0.01"/&gt;

&lt;!--The likelihood\_field model is usually more computationally
efficient and reliable--&gt;

&lt;!--for an environment such as the one you are working with. --&gt;

&lt;param name="laser\_model\_type" value="likelihood\_field"/&gt;

&lt;param name="laser\_min\_range" value="0.4"/&gt;

&lt;!--laser\_max\_beams--&gt;

&lt;param name="laser\_z\_hit" value="0.8"/&gt;

&lt;param name="lazer\_z\_rand" value="0.2" /&gt;

&lt;!--Since you are working with a differential drive mobile robot,
it’s best to use the diff-corrected type--&gt;

&lt;param name="odom\_model\_type" value="diff-corrected"/&gt;

&lt;param name="odom\_alpha1" value="0.02"/&gt;

&lt;param name="odom\_alpha2" value="0.02"/&gt;

&lt;param name="odom\_alpha3" value="0.02"/&gt;

&lt;param name="odom\_alpha4" value="0.02"/&gt;

## RESULTS ##

The robot model was able to reach the goal position while displaying the
PoseArray on RVIZ:

![](./media/image7.png)

## DISCUSSION ##

The AMCL would not work well for a kidnapped robot problem. MCL and AMCL
could be used in applications for factory floors or home robots (such as
the Roomba).

The use of MCL is better for this project because of the ease of
implementation and it’s robustness over EKF. MCL can also use raw
measurements vs. EKF’s landmarks. MCL also does Global Localization
whereas EKF can’t.

## CONCLUSION/FUTURE WORK ##

The AMCL model was able to reach the desired position goal in a timely
manner.

The robot could be improved by the addition of more sensors such as
sonar for close range measurements.

This ROS model could be deployed in actual hardware by using the
following hardware:

<https://ubiquityrobotics.com/magni.html>

The Magni is a robot capable of carrying 100kilos, it has motors,
sensors for navigation, battery, and it’s brains are a Raspberry Pi 3
running UBUNTU and ROS, so the simulation done on this project can be
moved over to the robot with ease.
