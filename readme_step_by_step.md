# STEP by STEP INSTRUCTIONS for Project: Where Am I?  #


## From How to Create a Catkin Workspace: ##
https://classroom.udacity.com/nanodegrees/nd209/parts/c199593e-1e9a-4830-8e29-2c86f70f489e/modules/2919466f-aa2b-4424-b86a-98b0a53ce335/lessons/658c94f5-f806-4273-9001-9e2838e56856/concepts/a777bc7a-95d4-44ca-b4e3-119718e3a213

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
ls -l
cd ~/catkin_ws
catkin_make
ls
```

You now have two new directories: build and devel. The aptly named build directory is the build space for C++ packages and, for the most part, you will not interact with it. The devel directory does contain something of interest, a file named setup.bash. This setup.bash script must be sourced before using the catkin workspace.  source devel/setup.bash




## 1- Overview ##
$ sudo apt-get install ros-kinetic-navigation
$ sudo apt-get install ros-kinetic-map-server
$ sudo apt-get install ros-kinetic-move-base
$ rospack profile
$ sudo apt-get install ros-kinetic-amcl






## 2- Gazebo Hello World ##
cd /home/workspace/catkin_ws/src/
catkin_create_pkg udacity_bot
$ cd udacity_bot
$ mkdir launch
$ mkdir worlds


$ cd worlds
$ nano udacity.world

Add the following to udacity.world
<?xml version="1.0" ?>

<sdf version="1.4">

 <world name="default">

   <include>
     <uri>model://ground_plane</uri>
   </include>

   <!-- Light source -->
   <include>
     <uri>model://sun</uri>
   </include>

   <!-- World camera -->
   <gui fullscreen='0'>
     <camera name='world_camera'>
       <pose>4.927360 -4.376610 3.740080 0.000000 0.275643 2.356190</pose>
       <view_controller>orbit</view_controller>
     </camera>
   </gui>

 </world>
</sdf>



$ cd ..
$ cd launch
$ nano udacity_world.launch

Add the following to your launch file.
<?xml version="1.0" encoding="UTF-8"?>

<launch>

 <arg name="world" default="empty"/> 
 <arg name="paused" default="false"/>
 <arg name="use_sim_time" default="true"/>
 <arg name="gui" default="true"/>
 <arg name="headless" default="false"/>
 <arg name="debug" default="false"/>

 <include file="$(find gazebo_ros)/launch/empty_world.launch">
   <arg name="world_name" value="$(find udacity_bot)/worlds/udacity.world"/>
   <arg name="paused" value="$(arg paused)"/>
   <arg name="use_sim_time" value="$(arg use_sim_time)"/>
   <arg name="gui" value="$(arg gui)"/>
   <arg name="headless" value="$(arg headless)"/>
   <arg name="debug" value="$(arg debug)"/>
 </include>

</launch>







$ cd /home/workspace/catkin_ws/
$ catkin_make
$ source devel/setup.bash
$ roslaunch udacity_bot udacity_world.launch




## 3-Robot Model: Basic Setup ##
$ cd /home/workspace/catkin_ws/src/udacity_bot/
$ mkdir urdf
$ cd urdf
$ nano udacity_bot.xacro


Copy the following code into your udacity_bot.xacro file. 
<?xml version='1.0'?>

<robot name="udacity_bot" xmlns:xacro="http://www.ros.org/wiki/xacro">

 <link name="robot_footprint"></link>

 <joint name="robot_footprint_joint" type="fixed">
   <origin xyz="0 0 0" rpy="0 0 0" />
   <parent link="robot_footprint"/>
   <child link="chassis" />
 </joint>

 <link name='chassis'>
   <pose>0 0 0.1 0 0 0</pose>

   <inertial>
     <mass value="15.0"/>
     <origin xyz="0.0 0 0" rpy=" 0 0 0"/>
     <inertia
         ixx="0.1" ixy="0" ixz="0"
         iyy="0.1" iyz="0"
         izz="0.1"
     />
   </inertial>

   <collision name='collision'>
     <origin xyz="0 0 0" rpy=" 0 0 0"/> 
     <geometry>
       <box size=".4 .2 .1"/>
     </geometry>
   </collision>

   <visual name='chassis_visual'>
     <origin xyz="0 0 0" rpy=" 0 0 0"/>
     <geometry>
       <box size=".4 .2 .1"/>
     </geometry>
   </visual>


   <collision name='back_caster_collision'>
     <origin xyz="-0.15 0 -0.05" rpy=" 0 0 0"/>
     <geometry>
       <sphere radius="0.0499"/>
     </geometry>
   </collision>

   <visual name='back_caster_visual'>
     <origin xyz="-0.15 0 -0.05" rpy=" 0 0 0"/>
     <geometry>
       <sphere radius="0.05"/>
     </geometry>
   </visual>

   <collision name='front_caster_collision'>
     <origin xyz="0.15 0 -0.05" rpy=" 0 0 0"/>
     <geometry>
       <sphere radius="0.0499"/>
     </geometry>
   </collision>

   <visual name='front_caster_visual'>
     <origin xyz="0.15 0 -0.05" rpy=" 0 0 0"/>
     <geometry>
       <sphere radius="0.05"/>
     </geometry>
   </visual>

 </link>

</robot>



$ cd /home/workspace/catkin_ws/src/udacity_bot/launch/
$ nano robot_description.launch


Copy the following into the above file.
<?xml version="1.0"?>
<launch>

 <!-- send urdf to param server -->
 <param name="robot_description" command="$(find xacro)/xacro --inorder '$(find udacity_bot)/urdf/udacity_bot.xacro'" />

</launch>

$ nano udacity_world.launch


Add the following to the launch file (after <launch>)
<include file="$(find udacity_bot)/launch/robot_description.launch"/>

Add the following to the launch file (before </launch>)
<!--spawn a robot in gazebo world-->

<node name="urdf_spawner" pkg="gazebo_ros" type="spawn_model" respawn="false" 
output="screen" args="-urdf -param robot_description -model udacity_bot"/>

$ cd /home/workspace/catkin_ws/
$ catkin_make
$ source devel/setup.bash
$ roslaunch udacity_bot udacity_world.launch




cd /catkin_ws/src/udacity_bot/urdf


edit udacity_bot.xacro, ADD:
  <link name='left_wheel'>
        <inertial>
          <mass value="5.0"/>
          <origin xyz="0.0 0 0" rpy=" 0 1.5707 1.5707"/>
          <inertia
              ixx="0.1" ixy="0" ixz="0"
              iyy="0.1" iyz="0"
              izz="0.1"
          />
        </inertial>
        <collision name='left_wheel_collision'>
          <origin xyz="0 0 0" rpy=" 0 1.5707 1.5707"/>
          <geometry>
            <cylinder radius="0.1" length="0.05"/>
          </geometry>
        </collision>
        <visual name='left_wheel_visual'>
          <origin xyz="0 0 0" rpy=" 0 1.5707 1.5707"/>
          <geometry>
            <cylinder radius="0.1" length="0.05"/>
          </geometry>
        </visual>    
  </link>


  <link name='right_wheel'>
        <inertial>
          <mass value="5.0"/>
          <origin xyz="0.0 0 0" rpy=" 0 1.5707 1.5707"/>
          <inertia
              ixx="0.1" ixy="0" ixz="0"
              iyy="0.1" iyz="0"
              izz="0.1"
          />
        </inertial>
        <collision name='right_wheel_collision'>
          <origin xyz="0 0 0" rpy=" 0 1.5707 1.5707"/>
          <geometry>
            <cylinder radius="0.1" length="0.05"/>
          </geometry>
        </collision>
        <visual name='right_wheel_visual'>
          <origin xyz="0 0 0" rpy=" 0 1.5707 1.5707"/>
          <geometry>
            <cylinder radius="0.1" length="0.05"/>
          </geometry>
        </visual>    
  </link>  


  <joint type="continuous" name="left_wheel_hinge">
        <origin xyz="0 0.15 0" rpy="0 0 0"/>
        <child link="left_wheel"/>
        <parent link="chassis"/>
        <axis xyz="0 1 0" rpy="0 0 0"/>
        <limit effort="10000" velocity="1000"/>
        <dynamics damping="1.0" friction="1.0"/>
  </joint>


  <joint type="continuous" name="right_wheel_hinge">
        <origin xyz="0 -0.15 0" rpy="0 0 0"/>
        <child link="right_wheel"/>
        <parent link="chassis"/>
        <axis xyz="0 1 0" rpy="0 0 0"/>
        <limit effort="10000" velocity="1000"/>
        <dynamics damping="1.0" friction="1.0"/>
  </joint>


$ cd /home/workspace/catkin_ws/
$ catkin_make
$ source devel/setup.bash
$ roslaunch udacity_bot udacity_world.launch




## 4- Let there be sight ##
$ cd /home/workspace/catkin_ws/src/udacity_bot/urdf
$ nano udacity_bot.xacro
And add a camera sensor based on the following specifications -
* link name - "camera"
* link origin - "[0, 0, 0, 0, 0, 0]"
* joint name - "camera_joint"
* joint origin - "[0.2, 0, 0, 0, 0, 0]"
* geometry - box with size "0.05"
* mass - "0.1"
* inertia - ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"
* joint parent link - "chassis", and joint child link - "camera"
As we covered in the previous section, each link should have its own visual, collision and inertial elements:
  <link name='camera'>
        <inertial>
          <mass value="0.1"/>
          <origin xyz="0.0 0 0" rpy=" 0 0 0"/>
          <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>
        </inertial>
        <collision name='camera_collision'>
          <origin xyz="0 0 0" rpy=" 0 0 0"/>
          <geometry>
            <box size="0.05 0.05 0.05"/>
          </geometry>
        </collision>
        <visual name='camera_visual'>
          <origin xyz="0 0 0" rpy=" 0 0 0"/>
          <geometry>
            <box size="0.05 0.05 0.05"/>
          </geometry>
        </visual>    
  </link>   


  <joint type="fixed" name="camera_joint">
        <origin xyz="0.2 0 0" rpy="0 0 0"/>
        <child link="camera"/>
        <parent link="chassis"/>
  </joint> 


The hokuyo sensor can be added to your robot model just like the camera sensor. Here are some of the specifications for the sensor that you can use -
* link name - "hokuyo"
* link origin - "[0, 0, 0, 0, 0, 0]"
* joint name - "hokuyo_joint"
* joint origin - "[.15, 0, .1, 0, 0, 0]"
* geometry - box with size "0.1" for <collision>, and a mesh file for <visual>
* mass - "0.1"
* inertia - ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"
Don't forget to define the joint type, and the parent and child links!
>mkdir ~/catkin_ws/src/udacity_bot/meshes
>cd meshes
>wget  https://raw.githubusercontent.com/udacity/RoboND-Localization-Project/master/meshes/hokuyo.dae


  <link name='hokuyo'>
        <inertial>
          <mass value="0.1"/>
          <origin xyz="0.0 0 0" rpy=" 0 0 0"/>
          <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>
        </inertial>
        <collision name='hokuyo_collision'>
          <origin xyz="0 0 0" rpy=" 0 0 0"/>
          <geometry>
            <box size="0.1 0.1 0.1"/>
          </geometry>
        </collision>
        <visual name='hokuyo_visual'>
          <origin xyz="0 0 0" rpy=" 0 0 0"/>
          <geometry>
            <mesh filename="package://udacity_bot/meshes/hokuyo.dae"/>
          </geometry>
        </visual>    
  </link>   


  <joint type="fixed" name="hokuyo_joint">
        <origin xyz="0.15 0 0.1" rpy="0 0 0"/>
        <child link="hokuyo"/>
        <parent link="chassis"/>
  </joint>


$ cd /home/workspace/catkin_ws/
$ catkin_make
$ source devel/setup.bash
$ roslaunch udacity_bot udacity_world.launch




* A plugin for the camera sensor.
* A plugin for the hokuyo sensor.
* A plugin for controlling the wheel joints.


>cd ~/catkin_ws/src/udacity_bot/urdf
>wget https://raw.githubusercontent.com/udacity/RoboND-Localization-Project/master/urdf/udacity_bot.gazebo


make sure that your plugins are imported by your URDF as well:
$ cd /home/workspace/catkin_ws/src/udacity_bot/urdf
$ nano udacity_bot.xacro

Add the following to the top of the file (right before you define the robot_footprint link)
<xacro:include filename="$(find udacity_bot)/urdf/udacity_bot.gazebo" />

$ cd /home/workspace/catkin_ws/
$ catkin_make
$ source devel/setup.bash
$ roslaunch udacity_bot udacity_world.launch


Add color to .gazebo. Example:
  <gazebo reference="chassis">
        <material>Gazebo/Blue</material>
  </gazebo>


  <gazebo reference="right_wheel">
        <material>Gazebo/Green</material>
  </gazebo>


  <gazebo reference="left_wheel">
        <material>Gazebo/Green</material>
  </gazebo>


$ cd /home/workspace/catkin_ws/
$ catkin_make
$ source devel/setup.bash
$ roslaunch udacity_bot udacity_world.launch


## 5-RVIZ ##


We will start with modifying the robot_description.launch file.
$ cd /home/workspace/catkin_ws/src/udacity_bot/launch/
$ nano robot_description.launch

Add the following after the first “param” definition.
<!-- Send fake joint values-->
 <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
   <param name="use_gui" value="false"/>
 </node>

<!-- Send robot states to tf -->
 <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" respawn="false" output="screen"/>





Next, you need to launch RViz along with Gazebo.
$ nano udacity_world.launch
Add the following at the end of the file. After the urdf_spawner node definition:
<!--launch rviz-->
<node name="rviz" pkg="rviz" type="rviz" respawn="false"/>

The above will create a node that launches the package rviz. Let's launch it:
$ cd /home/workspace/catkin_ws/
$ catkin_make
$ source devel/setup.bash
$ roslaunch udacity_bot udacity_world.launch


Select the RViz window, and on the left side, under Displays:
* Select “odom” for fixed frame
* Click the “Add” button and
   * add “RobotModel” add “Camera” and select the Image topic that was defined in the camera gazebo plugin 
   * add “LaserScan” and select the topic that was defined in the hokuyo gazebo plugin.


In Gazebo, click on “Insert” and from the list add any item in the world in front of the robot, such as “dumpster” or “jersey barrier”. You should be able to see the item in Rviz in the “Camera” viewer, and the Laser scan of that object as well.


While everything above is still running, open a new terminal window, and enter:
rostopic pub /cmd_vel geometry_msgs/Twist  "linear:
 x: 0.1
 y: 0.0
 z: 0.0
angular:
 x: 0.0
 y: 0.0
 z: 0.1" 



## 6-Localization Map ##


$ cd /home/workspace/catkin_ws/src/udacity_bot/
$ mkdir maps
$ cd maps
$ wget https://github.com/udacity/RoboND-Localization-Project/blob/master/maps/jackal_race.pgm
(Wget may not work for the .pgm , it may have to be downloaded from GITHUB and copy/paste into “maps” folder)


$ wget https://raw.githubusercontent.com/udacity/RoboND-Localization-Project/master/maps/jackal_race.yaml


Earlier on in this lesson, you created an empty Gazebo world, called udacity.world. The map that you will be working with is generated based on its own world.
$ cd ..
$ cd worlds

Copy the file jackal_race.world from the project repo into the “worlds” folder.
$ wget https://raw.githubusercontent.com/udacity/RoboND-Localization-Project/master/worlds/jackal_race.world


Next, you will have to modify the udacity_world.launch file and update the path to this new map/world (
$ cd ..
$ cd launch/
$ nano udacity_world.launch
Modify the argument world_name (from udacity.world TO jackal_race.world) 


$ cd /home/workspace/catkin_ws/
$ roslaunch udacity_bot udacity_world.launch



## 7-AMCL ##


We will start by creating a new launch file.
$ cd /home/workspace/catkin_ws/src/udacity_bot/launch/
$ nano amcl.launch

This launch file has three nodes, one of which is for the amcl package. Copy the following into your launch file.
<?xml version="1.0"?>
<launch>

 <!-- Map server -->
 <arg name="map_file" default="$(find udacity_bot)/maps/jackal_race.yaml"/>
 <node name="map_server" pkg="map_server" type="map_server" args="$(arg map_file)" />

 <!-- Localization-->
 <node pkg="amcl" type="amcl" name="amcl" output="screen">
   <remap from="scan" to="udacity_bot/laser/scan"/>
   <param name="odom_frame_id" value="odom"/>
   <param name="odom_model_type" value="diff-corrected"/>
   <param name="base_frame_id" value="robot_footprint"/>
   <param name="global_frame_id" value="map"/>
 </node>

the move_base package using which you can define a goal position for your robot in the map, and the robot will navigate to that goal position


In the same launch file as above, copy the following code.
<!-- Move base -->
 <node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen">
   <rosparam file="$(find udacity_bot)/config/costmap_common_params.yaml" command="load" ns="global_costmap" />
   <rosparam file="$(find udacity_bot)/config/costmap_common_params.yaml" command="load" ns="local_costmap" />
   <rosparam file="$(find udacity_bot)/config/local_costmap_params.yaml" command="load" />
   <rosparam file="$(find udacity_bot)/config/global_costmap_params.yaml" command="load" />
   <rosparam file="$(find udacity_bot)/config/base_local_planner_params.yaml" command="load" />

   <remap from="cmd_vel" to="cmd_vel"/>
   <remap from="odom" to="odom"/>
   <remap from="scan" to="udacity_bot/laser/scan"/>

   <param name="base_global_planner" type="string" value="navfn/NavfnROS" />
   <param name="base_local_planner" value="base_local_planner/TrajectoryPlannerROS"/>

 </node>


</launch>



move_base also has its own set of required parameters that help it perform efficiently. Let’s add those configuration files first:
$ cd ..
$ mkdir config
$ cd config

From the repo, copy the following files and add them here in config:
* local_costmap_params.yaml
* global_costmap_params.yaml
* costmap_common_params.yaml
* base_local_planner_params.yaml
The above files already have some set of parameters and some values defined for you to help you get started.


$ wget https://raw.githubusercontent.com/udacity/RoboND-Localization-Project/master/config/base_local_planner_params.yaml
$ wget https://raw.githubusercontent.com/udacity/RoboND-Localization-Project/master/config/costmap_common_params.yaml
$ wget https://raw.githubusercontent.com/udacity/RoboND-Localization-Project/master/config/global_costmap_params.yaml
$ wget https://raw.githubusercontent.com/udacity/RoboND-Localization-Project/master/config/local_costmap_params.yaml


$ cd /home/workspace/catkin_ws/
$ roslaunch udacity_bot udacity_world.launch
In a new terminal,
$ roslaunch udacity_bot amcl.launch

In Rviz,
* Select “odom” for fixed frame
* Click the “Add” button and
   * add “RobotModel”
   * add “Map” and select first topic/map
      * The second and third topics in the list will show the global costmap, and the local costmap. Both can be helpful to tune your parameters.
   * add “PoseArray” and select topic /particlecloud
      * This will display a set of arrows around the robot.
Note: As you discovered in the EKF lab, you can save the above RViz setup in a configuration file and launch RViz with the same configuration every time. This will make the process more efficient for you!


In RViz, in the toolbar,
* Select “2D Nav Goal” click anywhere else on the map and drag from there to define the goal position along with the orientation of the robot at the goal position.


Whoa! Those warnings don't look good. The amcl.launch file is throwing a lot of warnings, your map in RViz might be flickering, your robot might not be moving too well or at all etc. All of these are expected. We have provided you with a template for launching the necessary nodes and packages. However, it is up to you to explore some of the important parameters and tune them to effectively navigate and accurately localize the robot in this map.




## 8-Localization Parameter Tuning 1 ##


8A- Transform Timeout
This maximum amount of delay or latency allowed between transforms is defined by the transform_tolerance parameter.
-Tune amcl node in amcl.launch file (under src/udacity_bot/launch) 
transform_tolerance: 0.2
-Tune move_base node in the costmap_common_params.yaml file (under src/udacity_bot/config)
<param name="transform_tolerance" value="0.2"/>
Tuning the value for this parameter is usually dependent on your system. Once you have the transform_tolerance variable defined and tuned properly, you should be able to visualize all the three maps in RViz without any issues, and the warning should disappear. Only to be replaced by a new warning.


## 8B- Map Update Loop ##
The warning seems to indicate that your map or costmaps are not getting updated fast enough. The update loop is taking longer than the desired frequency rate of 50 Hz or 0.02 seconds. 
On costmap_common_params.yaml :
-update_frequency: 10.0 (frequency with which your map is getting updated)
-publish_frequency: 10.0 (frequency with which your map is getting published)


Try running your project again, and define a goal position using the "2D Nav Goal" button in RViz, a short distance from your robot. Your robot should start moving! That's brilliant! But, it doesn't seem to be following the path and might be hitting the walls.


## 8C-you can also modify: ##
Apart from tuning the frequency with which your map is getting updated and published, you can also modify the dimension and resolution of your global and local costmaps.
local_costmap_params.yaml :
   width:  5.0   # original 20.0
   height: 5.0   # original 20.0
   resolution: 0.05
global_costmap_params.yaml :
   width:  15.0 # original 40.0
   height: 15.0 # original 40.0
   resolution: 0.15 # original 0.05


Modifying these parameters can help free up some resources, however, decreasing the resolution of your map by too much can lead to loss of valuable information too. For example, in case of small passages, low resolution might cause the obstacle regions to overlap in the local costmap, and the robot might not be able to find a path through the passage.
Note: Modifying the above two parameters might help with the overall response, but it could potentially also help with ensuring that your robot is able to follow the defined local path. This might come in handy to experiment with later as well.




Try running your project again, and define a goal position using the "2D Nav Goal" button in RViz, a short distance from your robot. Your robot should start moving! That's brilliant! But, it doesn't seem to be following the path and might be hitting the walls.




## 9-Localization Parameter Tuning 2 ##
costmap_common_params.yaml file, and some of the parameters pre-defined (not tuned)
obstacle_range: 0.0
raytrace_range: 0.0
inflation_radius: 0.0

* obstacle_range - For example, if set to 0.1, that implies that if the obstacle detected by a laser sensor is within 0.1 meters from the base of the robot, that obstacle will be added to the costmap. Tuning this parameter can help with discarding noise, falsely detecting obstacles, and even with computational costs.
* raytrace_range - This parameter is used to clear and update the free space in the costmap as the robot moves.
* inflation_radius - This parameter determines the minimum distance between the robot geometry and the obstacles. Try setting a really high value for this parameter, and launch the project and select the global costmap selected. You will notice that the obstacles (the walls of the environment) seem to be "inflated" as can be seen below. An appropriate value for this parameter can ensure that the robot smoothly navigates through the map, without bumping into the walls and getting stuck, and can even pass through any narrow pathways.


To tune the above three parameters, we recommend the following approaches. In each case observe the differences to judge what value works best for this map -
1. Try setting a high value for inflation radius first (10.0), and have the robot navigate to your choice of a goal position. Then repeat the same with a low value(0.1)
2. Next, set the values for the obstacle (3.0) and raytracing (3.0) ranges and have the robot navigate to a position, again. Try to see if it can effectively navigate through the narrow passage and then take a turn.
3. Add an object, such as the Brick Box in Gazebo, as shown below. Try to observe how your previously selected parameter values behave with an object blocking part of the map. Note: This is only for testing purposes, and is not meant to be part of your project.
In each of the above cases, compare the results for both the global costmap and the local costmap.


It is highly recommended that you go through the documentation corresponding to these packages, and the associated parameters and identify which ones could help improve your results:
http://wiki.ros.org/costmap_2d
http://wiki.ros.org/navigation/Tutorials/RobotSetup


## 9B-Robot Localization AMCL parameters ##
identify and tune parameters for your amcl node in the amcl.launch file, to achieve better results. Three categories: overall filter, laser, and odometry


Overall Filter
* min_particles and max_particles - tuned based on your system specifications. A larger range, with a high maximum might be too computationally extensive for a low-end system.
* initial_pose - For the project, you should set the position to [0, 0]. Feel free to play around with the mean yaw value.
* update_min* - amcl relies on incoming laser scans. Upon receiving a scan, it checks the values for update_min_a and update_min_d and compares to how far the robot has moved. Based on this comparison it decides whether or not to perform a filter update or to discard the scan data. Discarding data could result in poorer localization results, and too many frequent filter updates for a fast moving robot could also cause computational problems.
Laser
There are two different types of models to consider under this - the likelihood_field and the beam. Each of these models defines how the laser rangefinder sensor estimates the obstacles in relation to the robot.
The likelihood_field model is usually more computationally efficient and reliable for an environment such as the one you are working with. So you can focus on parameters for that particular model such as the -
* laser_*_range
* laser_max_beams
* laser_z_hit and laser_z_rand
Tuning of these parameters will have to be experimental. While tuning them, observe the laser scan information in RViz and try to make sure that the laser scan matches or is aligned with the actual map, and how it gets updated as the robot moves. The better the estimation of where the obstacles are, the better the localization results.
Odometry
odom_model_type - Since you are working with a differential drive mobile robot, it’s best to use the diff-corrected type. There are additional parameters that are specific to this type - the odom_alphas (1 through 4). These parameters define how much noise is expected from the robot's movements/motions as it navigates inside the map.
Note: The odometry information for this project is received directly from Gazebo, and is equivalent to the ground truth value (no noise expected). So, you need not have to tune these parameters and can leave them at their default values. But feel free to experiment with some values and see if you notice any changes.
Important: The above set of parameters should help you get started, however they aren't the only ones that can improve your results. You are encouraged and required to go through the documentation, identify which parameters might help you improve your localization results, and experiment with them. All the remaining parameters and corresponding documentation can be found on the ROS wiki's amcl page.
Identifying and tuning all these parameters can take time and effort. But don't worry. Based on the information and resources provided uptil now, you are well-equipped to tackle the problem head-on! Make sure to discuss your approaches with your fellow students in the ND Slack, and to reach out to your mentor for any further help.




## 10A-Testing ##
test your implementation on a shorter path rather than the entire map.
The pose your robot starts with places it somewhere in the middle of a corridor. It is recommended that you carry out some initial tests across the length of that corridor before the robot takes a turn. This will help you figure out several aspects to improve your implementation. You can figure out if the robot gets stuck or not initially based on where it thinks are the walls with respect to its position, how quickly the robot moves and how quickly it sticks to the trajectory, and more importantly how good the PoseArray looks as the robot moves forward. Does it shrink or get worse?
You can easily carry out these tests using the 2D Nav Goal button in RViz’ toolbar, as we covered in another section.
## 10B-Launching ##
The above method is great for testing, but for your project submission your robot needs to navigate to a specific goal position while localizing itself along the way. In the project repo we have provided you with a C++ node that will navigate the robot to the goal position for you. You will need to create a new folder for that.
$ cd /home/workspace/catkin_ws/src/udacity_bot
$ mkdir src
$ cd src
$ wget https://raw.githubusercontent.com/udacity/RoboND-Localization-Project/master/src/navigation_goal.cpp


In order to use or launch this node, you will first need to compile it. Fortunately, ROS can handle that for you. You will have to modify your CMakeLists.txt file for that.
$ cd /home/workspace/catkin_ws/src/udacity_bot

Replace this file with the file present in the repo. 
$ wget https://raw.githubusercontent.com/udacity/RoboND-Localization-Project/master/CMakeLists.txt


Then,
$ cd /home/workspace/catkin_ws
$ catkin_make
$ source devel/setup.bash

Launch (in 3 terminals):
$ cd /home/workspace/catkin_ws/
$ source devel/setup.bash
$ roslaunch udacity_bot udacity_world.launch


$ cd /home/workspace/catkin_ws/
$ source devel/setup.bash
$ roslaunch udacity_bot amcl.launch
(NOTE: at this point you can still set the 2D navigation goal manually on RVIZ)


$ cd /home/workspace/catkin_ws/
$ source devel/setup.bash
$ rosrun udacity_bot navigation_goal

The above will run the node, and you will notice your robot moving to the goal position.
You can also display the goal position in RViz using the “Pose” display. Try it out!
Earlier we mentioned that you could create your own RViz configuration file instead of adding different elements everytime you launched the project. In the repo we have provided you with one such file that you can add to your package.
