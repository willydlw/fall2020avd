# Differential Drive Robot Simulation Using ROS

This unit uses ROS (Robot Operating System) features and tools to study the motion behavior of a two wheeled mobile robot. 


## Lessons

### Lesson 1

Build a robot model from scratch, using URDF (Unified Robot Description Format) and RViz to visualize it.




 Exploring ROS and Gazebo with a 2 Wheeled Robot



The my_worlds package was added in part 4. https://www.theconstructsim.com/exploring-ros-2-wheeled-robot-part-04-laser-scan-reading/

I am running everything locally, on my computer, from the command line, and occasionally have to modify a file. Modifications and run instructions are shown below.

This project is about using a Two Wheeled Mobile Robot to explore features and tools provided by ROS (Robot Operating System). We start building the robot from the scratch, using URDF (Unified Robot Description Format) and RViz to visualize it. Further, we describe the inertia and show how to simplify the URDF using XACROS. Later, motion planning algorithms, such as Obstacle Avoidance and Bugs 0, 1 and 2 are developed to be used in the built robot. Some ROS packages, like robot_localization, are used to built a map and localize on it.


### world.launch modifications

Changed simulation_gazebo to gazebo_ros 


### Terminal 1, Launch the gazebo world

My catkin workspace is named simulation_ws. My terminal's .bashrc file sources the ros path, each time a terminal is opened. If yours does not, you may need to run the command: `source /opt/ros/melodic/setup.bash` 

```
cd ~/simulation_ws
catkin_make
source ./devel/setup.bash
roslaunch gazebo_ros empty_world.launch
```


### Terminal 2

```
source ~/simulation_ws/devel/setup.bash
roslaunch m2wr_description spawn.launch
```

When the robot has spawned,

```
rosrun motion_plan reading_laser.py
``` 

You will see the laser output values, similar to those below.

```
[INFO] [1591718806.844912, 2080.227000]: [9.345121383666992, 9.467138290405273, 9.571355819702148, 9.384583473205566, 10]
[INFO] [1591718806.896321, 2080.276000]: [9.343762397766113, 9.462581634521484, 9.582467079162598, 9.370244026184082, 10]
[INFO] [1591718806.949253, 2080.327000]: [9.341439247131348, 9.457085609436035, 9.573403358459473, 9.385207176208496, 10]
[INFO] [1591718807.001094, 2080.376000]: [9.326991081237793, 9.461615562438965, 9.593852043151855, 9.379280090332031, 10]
```


### Terminal 3

To see the list of topics: `rostopic list` 

One of the topics is cmd_vel.


Use the keyboard to move the robot around. You will see the laser output values changing.

```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```