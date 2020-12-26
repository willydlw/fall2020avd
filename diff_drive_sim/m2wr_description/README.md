

### Terminal 1

```
source /opt/ros/melodic/setup.bash
roslaunch gazebo_ros empty_world.launch
```


### Terminal 2

```
source ~/simulation_ws/devel/setup.bash
roslaunch m2wr_description spawn.launch
```


### Terminal 3

To see the list of topics: `rostopic list` 

One of the topics is cmd_vel.


Use the keyboard to move the robot around

```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```