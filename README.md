# Courier Robot for Office Logistics

## Project Desciption 
This project is an implementation of navigation stack configured with real hardware. The aim of this project was to develop a prototype of a courier robot that could be used for logistics of documents in an office space.

![Simulation 1](/Media/AMR_Video.gif)

## Main Modules 
* Simulation of Turtlebot
* Developing ROS and hardware interface for control of wheel velocities
* Mapping environment
* Configuration of costmap and planner parameters 
* Navigation 


## Platform
* Ubuntu 18.04 LTS
* ROS Melodic
* Python 2.7 

## Hardware Used
* VEX Motors
* Arduino Mega 
* Intel i7 Processor 
* Li-Po Battery
* Robot Chassis
* RPLidar

## Implementation
1. To map your environment, execute the steps in the catkin_ws directory


```$ catkin_make```

```$ source devel/setup.bash```

```$ roslaunch agv agv_odom_ekf_spawn.launch```

```$ roslaunch agv agv_device_init.launch```

```$ roslaunch agv agv_hector.launch```

```$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py ```



2. To navigate the robot in your environment, execute the steps in the catkin_ws directory

```$ catkin_make```

```$ source devel/setup.bash```

```$ roslaunch agv agv_odom_ekf_spawn.launch```

```$ roslaunch agv agv_device_init.launch```

```$ roslaunch agv agv_agv_move_base_amcl_turtlebot.launch```


