# UR3 Admittance Controller [DEVEL]


This repository provides admittance contoller for the ur3 robotic-arm. 


* [admittance_control](https://github.com/woo-rookie/ur3_admittance_controller/tree/main/admittance_control): 
This package implements an admittance controller on the UR3 arm (see below for the control architecture). 
* [ur3_cartesian_velocity_control](https://github.com/woo-rookie/ur3_admittance_controller/tree/main/ur3_cartesian_velocity_control): This package provides a cartesian velocity controller (ros control) for the UR3 arm. 
* [ur3_bringup](https://github.com/woo-rookie/ur3_admittance_controller/tree/main/ur3_bringup): This package provides a series of launch files and ROS settings in order to start-up the real-robot as well as the simulator. 
* [cartesian_state_msgs](https://github.com/woo-rookie/ur3_admittance_controller/tree/main/cartesian_state_msgs): It contains the defintion of message type "PoseTwist" (combination of the standard ros/geometry_msgs pose and twist).
* [ur3_description](https://github.com/woo-rookie/ur3_admittance_controller/tree/main/ur3_description): It contains the ur3 description xacro files.

---

## Compliation and build

Clone the repository intor your catkin source directory
```bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/woo-rookie/ur3_admittance_controller.git
```

Finally complie
```bash
$ cd ~/catkin_ws
$ catkin_make
$ source devel/setup.bash
$ catkin_make
```
* you might need the source the bash file and compie again if the first compliation could not find some of in house dependencies.

---
---


## Running the controller


To bring up the robot in simulation run
```
roslaunch ur3_bringup ur3_bringup.launch
roslaunch admittance_control admittance_controller.launch
```
For the real robot launch on the CPR main PC run
```
roslaunch ur3_bringup ur3_bringup.launch sim:=false
roslaunch admittance_control admittance_controller_real.launch
```

# Expected behaviors

The behavior of different components are demonstrated here:
* [admittance control](https://youtu.be/e6_z8rCOoIs) and its [rviz](https://youtu.be/T10rhY_HqZo)


# Control Architecture

## Kinematics and transformations

Here is a short list of important frames and their usage.

| frame id      | Usage                         |
|---------------|-----------------------------------|
| world                          |            |
| ur3_arm_base_link              | Arm pose and twist                |
| base_link                      |            |
| robotiq_ft_frame_id  | External force applied to the end-effector           |





## Adamittance dynamics
The following figure shows the controller architecture for the admittance control on the robot.

<<<<<<< HEAD
![alt text](fig_admittance_controler_schematic.png "Control architecture")
=======
![alt text](fig_controler_schematics.png "Control architecture")
>>>>>>> 4231e8dbcb4c404a137a87df67370cead8b8b2fe

The two equations in the center describe the admittance dynamics which compute the desired accelaration for the arm. These accelerations are integrated in time to acheive the desired velocities for the robot. The low-level velocity controller fullfills these velocities.



The admittance parameters (shown in purple) are as follows: 

| Variable      | Parameter                         |
|---------------|-----------------------------------|
| M<sub>a</sub> | Desired mass of the arm           |
| D<sub>a</sub> | Desired damping of the arm        |
| D<sub>c</sub> | Desired damping of the coupling   |
| K<sub>c</sub> | Desired Stiffness of the coupling |

These parameters are load from a yaml through the launch file.


### External force
The external is initially measured by the force/torque sensor in its own frame reference. In admittance controller this force is transformed to "ur3_arm_base_link" where the control of arm takes place. To avoid reacting to small forces a deadzone is considered. Moreover, a low-pass filter is used to smooth the measurements. The parameters of the deadzone and the filter can be set from the launch file.

### Higher-level control (Motion planning )
Through a higher level controller, the position of the equilibrium can be can be changed to acheive a desired behavior. Also, F<sub>c</sub> can be used for control purposes; e.g., to compensate for the mass of an object carried by the arm.












