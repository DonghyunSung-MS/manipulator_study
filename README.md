manipulator_study
=================
## Enviroment
* ubuntu 16.04 ROS kinietic
* Coppeliasim Pro Edu (next version of V-rep)

## How to use it
```
 $ catkin_make  #in your caktin workspace
```
#### Forwad Kinematics with Franka in Coppeliasim
###### Discription
* This progam shows how forward Kinematics works.
* It uses pre-defined target joint position and execute time.
* it generate trajectory with cublic spline .
* Coppeliasim model has its own joint feedback loop with pid.

###### Before launch
* Open vrep and load model in the model folder.

###### Set Target Joint Positon once
```
 $ roslaunch manipulator_study demo1.launch
```
###### Set Target Joint Positon mutiple times
```
 $ roslaunch manipulator_study demo2.launch
```
```
 $ rosrun manipulator_study set_target_position.py
```
* You can type joints' target position and execute time in here.
