# autonomous_collision_avoidance

This is a gazebo simulation about UAV autonomous obstacle avoidance!.

As what depicted in the picture below, the task includes five parts, including flying through two columns(A), flying through two walls(B), flying through a circle with a determined heading(C), flying through three circles with diferent headings(D), flying through a corrider(E). The localization part is not considered in the gazebo simulation, and the control of the UAV is realized based on PX4 firmware and the simulation is a test about the planning part. The main work is based on the PX4 avoidance which is realized based on 3D VFH+. The original repo is here. 

Task A, B, C can be easily finished in the simulation, due to the uncertainty of the heading angles of the circles, the realization of task D is somewhat difficult. Considering the Aerodynamics changes in the corrider which makes a great impact on the control of the UAV, task E is abstained.

![image](https://github.com/ZhangWuji-1998/autonomous_collision_avoidance/blob/master/pictures/map.png)

# how to use

cd to your ros workspace
```
cd your_ws
```
clone the repo
```
git clone https://github.com/ZhangWuji-1998/autonomous_collision_avoidance.git
``` 
build the code
```
catkin_make
```
source the workspace
```
source devel/setup.bash
```
launch the collision avoidance file
```
roslaunch local_planner local_planner_depth-camera.launch
```
open a new terminal and launch the planning file
```
roslaunch local_planner cs_ll_controller_0921.launch
```
open a new terminal and launch the circle detection file
```
roslaunch huofu huofu.launch
```
the gazebo simulation is showed as follows.

![image](https://github.com/ZhangWuji-1998/autonomous_collision_avoidance/blob/master/pictures/snapshots.png)
