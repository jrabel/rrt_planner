# RRT Path Planner
This project is a ROS package that implements a global path planner plugin for the ROS navigation stack. The planner creates a rapidly-exploring random tree throughout the configuration space to search for a valid path to a goal pose.

### Installation
This package was built using Ubuntu 16.04 and the ROS Kinetic distribution. You must have a catkin workspace setup to build this package. Therefore, you can use an existing one or follow the instructions below to set one up.
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```
In the `src` directory of your workspace, clone this repository.
```
cd src
git clone https://github.com/jrabel/rrt_planner.git
```
From the main workspace directory, build the workspace. Then source your workspace environment.
```
cd ..
catkin_make
source devel/setup.bash
```

### Usage
The package is set up to export a global path planner plugin for use by the ROS navigation stack. However, you can run the default example of the RRT planner using the included launch file.
```
roslaunch rrt_planner rrt_planner.launch
```
This will launch a simulation within Rviz showing a robot (R2D2) inside a 2D maze map. You can send the planner goal poses by pressing the '2D Nav Goal' button on the top toolbar within Rviz. Now simply click on a valid point on the map to send a goal pose (note: you can click and hold to then select an orientation). Now watch as the planner displays the rapidly-exploring random tree as shown below.

![Example in Rviz](/images/rrt_planner_global_plan_closeup.png)
