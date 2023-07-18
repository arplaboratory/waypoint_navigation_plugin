# waypoint_navigation_plugin

This directory contains both waypoint_navigation_plugin and quadrotor_path

Prerequisite for 2D plotting
Install

```
sudo apt-get install gnuplot
cd /path/to/catkin_directory/
catkin build waypoint_navigation_plugin
source /path/to/catkin_directory/devel/setup.bash
```

Waypoint navigation contains a GUI dependent on ARPL_msgs. quadrotor_path contains a buffer for msgs of a certain odometry for visualization.
Waypoint navigation contains rqt_mav_manager. To use qp_tracker
1. Use Gui to Motors On
2. Use Gui to Takeoff
3. Drag drop waypoints.
4. Hit Pubish
