# waypoint_navigation_plugin

This directory contains both waypoint_navigation_plugin and quadrotor_path

Prerequisite for 2D plotting THIS WILL WIPE RVIZ1 PLUGIN
Install

```
sudo apt-get install gnuplot
cd /path/to/colcon_ws/
colcon build --symlink-install
source /path/to/colcon_ws/install/setup.bash
```

Waypoint navigation contains a GUI dependent on ARPL_msgs. OPEN RVIZ
open file at /path/to/way_nav_plugin/rviz/waypoint_tool.rviz


quadrotor_path contains a buffer for msgs of a certain odometry for visualization.
Waypoint navigation contains rqt_mav_manager. To use qp_tracker
1. Use Gui to Motors On
2. Use Gui to Takeoff
3. Drag drop waypoints.
4. Hit Pubish

