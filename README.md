# zm_dwa_local_planner

zm_dwa_local_planner is revised DWA local planner algorithm under ROS using DWA for AGV using mecanum wheel motion.

Only x-y linear moving to avoiding obstacles when AGV detect to have obstacles.

Software : Robot Operating System.

OS: Linux Ubuntu.

------

Step1. Download zm_dwa_local_planner github link.

``` bash
$ cd <catkin_ws>/src
```

``` bash
$ git clone https://github.com/qaz9517532846/zm_dwa_local_planner.git
```

``` bash
$ cd ..
```

``` bash
$ catkin_make
```

Step2. zm_dwa_local_planner add to move_base.launch file.

``` bash
  <node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen">
    <param name="base_local_planner" value="zm_dwa_local_planner/zmDWALocalPlannerROS" />
    <rosparam file="$(find zm_robot_navigation)/param/costmap_common_params.yaml" command="load" ns="global_costmap" />
    <rosparam file="$(find zm_robot_navigation)/param/costmap_common_params.yaml" command="load" ns="local_costmap" />
    <rosparam file="$(find zm_robot_navigation)/param/local_costmap_params.yaml" command="load" />
    <rosparam file="$(find zm_robot_navigation)/param/global_costmap_params.yaml" command="load" />
    <rosparam file="$(find zm_robot_navigation)/param/move_base_params.yaml" command="load" />
    <rosparam file="$(find zm_robot_navigation)/param/dwa_local_planner_params.yaml" command="load" />
    <remap from="cmd_vel" to="$(arg cmd_vel_topic)"/>
    <remap from="odom" to="$(arg odom_topic)"/>
    <param name="DWAPlannerROS/min_vel_x" value="0.0" if="$(arg move_forward_only)" />
  </node>
```

-----

# Reference

[1]. ros_navigation. https://github.com/ros-planning/navigation

------

This repository is for your reference only. copying, patent application, academic journals are strictly prohibited.

Copyright © 2020 ZM Robotics Software Laboratory.
