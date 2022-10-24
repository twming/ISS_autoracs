# Project AutoRACS

This ROS package provides the launch, world, and model files to
load the simple lake terrain into Gazebo and spawn 3 turtlebots onto the terrain.


## Installation

1. Clone this repository into the `src` in your ROS1 workspace. Assuming that you are using the ubuntu16 VM that we
   all spawn from

        $ cd /home/ubuntu16/catkin_ws/src/
        $ git clone git@github.com:powlook/project_autoracs.git

2. catkin make the project_autoracs

        $ cd /home/ubuntu16/catkin_ws
        $ catkin_make
   When you catkin_make, check that there are no errors in the make process. Fix the errors even if the catkin_make
   is successful.

3. Open a new terminal and launch the example below

        $ roslaunch multi_world multi_robots_rectangle.launch


At this point, Gazebo should have launched, loaded the multiple robots in the rectangle world. Project folder structures as below:
project_autoracs:
multi_world - store the world and maps
multi_slam - multiple robots SLAM
multi_nav - multiple robots navigation


## Multiple turtlebots SLAM

1. First, you need to install the ros-kinetic-multirobot-map-merge package (only need to do for the first time)

        $ sudo apt install ros-kinetic-multirobot-map-merge

2. Start the pillar10x10 world (pillar10x10.world) with 2 turtlebots. Utilize the turtlebot3_drive.cpp to navigate the turtlebots.

        $ roslaunch multi_slam robots_multi_moving.launch
        
or 

        $ roslaunch multi_slam robots_multi_moving.launch my_world:=rectangle_v2.world

3. Subcript to the map topics of both turtlebots and merge the 2 maps

        $ roslaunch multi_slam robots_multi_map_merge.launch

or 

        $ roslaunch multi_slam robots_multi_map_merge.launch first_tb3_x_pos:=<x1> first_tb3_y_pos:=<y1> first_tb3_z_pos:=<z1> first_tb3_yaw:=<yaw1> second_tb3_x_pos:=<x2> second_tb3_y_pos:=<y2> second_tb3_z_pos:=<z2> second_tb3_yaw=<yaw2>

4. Visualize the maps

        $ rosrun rviz rviz -d `rospack find multi_slam`/rviz/robots_multi_slam.rviz

5. Save the map

        $rosrun map_server map_saver -f ~/map/map_filename

## Multiple turtlebots Navigation

1. Launch the global map (after merging) and the pillar10x10.world

        $ roslaunch multi_nav robots_multi_static.launch

2. Launch the navigation node for multiple turtlebots navigation

        $ roslaunch multi_nav robots_multi_navigation.launch

FAQ: 
1. I have issue with map_merge package. Check if the ros-kinetic-multirobot-map-merge already installed.
2. Why initial_pose.py fail? Check the file, make sure it is executable.

