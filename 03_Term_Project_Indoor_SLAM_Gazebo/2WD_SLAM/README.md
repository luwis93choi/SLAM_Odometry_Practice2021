# Installation

(1) clone '2WD_SLAM' and 'map2gazebo' in /catkin_ws/src

(2) Build 2WD_SLAM
- catkin_me --only-pkg-with-deps 2WD_SLAM

(3) Build map2gazebo (https://github.com/shilohc/map2gazebo)
- catkin_make --only-pkg-with-deps map2gazebo

# Launch Commands for Gazebo 2WD with Gmapping

(1) roscore : Launch roscore

(2) Place 2D floor plan image in /map/raw_map/<'filename'>.png

(3) Place yaml of 2D floor plan image in /map/map_model/<'filename'>.yaml

(4) Write yaml for 2D floor plan image

(5) Robot navigation environment model stl generation from 2D floor plan
- Required packages : map_server, map2gazebo
- map_sever : Convert 2D floor plan image (png, pgm) into occupancy grid ROS topic (nav_msgs/OccupanyGrid).
- map2gazebo : Convert occupancy grid ROS topic into 3D mesh stl

- rosrun map_server map_server /home/luwis/catkin_ws/src/map2gazebo/src/raw_map.yaml
- roslaunch map2gazebo map2gazebo.launch

(6) Launch Gazebo world with 3D mesh model of 2D floor plan
- roslaunch 2WD_SLAM open_map.launch

(7) Launch main code for spawning 2WD model with Gmapping in current world
- roslaunch 2WD_SLAM 2wd_spawn.launch

(8) Open Rviz for Gmapping SLAM visualization
- rviz

(10) Use teleop package input commands to contorl 2WD in Gazebo world

# Acknowledgement

map2gazebo (https://github.com/shilohc/map2gazebo) is used to convert occupancy grid of 2D floor plan image into Gazebo stl mesh.

- Minor changes on map2gazebo : Remove 'image' in Line 72 of map2gazebo.py

    - cv2.findContours in OpenCV 2.4 returns only 2 values (contours, hierarchy). Remove 'image' to support the number of output.
    - cv2.findContours in OpenCV 3.x returns 3 output values (image, contours, hierarchy)
    - Reference : https://answers.opencv.org/question/32248/open-cv-30-findcontours-return-too-many-value-to-unpack/