# Gazebo 2WD with Gmapping Launch Commands

(1) roscore : Launch roscore

(2) roslaunch gazebo_ros empty_world.launch : Launch Gazebo with empty world

(3) roslaunch 2WD_SLAM 2wd_spawn.launch : Launch main code for spawning 2WD model with Gmapping and world models

(4) ~/.bashrc 파일에 다음 내용을 추가합니다. 

    export GAZEBO_MODEL_PATH=~/catkin_ws/src/SLAM_Odometry_Practice2021/03_Term_Project_Indoor_SLAM_Gazebo/map:$GAZEBO_MODEL_PATH

(5) source ~/.bashrc를 통해 변경 사항을 적용합니다.

(6) 다음의 명령어를 통해 world 생성 및 map2gazebo를 통해 생성한 stl파일을 world에 spawn합니다.

    roslaunch 2WD_SLAM open_map.launch
    roslaunch 2WD_SLAM spawn_wall.launch 