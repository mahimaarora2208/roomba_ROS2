# roomba_ros2
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
---
# Obstacle Avoidance in ROS2

### Overview

This is a ROS Package that defines a basic obstacle_avoidance using Gazebo and Turtlebot3.
- Use of all five logger levels: ```Fatal, Error, Warn, Info and Debug.```

### Dependencies/ Assumptions
- OS : Ubuntu 20.04 
- ROS2 Distro : ROS2 Galactic
- Package build type : ```ament_cmake ```
- Package dependencies : ```rclcpp```, ```std_msgs``` 
- ROS2 Galactic Installation : [link](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html)

## How to Run the ROS Package
### Build Instructions
```
cd <your_ROS2_ws>/src
git clone https://github.com/mahimaarora2208/roomba_ros2.git
cd ..   
rosdep install -i --from-path src --rosdistro galactic -y

export GAZEBO_MODEL_PATH=`ros2 pkg prefix turtlebot3_gazebo`/share/turtlebot3_gazebo/models/
export TURTLEBOT3_MODEL=waffle_pi

colcon build --packages-select roomba_ros2
source . install/setup.bash

```
#### Note:
In spawn_turtlebot3.launch.py file , You will need to change urdf_path = 'your model.sdf path'. This is hardcoded for now and it is meant to be changed to dynamic path. 

### Run Launch File to run simulation
To run the Launch file for obstacle avoidance node,run:
```

ros2 launch roomba_ros2 turtlebot3_world.launch.py
```

### ROS2 Bags and Enable/Disable them (and RESULTS)

To enable/disable rosbags, go to file ros_bag.launch.py and change "record_all_topics". (set to true by default)

Rosbag result files:[link](https://drive.google.com/drive/folders/1MS1ONd6Gny0ZCTlkEorER58Ab8U5tcnC?usp=share_link)

To run them separately, you can use:
```
ros2 bag record -o rosbag_files <topic>
```
To check topic information that is in the rosbag:
```
ros2 bag info rosbag_files
```
Lastly, to play back the data from rosbags:
```
ros2 bag play rosbag_files
``` 
## Results
The results after running the following commands are stored in the <your_package>/results folder.

### rqt Console
```
 ros2 run rqt_console rqt_console

```
### cppcheck
Run the following command from the root directory of your ROS package
```
cppcheck --enable=all --std=c++17 ./src/*.cpp --suppress=missingIncludeSystem --suppress=unmatchedSuppression --suppress=unusedFunction --suppress=missingInclude --suppress=useInitializationList > results/cppcheck.txt
```
### cpplint
Run the following command from the root directory of your ROS package
```
 cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order ./src/*.cpp > ./results/cpplint.txt
```
### Google Styling format
Run the following command from the directory where the .cpp files are present(src in this case)
```
clang-format -style=Google -i your_file.cpp
```
