# ROS2 Humble Hawksbill Tutorials
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)

## Overview
This repository is a ROS2 package to implement a walker node. The walker algorithm has emulate the Roomba robot. 
The walker node will subscribe to the laser scan topic and publish to the velocity topic. 
It will use Gazebos to simulate the robot and the environment. 
The robot used is the Turtlebot3 Burger. 

[1] [ROS2 Working with Gazebo](https://umd.instructure.com/courses/1336069/assignments/6188373)

## Personnel
- Sanchit Kedia (UID: 119159620) 

# Dependencies
- ROS2 (Humble Hawksbill Binary Install)
- Ubuntu 22.04 LTS

# Build Package
```
source /opt/ros/humble/setup.bash  # Source your ROS2 Installation
cd <Your ROS2 workspace src folder>
git clone https://github.com/Sanchitkedia/walker.git
cd ..
rosdep install -i --from-path src --rosdistro humble -y #Check for missing dependencies
colcon build --packages-select walker
```

## Run Package
```
cd <Your ROS2 workspace>
source /opt/ros/humble/setup.bash  # Source your ROS2 Installation
. install/setup.bash #Source the setup files
export TURTLEBOT3_MODEL=burger #export model name
ros2 launch walker roomba.launch 
ros2 launch walker roomba.launch record:=True # Run this to launch with bag recording
```

### To Play data from rosbag file or check the informating in the bag file
```
cd <Your ROS2 workspace>
source /opt/ros/humble/setup.bash  # Source your ROS2 Installation
. install/setup.bash #Source the setup files
ros2 bag info src/walker/bagfiles/walker_bagfile #To check the information in the bagfile
ros2 bag play src/walker/bagfiles/walker_bagfile #To play the bagfile
```
## Cpplint
```
pip3 install cpplint # If not already installed
cd <Your ROS2 workspace>/src/walker/src
cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order walker_node.cpp > ../results/walker_node_cpp.txt
```

## Cppcheck
```
sudo apt-get install cppcheck # If not already installed
cd <Your ROS2 workspace>/src/walker/src
cppcheck --enable=all --std=c++17 --force --suppress=missingIncludeSystem walker_node.cpp > ../results/cppcheck.txt
```

## License

MIT License

Copyright (c) 2022 Sanchit Kedia.

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
