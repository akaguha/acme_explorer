# acme_explorer
[![Build Status](https://travis-ci.org/akaguha/acme_explorer.svg?branch=master)](https://travis-ci.org/akaguha/acme_explorer)
[![Coverage Status](https://coveralls.io/repos/github/akaguha/acme_explorer/badge.svg?branch=master)](https://coveralls.io/github/akaguha/acme_explorer?branch=master)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-green.svg)](https://opensource.org/licenses/BSD-3-Clause)

# Overview
Increasing need for automated surveillance of indoor environments, such as airports, warehouses, production plants, etc. has stimulated the development of intelligent systems based on mobile sensors. Use of robots significantly expands the potential of surveillance systems, which can evolve from the traditional passive role, in which the system can only detect events and trigger alarms, to active surveillance, in which a robot can be used to interact with the environment, with humans or with other robots for more complex cooperative actions.

The Acme Explorer is a general robotic platform with localization, mapping and navigation capability. In its explorer mode, the robot autonomously navigates in an unknown environment using the exploration behavior. It actively avoids obstacles and at the same time maps the environment (2D SLAM).

Continuous video surveillance is carried out using the onboard camera module. An adhoc service can be called to take a snapshot of the environment at any point of time.

# License
```
BSD 3-Clause License

Copyright (c) 2018, Akash Guha
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```

# API and Other Developer Documentation

## Solo Iterative Process
SIP is a software process that single programmers use while working on a software project. The solo programmer who is the hero of this process is named “Sol”. Sol receives the requirements from the users and records them in the product backlog. From the iteration backlog, Sol selects a specific change request and implements the corresponding software changes. 

Product Backlog, Work log, Iteration log and Defect log for this project can be found [here][reference-id-for-here1].

The sprint planning notes can be viewed [here][reference-id-for-here2].

[reference-id-for-here1]: https://docs.google.com/spreadsheets/d/1DluQNZ4hPmuc6m9_1VbXkiNKjYlFCkAW2cCV9fm4xiM/edit?usp=sharing
[reference-id-for-here2]: https://docs.google.com/document/d/1Q8z-YIIRtX_Bhl94cPWdnv7DTf7MGm8ksgNLWIYC4Qo/edit?usp=sharing

# Dependencies
Following dependencies need to be installed before proceeding
- Ubuntu 16.04
- ROS Kinetic
- Gazebo version 7.0.0 or above
- rviz
- Catkin
- Turtlebot packages
- gmapping
- map_server
- OpenCV

Run the following command to install turtlebot packages
```
sudo apt-get install ros-kinetic-turtlebot-*
```

A complete installation guide for OpenCV can be found [here][reference-id-for-here].

[reference-id-for-here]: https://docs.opencv.org/3.3.1/d7/d9f/tutorial_linux_install.html

# Operation
The working pipeline is as follows,
On initialization, the robot rotates on the spot to get an initial understanding of its position and the surroundings. Then it starts moving around the world to simultaneously localize itself and map the environment. It actively avoids obstacles in the environment during navigation. After every 75 seconds the robot stops and rotates on the spot for a fixed period of time as part of the exploratory behavior. This also results in some degree of randomness to the robot's path.

Video captured by the onboard camera is viewed in a seperate window. This is part of the surveillance functionality of the robot. At any point in time the snapshot service can be called to click and save the image of the environment.

This process is represented in the given activity diagram

# Build Instructions
Create and build a catkin workspace.
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```
Source your new setup.*sh file
```
source devel/setup.bash
```
Clone the package in the src folder and build
```
cd src/
git clone --recursive https://github.com/akaguha/acme_explorer.git
cd ..
catkin_make
```

# Test Instructions
To run all unit tests
```
cd <path to catkin workspace>
catkin_make run_tests
```
To run tests using launch file
```
rostest acme_explorer AcmeExplorerTest.launch
rostest acme_explorer NavTest.launch 
rostest acme_explorer ImageTest.launch
```

# Demo Instructions
To run the demo follow the below instructions
```
cd <path to catkin workspace>
source ./devel/setup.bash
```
Initial setup: To bring up the turtlebot in an unknown environment, start gmapping and rviz run the following launch file
```
roslaunch acme_explorer setup.launch
```
Then to run the acme_explorer package launch the following file
```
roslaunch acme_explorer acmeExplorerLaunch.launch
```
Now the turtlebot starts exploring the environment and a small surveillance window opens

To take a snapshot of the environment at any point of time call the /snap service and pass the file name as an argument. Below TestImg1 is the file name to save the picture
```
rosservice call /Snap TestImg1
```
After you are satisfied with the map generated, run the following command to save the map. This saves the map 'shapeWorldSLAM_' in the custom_maps folder in the package
```
rosrun map_server map_saver -f /home/akash/ENPM808X_ROS_workspace/src/acme_explorer/custom_maps/shapeWorldSLAM_

```
# Rosbag record/playback
To run the demo and record rosbag file in the results folder
```
roslaunch acme_explorer acmeExplorerLaunch.launch record_bag:=true
```
To play the recorded rosbag file
```
cd <path to repository>/results
rosbag play rosbagRecordings.bag
```
# Known Issues/Bugs
Due to randomness in the exploratory behavior many times the robot is unable to cover all the locations in an environment and keeps exploring already known locations. Thus need an algorithm for more directed search.

Issue with coveralls for testing code coverage.

# Doxygen Documentation
To generate Doxygen Documentation,
```
cd <path to repository>
mkdir <documentation_folder_name>
cd <documentation_folder_name>
doxygen -g <config_file_name>

```
Update PROJECT_NAME and INPUT fields in the configuration file.

Then run the following command to generate the documentations,
```
doxygen <config_file_name>
```

# Testing Code Coverage

# About Me
I am a second-year graduate student, majoring in Robotics at University of Maryland College Park. I completed my undergraduation in Electronics and Telecommunication from University of Pune, India in May 2013. Since then I have worked as an Application Development Analyst at Accenture India for 3 years followed by 6 months as a Robotics Engineer at Cereble Robotics, India. My areas of interest are Robotics, Machine Learning, Deep Learning and Computer Vision.
