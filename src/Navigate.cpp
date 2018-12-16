/*********************************************************************************
*  BSD 3-Clause License
*
*  Copyright (c) 2018, Akash Guha
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions are met:
*
*1. Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
*2. Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
*3. Neither the name of the copyright holder nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
*  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
*  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
*  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
*  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
*  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
*  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
**********************************************************************************
*
*  @file    Navigate.cpp
*  @author  Akash Guha(akaguha@terpmail.umd.edu)
*  @version 1.0 
*
*  @brief ENPM808X, Final Project - Localization, mapping and navigation using turtlebot
*
*  @section DESCRIPTION
*
*  This is the cpp file showing the implementation of all the methods in the 
*  Navigate class
*
*/

#include "Navigate.hpp"

Navigate::Navigate(ros::NodeHandle& nh){
  ROS_INFO("Initializing the navigate object");
  exploreFlag = false;
  velLinear = 0.1;
  velAngular = 0.5;
  obsDetectedFlag = false;
  scanData = 0.0;
  minDist = 0.7;
  //  Setting up a velocity publisher with the master
  velPub = nh.advertise <geometry_msgs::Twist>
  ("/mobile_base/commands/velocity", 1000);
  //  Setting up a laser scanner subscriber with the master
  scanSub = nh.subscribe <sensor_msgs::LaserScan>
  ("/scan", 100, &Navigate::scanCallBack, this);
  //  Timers allow you to get a callback at a specified rate.
  timer1 = nh.createTimer(ros::Duration(75), &Navigate::turnCallback, this);
  //  Initializing the robot with zero velocity
  velocityInput.linear.x = 0.0;
  velocityInput.angular.z = 0.0;
  velPub.publish(velocityInput);
  setnavCheckFlag();
}

Navigate::~Navigate(){
  //  Stopping the robot on node closure
  velocityInput.linear.x = 0.0;
  velocityInput.angular.z = 0.0;
  velPub.publish(velocityInput);
}

void Navigate::scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan){
  dist = minDist;  //  Initialize dist with threshold distance value
  //int sz = scan->ranges.size();  //  Stores the size of laser scan array
  for (const auto& scanData: scan->ranges){
    //scanData = scan->ranges[i];  //  Stores the laser scan data
    //  Based on the received data assigning value to dist variable
    if (scanData < dist) {
      dist = scanData;
      ROS_WARN_STREAM("Distance " << dist << " less than the threshold value");
    }
  }	
}

void Navigate::setnavCheckFlag(){
  navCheckFlag = true;
}

bool Navigate::getnavCheckFlag(){
  if (navCheckFlag == true) {
    return true;
  } else {
  	return false;
  }
}

void Navigate::setobsDetectedFlag(){
  obsDetectedFlag = true;
}

bool Navigate::getobsDetectedFlag(){
  if (obsDetectedFlag == true) {
    return true;
  } else {
  	return false;
  }
}

void Navigate::explore(){
  scanData = 0.0;
  dist = minDist;  //  Initialize dist with threshold distance value  
  exploreFlag = true;  //  initially bot turns for 50 iterations to scout the world
  count = 0;
  ros::Rate loop_rate(5);  //  Setting the looping rate
  while (ros::ok()) {
    if(exploreFlag) {
      ROS_INFO_STREAM("Exploring");
      turn();
    } else if (obstacleDetected()) {
      ROS_INFO_STREAM("Obstacle ahead, turning");
      turn();  //  Calling the turn method to avoid obstacle
    } else {
      //  Calling the moving forward method in absence of an obstacle
        moveForward();
      }
      //  Using publish method to send velocity values to turtlebot
    velPub.publish(velocityInput);
    ros::spinOnce();
    loop_rate.sleep();
    count++;  //  increments the counter on every iteration
    if (count == 50) {
      count = 0;
      exploreFlag = false;  //  
    }
  }

}

bool Navigate::obstacleDetected() {
  if (dist < minDist) {
    setobsDetectedFlag();
    return true;
  } else {
      obsDetectedFlag = false;
      return false;
    }
}

void Navigate::moveForward(){
  ROS_INFO_STREAM("Path is clear to go forward");
  //  Setting a linear velocity and making angular velocity zero
  velocityInput.linear.x = velLinear;
  velocityInput.angular.z = 0.0;
}

void Navigate::turn(){
  //  Setting an angular velocity and making linear velocity zero
  velocityInput.linear.x = 0.0;
  velocityInput.angular.z = velAngular;
}

geometry_msgs::Twist Navigate::getVelocity() {
  return velocityInput;
}

void Navigate::turnCallback(const ros::TimerEvent&) {
  exploreFlag = true;  //  Setting the explore flag after every 75sec 
  count = 0;  //  resetting the counter
  ROS_INFO("turnCallback triggered");
}