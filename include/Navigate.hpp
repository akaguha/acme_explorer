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
*  @file    Navigate.hpp
*  @author  Akash Guha(akaguha@terpmail.umd.edu)
*  @version 1.0 
*
*  @brief ENPM808X, Final Project - Localization, mapping and navigation using turtlebot
*
*  @section DESCRIPTION
*
*  This program is the header implementation of the Navigate class
*
*/
#ifndef INCLUDE_NAVIGATE_HPP_
#define INCLUDE_NAVIGATE_HPP_

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>

/**
 * @brief  Navigate class for autonomous navigation and obstacle avoidance
 */
class Navigate {
 private:
  //  Flags to check the status
  bool navCheckFlag = false;
  bool obsDetectedFlag;
  geometry_msgs::Twist velocityInput;  //  Velocity values to be published to the bot
  ros::Publisher velPub;  //  Publisher for turtlebot's velocity topic
  ros::Subscriber scanSub;  //  Subscriber for laser scanner sensor
  double velLinear;  //  linera velocity
  double velAngular;  //  angular velocity
  float minDist; //= 0.7;  //  Stores the threshold distance from obstacle
  float scanData;  //  Stores data from turtlebot's laser scanner
  float dist;
 public:
  /**
   *   @brief Default constructor for Navigate class
   *
   *   @param nothing
   *   @return nothing
   */
  Navigate(ros::NodeHandle& nh);
  /**
   *   @brief Default destructor for Navigate class
   *
   *   @param nothing
   *   @return nothing
   */
  ~Navigate();
  /**
   *   @brief Function to set navigation status flag
   *
   *   @param nothing
   *   @return nothing
   */
  void setnavCheckFlag();
  /**
   *   @brief Function to check navigation node status
   *
   *   @param nothing
   *   @return true or false
   */
  bool getnavCheckFlag();
  /**
   *   @brief Function to set obstacle detected flag
   *
   *   @param nothing
   *   @return nothing
   */
  void setobsDetectedFlag();
  /**
   *   @brief Function to check obstacle detected flag
   *
   *   @param nothing
   *   @return true or false
   */
  bool getobsDetectedFlag();
  /**
   *   @brief Function to explore the unknown environment
   *
   *   @param nothing
   *   @return void
   */
  void explore();
  /**
   *   @brief Function to command the bot to move forward
   *
   *   @param nothing
   *   @return void
   */
  void moveForward();
  /**
   *   @brief Function to command the bot to keep rotating
   *
   *   @param nothing
   *   @return void
   */
  void turn();
  /**
   *   @brief Callback function that will get called when a new laser 
   *   scan data is available 
   *
   *   @param Pointer to the laser scan data received on sensor_msgs topic
   *   @return void
   */
  void scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan);
  /**
   *   @brief  Function to set or clear the obstacle flag
   *
   *   @param  none
   *   @return void
   */
  bool obstacleDetected();

};

#endif // INCLUDE_NAVIGATE_HPP_