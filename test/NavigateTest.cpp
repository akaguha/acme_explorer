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
*  @file    NavigateTest.cpp
*  @author  Akash Guha(akaguha@terpmail.umd.edu)
*  @version 1.0 
*
*  @brief ENPM808X, Final Project - Localization, mapping and navigation using turtlebot
*
*  @section DESCRIPTION
*
*  This is a test program to check the working of Navigate class.
*
*/

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>
#include <gtest/gtest.h>
#include "Navigate.hpp"


/**
*  @brief Testing if the node is initialized correctly 
*/
TEST(NavTest, navNodeCheck) {
  //  Create
  ros::NodeHandle nH;  //  Create a node handle
  Navigate nav(nH);  //  Create a navigate class object
  //  Act
  bool returnVal = nav.getnavCheckFlag();
  //  Assert
  EXPECT_EQ(returnVal, true);
}

/**
*  @brief Test for obstacle detected flag
*/
TEST(NavTest, obsDetectedTest) {
  //  Create
  ros::NodeHandle nH;  //  Create a node handle
  Navigate nav(nH);
  nav.setobsDetectedFlag();
  //  Act
  bool returnVal = nav.getnavCheckFlag();
  //  Assert
  EXPECT_EQ(returnVal, true);
}

/**
*  @brief Test to check if linear velocity is set correctly
*/
TEST(NavTest, moveForwardTest) {
  //  Create
  ros::NodeHandle nH;  //  Create a node handle
  Navigate nav(nH);
  geometry_msgs::Twist velocity;
  //  Act
  nav.moveForward();
  velocity = nav.getVelocity();
  //  Assert
  EXPECT_EQ(velocity.linear.x, 0.1);
}

/**
*  @brief Test to check if the angular velocity is set correctly
*/
TEST(NavTest, turnTest) {
  //  Create
  ros::NodeHandle nH;  //  Create a node handle
  Navigate nav(nH);
  geometry_msgs::Twist velocity;
  //  Act
  nav.turn();
  velocity = nav.getVelocity();
  //  Assert
  EXPECT_EQ(velocity.angular.z, 0.5);
}

// /**
// *  @brief Test to check the turn behavior of the robot
// */
// TEST(NavTest, exploreTestTurn) {
//   //  Create
//   ros::NodeHandle nH;  //  Create a node handle
//   Navigate nav(nH);
//   double minDist = 0.7;
//   double dist = 0.5;
//   geometry_msgs::Twist velocity;
//   //  Act
//   nav.explore();  // turn condition satisfied
//   velocity = nav.getVelocity();  
//   //  Assert
//   EXPECT_EQ(velocity.angular.z, 0.5);
// }

// *
// *  @brief Test to check the move forward behavior of the robot

// TEST(NavTest, exploreTestForward) {
//   //  Create
//   ros::NodeHandle nH;  //  Create a node handle
//   Navigate nav(nH);
//   double minDist = 0.7;
//   double dist = 0.8;
//   geometry_msgs::Twist velocity;
//   //  Act
//   nav.explore();  // turn condition satisfied
//   velocity = nav.getVelocity();  
//   //  Assert
//   EXPECT_EQ(velocity.linear.x, 0.1);
// }

/**
*  @brief Test to check the obstacleDetected function
*/
TEST(NavTest, obstDetectedTest) {
  //  Create
  ros::NodeHandle nH;  //  Create a node handle
  Navigate nav(nH);
  nav.setobsDetectedFlag();  //  flag set to true
  //  Act
  bool returnVal = nav.obstacleDetected();
  //  Assert
  EXPECT_EQ(returnVal, true);
}

/**
 * @brief  main function
 *
 * @param  count of arguments as argc
 *         argument vector as argv
 *
 * @return status of the program as int, returns 0 if all tests pass
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "NavTest");  //  Initialize the node
  //ros::NodeHandle nh;  //  Create a node handle
  testing::InitGoogleTest(&argc, argv);
  //  Run all the declared tests with TEST()
  return RUN_ALL_TESTS();
}