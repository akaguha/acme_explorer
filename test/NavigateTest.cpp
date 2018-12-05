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

#include <ros/ros.h>
#include <gtest/gtest.h>
#include "../include/Navigate.hpp"

/**
*  @brief Testing if the node is initialized correctly 
*/
TEST(NavTest, navNodeCheck) {
  //  Create
  Navigate nav;  //  Create a navigate class object
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
  Navigate nav;
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
  Navigate nav;
  //  Act
  nav.moveForward();
  //  Assert
  EXPECT_EQ(, true);
}

/**
*  @brief Test to check if teh angular velocity is set correctly
*/
TEST(NavTest, turnTest) {
  //  Create
  Navigate nav;
  //  Act
  nav.turn();
  //  Assert
  EXPECT_EQ(, true);
}

/**
*  @brief Test to check the explore behavior of the robot
*/
TEST(NavTest, exploreTest) {
  //  Create
  Navigate nav;
  //  Act
  //  Assert

}