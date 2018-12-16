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
*  @file    ViewImageTest.cpp
*  @author  Akash Guha(akaguha@terpmail.umd.edu)
*  @version 1.0 
*
*  @brief ENPM808X, Final Project - Localization, mapping and navigation using turtlebot
*
*  @section DESCRIPTION
*
*  This is a test program to check the working of ViewImage class.
*
*/

//  header to publish and subscribe images
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <string>
#include "acme_explorer/Snap.h"
//  header to display images using OpenCV's GUI
#include <opencv2/highgui/highgui.hpp>
#include "ViewImage.hpp"

/**
*  @brief Testing if the node is initialized correctly 
*/
TEST(ImgTest, camCheck) {
  //  Create
  ViewImage imgView;  //  Create a ViewImage object
  //  Act
  bool returnVal = imgView.getcamCheckFlag();
  //  Assert
  EXPECT_EQ(returnVal, true);
}


/**
*  @brief Test for image received flag 
*/
TEST(ImgTest, imgReceivedCheck) {
  //  Create
  ViewImage imgView;
  //  Act
  imgView.setimgReceivedFlag();
  bool returnVal = imgView.getimgReceivedFlag();
  //  Assert
  EXPECT_EQ(returnVal, true);
}

/**
*  @brief Test for picture saved flag 
*/
TEST(ImgTest, picSavedCheck) {
  //  Create
  ViewImage imgView;
  //  Act
  imgView.setpicSavedFlag();
  bool returnVal = imgView.getpicSavedFlag();
  //  Assert
  EXPECT_EQ(returnVal, true);
}

// /**
// *  @brief Test to check if the picture was saved successfully
// */
// TEST(ImgTest, picSaveCheck) {
//   //  Create
//   ros::NodeHandle nH;  //  Create a node handle
//   ViewImage imgView(nH);
//   std::string req = "TestImg";
//   bool resp = true;
//   //  Act
//   imgView.setimgReceivedFlag();
//   bool returnVal = imgView.takePic(&req,&resp);
//   //  Assert
//   EXPECT_EQ(returnVal, true);
// }

/**
 * @brief  main function
 *
 * @param  count of arguments as argc
 *         argument vector as argv
 *
 * @return status of the program as int, returns 0 is all tests pass
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "ViewImageTest");  //  Initialize the node
  testing::InitGoogleTest(&argc, argv);
  //  Run all the declared tests with TEST()
  return RUN_ALL_TESTS();
}
