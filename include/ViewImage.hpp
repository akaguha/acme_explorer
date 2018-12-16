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
*  @file    ViewImage.hpp
*  @author  Akash Guha(akaguha@terpmail.umd.edu)
*  @version 1.0 
*
*  @brief ENPM808X, Final Project - Localization, mapping and navigation using turtlebot
*
*  @section DESCRIPTION
*
*  This program is the header implementation of the ViewImage class
*
*/
#ifndef INCLUDE_VIEWIMAGE_HPP_
#define INCLUDE_VIEWIMAGE_HPP_

//  header to publish and subscribe images
#include <image_transport/image_transport.h>
//  header to display images using OpenCV's GUI 
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <ros/ros.h>
#include <string>
#include "acme_explorer/Snap.h"

/**
 * @brief  ViewImage class to stream video feed and click pictures
 */
class ViewImage {
 private:
  //  Flags to check the status
  bool camCheckFlag = false;
  bool imgReceivedFlag;
  bool picSavedFlag; 
  image_transport::Subscriber imgSub;  //  Subscriber to the image topic to get pictures
  std::string imgTitle;  //  Save picture with this file name
  ros::ServiceServer service;  //  Create a service with master
  cv_bridge::CvImagePtr imagePtr;  //  Pointer to store the image
  ros::NodeHandle viewNh;
 public:
  /**
   *   @brief Default constructor for ViewImage class
   *
   *   @param node handle as nh
   *   @return nothing
   */
  ViewImage();
  /**
   *   @brief Default destructor for ViewImage class
   *
   *   @param nothing
   *   @return nothing
   */
  ~ViewImage();
  /**
   *   @brief Function to check camera status
   *
   *   @param nothing
   *   @return true or false
   */
  bool getcamCheckFlag();
  /**
   *   @brief Function to set camera status flag
   *
   *   @param nothing
   *   @return nothing
   */
  void setcamCheckFlag();
  /**
   *   @brief Function to check status of received image
   *
   *   @param nothing
   *   @return true or false
   */
  bool getimgReceivedFlag();
  /**
   *   @brief Function to set received image status
   *
   *   @param nothing
   *   @return nothing
   */
  void setimgReceivedFlag();
  /**
   *   @brief Function to check status of saved picture
   *
   *   @param nothing
   *   @return true or false
   */
  bool getpicSavedFlag();
  /**
   *   @brief Function to set saved picture status
   *
   *   @param nothing
   *   @return nothing
   */
  void setpicSavedFlag();
  /**
   *   @brief Callback function that will get called when a new 
   *   image has arrived on the "camera/image" topic
   *
   *   @param Pointer to the image data received on sensor_msgs topic
   *   @return void
   */
  void imageCallback(const sensor_msgs::ImageConstPtr& img);
  /**
   *   @brief Function to click a picture when the Snap service is called 
   *
   *   @param 
   *   @return void
   */
  bool takePic(acme_explorer::Snap::Request &req,
               acme_explorer::Snap::Response &resp);
  /**
   *   @brief Function for video  surveillance
   *
   *   @param nothing
   *   @return void
   */
  void viewImg();

};

#endif // INCLUDE_VIEWIMAGE_HPP_