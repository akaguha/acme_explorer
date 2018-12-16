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
*  @file    ViewImage.cpp
*  @author  Akash Guha(akaguha@terpmail.umd.edu)
*  @version 1.0 
*
*  @brief ENPM808X, Final Project - Localization, mapping and navigation using turtlebot
*
*  @section DESCRIPTION
*
*  This is the cpp file showing the implementation of all the methods in the 
*  ViewImage class
*
*/

//  header to publish and subscribe images
#include <image_transport/image_transport.h>
//  header to display images using OpenCV's GUI 
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <ros/ros.h>
#include <string>
#include "acme_explorer/Snap.h"
#include "ViewImage.hpp"

ViewImage::ViewImage(){
  ROS_INFO("Initializing the ViewImage object");
  imgReceivedFlag = false;
  picSavedFlag = false;
  //  create an ImageTransport instance, initializing it with the NodeHandle
  image_transport::ImageTransport it(viewNh);
  //  subscribe to the "camera/rgb/image_raw" topic
  imgSub = it.subscribe("camera/rgb/image_raw", 10, &ViewImage::imageCallback, this);
  //  register service with the master
  service = viewNh.advertiseService("Snap", &ViewImage::takePic, this);
  setcamCheckFlag();
}

ViewImage::~ViewImage(){
//  cv::destroyWindow("view");  //  Dispose the display window
}

bool ViewImage::getcamCheckFlag(){
  if (camCheckFlag == true) {
  	return true;
  } else {
  	false;
  }
}

void ViewImage::setcamCheckFlag(){
  camCheckFlag = true;
}

bool ViewImage::getimgReceivedFlag(){
  if (imgReceivedFlag == true) {
  	return true;
  } else {
  	false;
  }
}

void ViewImage::setimgReceivedFlag(){
  imgReceivedFlag = true;
}

bool ViewImage::getpicSavedFlag(){
  if (picSavedFlag == true) {
  	return true;
  } else {
  	false;
  }
}

void ViewImage::setpicSavedFlag(){
  picSavedFlag = true;
}

void ViewImage::imageCallback(const sensor_msgs::ImageConstPtr& img){
  try
  {
    //  convert the ROS image message to an OpenCV image with BGR pixel encoding, then show it in a display window
    cv::imshow("view", cv_bridge::toCvShare(img, "bgr8")->image);
    cv::waitKey(30);
    imgReceivedFlag = true;  //  Sets the image received flag
    imagePtr = cv_bridge::toCvCopy(img, "bgr8");  //  Pointer to the image to be stored
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", img->encoding.c_str());
  }

}

bool ViewImage::takePic(acme_explorer::Snap::Request& req,
                        acme_explorer::Snap::Response& resp){
  imgTitle = req.fileTitle + ".jpg";  //  file name received from service request
  ROS_INFO("Taking a snapshot");
  if(imgReceivedFlag) {  //  executes only if the image is getting received
    cv::imwrite(imgTitle, imagePtr->image); // saves the image
    setpicSavedFlag();  //  Sets the picture saved flag
    resp.respFlag = true;  //  respondes with a boolean true
    return true;
  }
}

void ViewImage::viewImg() {
  //  openCV display window
  cv::namedWindow("view");
  cv::startWindowThread();
  ros::spin();
}
