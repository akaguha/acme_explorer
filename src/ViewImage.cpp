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

#include "ViewImage.hpp"

/**
*   @brief Default constructor for ViewImage class
*
*   @param nothing
*   @return nothing
*/
ViewImage::ViewImage(){
  ROS_INFO("Initializing the ViewImage object");

  setcamCheckFlag();
}

/**
*   @brief Default destructor for ViewImage class
*
*   @param nothing
*   @return nothing
*/
ViewImage::~ViewImage(){

}

/**
*   @brief Function to check camera status
*
*   @param nothing
*   @return true or false
*/
bool ViewImage::getcamCheckFlag(){
  if (camCheckFlag == true) {
  	return true;
  } else {
  	false;
  }
}

/**
*   @brief Function to set camera status flag
*
*   @param nothing
*   @return nothing
*/
void ViewImage::setcamCheckFlag(){
  camCheckFlag = true;
}

/**
*   @brief Function to check status of received image
*
*   @param nothing
*   @return true or false
*/
bool ViewImage::getimgReceivedFlag(){
  if (imgReceivedFlag == true) {
  	return true;
  } else {
  	false;
  }
}

/**
*   @brief Function to set received image status
*
*   @param nothing
*   @return nothing
*/
void ViewImage::setimgReceivedFlag(){
  imgReceivedFlag = true;
}

/**
*   @brief Function to check status of saved picture
*
*   @param nothing
*   @return true or false
*/
bool ViewImage::getpicSavedFlag(){
  if (picSavedFlag == true) {
  	return true;
  } else {
  	false;
  }
}

/**
*   @brief Function to set saved picture status
*
*   @param nothing
*   @return nothing
*/
void ViewImage::setpicSavedFlag(){
  picSavedFlag = true;
}

/**
*   @brief Callback function that will get called when a new 
*   image has arrived on the "camera/image" topic
*
*   @param Pointer to the image data received on sensor_msgs topic
*   @return void
*/
void ViewImage::imageCallback(const sensor_msgs::ImageConstPtr& img){

}

/**
*   @brief Function to click a picture when the click service is called 
*
*   @param file name as string
*   @return void
*/
void ViewImage::takePic(std::string imgTitle){
	
}