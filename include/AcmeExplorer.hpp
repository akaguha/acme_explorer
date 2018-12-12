/**********************************************************************************
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
***********************************************************************************
*
*  @file    AcmeExplorer.hpp
*  @author  Akash Guha(akaguha@terpmail.umd.edu)
*  @version 1.0 
*
*  @brief ENPM808X, Final Project - Localization, mapping and navigation using turtlebot
*
*  @section DESCRIPTION
*
*  This program is the header implementation of the AcmeExplorer class
*
*/
#ifndef INCLUDE_ACMEEXPLORER_HPP_
#define INCLUDE_ACMEEXPLORER_HPP_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include "Navigate.hpp"
#include "ViewImage.hpp"

/**
 * @brief  Class for AcmeExplorer
 */
class AcmeExplorer {
 private:
  bool botCheckFlag = false;  //  Flag to check the robot status
  ros::NodeHandle nH;  //  Create a node handle
  // Navigate nav;  //  Create Navigate class object
  // ViewImage viewImg; //  Create ViewImage class object

 public:
  /**
   *   @brief Default constructor for AcmeExplorer class
   *
   *   @param nothing
   *   @return nothing
   */
  AcmeExplorer();
  /**
   *   @brief Default destructor for AcmeExplorer class
   *
   *   @param nothing
   *   @return nothing
   */
  ~AcmeExplorer();
  /**
   *   @brief Function to get the robot's status
   *
   *   @param nothing
   *   @return true or flase based on the bot's status
   */
  bool getbotCheckFlag();
  /**
   *   @brief Function to set the status flag
   *
   *   @param nothing
   *   @return nothing
   */
  void setbotCheckFlag();
};

#endif // INCLUDE_ACMEEXPLORER_HPP_