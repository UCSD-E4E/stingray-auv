/**
 *  \file jsp_core.h
 *  \brief Common class functions for simple joint state publisher.
 */

/*
 *
 *      Copyright (c) 2010 <iBotics -- www.sdibotics.org>
 *      All rights reserved.
 *
 *      Redistribution and use in source and binary forms, with or without
 *      modification, are permitted provided that the following conditions are
 *      met:
 *
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above
 *        copyright notice, this list of conditions and the following disclaimer
 *        in the documentation and/or other materials provided with the
 *        distribution.
 *      * Neither the name of the Stingray, iBotics nor the names of its
 *        contributors may be used to endorse or promote products derived from
 *        this software without specific prior written permission.
 *
 *      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *      "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *      LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *      A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *      OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *      SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *      LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *      DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *      THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *      (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *      OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef JSP_CORE_H
#define JSP_CORE_H

// ROS includes.
#include <ros/ros.h>
#include <ros/time.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include "tf/transform_datatypes.h"

// Dynamic reconfigure includes.
#include <dynamic_reconfigure/server.h>
// Auto-generated from cfg/ directory.
#include <jsp/jsp_paramsConfig.h>

using std::string;

class JSP
{
public:
  //! Constructor.
  JSP();

  //! Destructor.
  ~JSP();

  //! Callback function for dynamic reconfigure server.
  void configCallback(jsp::jsp_paramsConfig &config, uint32_t level);

  //! Publish the message.
  void publishOdom(ros::Publisher *pub_odom);

  //! Publish the message.
  void publishJointState(ros::Publisher *pub_joint_states);

  //! Callback function for subscriber.
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg);

  //! Callback function for subscriber.
  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);

  //! Store the latest JointState message.
  sensor_msgs::JointState joint_state_received;

  //! The number of joints determined by looking at the most
  //! recent received JointState message.
  size_t number_joints;

  //! Flag indicating whether we have received a JointState message.
  bool got_first_joint_msg;

  //! Store the latest Odometry message.
  nav_msgs::Odometry odom_received;

  //! Keep track of motion direction.
  bool going_forward;

  // States to send.
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
};

#endif // JSP_CORE_H
