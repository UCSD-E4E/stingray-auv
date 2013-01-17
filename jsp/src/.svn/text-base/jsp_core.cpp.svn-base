/*------------------------------------------------------------------------------
 *  Title:        jsp_core.cpp
 *  Description:  Common class functions for simple joint state publisher.
 *----------------------------------------------------------------------------*/

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

#include "jsp_core.h"

/*------------------------------------------------------------------------------
 * JSP()
 * Constructor.
 *----------------------------------------------------------------------------*/

JSP::JSP()
{
  number_joints = 0;
  got_first_joint_msg = false;
  going_forward = true;

  x = 0.0;
  y = 0.0;
  z = 0.0;
  roll = 0.0;
  pitch = 0.0;
  yaw = 0.0;
} // end JSP()

/*------------------------------------------------------------------------------
 * ~JSP()
 * Destructor.
 *----------------------------------------------------------------------------*/

JSP::~JSP()
{
} // end ~JSP()

/*------------------------------------------------------------------------------
 * publishOdom()
 * Publish the message.
 *----------------------------------------------------------------------------*/

void JSP::publishOdom(ros::Publisher *pub_odom)
{
  nav_msgs::Odometry msg;// = odom_received;
  msg.header.stamp = ros::Time::now();
  if (msg.pose.pose.position.x > 1.0)
  {
    going_forward = false;
  }
  else if (msg.pose.pose.position.x < -1.0)
  {
    going_forward = true;
  }
  if (going_forward)
  {
    msg.pose.pose.position.x = odom_received.pose.pose.position.x + 0.0001;
  }
  else
  {
    msg.pose.pose.position.x = odom_received.pose.pose.position.x - 0.0001;
  }
  ROS_DEBUG("Sending to (%lf, %lf, %lf)",
    msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);

  msg.pose.pose.position.x = x;
  msg.pose.pose.position.y = y;
  msg.pose.pose.position.z = z;
  msg.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

  pub_odom->publish(msg);
} // end publishOdom()

/*------------------------------------------------------------------------------
 * publishJointState()
 * Publish the message.
 *----------------------------------------------------------------------------*/

void JSP::publishJointState(ros::Publisher *pub_joint_states)
{
  sensor_msgs::JointState msg = joint_state_received;
  for (size_t i = 0; i < number_joints; i++)
  {
    msg.position[i] += 0.1;
  }

  if (pub_joint_states->getNumSubscribers() > 0)
  {
    pub_joint_states->publish(msg);
  }
} // end publishJointState()

/*------------------------------------------------------------------------------
 * odomCallback()
 * Callback function for subscriber.
 *----------------------------------------------------------------------------*/

void JSP::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  ROS_DEBUG("Got odometry data with (x,y,z) = (%lf, %lf, %lf)",
  msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  // Store the message.
  odom_received = *msg;
} // end odomCallback()

/*------------------------------------------------------------------------------
 * jointStateCallback()
 * Callback function for subscriber.
 *----------------------------------------------------------------------------*/

void JSP::jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  number_joints = msg->name.size();
  joint_state_received = *msg;
  if (number_joints > 0)
  {
    got_first_joint_msg = true;
  }
} // end jointStateCallback()

/*------------------------------------------------------------------------------
 * configCallback()
 * Callback function for dynamic reconfigure server.
 *----------------------------------------------------------------------------*/

void JSP::configCallback(jsp::jsp_paramsConfig &config, uint32_t level)
{
  // Set class variables to new values. They should match what is input at the dynamic reconfigure GUI.
  x = config.x;
  y = config.y;
  z = config.z;
  roll  = config.roll;
  pitch = config.pitch;
  yaw   = config.yaw;
} // end configCallback()
