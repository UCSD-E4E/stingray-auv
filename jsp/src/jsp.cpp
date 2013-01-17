/*------------------------------------------------------------------------------
 *  Title:        jsp.cpp
 *  Description:  ROS node for simple joint state publisher.
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
 * main()
 * Main function to set up ROS node.
 *----------------------------------------------------------------------------*/

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "jsp");
  ros::NodeHandle n;

  // Create a new NodeExample object.
  JSP *jsp = new JSP();

  // Set up a dynamic reconfigure server before reading parameter server values.
  // The callback gets called and dynamic reconfigure defaults will overwrite parameter server values.
  dynamic_reconfigure::Server<jsp::jsp_paramsConfig> reconfig_srv;
  dynamic_reconfigure::Server<jsp::jsp_paramsConfig>::CallbackType f;
  f = boost::bind(&JSP::configCallback, jsp, _1, _2);
  reconfig_srv.setCallback(f);

  // Declare variables that can be modified by launch file or command line.
  int rate;
  string sub_topic_joint_state;
  string sub_topic_odom;
  string pub_topic_joint_state;
  string pub_topic_odom;

  // Initialize node parameters from launch file or command line.
  // Use a private node handle so that multiple instances of the node can be run simultaneously
  // while using different parameters.
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("rate", rate, int(40));
  private_node_handle_.param("sub_topic_joint_state", sub_topic_joint_state, string("uwsim/joint_state"));
  private_node_handle_.param("sub_topic_odom", sub_topic_odom, string("uwsim/girona500_odom"));
  private_node_handle_.param("pub_topic_joint_state", pub_topic_joint_state, string("uwsim/joint_state_command"));
  private_node_handle_.param("pub_topic_odom", pub_topic_odom, string("uwsim/dataNavigator"));

  // Create subscribers.
  ros::Subscriber sub_joint_states = n.subscribe(sub_topic_joint_state.c_str(), 1, &JSP::jointStateCallback, jsp);
  ros::Subscriber sub_odom = n.subscribe(sub_topic_odom.c_str(), 1, &JSP::odomCallback, jsp);

  // Create publishers.
  ros::Publisher pub_joint_states = n.advertise<sensor_msgs::JointState>(pub_topic_joint_state.c_str(), 1);
  ros::Publisher pub_odom = n.advertise<nav_msgs::Odometry>(pub_topic_odom.c_str(), 1);
  
  // Tell ROS how fast to run this node.
  ros::Rate r(rate);

  // Main loop.
  while (n.ok())
  {
    // Make sure we don't try to publish before knowing joints available.
    if (jsp->got_first_joint_msg)
    {
      jsp->publishJointState(&pub_joint_states);
      jsp->publishOdom(&pub_odom);
    }

    ros::spinOnce();
    r.sleep();
  }

  return 0;
} // end main()
