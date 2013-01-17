/*------------------------------------------------------------------------------
 *  Title:        cse_gestures.cpp
 *  Description:  ROS node for subscribing to Kinect skeleton transforms and
                  finding gestures.
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

#include "cse_gestures.h"

/*------------------------------------------------------------------------------
 * CseGestures()
 * Constructor.
 *----------------------------------------------------------------------------*/

CseGestures::CseGestures()
{
} // end CseGestures()


/*------------------------------------------------------------------------------
 * ~CseGestures()
 * Destructor.
 *----------------------------------------------------------------------------*/

CseGestures::~CseGestures()
{
} // end ~CseGestures()


/*---------------------------------------------------------------------
* poseDataCallback()
* Callback for when a pose is found.
* -------------------------------------------------------------------*/

void CseGestures::poseDataCallback(const cse_kinect::PoseData::ConstPtr &msg)
{
  pose1 =  msg->pose1;
  pose2 =  msg->pose2;
  lVoila = msg->lVoila;
  rVoila = msg->rVoila;
  flat =   msg->flat;
} // end poseDataCallback()


/*---------------------------------------------------------------------
* main()
* Main function for ROS node.
* -------------------------------------------------------------------*/

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "cse_gestures");
  ros::NodeHandle n;

  // Declare variables.
  CseGestures *cse_gestures;

  // Set up a CseGestures object.
  cse_gestures = new CseGestures();

  ros::Subscriber pose_data_sub = n.subscribe("poseData", 1000, &CseGestures::poseDataCallback, cse_gestures);

  // Tell ROS to run this node at the desired rate.
  ros::Rate r(20);
  
  while (n.ok())
  {
    if (cse_gestures->rVoila)
    {
      ros::Time rVoilaTime = ros::Time::now();
    }
    if (cse_gestures->flat)
    {
      ros::Time flatTime = ros::Time::now();
    }
    if (cse_gestures->lVoila)
    {
      ros::Time lVoilaTime = ros::Time::now();
    }

    // Let ROS run all of its background threads to send out data now.
    ros::spinOnce();
    r.sleep();
  }

  return 0;
} // end main()
