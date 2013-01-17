/**
 *  \file cse_kinect.h
 *  \brief ROS node for subscribing to Kinect skeleton transforms and finding gestures.
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

#ifndef CSEKINECT_H
#define CSEKINECT_H

// System libraries.
#include <cstdlib>

// ROS includes.
#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

// Local includes.
#include "cse_kinect/PoseData.h"

class CseKinect
{
public:
    //! Constructor.
    CseKinect();
  
    //! Destructor.
    ~CseKinect();
  
    void publishPoseData(ros::Publisher *pubPoseData);
  
    void lookForPoses();

private:
    //! A listener for the transform topic.
    tf::TransformListener listener;
  
    //! Transforms for each body part.
    tf::StampedTransform tf_right_shoulder;
    tf::StampedTransform tf_right_elbow;
    tf::StampedTransform tf_right_hand;
    tf::StampedTransform tf_left_shoulder;
    tf::StampedTransform tf_left_elbow;
    tf::StampedTransform tf_left_hand;
    tf::StampedTransform tf_neck;
    tf::StampedTransform tf_right_hip;
    tf::StampedTransform tf_left_hip;
    tf::StampedTransform tf_right_foot;
    tf::StampedTransform tf_left_foot;
    tf::StampedTransform tf_right_knee;
    tf::StampedTransform tf_left_knee;
    tf::StampedTransform tf_head;    
  
    bool pose1;
    bool pose2;
    bool lVoila;
    bool rVoila;
    bool flat;
  
    double x_right_shoulder;
    double y_right_shoulder;
    double x_right_elbow;
    double y_right_elbow;
    double x_right_hand;
    double y_right_hand;
    double x_left_shoulder;
    double y_left_shoulder;
    double x_left_elbow;
    double y_left_elbow;
    double x_left_hand;
    double y_left_hand;
    double x_neck;
    double y_neck;
    double x_right_hip;
    double y_right_hip;
    double x_left_hip;
    double y_left_hip;
    double x_right_foot;
    double y_right_foot;
    double x_left_foot;
    double y_left_foot;
    double x_right_knee;
    double y_right_knee;
    double x_left_knee;
    double y_left_knee;
    double x_head;
    double y_head;
};

#endif // CSEKINECT_H
