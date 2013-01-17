/*------------------------------------------------------------------------------
 *  Title:        cse_kinect.cpp
 *  Description:  ROS node for subscribing to Kinect skeleton transforms and
 *                finding gestures.
 *----------------------------------------------------------------------------*/

/*
 *
 *    Copyright (c) 2010 <iBotics -- www.sdibotics.org>
 *    All rights reserved.
 *
 *    Redistribution and use in source and binary forms, with or without
 *    modification, are permitted provided that the following conditions are
 *    met:
 *
 *    * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer
 *    in the documentation and/or other materials provided with the
 *    distribution.
 *    * Neither the name of the Stingray, iBotics nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *    A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *    OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *    SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *    LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *    DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *    THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "cse_kinect.h"

/*------------------------------------------------------------------------------
 * CseKinect()
 * Constructor.
 *----------------------------------------------------------------------------*/

CseKinect::CseKinect()
{
} // end CseKinect()


/*------------------------------------------------------------------------------
 * ~CseKinect()
 * Destructor.
 *----------------------------------------------------------------------------*/

CseKinect::~CseKinect()
{
} // end ~CseKinect()


/*---------------------------------------------------------------------
* publishPoseData()
* Publishes poses that have been found.
* -------------------------------------------------------------------*/

void CseKinect::publishPoseData(ros::Publisher *pub_pose_data)
{
    cse_kinect::PoseData msg;
  
    // Send out the pose data.
    msg.pose1  = pose1;
    msg.pose2  = pose2;
    msg.lVoila = lVoila;
    msg.rVoila = rVoila;
    msg.flat   = flat;
    pub_pose_data->publish(msg);
  
    // Reset whether we have found the poses.
    pose1  = false;
    pose2  = false;
    lVoila = false;
    rVoila = false;
    flat   = false;
} // end publishPoseData()


/*---------------------------------------------------------------------
* lookForPoses()
* Looks for poses in Kinect data.
* -------------------------------------------------------------------*/

void CseKinect::lookForPoses()
{
    try
    {
        // Right shoulder.
        listener.lookupTransform("/openni_depth_frame", "/right_shoulder_1", ros::Time(0), tf_right_shoulder);
        x_right_shoulder = tf_right_shoulder.getOrigin().x();
        y_right_shoulder = tf_right_shoulder.getOrigin().y();
  
        // Right elbow.
        listener.lookupTransform("/openni_depth_frame", "/right_elbow_1", ros::Time(0), tf_right_elbow);
        x_right_elbow = tf_right_elbow.getOrigin().x();
        y_right_elbow = tf_right_elbow.getOrigin().y();
  
        // Right hand.
        listener.lookupTransform("/openni_depth_frame", "/right_hand_1", ros::Time(0), tf_right_hand);
        x_right_hand = tf_right_hand.getOrigin().x();
        y_right_hand = tf_right_hand.getOrigin().y();
  
        // Left shoulder.
        listener.lookupTransform("/openni_depth_frame", "/left_shoulder_1", ros::Time(0), tf_left_shoulder);
        x_left_shoulder = tf_left_shoulder.getOrigin().x();
        y_left_shoulder = tf_left_shoulder.getOrigin().y();
  
        // Left elbow.
        listener.lookupTransform("/openni_depth_frame", "/left_elbow_1", ros::Time(0), tf_left_elbow);
        x_left_elbow = tf_left_elbow.getOrigin().x();
        y_left_elbow = tf_left_elbow.getOrigin().y();
        
        // Left hand.
        listener.lookupTransform("/openni_depth_frame", "/left_hand_1", ros::Time(0), tf_left_hand);
        x_left_hand = tf_left_hand.getOrigin().x();
        y_left_hand = tf_left_hand.getOrigin().y();
  
        // Neck.
        listener.lookupTransform("/openni_depth_frame", "/neck_1", ros::Time(0), tf_neck);
        x_neck = tf_neck.getOrigin().x();
        y_neck = tf_neck.getOrigin().y();
  
        // Right hip.
        listener.lookupTransform("/openni_depth_frame", "/right_hip_1", ros::Time(0), tf_right_hip);
        x_right_hip = tf_right_hip.getOrigin().x();
        y_right_hip = tf_right_hip.getOrigin().y();
  
        // Left hip.
        listener.lookupTransform("/openni_depth_frame", "/left_hip_1", ros::Time(0), tf_left_hip);
        x_left_hip = tf_left_hip.getOrigin().x();
        y_left_hip = tf_left_hip.getOrigin().y();
  
        // Right foot.
        listener.lookupTransform("/openni_depth_frame", "/right_foot_1", ros::Time(0), tf_right_foot);
        x_right_foot = tf_right_foot.getOrigin().x();
        y_right_foot = tf_right_foot.getOrigin().y();
  
        // Left foot.
        listener.lookupTransform("/openni_depth_frame", "/left_foot_1", ros::Time(0), tf_left_foot);
        x_left_foot = tf_left_foot.getOrigin().x();
        y_left_foot = tf_left_foot.getOrigin().y();
  
        // Right knee.
        listener.lookupTransform("/openni_depth_frame", "/right_knee_1", ros::Time(0), tf_right_knee);
        x_right_knee = tf_right_knee.getOrigin().x();
        y_right_knee = tf_right_knee.getOrigin().y();
  
        // Left knee.
        listener.lookupTransform("/openni_depth_frame", "/left_knee_1", ros::Time(0), tf_left_knee);
        x_left_knee = tf_left_knee.getOrigin().x();
        y_left_knee = tf_left_knee.getOrigin().y();
  
        // Head.
        listener.lookupTransform("/openni_depth_frame", "/head_1", ros::Time(0), tf_head);
        x_head = tf_head.getOrigin().x();
        y_head = tf_head.getOrigin().y();
  
        // Start looking for poses now that we have positions of body parts.
        float a = 0.0, b = 0.0, c = 0.0, d = 0.0, z = 0.0;
        float diff = 0.0, diff2 = 0.0, diff4 = 0.0, diff5 = 0.0, diffHRH = 0.0;
        float machoAngle = 0.0, diagonal = 0.0;
        a = x_right_elbow - x_right_shoulder;
        b = y_right_elbow - y_right_shoulder;
        c = x_right_hand - x_right_elbow;
        d = y_right_hand - y_right_elbow;
        z = ((a*c + b*d) / (sqrt(a*a + b*b) * sqrt(c*c + d*d)));
        machoAngle = acos(z) * 180.0 / M_PI;
        
        diff = (x_left_elbow) - (x_left_hand);
        diff2 =   x_left_shoulder - x_left_elbow;
        diff4 =   x_right_elbow - x_right_hand;
        diff5 =   x_right_shoulder - x_right_elbow;
        diffHRH = y_head - y_right_hand;
        
        // If angle is between this and .
        if (machoAngle > 75.0 && machoAngle < 100.0 && diff < 0.1 && diff2 < 0.1 && diffHRH < 0.2)
        {
            pose1 = true;
            ROS_WARN("Macho Pose Detected!");
        }
  
        // Right upper arm.
        float y, libertyAngle = 0;
        diagonal = sqrt(((x_right_hand-x_right_shoulder) * (x_right_hand-x_right_shoulder)) + (y_right_hand*y_right_hand));
        y = (x_right_hand-x_right_shoulder) / diagonal;
        libertyAngle = acos(y) * 180.0 / M_PI;
  
        // Left lower arm.
        float x, leftAngle, leftDiagonal = 0;
        leftDiagonal = sqrt(((x_left_shoulder-x_left_hand) * (x_left_shoulder-x_left_hand)) + (y_left_hand*y_left_hand));
        x = (x_left_hand-x_left_shoulder) / leftDiagonal;
        leftAngle = acos(x) * 180.0 / M_PI;
        leftAngle = 180.0 - leftAngle;
  
        // Voila pose, right arm up.
        float voilaAngle;
        voilaAngle = libertyAngle - leftAngle;
        
        if (voilaAngle < 10.0 && y_right_hand > y_head && y_left_hand < y_head)
        {
            ROS_WARN("Right Voila Pose Detected!");    
            rVoila = true;
        }
  
        if (voilaAngle < 20.0 && y_left_hand > y_head && y_right_hand < y_head)
        {
            ROS_WARN("Left Voila Pose Detected!");
            lVoila = true;
        }
  
        // Left hip.
        if(voilaAngle < 10.0 && y_right_hand > 0.0 && y_left_hand > 0.0)
        {
          ROS_WARN("High V Pose Detected!");
        }
  
        if(voilaAngle < 10.0 && y_right_hand < y_right_hip && y_left_hand < y_left_hip)
        {
          ROS_WARN("Low V Pose Detected!");
        }
  
        float diff6 = 0.0, diff7 = 0.0, diff13 = 0.0, diff14 = 0.0;
        float flatAngle1 = 0.0, flatAngle2 = 0.0, flatAngle3 = 0.0;
        diff6  = y_right_hand - y_right_elbow;
        diff13 = y_right_elbow - y_right_shoulder;
        diff7  = y_left_shoulder - y_left_elbow;
        diff14 = y_left_elbow - y_left_hand;
  
        flatAngle1 = acos(y_head/x_right_hand) * 180.0 / M_PI;
        flatAngle2 = acos(y_head/x_left_hand) * 180.0 / M_PI;
        flatAngle3 = flatAngle1 - flatAngle2;
        if(diff6 < 0.1 && diff7 < 0.1 && diff13 < 0.1 && diff14 < 0.1 && y_right_hand < y_head  && y_left_hand < y_head)
        {
          ROS_WARN("Flat Pose Detected!");
          flat = true;
        }
    
        if (((x_right_shoulder - x_right_elbow) < .1) &&
            ((x_right_elbow - x_left_hand) < .2) &&
            ((x_left_shoulder - x_left_elbow) < .1) &&
             (x_left_elbow > x_head) && 
             (x_right_elbow > x_head) && 
             (x_right_hand > x_head) && 
             (x_left_hand > x_head))
        {
            ROS_WARN("Stop Pose Detected!");
        }
    }
  
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
    }
} // end lookForPoses()


/*---------------------------------------------------------------------
* main()
* Main function for ROS node.
* -------------------------------------------------------------------*/

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "cse_kinect");
    ros::NodeHandle n;
  
    // Set up a CseKinect object.
    CseKinect *cse_kinect = new CseKinect();
  
    // Set up parameter server variables.
    ros::NodeHandle pnh("~");
    int rate = 20;
    std::string pub_name_poses;
    pnh.param("pub_name_poses", pub_name_poses, std::string("cse_pose_data"));
    pnh.param("rate",           rate,           int(20));
    
    // Set up a publisher.
    ros::Publisher pub_pose_data = n.advertise<cse_kinect::PoseData>(pub_name_poses.c_str(), 1);
  
    // Tell ROS to run this node at the desired rate.
    ros::Rate r(rate);
  
    // Main loop.
    while (n.ok())
    {
        cse_kinect->lookForPoses();
        ros::spinOnce();
        r.sleep();
    }
  
    return 0;
} // end main()
