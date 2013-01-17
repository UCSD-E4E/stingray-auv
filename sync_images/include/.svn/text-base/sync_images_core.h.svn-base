/**
 *  \file sync_images_core.h
 *  \brief Common class functions for synchronizing images.
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

#ifndef SYNC_IMAGES_CORE_H
#define SYNC_IMAGES_CORE_H

// ROS includes.
#include <ros/ros.h>

#include <cv_bridge/CvBridge.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <stereo_image_proc/processor.h>
#include <image_geometry/stereo_camera_model.h>

using namespace sensor_msgs;
using namespace stereo_msgs;
using namespace message_filters::sync_policies;

class SyncImages
{
public:
  //! Constructor.
  SyncImages();

  //! Destructor.
  ~SyncImages();

  //! Image callback.
  void imageCb(const sensor_msgs::ImageConstPtr& imgl, const sensor_msgs::ImageConstPtr& imgr);

  //! Left info callback.
  void leftInfoCb(const sensor_msgs::CameraInfoConstPtr& info_left);

  //! Right info callback.
  void rightInfoCb(const sensor_msgs::CameraInfoConstPtr& info_right);

  //! Image subscriber for left image.
  image_transport::CameraSubscriber sub_l;
  //! Image subscriber for right image.
  image_transport::CameraSubscriber sub_r;
  //! Image publisher for left image.
  image_transport::Publisher pub_l;
  //! Image publisher for right image.
  image_transport::Publisher pub_r;
  ros::Publisher pub_info_l;
  ros::Publisher pub_info_r;
  ros::Subscriber sub_info_l;
  ros::Subscriber sub_info_r;
  message_filters::Subscriber<CameraInfo> sub_l_info;
  message_filters::Subscriber<CameraInfo> sub_r_info;
  sensor_msgs::CameraInfo info_l;
  sensor_msgs::CameraInfo info_r;

private:
  bool have_first_left_info;
  bool have_first_right_info;
  bool have_first_both_info;
};

#endif // SYNC_IMAGES_CORE_H
