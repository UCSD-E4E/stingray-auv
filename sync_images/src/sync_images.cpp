/*------------------------------------------------------------------------------
 *  Title:        sync_images.cpp
 *  Description:  Node to synchronize images.
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

#include "sync_images_core.h"

/*------------------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *----------------------------------------------------------------------------*/

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "sync_images");
  ros::NodeHandle nh;
  std::string pub_topic_left;
  std::string pub_topic_right;
  std::string pub_info_left;
  std::string pub_info_right;
  std::string sub_topic_left;
  std::string sub_topic_right;
  std::string sub_info_left;
  std::string sub_info_right;
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("pub_topic_left", pub_topic_left, std::string("sync/left"));
  private_node_handle_.param("pub_topic_right", pub_topic_right, std::string("sync/right"));
  private_node_handle_.param("pub_info_left", pub_info_left, std::string("sync/left/camera_info"));
  private_node_handle_.param("pub_info_right", pub_info_right, std::string("sync/right/camera_info"));
  private_node_handle_.param("sub_topic_left", sub_topic_left, std::string("left/image_raw"));
  private_node_handle_.param("sub_topic_right", sub_topic_right, std::string("right/image_raw"));
  private_node_handle_.param("sub_info_left", sub_info_left, std::string("left/camera_info"));
  private_node_handle_.param("sub_info_right", sub_info_right, std::string("right/camera_info"));

  // Create a new SyncImages object.
  SyncImages *sync_images = new SyncImages();

  // Create an ImageTransport object.
  image_transport::ImageTransport it(nh);

  // Create image publishers.
  sync_images->pub_l = it.advertise(pub_topic_left, 1);
  sync_images->pub_r = it.advertise(pub_topic_right, 1);
  sync_images->pub_info_l = nh.advertise<sensor_msgs::CameraInfo>(pub_info_left, 1);
  sync_images->pub_info_r = nh.advertise<sensor_msgs::CameraInfo>(pub_info_right, 1);

  // Subscribe to camera_info messages.
  sync_images->sub_info_l = nh.subscribe(sub_info_left.c_str(), 1, &SyncImages::leftInfoCb, sync_images);
  sync_images->sub_info_r = nh.subscribe(sub_info_right.c_str(), 1, &SyncImages::rightInfoCb, sync_images);

  // Tell ROS how fast to run this node.
  ros::Rate r(100);

  image_transport::SubscriberFilter sub_l_image, sub_r_image;
  typedef ApproximateTime<Image, Image> ApproximatePolicy;  
  typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
  boost::shared_ptr<ApproximateSync> approximate_sync;

  approximate_sync.reset( new ApproximateSync(ApproximatePolicy(5),
                                              sub_l_image,
                                              sub_r_image) );

  approximate_sync->registerCallback(boost::bind(&SyncImages::imageCb, sync_images, _1, _2));

  boost::shared_ptr<image_transport::ImageTransport> it_;
  it_.reset(new image_transport::ImageTransport(nh)); 

  // Queue size 1 should be OK; the one that matters is the synchronizer queue size.
  sub_l_image.subscribe(*it_, sub_topic_left, 1);
  sub_r_image.subscribe(*it_, sub_topic_right, 1);
  
  // Spin until done.
  ros::spin();
} // end main()
