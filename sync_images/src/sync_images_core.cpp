/*------------------------------------------------------------------------------
 *  Title:        sync_images_core.cpp
 *  Description:  Common class functions for synchronizing images.
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
 * SyncImages()
 * Constructor.
 *----------------------------------------------------------------------------*/

SyncImages::SyncImages()
{
  have_first_left_info = false;
  have_first_right_info = false;
  have_first_both_info = false;
} // end SyncImages()

/*------------------------------------------------------------------------------
 * ~SyncImages()
 * Destructor.
 *----------------------------------------------------------------------------*/

SyncImages::~SyncImages()
{
} // end ~SyncImages()


/*------------------------------------------------------------------------------
 * imageCb()
 * Callback for received images.
 *----------------------------------------------------------------------------*/

void SyncImages::imageCb(const sensor_msgs::ImageConstPtr& imgl, const sensor_msgs::ImageConstPtr& imgr)
{
  // Do not publish anything until we have both camera_info messages.
  // Note: this function does not get called unless both images are present already.
  if (have_first_both_info)
  {
    // Create new right image so we can modify the header.
    sensor_msgs::Image sync_img_r = *imgr;

    // Synchronize all the headers to match the left image timestamp.
    sync_img_r.header.stamp = imgl->header.stamp;
    info_l.header.stamp = imgl->header.stamp;
    info_r.header.stamp = imgl->header.stamp;

    // Publish all of the images on the new topic with exactly synchronized headers.
    pub_l.publish(imgl);
    pub_r.publish(sync_img_r);
    pub_info_l.publish(info_l);
    pub_info_r.publish(info_r);
  }
} // end imageCb()


/*------------------------------------------------------------------------------
 * leftInfoCb()
 * Callback for received left camera_info.
 *----------------------------------------------------------------------------*/

void SyncImages::leftInfoCb(const sensor_msgs::CameraInfoConstPtr& info_left)
{
  have_first_left_info = true;
  if (have_first_right_info)
  {
    have_first_both_info = true;
    // Save the camera info so we can modify it. Originally const so we can't touch it without compiler errors.
    info_l = *info_left;
  }
} // end leftInfoCb()


/*------------------------------------------------------------------------------
 * rightInfoCb()
 * Callback for received right camera_info.
 *----------------------------------------------------------------------------*/

void SyncImages::rightInfoCb(const sensor_msgs::CameraInfoConstPtr& info_right)
{
  have_first_right_info = true;
  if (have_first_left_info)
  {
    have_first_both_info = true;
    // Save the camera info so we can modify it. Originally const so we can't touch it without compiler errors.
    info_r = *info_right;
  }
} // end rightInfoCb()
