/**
 *  \file vision_port.h
 *  \brief ROS node for subscribing to images.
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

#ifndef VISIONPORT_H
#define VISIONPORT_H

// System libraries.
#include <cstdlib>

// ROS.
#include <ros/ros.h>
#include <ros/time.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <actionlib/server/simple_action_server.h>

// Custom messages.
#include "vision_port/visionAction.h"

// Dynamic reconfigure.
#include <dynamic_reconfigure/server.h>
#include <vision_port/visionPortParamsConfig.h>

// Define for task solution techniques
// 0 = DEFAULT (From AUVSI 2009)
// 1 = Simple Boost off HSV
#define GATE_TECHNIQUE 1
#define BUOY_TECHNIQUE 1
#define PIPE_TECHNIQUE 1

#ifndef VISION_BUOY_COLOR
#define VISION_BUOY_RED 1
#define VISION_BUOY_YELLOW 2
#define VISION_BUOY_GREEN 3
#endif // VISION_BUOY_COLOR

class VisionPort
{
public:
    //! Constructor.
    VisionPort();

    //! Destructor.
    ~VisionPort();

    //! Callback for new images.
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

    //! Callback function for dynamic reconfigure server.
    void configCallback(vision_port::visionPortParamsConfig &config, uint32_t level);

    int vision_find_buoy(int *dotx, int *doty, int color);

    int vision_boost_buoy(IplImage *src_img, IplImage *bin_img, CvPoint *center, int color);

    int vision_find_hedge(int *dotx, int *doty);

    int vision_boost_hedge(IplImage *src_img, IplImage *bin_img, CvPoint *center);

    int vision_find_pipe(CvPoint center[2], double bearing[4]);

    int vision_boost_pipe(IplImage *src_img, IplImage *bin_img, CvPoint center[2], double bearing[4]);

    CvPoint vision_find_centroid(IplImage *bin_img_ptr, int thresh);

    double vision_get_bearing(IplImage *input_bin_img);

    //! Image subscriber.
    image_transport::Subscriber sub;

	//! If boost is wanted
    int boost_wanted;

    //! Image to view.
    int image_to_view;

    //! HSV (Hue, Saturation, Value) thresholds.
    double h_low;
    double h_high;
    double s_low;
    double s_high;
    double v_low;
    double v_high;

    IplImage img_in;
    IplImage bin_img;
    IplImage img_hsv;

    IplImage *img_in_ptr;
    IplImage *bin_img_ptr;
    IplImage *img_hsv_ptr;
    
    IplImage *cv_input;
    IplImage cv_output;
};

#endif // VISIONPORT_H
