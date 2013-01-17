/**
 *  \file camera_model.h
 *  \brief ROS node for running a model of the Camera.
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
 *      * Neither the name of the Camera, iBotics nor the names of its
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

#ifndef CAMERA_MODEL_H
#define CAMERA_MODEL_H

// System libraries.
#include <cstdlib>
#include <string>

// ROS.
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

class CameraModel
{
public:
    //! Constructor.
    CameraModel();

    //! Destructor.
    ~CameraModel();

    //! Callback function for standard IMU data.
    void ms3dmgCallback(const sensor_msgs::Imu::ConstPtr &msg);

    //! Callback function for standard compass data.
    void os5000Callback(const sensor_msgs::Imu::ConstPtr &msg);

    //! Quaternion X.
    float orientation_x;
    //! Quaternion Y.
    float orientation_y;
    //! Quaternion Z.
    float orientation_z;
    //! Quaternion W.
    float orientation_w;
    //! Depth of the path.
    double path_depth;
    //! Radius of the path.
    double path_radius;
};

#endif // CAMERA_MODEL_H 
