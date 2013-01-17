/**
 *  \file os5000_ros.h
 *  \brief ROS node for running the Ocean Server OS5000 digital compass.
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

#ifndef OS5000_ROS_H
#define OS5000_ROS_H

// ROS includes.
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>

// Local includes.
#include "os5000_core.h"
#include "timing.h"

// Dynamic reconfigure.
#include <dynamic_reconfigure/server.h>
#include <os5000/os5000Config.h>

using std::string;


/******************************
 *
 * #defines
 *
 *****************************/


/******************************
 *
 * Classes
 *
 *****************************/

class OSCompass : public Compass
{
public:
    //! Constructor.
    OSCompass(string _portname, int _baud, int _rate, int _init_time);

    //! Destructor.
    ~OSCompass();

    //! Callback function for dynamic reconfigure server.
    void configCallback(os5000::os5000Config &config, uint32_t level);

    //! Publish the data from the compass in a ROS standard format.
    void publishImuData(ros::Publisher *pubImuData);
};

#endif // OS5000_ROS_H
