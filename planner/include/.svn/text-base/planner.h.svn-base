/**
 *  \file planner.h
 *  \brief ROS node for the generation of target vehicle states.
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

#ifndef PLANNER_H
#define PLANNER_H

// System libraries.
#include <cstdlib>
#include <math.h>

// ROS.
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>

// Custom messages.
#include "mission_controller/TargetWaypoints.h"
#include "planner/TargetStates.h"
#include "state_estimator/State.h"

// Dynamic reconfigure.
#include <dynamic_reconfigure/server.h>
#include <planner/plannerParamsConfig.h>

using std::string;


/******************************
 * Classes
 *****************************/

class Planner
{
public:
    //! Constructor.
    Planner();

    //! Destructor.
    ~Planner();

    //! State initialization.
    void initStates();

    //! Publisher for target states
    void publishTargetStates(ros::Publisher *pubTargetStates);

    //! Follow waypoints from missionController
    void followPathCallback(const mission_controller::TargetWaypoints::ConstPtr& msg);

    //! Callback function for dynamic configuration of gains
    void configCallback(planner::plannerParamsConfig& config, uint32_t level);

    //! Callback function for State Estimate
    void stateEstimatorCallback(const state_estimator::State::ConstPtr& msg);

    //! Callback function for compass data.
    void microstrainCallback(const sensor_msgs::Imu::ConstPtr& msg);

    //! Callback function for compass data.
    void compassCallback(const sensor_msgs::Imu::ConstPtr& msg);

    // Target states.
    bool dynamic_reconfigure_enable;
    //! Desired roll angle.
    double target_roll;
    //! Deisred pitch angle.
    double target_pitch;
    //! Desired yaw angle.
    double target_yaw;
    //! Desired depth.
    double target_depth;
    //! Desired forward velocity.
    double target_surge; // PIC motor controller uses ints, typecasting for now.

    //! Desired Global Position latitude.
    double target_lat;
    //! Desired Global Position longitude.
    double target_lon;

    //! Waypoint errors
    double lat_err;
    double lon_err;
    double depth_err;

    //! Minimum distance to a waypoint before success flag set
    double waypoint_radius;

    //! Distance between lawnmower segments
    double search_width;

    //! Maximum ammount of time to perform a task
    double task_time;

    //! Mission status flags
    bool recharge_flag;
    bool mission_complete;

    // Current States (For future use with more spohisticated path planning).
    double roll;
    double pitch;
    double yaw;
    double depth;
    double surge;
    double lat;
    double lon;

    //! Pointer to a Planner class.
    Planner *planner;
};

#endif // PLANNER_H
