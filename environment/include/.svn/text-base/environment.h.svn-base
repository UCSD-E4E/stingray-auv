/**
 *  \file environment.h
 *  \brief ROS node for environmental simulation.
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

#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

// System libraries.
#include <cstdlib>
#include <math.h>

// ROS.
#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

// Dynamic reconfigure.
#include <dynamic_reconfigure/server.h>
#include <environment/environmentParamsConfig.h>

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

class Environment
{
public:
    //! Constructor.
    Environment();

    //! Destructor.
    ~Environment();

    // Member Functions

    //! State initialization.
    void InitEnvironment();

    //! Publisher for simulated IMU data
    void PublishSimulatedMicrostrainData(ros::Publisher *_pubSimEuler);

    //! Publisher for simulated DVL data
    void PublishSimulatedDVLData(ros::Publisher *_pubSimDVL);

    //! Callback function for custom Microstrain data.
    void StateEstimatorCallback(const nav_msgs::Odometry::ConstPtr& _msg);

    //! Callback function for dynamic configuration of gains
    void DisturbanceCallback(environment::environmentParamsConfig &_config, uint32_t _level);

    //! Earth Fixed Frame
    float lat;   //gps latitude
    float lon;   //gps longitude
    float depth; //depth is positive below the surface (in meters)

    //! Roll state. In radians (-pi,pi], measured around longitudinal axis, positive is left wing up, negative is left wing down.
    float roll;

    //! Pitch state. In radians (-pi,pi], measured around lateral axis, positive is nose up, negative is nose down.
    float pitch;

    //! Yaw state. In radians (-pi,pi], measured around vertical axis, positive is left turn, negative is right turn.
    float yaw;

    //! Quaternion X.
    float orientationX;

    //! Quaternion Y.
    float orientationY;

    //! Quaternion Z.
    float orientationZ;

    //! Quaternion W.
    float orientationW;

    //! Forward speed In meters/second.
    float surge;

    //! Sway speed In meters/second.
    float sway;

    //! Heave velocity In meters/second (depth dot)
    float heave;


    //! Earth Fixed Frame: Magnetometer Readings (Absolute Angles)
    float mag_x;
    float mag_y;
    float mag_z;

    //! Linear Accelerations in meters/second^2
    float surge_dot;
    float sway_dot;
    float heave_dot;

    //! Pointer to an environment class.
    Environment *environment;

    //! Provide a map of the environment:Add detail later
    float ocean_depth;

    float sea_floor_height;

    //xyz + width + height + depth (location and size of an obstacle)
    float obstacle1[6];

    //! State disturbances
    double disturbanceRoll;
    double disturbancePitch;
    double disturbanceYaw;
    double disturbanceDepth;
    double disturbanceSpeed;
    double disturbanceSway;

    //Magnitude of electrical noise
    double noiseScale;
};

#endif // ENVIRONMENT_H
