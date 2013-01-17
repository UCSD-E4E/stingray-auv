/**
 *  \file stateEstimator.h
 *  \brief ROS node for estimating vehicle state
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

#ifndef STATEESTIMATOR_H
#define STATEESTIMATOR_H

// System libraries.
#include <cstdlib>

// ROS.
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>

// Custom messages.
#include "state_estimator/State.h"
#include "state_estimator/ControlInputs.h"
#include "state_estimator/SimDVLData.h"

/******************************
 * #defines
 *****************************/
#define lat2meters  111131.75 //length of a deg of latitude in meters at 45 degree lat
#define lon2meters  78846.81 //length of a deg of longitute in meters at 45 degree lat

/******************************
 * Classes
 *****************************/

class StateEstimator
{
public:
    //! Constructor.
    StateEstimator();

    //! Destructor.
    ~StateEstimator();

    // Member Functions

    //! State initialization.
    void initStates();

    //! Run steps.
    void run();

    //! Callback function for Control Inputs.
    void controlInputsCallback(const state_estimator::ControlInputs::ConstPtr &_msg);

    //! Callback function for standard compass data.
    void imuCallback(const sensor_msgs::Imu::ConstPtr &_msg);

    //! Callback function for simulated DVL data
    void simDVLCallback(const state_estimator::SimDVLData::ConstPtr &_msg);

    // Add Depth Sensor callback.

    // Add DVL Sensor callback (or other speed measurement) when vehicle becomes equiped.

    ros::Publisher pubStateEstimate;

private:
    //! Publisher for StateEstimate
    void predictionUpdate();

    //! Publisher for StatePrediction
    void measurementUpdate();

    // States.

    //! Earth Fixed Frame
    //! GPS latitude.
    float lat;

    //! GPS longitude.
    float lon;

    //! Depth state. In meters, surface is 0m, positive is below the surface.
    float depth;

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

    //! Differential change in yaw angle
    float yaw_dot;

    //! Duplicate of all states to hold predicted values
    float lat_next;
    float lon_next;
    float depth_next;

    float roll_next;
    float pitch_next;
    float yaw_next;

    float orientationX_next;
    float orientationY_next;
    float orientationZ_next;
    float orientationW_next;

    float surge_next;
    float sway_next;
    float heave_next;

    float mag_x_next;
    float mag_y_next;
    float mag_z_next;

    float surge_dot_next;
    float sway_dot_next;
    float heave_dot_next;
    float yaw_dot_next;

    //! Duplicate states to hold previous values
    float surge_prev;
    float lat_prev;
    float lon_prev;
    float yaw_prev;
    float yaw_dot_prev;

    //! Viscous Damping Coefficient for Velocity Model
    float beta;

    //! Motor Speed to Thrust Conversion
    float tau;

    //! Observer gain (static for now)
    float K;

    //! 1/Vehicle Mass in kG
    float mass_reciprocal;

    //! 1/Vehicle inertia along yaw axis
    float yaw_inertia_reciprocal;

    //! Area of the yaw control surface
    float rudder_area;

    //! Density of seawater
    float H2O_density;

    //! Distance from CG to rudder axis
    float r;

    //! The following five variables are used intermediate yaw and drag calculations
    float rudder_force; //Force normal to rudder
    float yaw_force; // Force perp to surge axis
    float rudder_drag; //Force parallel to surge axis
    float yaw_torque; // Torque exerted on vehicle
    float total_drag; // Sum of drag forces on vehicle, pun intended

    // Variables for storing control outputs u.

    //! Output command for roll.
    float uRoll;

    //! Output command for pitch.
    float uPitch;

    //! Output command for yaw.
    float uYaw;

    //! Output command for depth.
    float uDepth;

    //! Output command for speed.
    float uSurge;

    float testVariable1;
    float testVariable2;

    //! Elapsed time between estimation steps.
    float dt;

    //! Elapsed time between prediction steps
    float dt_pred;

	//! Time scale speed up for non-realtime simulation
	float timeScale;

    //! variables for measuring time in estimation and prediction
    ros::Time current_time;
    ros::Time last_time;

    ros::Time current_time_pred;
    ros::Time last_time_pred;

    int number_new_measurements;
};

#endif // STATEESTIMATOR_H
