/*------------------------------------------------------------------------------
 *  Title:        stateEstimator.cpp
 *  Description:  ROS node for vehicle state estimation.
 * 				  Subscribes to sensors and control inputs
 * 				  Broadcasts current state estimate and next state prediction
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

#include "stateEstimator.h"

StateEstimator::StateEstimator()
{
    // Initialize Earth Fixed Position
    lat = 32.65694; //Waypoint near scripps 
    lon = -117.34916;
    depth = 0.;

    // Initialize Euler angles.
    roll = 0.;
    pitch = 0.;
    yaw = 3*M_PI/2; // change this to dynamic reconfig

    // Initialize Linear Velocities
    surge = 0.;
    sway = 0.;
    heave = 0.;

    // Initialize Quaternions.
    orientationX = 0.;
    orientationY = 0.;
    orientationZ = 0.;
    orientationW = 0.;

    // Initialize Linear Accelerations
    surge_dot = 0.;
    sway_dot = 0.;
    heave_dot = 0.;

    // Initialize Angular Acceleration
    yaw_dot = 0.;

    // Initialize Earth Fixed Angles from Magnetometer
    mag_x = 0.;
    mag_y = 0.;
    mag_z = 0.;

    // Initialize all control outputs to zero.
    uRoll = 0.;
    uPitch = 0.;
    uYaw = 0.;
    uDepth = 0.;
    uSurge = 0.;

    // Initialize all predicted states
    lat_next = 0.;
    lon_next = 0.;
    depth_next = 0.;

    roll_next = 0.;
    pitch_next = 0.;
    yaw_next = 0.;

    orientationX_next = 0.;
    orientationY_next = 0.;
    orientationZ_next = 0.;
    orientationW_next = 0.;

    surge_next = 0.;
    sway_next = 0.;
    heave_next = 0.;

    mag_x_next = 0.;
    mag_y_next = 0.;
    mag_z_next = 0.;

    surge_dot_next = 0.;
    sway_dot_next = 0.;
    heave_dot_next = 0.;
    yaw_dot_next = 0.;

    //Initialize previous states
    surge_prev = 0.;
    lat_prev = lat;
    lon_prev = lon;
    yaw_prev = yaw;
    yaw_dot_prev = 0.;

    //Initialize model parameters
    beta = 5; // coefficient of viscous friction (bogus number)
    tau = 10;  // RPM to thrust conversion, bogus number, should depend on motor output power, prop efficiency, motor efficiency and RPM.
    K = 1.0; // Observer gain, unimplemented for now

    mass_reciprocal = .0001; // 10,000 lb vehicle
    yaw_inertia_reciprocal = .01; // made up val, get better number
    rudder_area = 4.; //4 meter^2
    H2O_density = 1026.; //in kg/m^3
    r = 4.5; //meters, 10 meter vehicle, center point to axis of rotation of rudder

    // Initialize delta time
    dt = 0.;
    dt_pred = 0.;
	timeScale = 4.0;

    current_time = ros::Time::now();
    last_time = current_time;
    current_time_pred = current_time;
    last_time_pred = current_time;

    rudder_force = 0.; //Force normal to rudder
    yaw_force = 0.; // Force perp to surge axis
    rudder_drag = 0.; //Force parallel to surge axis
    yaw_torque = 0.; // Torque exerted on vehicle
    total_drag = 0.; // Sum of drag forces on vehicle, pun intended

    testVariable1 = 0.;
    testVariable2 = 0.;
    number_new_measurements = 0;
}

StateEstimator::~StateEstimator()
{
}

void StateEstimator::run()
{
    predictionUpdate();
    if (number_new_measurements)
    {
        measurementUpdate();
        number_new_measurements = 0;
    }
}

void StateEstimator::predictionUpdate()
{
    current_time = ros::Time::now();
    dt = (float)(current_time.toSec() - last_time.toSec());
    ROS_DEBUG("dt = %f", dt);

    // Compute state estimates

    // Note: not a good estimator yet. using Euler V = V_prev + a*dt
    // Note: all trig functions converted to degrees.
    // Control input uYaw is also in degrees, so convert to rads before eval

    //! Step 1, compute force normal to rudder as Velocity^2 * Area * density * sin(rudder_angle)
    //rudder_force = surge_prev * surge_prev * rudder_area * H2O_density * sinf(uYaw);
    rudder_force = surge_prev * surge_prev * rudder_area * sinf(uYaw);

    //! Step 2, get components parallel and perpindicular to vehicle surge axis
    yaw_force = rudder_force * cosf(uYaw);
    rudder_drag = rudder_force * sinf(uYaw);

    //! Step 3, compute torque on vehicle (rXF), negative sign comes from control surface being in the back of the vehicle (positive theta torques in negative direction), this model property needs to be considered when computing yaw commands
    yaw_torque = -1.0 * r * yaw_force;

    //! Step 4, integrate yaw accelleration, using accel = yaw_torque/Inertia (Newtons second law)
    yaw_dot = yaw_dot_prev + (yaw_torque * yaw_inertia_reciprocal) * dt;

    //! Step 5, integrate yaw velocity to yaw angle
    yaw = yaw_prev + yaw_dot * dt;

    //! Step 6, wrap angle back to zero if we cross 2pi
    while (yaw < 0.)
    {
        yaw += 2*M_PI;
    }
    while (yaw > 2*M_PI)
    {
        yaw -= 2*M_PI;
    }

    //! Step 7, compute total drag on vehicle  as beta*V^2 + controlSurfaceDrag
    total_drag = beta * surge_prev * surge_prev + rudder_drag;

    //! Step8, integrate surge velocity using accel = (thrust-totalDrag)/Mass : thrust proportional to prop speed uSurge by factor tau
    surge = surge_prev + (tau * uSurge - total_drag) * mass_reciprocal * dt;

    //! Step 9, integrate latitude and longitude using surge and yaw angle (lat and lon to meters only valid close to 45 degree latitude, make dynamic in future)
    lat = lat_prev + surge * cosf(yaw) * dt / lat2meters * timeScale;
    lon = lon_prev + surge * sinf(yaw) * dt / lon2meters * timeScale;

    // Make sure states are in bounds, these are the only variables in degrees
    if (lat > 90.)
    {
        lat = 90.;
    }
    if (lat < -90.)
    {
        lat = -90.;
    }
    while (lon > 180.)
    {
        lon -= 360.;
    }
    while (lon < -180.)
    {
        lon += 360.;
    }


    // Replace previous values with current ones for next calculation.
    yaw_dot_prev = yaw_dot;
    yaw_prev = yaw;
    surge_prev = surge;
    lat_prev = lat;
    lon_prev = lon;

    //Publish state estimates
    state_estimator::State msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "base_link";

    msg.lat = lat;
    msg.lon = lon;
    msg.depth = depth;

    msg.roll = roll;
    msg.pitch = pitch;
    msg.yaw = yaw;

    msg.surge = surge;
    msg.sway = sway;
    msg.heave = heave;

    msg.surge_dot = surge_dot;
    msg.sway_dot = sway_dot;
    msg.heave_dot = heave_dot;

    ROS_DEBUG("Yaw = %f ", yaw * 180.0 / M_PI);
    ROS_DEBUG("uYaw = %f ", uYaw * 180.0 / M_PI);
    ROS_DEBUG("Surge = %f ", surge);
    ROS_DEBUG("uSurge = %f ", uSurge);

    ROS_DEBUG("Yaw dot = %f ", yaw_dot * 180.0 / M_PI);
    ROS_DEBUG("RudderForce = %f ", rudder_force);
    ROS_DEBUG("Yaw Force = %f ", yaw_force);
    ROS_DEBUG("Lat = %f ", lat);
    ROS_DEBUG("Lon = %f ", lon);

    pubStateEstimate.publish(msg);

    last_time = current_time;
}

void StateEstimator::measurementUpdate()
{
    // March estimate forward in time for prediction.
    //Assumes constant control over one time step horizon.
    //Used in faking sensor data

    current_time_pred = ros::Time::now();
    dt_pred = (float)(current_time_pred.toSec() - last_time_pred.toSec());
    ROS_DEBUG("dt_pred = %f", dt_pred);

    surge_next = surge + (tau * uSurge - total_drag) * mass_reciprocal * dt_pred;
    yaw_next = yaw + yaw_dot * dt_pred;

    //! Wrap angle back to 0-2pi
    while (yaw_next < 0.)
    {
        yaw_next += 2*M_PI;
    }
    while (yaw_next > 2*M_PI)
    {
        yaw_next -= 2*M_PI;
    }

    lat_next = lat + surge_next * cosf(yaw_next) * dt_pred;
    lon_next = lon + surge_next * sinf(yaw_next) * dt_pred;

    // Make sure lat and lon remain on planet earth
    if (lat_next > 90.)
    {
        lat_next = 90.;
    }
    if (lat_next < -90.)
    {
        lat_next = -90.;
    }
    while (lon_next > 180.)
    {
        lon_next -= 360.;
    }
    while (lon_next < -180.)
    {
        lon_next += 360.;
    }

    state_estimator::State msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "base_link";

    msg.lat = lat_next;
    msg.lon = lon_next;
    msg.depth = depth_next;

    msg.roll = roll_next;
    msg.pitch = pitch_next;
    msg.yaw = yaw_next;

    msg.surge = surge_next;
    msg.sway = sway_next;
    msg.heave = heave_next;

    msg.surge_dot = surge_dot_next;
    msg.sway_dot = sway_dot_next;
    msg.heave_dot = heave_dot_next;

    pubStateEstimate.publish(msg);

    ROS_DEBUG("Yaw_next = %f ", yaw_next * 180.0 / M_PI);

    last_time_pred = current_time_pred;
}

void StateEstimator::controlInputsCallback(const state_estimator::ControlInputs::ConstPtr &_msg)
{
    number_new_measurements++;

    uRoll  = _msg->uRoll;
    uPitch = _msg->uPitch;
    uYaw   = _msg->uYaw;
    uDepth = _msg->uDepth;
    uSurge = _msg->uSurge;
}

void StateEstimator::imuCallback(const sensor_msgs::Imu::ConstPtr &_msg)
{
    number_new_measurements++;

    orientationX = _msg->orientation.x;
    orientationY = _msg->orientation.y;
    orientationZ = _msg->orientation.z;
    orientationW = _msg->orientation.w;
}

void StateEstimator::simDVLCallback(const state_estimator::SimDVLData::ConstPtr &_msg)
{
    number_new_measurements++;

    surge = _msg->surge;
    sway  = _msg->sway;
    heave = _msg->heave;

    //No altitude for now, but it is available in the message
}

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "stateEstimator");
    ros::NodeHandle n;
    ros::NodeHandle pnh("~");

    // Declare variables.
    int rate;
    StateEstimator *stateEstimator;

    // Set up a StateEstimator object.
    stateEstimator = new StateEstimator();

    // Initialize node parameters.
    pnh.param("rate", rate, int(15));

    // Tell ROS to run this node at the desired rate.
    ros::Rate r(rate);

    // Set up ROS subscribers and clients so that data can be found.
    ros::Subscriber compass_data_sub = n.subscribe("imu", 1, &StateEstimator::imuCallback, stateEstimator);
    ros::Subscriber ctrl_input_sub = n.subscribe("controlInputs", 1, &StateEstimator::controlInputsCallback, stateEstimator);
    ros::Subscriber simDVL_sub = n.subscribe("simDVLData", 1, &StateEstimator::simDVLCallback, stateEstimator);

    // Set up ROS publisher for state estimates and state prediction.
    stateEstimator->pubStateEstimate = pnh.advertise<state_estimator::State>("state", 1);

    // Main loop.
    while (n.ok())
    {
        stateEstimator->run();
        ros::spinOnce();
        r.sleep();
    }

    return 0;
} // end main()
