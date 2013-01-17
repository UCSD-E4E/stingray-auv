/*------------------------------------------------------------------------------
 *  Title:        planner.cpp
 *  Description:  ROS node for running Planner.
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

#include "planner.h"

/*------------------------------------------------------------------------------
 * Planner()
 * Constructor.
 *----------------------------------------------------------------------------*/

Planner::Planner()
{
    initStates();
} // end Planner()


/*------------------------------------------------------------------------------
 * ~Planner()
 * Destructor.
 *----------------------------------------------------------------------------*/

Planner::~Planner()
{
} // end ~Planner()


/*---------------------------------------------------------------------
* initStates()
* Initializes measured states, target states, and errors to zero.
* -------------------------------------------------------------------*/

void Planner::initStates()
{
    // States.
    roll = 0.;
    pitch = 0.;
    yaw = 0.;
    depth = 0.;
    surge = 0.;
    lat = 0.;
    lon = 0.;
} //end initStates()


/*---------------------------------------------------------------------
* buildMissions()
* Constructs Missions out of subtasks
* --------------------------------------------------------------------*/



/*---------------------------------------------------------------------
* publishTargetStates()
* Publishes target information.
* -------------------------------------------------------------------*/

void Planner::publishTargetStates(ros::Publisher *pubTargetStates)
{
    planner::TargetStates msg;

    msg.target_roll  = target_roll;
    msg.target_pitch = target_pitch;
    msg.target_yaw   = target_yaw;
    msg.target_depth = target_depth;
    msg.target_surge = target_surge;

    pubTargetStates->publish(msg);
} // end publishTargetStates()


/*---------------------------------------------------------------------
* stateEstimatorCallback()
* Callback when state estimation data is received.
* -------------------------------------------------------------------*/

void Planner::stateEstimatorCallback(const state_estimator::State::ConstPtr& msg)
{
    roll  = msg->roll;
    pitch = msg->pitch;
    yaw   = msg->yaw;
    depth = msg->depth;
    surge = msg->surge;
    lat   = msg->lat;
    lon   = msg->lon;

    ROS_DEBUG("Planner: State Estimates Received roll = %f , pitch = %f , yaw = %f , depth = %f , surge = %f , latitude = %f , longitude = %f", roll, pitch, yaw, depth, surge, lat, lon);
} // end stateEstimatorCallback()


/*-----------------------------------------------------------------------
 * computeTargets()
 * Computes the target states from mission requests in mission controller
 *---------------------------------------------------------------------*/

void Planner::followPathCallback(const mission_controller::TargetWaypoints::ConstPtr& msg)
{
    //Check if setting of targets has been disabled by dynamicReconfigure
    if (!dynamic_reconfigure_enable)
    {
        // Update targets.
        target_surge = msg->target_surge;
        target_lat   = msg->target_lat;
        target_lon   = msg->target_lon;
        target_depth = msg->target_depth;
        //waypoint_radius = _msg->waypoint_radius;
        ROS_DEBUG("Received message from Mission Controller, target Surge = %f, target Latitude = %f, target Longitude = %f, target Depth = %f", target_surge, target_lat, target_lon, target_depth);

        target_roll = 0.;
        target_pitch = 0.;
        //compute target yaw here

        lat_err = target_lat - lat;
        lon_err = target_lon - lon;

        //Check if converging to target crosses international date line (-180->180)
        if (lon_err > 180.0)
        {
            lon_err -= 360.0;
        }
        if (lon_err < -180.0)
        {
            lon_err += 360.0;
        }
        // Only lat and lon are in degrees, yaw target based on vectors
        // (coordinate errors) in lat and lon, so no conversion from
        // lat lon units to yaw in radians necessary.

        // Compute angle wrt longitude ('x' axis).
        // Then rotate wrt latitude, so zero radians is (magnetic) north.
        target_yaw = M_PI_2 - atan2(lat_err, lon_err);
    }
    else
    {
        ROS_DEBUG("Dynamic Reconfigure Override.");
    }
} // end followPath()


/*------------------------------------------------------------------------------
 * microstrainCallback()
 * Callback for IMU data.
 *----------------------------------------------------------------------------*/

void Planner::microstrainCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
} // end microstrainCallback()


/*------------------------------------------------------------------------------
 * compassCallback()
 * Callback for compass data.
 *----------------------------------------------------------------------------*/

void Planner::compassCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
} // end compassCallback()


/*---------------------------------------------------------------------
* configCallback()
* Callback for dynamic reconfigure server.
* -------------------------------------------------------------------*/

void Planner::configCallback(planner::plannerParamsConfig &config, uint32_t level)
{
    ROS_INFO("Changing Reconfigure Enable to : %d", config.dynamic_reconfigure_enable);
    dynamic_reconfigure_enable = config.dynamic_reconfigure_enable;

    // Note: we convert user commands from gui to radians for nav calculations.
    // Users like commands in degrees, models like radians.
    if (dynamic_reconfigure_enable)
    {
        ROS_INFO("Reconfiguring Target Roll to : %f", config.target_roll);
        target_roll = config.target_roll * M_PI / 180.0;

        ROS_INFO("Reconfiguring Target Pitch to : %f", config.target_pitch);
        target_pitch = config.target_pitch * M_PI / 180.0;

        ROS_INFO("Reconfiguring Target Yaw to : %f", config.target_yaw);
        target_yaw = config.target_yaw * M_PI / 180.0;

        ROS_INFO("Reconfiguring Target Depth to : %f", config.target_depth);
        target_depth = config.target_depth;

        ROS_INFO("Reconfiguring Target Surge to : %f", config.target_surge);
        target_surge = config.target_surge;

        ROS_INFO("Reconfiguring Target Latitude to : %f", config.target_lat);
        target_lat = config.target_lat;

        ROS_INFO("Reconfiguring Target Longitude to : %f", config.target_lon);
        target_lon = config.target_lon;
    }
    else
    {
        ROS_DEBUG("Dynamic Reconfigure Disabled");
    }
} // end configCallback()


/*---------------------------------------------------------------------
* main()
* Main function for ROS node.
* -------------------------------------------------------------------*/

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "planner");
    ros::NodeHandle n;
    ros::NodeHandle private_node_handle_("~");

    // Declare variables.
    int rate;
    Planner *planner;
    //Waypoints stationKeepWayPoints;
    //Waypoints rechargeWayPoints;

    // Create a Planner object.
    planner = new Planner();

    // Initialize node parameters.
    private_node_handle_.param("rate", rate, int(2));
    private_node_handle_.param("target_roll",                planner->target_roll, double(0.));
    private_node_handle_.param("target_pitch",               planner->target_pitch, double(0.));
    private_node_handle_.param("target_yaw",                 planner->target_yaw, double(0.));
    private_node_handle_.param("target_depth",               planner->target_depth, double(0.));
    private_node_handle_.param("target_surge",               planner->target_surge, double(0.));
    private_node_handle_.param("target_lat",                 planner->target_lat, double(0.));
    private_node_handle_.param("target_lon",                 planner->target_lon, double(0.));
    private_node_handle_.param("waypoint_radius",            planner->waypoint_radius, double(0.));
    private_node_handle_.param("task_time",                  planner->task_time, double(0.));
    private_node_handle_.param("recharge_flag",              planner->recharge_flag, bool(0));
    private_node_handle_.param("dynamic_reconfigure_enable", planner->dynamic_reconfigure_enable, bool(true));

    // Set node execution rate
    ros::Rate r(rate);

    // Set up ROS subscribers so that data can be found.
    ros::Subscriber state_estimator_sub = n.subscribe("state", 1, &Planner::stateEstimatorCallback, planner);
    ros::Subscriber waypoint_sub = n.subscribe("targetWaypoints", 1, &Planner::followPathCallback, planner);
    ros::Subscriber compass_sub = n.subscribe("compassData", 1, &Planner::compassCallback, planner);
    ros::Subscriber microstrain_sub = n.subscribe("mstrain_data", 1, &Planner::microstrainCallback, planner);

    // Set up ROS publisher for target states
    ros::Publisher pub_target_states = n.advertise<planner::TargetStates>("targetStates", 1000);

    // Set up a dynamic reconfigure server.
    dynamic_reconfigure::Server<planner::plannerParamsConfig> gain_srv;
    dynamic_reconfigure::Server<planner::plannerParamsConfig>::CallbackType f;
    f = boost::bind(&Planner::configCallback, planner, _1, _2);
    gain_srv.setCallback(f);

    // Main loop.
    while (n.ok())
    {
        planner->publishTargetStates(&pub_target_states);
        ros::spinOnce();
        r.sleep();
    }

    return 0;
} // end main()
