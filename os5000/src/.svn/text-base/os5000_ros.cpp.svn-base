/*------------------------------------------------------------------------------
 *  Title:        os5000_ros.cpp
 *  Description:  ROS node for running the Ocean Server OS5000 digital compass.
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

#include "os5000_ros.h"

/*------------------------------------------------------------------------------
 * OSCompass()
 * Constructor.
 *----------------------------------------------------------------------------*/

OSCompass::OSCompass(string _portname, int _baud, int _rate, int _init_time) : Compass::Compass(_portname, _baud, _rate, _init_time)
{
} // end OSCompass()


/*------------------------------------------------------------------------------
 * ~OSCompass()
 * Destructor.
 *----------------------------------------------------------------------------*/

OSCompass::~OSCompass()
{
} // end ~OSCompass()


/*------------------------------------------------------------------------------
 * publishImuData()
 * Publish standard compass message.
 *----------------------------------------------------------------------------*/

void OSCompass::publishImuData(ros::Publisher *pub_imu_data)
{
    sensor_msgs::Imu imudata;
    double linear_acceleration_covariance = 10000.;
    double angular_velocity_covariance = 10000.;
    double orientation_covariance = 1.;
    uint64_t time = 0;

    imudata.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll * M_PI / 180., pitch * M_PI / 180., yaw * M_PI / 180.);
    
    imudata.orientation_covariance[0] = orientation_covariance;
    imudata.orientation_covariance[4] = orientation_covariance;
    imudata.orientation_covariance[8] = orientation_covariance;

    imudata.angular_velocity.x = 0.;
    imudata.angular_velocity.y = 0.;
    imudata.angular_velocity.z = 0.;

    imudata.angular_velocity_covariance[0] = angular_velocity_covariance;
    imudata.angular_velocity_covariance[4] = angular_velocity_covariance;
    imudata.angular_velocity_covariance[8] = angular_velocity_covariance;

    imudata.linear_acceleration.x = 0.;
    imudata.linear_acceleration.y = 0.;
    imudata.linear_acceleration.z = 0.;

    imudata.linear_acceleration_covariance[0] = linear_acceleration_covariance;
    imudata.linear_acceleration_covariance[4] = linear_acceleration_covariance;
    imudata.linear_acceleration_covariance[8] = linear_acceleration_covariance;

    ROS_DEBUG("OS5000 Quaternions = %.1f, %.1f, %.1f, %.1f", imudata.orientation.x, imudata.orientation.y, imudata.orientation.z, imudata.orientation.w);
    ROS_DEBUG("OS5000 (RPY) = (%lf, %lf, %lf)", roll, pitch, yaw);

    imudata.header.stamp = ros::Time::now().fromNSec(time);

    pub_imu_data->publish(imudata);
} // end publishImuData()


/*------------------------------------------------------------------------------
 * configCallback()
 * Callback function for dynamic reconfigure server.
 *----------------------------------------------------------------------------*/

void OSCompass::configCallback(os5000::os5000Config &config, uint32_t level)
{
    ROS_INFO("Reconfiguring port, baud, rate, init_time, reconnect to %s, %d, %d, %d, %d", config.port.c_str(), config.baud, config.rate, config.init_time, config.reconnect);

    // Set class variables to new values.
    baud      = config.baud;
    init_time = config.init_time;
    portname  = config.port.c_str();
    rate      = config.rate;

    // Check to see if we should attempt to reconnect to the compass.
    if (config.reconnect)
    {
        // Use the new compass settings to reconnect.
        setup();
        ROS_INFO("Using new settings to reconnect to compass. Got fd = %d", fd);

        // Reset the reconnect variable.
        config.reconnect = false;
    }
} // end configCallback()


/*------------------------------------------------------------------------------
 * main()
 * Main function to set up ROS node, get compass data and use callbacks to
 * publish compass data.
 *----------------------------------------------------------------------------*/

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "compass_node");
    ros::NodeHandle n;
    ros::NodeHandle private_node_handle_("~");

    // Local variables.
    int baud;
    int init_time;
    string portname;
    string pub_topic_name;
    int rate;

    // Initialize node parameters.
    private_node_handle_.param("baud",           baud,           int(115200));
    private_node_handle_.param("init_time",      init_time,      int(3));
    private_node_handle_.param("port",           portname,       string("/dev/ttyUSB0"));
    private_node_handle_.param("pub_topic_name", pub_topic_name, string("os5000_data"));
    private_node_handle_.param("rate",           rate,           int(40));

    // Create a new OSCompass object.
    OSCompass *oscompass = new OSCompass(portname, baud, rate, init_time);

    // Set up a dynamic reconfigure server.
    dynamic_reconfigure::Server<os5000::os5000Config> gain_srv;
    dynamic_reconfigure::Server<os5000::os5000Config>::CallbackType f;
    f = boost::bind(&OSCompass::configCallback, oscompass, _1, _2);
    gain_srv.setCallback(f);

    // Set up publishers.
    ros::Publisher pubImuData = n.advertise<sensor_msgs::Imu>(pub_topic_name.c_str(), 1);

    // Tell ROS to run this node at the rate that the compass is sending messages to us.
    ros::Rate r(rate);

    // Connect to the Ocean Server compass.
    if (oscompass->fd < 0)
    {
        ROS_ERROR("Could not connect to compass on port %s at %d baud. You can try changing the parameters using the dynamic reconfigure gui.", oscompass->portname.c_str(), oscompass->baud);
    }

    // Main loop.
    while (n.ok())
    {
        // Get compass data.
        if (oscompass->fd > 0)
        {
            oscompass->getData();

            if (oscompass->yaw > 180.)
            {
                oscompass->yaw -= 360.;
            }

            // Publish the message.
            oscompass->publishImuData(&pubImuData);
        }

        ros::spinOnce();
        r.sleep();
    }

    return 0;
} // end main()
