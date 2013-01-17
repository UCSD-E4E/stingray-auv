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

#include "ms3dmg_ros.h"

using std::string;

/*------------------------------------------------------------------------------
 * MS3dmgROS()
 * Constructor.
 *----------------------------------------------------------------------------*/

MS3dmgROS::MS3dmgROS(string _portname, int _baud, bool _use_gyro_stab) : MS3dmgCore::MS3dmgCore(_portname, _baud, _use_gyro_stab)
{
    ROS_INFO("Microstrain fd = %d", Serial::fd);
} // end MS3dmgROS()


/*------------------------------------------------------------------------------
 * ~MS3dmgROS()
 * Destructor.
 *----------------------------------------------------------------------------*/

MS3dmgROS::~MS3dmgROS()
{
} // end ~MS3dmgROS()


/*------------------------------------------------------------------------------
 * publishImuData()
 * Publish standard IMU message.
 *----------------------------------------------------------------------------*/

void MS3dmgROS::publishImuData(ros::Publisher *pub_imu_data)
{
    // Some other node that gets all sensor data needs to publish a geometry_msgs/PoseWithCovarianceStamped message
    // for AMCL, Navigation and Rviz. The initialpose topic especially needs to have that message type
    // so RViz knows where the robot is.
    sensor_msgs::Imu msg;
    double linear_acceleration_covariance = 0.1;
    double angular_velocity_covariance = 0.01;
    double orientation_covariance = 1.;

    msg.header.stamp = ros::Time::now();

    msg.orientation.x = quat[0];
    msg.orientation.y = quat[1];
    msg.orientation.z = quat[2];
    msg.orientation.w = quat[3];

    msg.orientation_covariance[0] = orientation_covariance;
    msg.orientation_covariance[4] = orientation_covariance;
    msg.orientation_covariance[8] = orientation_covariance;

    msg.angular_velocity.x = ang_rate[0];
    msg.angular_velocity.y = ang_rate[1];
    msg.angular_velocity.z = ang_rate[2];

    msg.angular_velocity_covariance[0] = angular_velocity_covariance;
    msg.angular_velocity_covariance[4] = angular_velocity_covariance;
    msg.angular_velocity_covariance[8] = angular_velocity_covariance;

    msg.linear_acceleration.x = accel[0];
    msg.linear_acceleration.y = accel[1];
    msg.linear_acceleration.z = accel[2];

    msg.linear_acceleration_covariance[0] = linear_acceleration_covariance;
    msg.linear_acceleration_covariance[4] = linear_acceleration_covariance;
    msg.linear_acceleration_covariance[8] = linear_acceleration_covariance;

    ROS_INFO("Microstrain Quaternions: %f, %f, %f, %f", msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);
    ROS_INFO("Microstrain Angular velocity: %f, %f, %f", msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
    ROS_INFO("Microstrain Linear acceleartion: %f, %f, %f", msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);

    pub_imu_data->publish(msg);
} // end publishImuData()


/*------------------------------------------------------------------------------
 * main()
 * Main function to set up ROS node, get IMU data and use callbacks to
 * publish IMU data.
 *----------------------------------------------------------------------------*/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ms3dmg_node");
    ros::NodeHandle n;

    // Declare variables.
    int baud;
    string portname;
    string pub_topic_name;
    int rate;
    int serial_number;
    bool use_gyro_stab;

    // Set up variables.
    ros::NodeHandle private_node_handle_("~");
    private_node_handle_.param("baud",           baud,           int(115200));
    private_node_handle_.param("port",           portname,       string("/dev/ttyUSB1"));
    private_node_handle_.param("pub_topic_name", pub_topic_name, string("mstrain_data"));
    private_node_handle_.param("rate",           rate,           int(40));
    private_node_handle_.param("serial_number",  serial_number,  int(1397));
    private_node_handle_.param("use_gyro_stab",  use_gyro_stab,  bool(true));

    // Create a new MS3dmgROS object.
    MS3dmgROS *mstrain = new MS3dmgROS(portname, baud, use_gyro_stab);

    // Set up publisher.
    ros::Publisher pub_imu = n.advertise<sensor_msgs::Imu>(pub_topic_name.c_str(), 1);

    // Tell ROS how fast to run this node.
    ros::Rate r(rate);

    // Check that the serial number matches with what we expect.
    mstrain->serialNumber();
    ROS_INFO("Serial number of Microstrain = %d, expected %d", mstrain->serial_number, serial_number);

    // Main loop.
    while (n.ok())
    {
        // Verify that Microstrain data is valid before publishing anything. fd is not good enough with USB ports.
        if (serial_number == mstrain->serial_number)
        {
            // Get IMU data.
            mstrain->quaternionsVectors();
            //mstrain->vectors();

            // Publish the message.
            mstrain->publishImuData(&pub_imu);
        }
        else
        {
            ROS_ERROR("Microstrain IMU not initialized properly. No data is being published.");
        }

        ros::spinOnce();
        r.sleep();
    }

    return 0;
} // end main()
