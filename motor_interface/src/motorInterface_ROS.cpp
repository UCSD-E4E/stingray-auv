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

#include "motorInterface_ROS.h"

using std::string;

/*------------------------------------------------------------------------------
 * MotorInterface_ROS()
 * Constructor.
 *----------------------------------------------------------------------------*/

MotorInterface_ROS::MotorInterface_ROS(string _portname, int _baud) : MotorInterface::MotorInterface(_portname, _baud)
{
    ROS_INFO("MotorInterface fd = %d", Serial::fd);
} // end MotorInterface_ROS()


/*------------------------------------------------------------------------------
 * ~MotorInterface_ROS()
 * Destructor.
 *----------------------------------------------------------------------------*/

MotorInterface_ROS::~MotorInterface_ROS()
{
} // end ~MotorInterface_ROS()

/*---------------------------------------------------------------------
* ControlInputsCallback()
* Callback for when control input data is received.
* -------------------------------------------------------------------*/

void MotorInterface_ROS::ControlInputsCallback(const stateEstimator::ControlInputs::ConstPtr &_msg)
{
    uSurge = (int)(_msg->uSurge * radPsec2RPM);
	ROS_DEBUG("MOTOR INTERFACE Variable uSURGE = %d", uSurge);
} // end ControlInputsCallback()

/*------------------------------------------------------------------------------
 * main()
 * Main function to set up ROS node, speed commands and use callbacks to
 * send speed commands.
 *----------------------------------------------------------------------------*/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motorInterface_node");
    ros::NodeHandle n;
    ros::NodeHandle private_node_handle_("~");


    // Declare variables.
    string portname;
    int baud;
    double rate;
    int uSurge;
    MotorInterface_ROS *interface;

    int prescalar; // for decreasing the frequency of serial messages sent to serial port, val of 100 sends every 100th command
    int commandCount;

    // Set up variables.
    private_node_handle_.param("port", portname, string("/dev/ttyS0"));
    private_node_handle_.param("baud", baud, int(9600));
    private_node_handle_.param("rate", rate, double(1));
    private_node_handle_.param("uSurge", uSurge, int(0));
    private_node_handle_.param("prescalar", prescalar, int(5));
    private_node_handle_.param("commandCount", commandCount, int(1));


    // Create a new MotorInterface_ROS object.
    interface = new MotorInterface_ROS(portname, baud);

    // Tell ROS to run this node at the rate from the launch file.
    ros::Rate r(rate);

    ros::Subscriber ctrl_input_sub = n.subscribe("controlInputs", 1000, &MotorInterface_ROS::ControlInputsCallback, interface);

    // Main loop.
    while (n.ok())
    {
        ROS_DEBUG("uSurge in RPM = %d,", interface->uSurge);

        commandCount--;
        if (commandCount <= 0)
        {
            // Send speed command out through serial port
            interface->SpeedCommand(interface->uSurge);
            commandCount = prescalar;
        }

        ros::spinOnce();
        r.sleep();
    }

    return 0;
} // end main()
