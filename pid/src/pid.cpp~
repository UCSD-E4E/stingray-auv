/*------------------------------------------------------------------------------
 *  Title:        pid.cpp
 *  Description:  ROS node for running PID server.
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

#include "ros/ros.h"
#include "pid/PID.h"

/*------------------------------------------------------------------------------
 * computePIDOut()
 * Computes the output that should be sent using PID controller.
 *----------------------------------------------------------------------------*/

bool computePIDOut(pid::PID::Request& req, pid::PID::Response& res)
{
    // Declare variables.
    float error = req.target_val - req.current_val;
    float integral_term = req.previous_integrator_val + error * req.dt;

    // Apply integrator limits.
    if (integral_term < req.integral_term_min)
    {
        integral_term = req.integral_term_min;
    }
    if (integral_term > req.integral_term_max)
    {
        integral_term = req.integral_term_max;
    }

    // Compute the output.
    //! \todo Still need to add anti-windup feature.
    res.u = req.gain_p * error + req.gain_i * integral_term + req.gain_d * (error - req.previous_error) / req.dt;
    res.current_integrator_val = integral_term;
    res.current_error = error;

    ROS_DEBUG("Request contains current_state = %f, target_state = %f, Kp = %f", req.current_val, req.target_val, req.gain_p);
    ROS_DEBUG("Integrator limits with min = %f, max = %f", req.integral_term_min, req.integral_term_max);
    ROS_DEBUG("Sending back control input -- u = %f", res.u);

    return true;
} // end computePIDOut()


/*------------------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *----------------------------------------------------------------------------*/

int main(int argc, char **argv)
{
    // Start ROS.
    ros::init(argc, argv, "pidserver");
    ros::NodeHandle n;

    // Start PID service.
    ros::ServiceServer service = n.advertiseService("PID", computePIDOut);
    ros::spin();

    return 0;
} // end main()
