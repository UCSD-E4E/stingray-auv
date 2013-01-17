/**
 *  \file motorInterface_ROS.h
 *  \brief ROS node for serial interfacing with motor controller.
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

#ifndef MOTORINTERFACE_ROS_H
#define MOTORINTERFACE_ROS_H

// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"
#include "stateEstimator/ControlInputs.h"

// Local includes.
#include "motorInterface.h"


using std::string;


/******************************
 *
 * #defines
 *
 *****************************/

#define radPsec2RPM 10 // approximate conversion, speed commands are ints so this rounding is inevitable
/******************************
 *
 * Classes
 *
 *****************************/

class MotorInterface_ROS : public MotorInterface
{
public:
    //! Constructor.
    MotorInterface_ROS(string _portname, int _baud);

    //! Destructor.
    ~MotorInterface_ROS();

    //! Callback function for Control Inputs.
    void ControlInputsCallback(const stateEstimator::ControlInputs::ConstPtr &_msg);

    //! Pointer to an Interface class.
    MotorInterface_ROS *interface;

    //! The baud rate to use with the port.
    int baud;

    //! The name of the port to open.
    string portname;

    //! Speed command (int for now due to PIC speed command format)
    int uSurge;
};

#endif // MOTORINTERFACE_ROS
