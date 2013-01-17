/**
 *  \file usb1208LS_node.h
 *  \brief ROS node for Controlling USB Analog and Digital IO board
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

#ifndef USB1208LSNODE_H
#define USB1208LSNODE_H

// System libraries.
#include <cstdlib>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <ctype.h>
#include <sys/types.h>
#include <asm/types.h>

#include "pmd.h"

// ROS.
#include "ros/ros.h"
#include "ros/time.h"
#include "tf/transform_datatypes.h"

#include "usb-1208LS.h"
//#include "timing.h"


// Custom messages.
#include "stateEstimator/LoadTorque.h"
#include "stateEstimator/PrimePowerStartStop.h"

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

class Usb1208ls_node
{
public:
    //! Constructor.
    Usb1208ls_node();

    //! Destructor.
    ~Usb1208ls_node();

    // Member Functions

    //! Callback function for Load Torque Command
    void LoadTorqueControlCallback(const stateEstimator::LoadTorque::ConstPtr& msg);

    //! Callback function for Prime Power Control
    void PrimePowerControlCallback(const stateEstimator::PrimePowerStartStop::ConstPtr& msg);

    bool primePowerStartStop;
    uint16_t loadTorque;
    HIDInterface*  hid;
    hid_return ret;


    uint8_t input;
    uint8_t pin;
    uint8_t channel;
    uint8_t gain;

    //! Pointer to a usb1208ls_node class.
    Usb1208ls_node *usb1208ls_node;
};

#endif // USB1208LSNODE_H
