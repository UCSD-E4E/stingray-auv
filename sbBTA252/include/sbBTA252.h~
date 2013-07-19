/**
 *  \file sbBTA252.h
 *  \brief Sending and receiving data with the Seabotix BTA252 thrusters.
 */

/*
 *
 *    Copyright (c) 2010 <iBotics -- www.sdibotics.org>
 *    All rights reserved.
 *
 *    Redistribution and use in source and binary forms, with or without
 *    modification, are permitted provided that the following conditions are
 *    met:
 *
 *    * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer
 *    in the documentation and/or other materials provided with the
 *    distribution.
 *    * Neither the name of the Stingray, iBotics nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *    A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *    OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *    SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *    LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *    DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *    THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef SR_SBBTA252_H
#define SR_SBBTA252_H

// System includes.
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sstream>

// ROS includes.
#include <ros/ros.h>
#include <ros/time.h>

// Local includes.
#include "sbBTA252/SBBTA252Data.h"
#include "serial.h"
#include "timing.h"

// Custom messages.
#include "nav/ControlInputs.h"

// Dynamic reconfigure.
#include <dynamic_reconfigure/server.h>
#include <sbBTA252/sbBTA252ParamsConfig.h>

using std::string;

/******************************
 * #defines
 *****************************/
#ifndef SBBTA252_STRING_SIZE
#define SBBTA252_STRING_SIZE 128
#endif // SBBTA252_STRING_SIZE

#ifndef SBBTA252_SERIAL_DELAY
#define SBBTA252_SERIAL_DELAY 100000
#endif // SBBTA252_SERIAL_DELAY

#ifndef SBBTA252_SC
#define SBBTA252_SC '$'
#endif // SBBTA252_SC

#ifndef SBBTA252_EC
#define SBBTA252_EC '!'
#endif // SBBTA252_EC

/******************************
 * Classes
 *****************************/

class SBBTA252 : public Serial
{
public:
  //! Constructor.
  //! \param _portname The name of the port that the thruster are connected to.
  //! \param _baud The baud rate that the thruster are configured to communicate at.
  //! \param _initTime The amount of time to try establishing communications with the thruster before timing out.
  SBBTA252(string portname, int baud, int init_time);

  //! Destructor.
  ~SBBTA252();

  //! Establish communications with the thruster.
  void setup();

  //! Looks for valid data from the thruster for a specified amount of time.
  void init(int thruster);
  
  //! Get status data from the thruster.
  bool getStatus();
  
  //! Callback function for "controlInputs" topic.
  void controlsCallback(const nav::ControlInputs::ConstPtr& msg);

  //! Callback function for dynamic reconfigure server.
  void configCallback(sbBTA252::sbBTA252ParamsConfig& config, uint32_t level);

  //! Publish the data from the thruster in our custom message format.
  void publishMessage(ros::Publisher *pub_thruster_data);

  //! Send the Speed Command to the thruster.
  void sendSpeedCommand(int thruster, int speed);
  
  //! Sends the command for to send status.
  void sendReadCommand(int thruster);
  
  //! Sends the command to change the given address to a new address.
  void sendChangeAddressCommand(int thruster, int new_address);

  //! The file descriptor used to communicate with the thruster.
  int fd;
  
  /// !!!!!!!!!!!!!!!!!!!!!!!!!!
  /// BEGIN GUI CONFIG VALUES !!
  /// !!!!!!!!!!!!!!!!!!!!!!!!!!
  
  int depth_frequency;

  //! The address of the left yaw thruster.
  int t_left_yaw;
  //! The address of the right yaw thruster.
  int t_right_yaw;
  //! The address of the left roll thruster.
  int t_left_roll;
  //! The address of the right roll thruster.
  int t_right_roll;
  //! The address of the pitch thruster.
  int t_pitch;

  //! The amount of time attempting to set up communications with the thruster.
  int init_time;
  //! Should we manually control the thrusters flag.
  bool manual;
  //! Thruster address to talk to.
  int  thrust_addr;
  //! Flag to change the address.
  bool change_addr;
  //! New address to change to.
  int new_addr;
  //! The speed of the thruster in percent.
  int speed_perc;
  //! Flag to get the status of this thruster.
  bool get_status;
  
  /// !!!!!!!!!!!!!!!!!!!!!!!!
  /// END GUI CONFIG VALUES !!
  /// !!!!!!!!!!!!!!!!!!!!!!!!

  //! Pointer to a timer.
  Timing *timer;
  //! The speed of the motor in RPMs.
  string speed;
  //! The current draw in Amps*10.
  string current;
  //! Motor winding temperature in Celsius.
  string motor_temp;
  //! Controller FET temperature in Celsius.
  string controller_temp;
  //! Controller voltage.
  string controller_volt;
  //! Water detect ADC reading. Values range from 511 (5V) to 0. Water detect fails at 475, GFI fails at 340.
  string water;
  //! Status byte reguarding motor control software.
  string status;
  
  //! Faults byte. Indicates the fault status of the thruster.
  //! Bit is set to indicate fault.
  //! Bit 0 – Overtemp
  //! Bit 1 – Stalled Motor
  //! Bit 2 – Hall Sensor Error
  //! Bit 3 – Ground Fault
  //! Bit 4 – Water Detect
  //! Bit 5 – (future growth)
  //! Bit 6 – (future growth)
  //! Bit 7 – (future growth)
  string faults;
  
  //! The checksum for the message.
  string checksum;

  //! Output command for roll.
  float u_roll;
  //! Output command for pitch.
  float u_pitch;
  //! Output command for yaw.
  float u_yaw;
  //! Output command for depth.
  float u_depth;
  //! Output command for surge.
  float u_surge;

private:
  //! Converts the thruster number to hex address.
  char getAddress(int bit, int thruster);
  
  //! Converst the decimal (0-15) to hex.
  char dec2hex(int dec);
  
  //! Converst the hex (0x00 - 0xFF) to decimal (0 - 255)
  int hex2dec(char hex[2]);
  
  //! Caculates the checksum in hex based on the command.
  char getChecksum(int bit, char *cmd, int cmd_length);
  
  //! Get the integer value encoded in str ignoring spaces.
  int getDecimal(char* str, int start, int end);
  
  //! Get the speed in hex based on speed in percent.
  char getSpeed(int bit, int speed);
  
  //! Searches a buffer looking for the start and end sequences.
  bool findMsg();

  //! Parses a message for thruster status.
  bool parseStatus();
  
  //! Whether the thruster was initialized correctly.
  bool b_thruster_initialized;
};

#endif // SR_SBBTA252_H
