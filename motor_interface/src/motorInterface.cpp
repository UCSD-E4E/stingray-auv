/*------------------------------------------------------------------------------
 *
 *  Title:        motorInterface.cpp
 *
 *  Description:  Sending and receiving data with to a motor controller through
 *		          a serial interface.
 *
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

#include "motorInterface.h"

/*------------------------------------------------------------------------------
 * MotorInterface()
 * Constructor.
 *----------------------------------------------------------------------------*/

MotorInterface::MotorInterface(string _portname, int _baud) : Serial::Serial(_portname, _baud)
{
    // Initialize variables.
    portname = _portname;
    baud = _baud;
} // end MotorInterface()


/*------------------------------------------------------------------------------
 * ~MotorInterface()
 * Destructor.
 *----------------------------------------------------------------------------*/

MotorInterface::~MotorInterface()
{
    // Close the open file descriptors.
    if (fd > 0)
    {
        close(fd);
    }
    if (Serial::fd > 0)
    {
        close(Serial::fd);
    }
} // end ~MotorInterface()


/*------------------------------------------------------------------------------
 * void SerialNumber()
 * Asks for serial number from IMU.
 *----------------------------------------------------------------------------*/

void MotorInterface::SpeedCommand(int command)
{
    //TODO, allow floating point (5 digits 000.00) commands
    //FOR NOW, use integers (3 digits 000) ranging from 0-900 because
    //current control code on PIC takes int valued speed commands

    // for parsing at the other end,
    // all commands start with 's' and end with '\n'


    // Declare variables.
    int BufLen = 5;  //three digits, one start char and one end char
    char cmd[BufLen];

    MotorInterface::Int2Char(command, cmd);
    Serial::bufSend = cmd;

    Serial::lengthSend = BufLen;
    Send();

    if (Serial::bytesSent > 0)
    {
        usleep(SERIAL_EXTRA_DELAY_LENGTH);
    }


} // end SpeedCommand()

/*------------------------------------------------------------------------------
 * char* Float2Char(float_t)()
 * Convert float to char array.
 *----------------------------------------------------------------------------*/

//char* MotorInterface::Float2Char(float_t) //Not Used for now
//{

//} // end Float2Char()

/*------------------------------------------------------------------------------
 * char* Int2Char(int, char*, int)()
 * Convert int to char array.
 *----------------------------------------------------------------------------*/

void MotorInterface::Int2Char(int command, char *cmd)
{

    int temp1 = 0; //for hundreds digit
    int temp2 = 0; //for tens digit
    int temp3 = 0; //for ones digit
    int remainder = 0; //for storing remainder in intermediate steps

    //Make sure command is in bounds
    if (command > MAXSPEED)
    {
        command = MAXSPEED;
    }

    if (command < MINSPEED)
    {
        command = MINSPEED;
    }

    //!Example: command = 123

    temp1 = command / 100 ; // get hundreds digit
    //! Now temp1 = 1

    remainder = command - (temp1 * 100); // get remainder
    //! remainder = 123 - 100 = 23

    temp2 = remainder / 10; // get tens digit
    //! temp2 = 2

    temp3 = remainder - (temp2 * 10);
    //! temp3 = 3

    //printf("Temp1 Temp2 Temp3 = (%d %d %d)", temp1, temp2, temp3);
    //Note: Using hex notation to keep consistent with PIC code

    //Always begin message with 's' character
    cmd[0] = 0x73; //0x73 = 's'

    //add 0x30 to convert ints to ASCII values
    cmd[1] = temp1 + 0x30;
    cmd[2] = temp2 + 0x30;
    cmd[3] = temp3 + 0x30;

    //Always end messages with newline
    cmd[4] = 0x0A; //0x0A = linefeed = '\n'

} // end Int2Char()

/*------------------------------------------------------------------------------
 * int Convert2Int()
 * Convert two adjacent bytes to an integer.
 *----------------------------------------------------------------------------*/

int MotorInterface::Convert2Int(char *buffer)
{
    int retval = (buffer[0] & LSB_MASK) * 256 + (buffer[1] & LSB_MASK);

    return retval;
} // end Convert2Int()


/*------------------------------------------------------------------------------
 * short Convert2Short()
 * Convert two adjacent bytes to a short.
 *----------------------------------------------------------------------------*/

short MotorInterface::Convert2Short(char *buffer)
{
    short retval = (buffer[0] & LSB_MASK) * 256 + (buffer[1] & LSB_MASK);

    return retval;
} // end Convert2Short()
