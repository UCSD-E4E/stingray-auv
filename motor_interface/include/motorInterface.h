/**
 *  \file motorInterface.h
 *  \brief Sending and receiving data with to a motor controller through
 * 		   a serial interface. Motor controller is currently a Microchip
 * 		   dsPIC30f6010A. Only message sent to motor is speed command
 * 		   sent as a char array beginning with 's' and ending with '\n'
 * 		   maximum motor command is 900, actual rotor speed tbd
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

#ifndef MOTORINTERFACE_H
#define MOTORINTERFACE_H

#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>

#include "serial_SR.h"


/******************************
 *
 * #defines
 *
 *****************************/
//@{
#ifndef LSB_MASK
#define LSB_MASK 0xFF
#endif /* LSB_MASK */

#ifndef MSB_MASK
#define MSB_MASK 0xFF00
#endif /* MSB_MASK */

#ifndef CHECKSUM_MASK
#define CHECKSUM_MASK 0xFFFF
#endif /* CHECKSUM_MASK */
//@}



#ifndef MOTORCONTROLLER_SERIAL_DELAY
#define MOTORCONTROLLER_SERIAL_DELAY 20000
#endif /* MOTORCONTROLLER_SERIAL_DELAY */

#ifndef MOTORCONTROLLER_ERROR_HEADER
#define MOTORCONTROLLER_SUCCESS			1
#define MOTORCONTROLLER_ERROR_HEADER	-1
#define MOTORCONTROLLER_ERROR_CHECKSUM	-2
#define MOTORCONTROLLER_ERROR_LENGTH	-3
#endif /* MOTORCONTROLLER_ERROR_HEADER */

#define MAXSPEED 900
#define MINSPEED 0



/******************************
 *
 * Classes
 *
 *****************************/

class MotorInterface : public Serial
{
public:
    //! Constructor.
    //! \param _portname The name of the port that the IMU is connected to.
    //! \param _baud The baud rate that the IMU is configured to communicate at.
    MotorInterface(string _portname, int _baud);

    //! Destructor.
    ~MotorInterface();

    //!Sends speed commands to motor controller
    void SpeedCommand(int);

    //! Convert floating point value to a char array
    //! \param floating point value to be converted
    //! \param integer length of resulting character array
    //! \return buffer Pointer to first byte.
    //char* Float2Char(float_t, int);

    //! Convert integer value to a char array
    //! \param integer value to be converted
    //! \param character array to store result
    //! \param integer length of resulting character array

    void Int2Char(int command, char *buffer);

    //! Convert two adjacent bytes to an integer value.
    //! \param buffer Pointer to first byte.
    //! \return Resulting integer.
    int Convert2Int(char *buffer);

    //! Convert two adjacent bytes to a short value.
    //! \param buffer Pointer to first byte.
    //! \return Resulting short integer.
    short Convert2Short(char *buffer);

};


#endif // MOTORINTERFACE_H
