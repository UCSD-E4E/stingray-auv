/**
 *  \file ms3dmg_core.h
 *  \brief Sending and receiving data with the MicroStrain IMU. We are using
 *         model 3DM-GX1. The API is described in the document
 *         3DM_GX1_Data_Communication_Protocol_203101.pdf. See
 *         <a href="http://www.microstrain.com/3dm-gx1.aspx">Microstrain</a>
 *         for more details.
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

#ifndef MS3DMG_CORE_H
#define MS3DMG_CORE_H

#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>

#include "serial.h"


/******************************
 *
 * #defines
 *
 *****************************/

// Command Set Summary for the MicroStrain 3DM-GX1 MSTRAIN.
#ifndef MSTRAIN_COMMANDS
#define MSTRAIN_COMMANDS
/** @name Command values for the MSTRAIN. */
//@{
#define MSTRAIN_RAW_SENSOR                  0x01
#define MSTRAIN_GYRO_STAB_VECTORS           0x02
#define MSTRAIN_INST_VECTORS                0x03
#define MSTRAIN_INST_QUAT                   0x04
#define MSTRAIN_GYRO_STAB_QUAT              0x05
#define MSTRAIN_GYRO_BIAS                   0x06
#define MSTRAIN_TEMPERATURE                 0x07
#define MSTRAIN_READ_EEPROM                 0x08
#define MSTRAIN_WRITE_EEPROM                0x09
#define MSTRAIN_INST_ORIENT_MATRIX          0x0A
#define MSTRAIN_GYRO_STAB_ORIENT_MATRIX     0x0B
#define MSTRAIN_GYRO_STAB_QUAT_VECTORS      0x0C
#define MSTRAIN_INST_EULER_ANGLES           0x0D
#define MSTRAIN_GYRO_STAB_EULER_ANGLES      0x0E
#define MSTRAIN_TARE_COORDINATE_SYSTEM      0x0F
#define MSTRAIN_CONTINUOUS_MODE             0x10
#define MSTRAIN_REMOVE_TARE                 0x11
#define MSTRAIN_GYRO_STAB_QUAT_INST_VECTORS 0x12
#define MSTRAIN_WRITE_SYSTEM_GAINS          0x24
#define MSTRAIN_READ_SYSTEM_GAINS           0x25
#define MSTRAIN_SELF_TEST                   0x27
#define MSTRAIN_READ_EEPROM_CHECKSUM        0x28
#define MSTRAIN_WRITE_EEPROM_CHECKSUM       0x29
#define MSTRAIN_GYRO_STAB_EULER_VECTORS     0x31
#define MSTRAIN_INIT_HARD_IRON_CALIB        0x40
#define MSTRAIN_HARD_IRON_CALIB_DATA        0x41
#define MSTRAIN_HARD_IRON_CALIB             0x42
#define MSTRAIN_FIRMWARE_VERSION            0xF0
#define MSTRAIN_SERIAL_NUMBER               0xF1
//@}
#endif // MSTRAIN_COMMANDS

#ifndef MSTRAIN_CMD_LENGTH
#define MSTRAIN_CMD_LENGTH
/** @name Response lengths for the MSTRAIN messages. */
//@{
#define MSTRAIN_LENGTH_01 23
#define MSTRAIN_LENGTH_02 23
#define MSTRAIN_LENGTH_03 23
#define MSTRAIN_LENGTH_04 13
#define MSTRAIN_LENGTH_05 13
#define MSTRAIN_LENGTH_06  5
#define MSTRAIN_LENGTH_07  7
#define MSTRAIN_LENGTH_08  0
#define MSTRAIN_LENGTH_09  2
#define MSTRAIN_LENGTH_0A 23
#define MSTRAIN_LENGTH_0B 23
#define MSTRAIN_LENGTH_0C 31
#define MSTRAIN_LENGTH_0D 11
#define MSTRAIN_LENGTH_0E 11
#define MSTRAIN_LENGTH_0F  5
#define MSTRAIN_LENGTH_10  7
#define MSTRAIN_LENGTH_11  5
#define MSTRAIN_LENGTH_12 31
#define MSTRAIN_LENGTH_24_CMD  7
#define MSTRAIN_LENGTH_24_RSP  5
#define MSTRAIN_LENGTH_25 11
#define MSTRAIN_LENGTH_27  5
#define MSTRAIN_LENGTH_28  7
#define MSTRAIN_LENGTH_29  7
#define MSTRAIN_LENGTH_31 23
#define MSTRAIN_LENGTH_40  5
#define MSTRAIN_LENGTH_41 23
#define MSTRAIN_LENGTH_42 11
#define MSTRAIN_LENGTH_F0  5
#define MSTRAIN_LENGTH_F1  5
//@}
#endif // MSTRAIN_CMD_LENGTH

/** @name Masks for moving data around. */
//@{
#ifndef LSB_MASK
#define LSB_MASK 0xFF
#endif // LSB_MASK

#ifndef MSB_MASK
#define MSB_MASK 0xFF00
#endif // MSB_MASK

#ifndef CHECKSUM_MASK
#define CHECKSUM_MASK 0xFFFF
#endif // CHECKSUM_MASK
//@}

/** @name Bytes for setting and removing tare for coordinate system. */
//@{
#define MSTRAIN_TARE_BYTE1	0xC1
#define MSTRAIN_TARE_BYTE2	0xC3
#define MSTRAIN_TARE_BYTE3	0xC5
//@}

#ifndef MSTRAIN_SERIAL_DELAY
#define MSTRAIN_SERIAL_DELAY 40000
#endif // MSTRAIN_SERIAL_DELAY

#ifndef MSTRAIN_ERROR_HEADER
#define MSTRAIN_SUCCESS			1
#define MSTRAIN_ERROR_HEADER	-1
#define MSTRAIN_ERROR_CHECKSUM	-2
#define MSTRAIN_ERROR_LENGTH	-3
#endif // MSTRAIN_ERROR_HEADER

/** @name Serial number of Microstrain IMU. */
//@{
#ifndef MSTRAIN_SERIAL
//#define MSTRAIN_SERIAL 2104
#define MSTRAIN_SERIAL 1397
#endif // MSTRAIN_SERIAL
//@}

/** @name Conversion factors from Microstrain manual. */
//@{
#ifndef MSTRAIN_CONVERSION_FACTORS
#define MSTRAIN_CONVERSION_FACTORS 1
#define MSTRAIN_CONVERSION_FACTOR_QUATERNION 8192.0
#define MSTRAIN_CONVERSION_FACTOR_ORIENTATION 8192.0
#define MSTRAIN_CONVERSION_FACTOR_STAB 8192.0
#define MSTRAIN_CONVERSION_FACTOR_EULER_ANGLES (360.0 / 65536.0)
#define MSTRAIN_CONVERSION_FACTOR_ACCEL (3276800.0 / 7000.0)
#define MSTRAIN_CONVERSION_FACTOR_ANG_RATE (3276800.0 / 8500.0)
#define MSTRAIN_CONVERSION_FACTOR_MAG (3276800.0 / 2000.0)
#endif // MSTRAIN_CONVERSION_FACTORS
//@}

/******************************
 *
 * Classes
 *
 *****************************/

class MS3dmgCore : public Serial
{
public:
    //! Constructor.
    //! \param _portname The name of the port that the IMU is connected to.
    //! \param _baud The baud rate that the IMU is configured to communicate at.
    //! \param _use_gyro_stab Whether to use gyro-stablilized values (true)
    //! or instantaneous values (false).
    MS3dmgCore(string _portname, int _baud, bool _use_gyro_stab);

    //! Destructor.
    ~MS3dmgCore();

    //! Get the Euler angles from the IMU.
    void eulerAngles();

    //! Get the serial number from the IMU.
    void serialNumber();

    //! Get quaternions from the IMU.
    void quaternions();

    //! Get the temperature inside the IMU housing.
    void temperature();

    //! Get quaternions and accelerations from the IMU.
    void quaternionsVectors();

    //! Get Euler angles and accelerations from the IMU.
    void eulerVectors();

    //! Get gyro-stabilized orientation matrix from the IMU.
    void orientation();

    //! Gets vectors from IMU.
    void vectors();

    //! Convert two adjacent bytes to an integer value.
    //! \param buffer Pointer to first byte.
    //! \return Resulting integer.
    int convert2Int(char *buffer);

    //! Convert two adjacent bytes to a short value.
    //! \param buffer Pointer to first byte.
    //! \return Resulting short integer.
    short convert2Short(char *buffer);

    //! Serial number.
    int serial_number;
    //! Temperature inside the IMU housing.
    float temp;
    //! Timer tick interval.
    float ticks;
    //! Magnetometer vector.
    float mag[3];
    //! Acceleration vector.
    float accel[3];
    //! Angular rate vector.
    float ang_rate[3];
    //! Quaternion vector.
    float quat[4];
    //! Transform matrix.
    float transform[3][3];
    //! Orientation matrix.
    float orient[3][3];
    //! Pitch angle, from Euler angles, in degrees (-180, 180].
    float pitch;
    //! Roll angle, from Euler angles, in degrees (-180, 180].
    float roll;
    //! Yaw angle, from Euler angles, in degrees (-180, 180].
    float yaw;
    //! Whether to use gyro-stabilized values or not.
    bool gyro_stab;
};


#endif // MS3DMG_CORE_H
