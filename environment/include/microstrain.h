/**
 *  \file microstrain.h
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

#ifndef MICROSTRAIN_H
#define MICROSTRAIN_H

#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>

//#include "serial.h"


/******************************
 *
 * #defines
 *
 *****************************/

/* Command Set Summary for the MicroStrain 3DM-GX1 MSTRAIN. */
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
#endif /* MSTRAIN_COMMANDS */

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
#endif /* MSTRAIN_CMD_LENGTH */

/** @name Masks for moving data around. */
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

/** @name Bytes for setting and removing tare for coordinate system. */
//@{
#define MSTRAIN_TARE_BYTE1	0xC1
#define MSTRAIN_TARE_BYTE2	0xC3
#define MSTRAIN_TARE_BYTE3	0xC5
//@}

#ifndef MSTRAIN_SERIAL_DELAY
#define MSTRAIN_SERIAL_DELAY 40000
#endif /* MSTRAIN_SERIAL_DELAY */

#ifndef MSTRAIN_ERROR_HEADER
#define MSTRAIN_SUCCESS			1
#define MSTRAIN_ERROR_HEADER	-1
#define MSTRAIN_ERROR_CHECKSUM	-2
#define MSTRAIN_ERROR_LENGTH	-3
#endif /* MSTRAIN_ERROR_HEADER */

/** @name Serial number of Microstrain IMU. */
//@{
#ifndef MSTRAIN_SERIAL
//#define MSTRAIN_SERIAL 2104
#define MSTRAIN_SERIAL 1397
#endif /* MSTRAIN_SERIAL */
//@}

/******************************
 *
 * Data types
 *
 *****************************/

#ifndef _MSTRAIN_DATA_
#define _MSTRAIN_DATA_
/*! Struct to store data from the IMU. */
typedef struct
{
    int   serial_number;      //!< Serial number
    float temp;               //!< Temperature inside the IMU housing
    float ticks;              //!< Timer tick interval
    float mag[3];             //!< Magnetometer vector
    float accel[3];           //!< Acceleration vector
    float ang_rate[3];        //!< Angular rate vector
    float quat[4];            //!< Quaternion vector
    float transform[3][3];    //!< Transform matrix
    float orient[3][3];       //!< Orientation matrix
    float pitch;              //!< Pitch angle, from Euler angles
    float roll;               //!< Roll angle, from Euler angles
    float yaw;                //!< Yaw angle, from Euler angles
    short eeprom_address;     //!< EEPROM address
    short eeprom_value;       //!< EEPROM value
    short int accel_gain;	  //!< Accelerometer gain used by IMU filter
    short int mag_gain;		  //!< Magnetometer gain used by IMU filter
    short int bias_gain;	  //!< Bias gain used by IMU filter
} MSTRAIN_DATA;
#endif /* _MSTRAIN_DATA_ */


/******************************
 *
 * Function prototypes
 *
 *****************************/

//! Get the Euler angles from the IMU.
//! \param fd A file descriptor for the IMU port.
//! \param roll A pointer to store the roll value.
//! \param pitch A pointer to store the pitch value.
//! \param yaw A pointer to store the yaw value.
//! \return 1 on success, 0 on failure.
int mstrain_euler_angles(int fd, float *roll, float *pitch, float *yaw);

//! Get the serial number from the IMU.
//! \param fd A file descriptor for the IMU port.
//! \param serial_number A pointer to store the serial number value.
//! \return 1 on success, 0 on failure.
int mstrain_serial_number(int fd, int *serial_number);

//! Establish communications with the IMU.
//! \param portname The name of the port that the IMU is plugged into.
//! \param baud The baud rate to use for the IMU serial port.
//! \return A file descriptor for the Microstrain.
int mstrain_setup(const char *portname, int baud);

//! Get quaternions from the IMU.
//! \param fd A file descriptor for the IMU port.
//! \param gyro_stab Whether to use gyro-stabilized values or not.
//! \param quat A pointer to store the quaternion values.
//! \return 1 on success, 0 on failure.
int mstrain_quaternions(int fd, int gyro_stab, float *quat[4]);

//! Get the temperature inside the IMU housing.
//! \param fd A file descriptor for the IMU port.
//! \param temp A pointer to store the temperature value.
//! \return 1 on success, 0 on failure.
int mstrain_temperature(int fd, float *temp);

//! Get quaternions and accelerations from the IMU.
//! \param fd A file descriptor for the IMU port.
//! \param quat A pointer to store the quaternion values.
//! \param mag A pointer to store the magnetic values.
//! \param accel A pointer to store the acceleration values.
//! \param ang_rate A pointer to store the angular rate values.
//! \return 1 on success, 0 on failure.
int mstrain_quaternions_vectors(int fd, float *quat[4], float *mag[3], float *accel[3], float *ang_rate[3]);

//! Get Euler angles and accelerations from the IMU.
//! \param fd A file descriptor for the IMU port.
//! \param roll A pointer to store the roll value.
//! \param pitch A pointer to store the pitch value.
//! \param yaw A pointer to store the yaw value.
//! \param accel A pointer to store the acceleration values.
//! \param ang_rate A pointer to store the angular rate values.
//! \return 1 on success, 0 on failure.
int mstrain_euler_vectors(int fd, float *roll, float *pitch, float *yaw, float *accel, float *ang_rate);

//! Get gyro-stabilized orientation matrix from the IMU.
//! \param fd A file descriptor for the IMU port.
//! \param gyro_stab Whether to use gyro-stabilized values or not.
//! \param orient A pointer to store the orientation values.
//! \return 1 on success, 0 on failure.
int mstrain_orientation(int fd, int gyro_stab, float *orient[3][3]);

//! Sets tare for the Microstrain.
int mstrain_set_tare(int fd);

//! Removes tare for the Microstrain.
int mstrain_remove_tare(int fd);

//! Gets vectors from IMU.
//! \param fd A file descriptor for the IMU port.
//! \param gyro_stab Whether to use gyro-stabilized values or not.
//! \param mag A pointer to store the magnetic values.
//! \param accel A pointer to store the acceleration values.
//! \param ang_rate A pointer to store the angular rate values.
//! \return 1 on success, 0 on failure.
int mstrain_vectors(int fd, int gyro_stab, float *mag, float *accel, float *ang_rate);

//! Calculate the checksum for a message.
//! \param buffer The response buffer with the IMU message data.
//! \param length The size of the message.
//! \return 1 on success, error code on failure.
int mstrain_calc_checksum(char *buffer, int length);

//! Convert two adjacent bytes to an integer value.
//! Returns an integer value.
//! \param buffer Pointer to first byte.
//! \return Resulting integer.
int mstrain_convert2int(char *buffer);

//! Convert two adjacent bytes to a short value.
//! Returns a short integer value.
//! \param buffer Pointer to first byte.
//! \return Resulting short integer.
short mstrain_convert2short(char *buffer);

//! Get the Euler angles from the IMU.
//! \param fd A file descriptor for the IMU port.
//! \param accel_gain The gain value that the IMU is using for accelerometers.
//! \param mag_gain The gain value that the IMU is using for magnetometers.
//! \param bias_gain The gain value that the IMU is using for bias.
//! \return 1 on success, 0 on failure.
int mstrain_read_system_gains(int fd, short int *accel_gain, short int *mag_gain, short int *bias_gain);

//! Get the Euler angles from the IMU.
//! \param fd A file descriptor for the IMU port.
//! \param accel_gain The gain value to use for accelerometers.
//! \param mag_gain The gain value to use for magnetometers.
//! \param bias_gain The gain value to use for bias.
//! \return 1 on success, 0 on failure.
int mstrain_write_system_gains(int fd, short int accel_gain, short int mag_gain, short int bias_gain);

//! Zeroes out the manetometer gains.
//! \param fd A file descriptor for the IMU port.
//! \return 1 on success, 0 on failure.
int mstrain_zero_mag_gain(int fd);

//! Captures the gyro bias.
//! \param fd A file descriptor for the IMU port.
//! \return 1 on success, 0 on failure.
int mstrain_capture_gyro_bias(int fd);

//! Simulates IMU data when no Microstrain is available.
//! \param pitch Pointer to pitch state.
//! \param roll Pointer to roll state.
//! \param yaw Pointer to yaw state.
//! \param accel Pointer to acceleration state.
//! \param ang_rate Pointer to angular rate state.
//! \param tpitch Target pitch state.
//! \param troll Target roll state.
//! \param tyaw Target yaw state.
void mstrain_simulate(float *pitch, float *roll, float *yaw, float *accel, float *ang_rate, float tpitch, float troll, float tyaw);

#endif /* MICROSTRAIN_H */
