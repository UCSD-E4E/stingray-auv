/*------------------------------------------------------------------------------
 *
 *  Title:        ms3dmg_core.cpp
 *
 *  Description:  Sending and receiving data with the MicroStrain IMU. We are
 *                using model 3DM-GX1. The API is described in the document
 *                3DM_GX1_Data_Communication_Protocol_203101.pdf. See
 *                www.microstrain.com for more details.
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

#include "ms3dmg_core.h"

/*------------------------------------------------------------------------------
 * MS3dmgCore()
 * Constructor.
 *----------------------------------------------------------------------------*/

MS3dmgCore::MS3dmgCore(string _portname, int _baud, bool _use_gyro_stab) : Serial::Serial(_portname, _baud)
{
    // Initialize variables.
    baud      = _baud;
    portname  = _portname;
    gyro_stab = _use_gyro_stab;
} // end MS3dmgCore()


/*------------------------------------------------------------------------------
 * ~MS3dmgCore()
 * Destructor.
 *----------------------------------------------------------------------------*/

MS3dmgCore::~MS3dmgCore()
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
} // end ~MS3dmgCore()


/*------------------------------------------------------------------------------
 * void serialNumber()
 * Asks for serial number from IMU.
 *----------------------------------------------------------------------------*/

void MS3dmgCore::serialNumber()
{
    // Declare variables.
    int response_length = (int)MSTRAIN_LENGTH_F1;
    int bytes_to_discard = 0;
    char cmd = 0;
    Serial::buf_send = &cmd;

    // Send request to and receive data from IMU.
    cmd = (char)MSTRAIN_SERIAL_NUMBER;
    Serial::length_send = 1;
    send();

    if (Serial::bytes_sent > 0)
    {
        // Make sure we don't read too many bytes and overrun the buffer.
        usleep(SERIAL_EXTRA_DELAY_LENGTH);
        getBytesAvailable();
        if (bytes_available >= SERIAL_MAX_DATA)
        {
            bytes_available = SERIAL_MAX_DATA - 1;
        }
        if (bytes_available + strlen(buf_recv) >= SERIAL_MAX_DATA)
        {
            bytes_to_discard = bytes_available + strlen(buf_recv) - SERIAL_MAX_DATA - 1;
            memmove(buf_recv, &buf_recv[bytes_to_discard], bytes_to_discard);
        }
        Serial::length_recv = Serial::bytes_available;
        recv();
    }

    if (Serial::bytes_recv == response_length)
    {
        serial_number = convert2Int(&Serial::buf_recv[1]);
    }
} // end serialNumber()


/*------------------------------------------------------------------------------
 * void temperature()
 * Gets temperature from IMU.
 *----------------------------------------------------------------------------*/

void MS3dmgCore::temperature()
{
    // Declare variables.
    int response_length = (int)MSTRAIN_LENGTH_07;
    int bytes_to_discard = 0;
    char cmd = 0;
    Serial::buf_send = &cmd;

    // Send request to and receive data from IMU.
    cmd = (char)MSTRAIN_TEMPERATURE;
    Serial::length_send = 1;
    send();

    if (Serial::bytes_sent > 0)
    {
        // Make sure we don't read too many bytes and overrun the buffer.
        usleep(SERIAL_EXTRA_DELAY_LENGTH);
        getBytesAvailable();
        if (bytes_available >= SERIAL_MAX_DATA)
        {
            bytes_available = SERIAL_MAX_DATA - 1;
        }
        if (bytes_available + strlen(buf_recv) >= SERIAL_MAX_DATA)
        {
            bytes_to_discard = bytes_available + strlen(buf_recv) - SERIAL_MAX_DATA - 1;
            memmove(buf_recv, &buf_recv[bytes_to_discard], bytes_to_discard);
        }
        Serial::length_recv = Serial::bytes_available;
        recv();
    }

    if (Serial::bytes_recv == response_length)
    {
        temp = convert2Int(&Serial::buf_recv[1]);
        // This conversion is from the 3DM-GX1 manual.
        temp = (((temp * 5.0 / 65536.0) - 0.5) * 100.0);
    }
} // end temperature()


/*------------------------------------------------------------------------------
 * void orientation()
 * Asks for the gyro-stabilized orientation matrix from the IMU.
 *----------------------------------------------------------------------------*/

void MS3dmgCore::orientation()
{
    // Declare variables.
    int bytes_to_discard = 0;
    char cmd = 0;
    Serial::buf_send = &cmd;
    int response_length = 0;

    if (gyro_stab)
    {
        cmd = MSTRAIN_GYRO_STAB_ORIENT_MATRIX;
        response_length = (int)MSTRAIN_LENGTH_0B;
    }
    else
    {
        cmd = MSTRAIN_INST_ORIENT_MATRIX;
        response_length = (int)MSTRAIN_LENGTH_0A;
    }

    // Send request to and receive data from IMU.
    Serial::length_send = 1;
    send();

    if (Serial::bytes_sent > 0)
    {
        // Make sure we don't read too many bytes and overrun the buffer.
        usleep(SERIAL_EXTRA_DELAY_LENGTH);
        getBytesAvailable();
        if (bytes_available >= SERIAL_MAX_DATA)
        {
            bytes_available = SERIAL_MAX_DATA - 1;
        }
        if (bytes_available + strlen(buf_recv) >= SERIAL_MAX_DATA)
        {
            bytes_to_discard = bytes_available + strlen(buf_recv) - SERIAL_MAX_DATA - 1;
            memmove(buf_recv, &buf_recv[bytes_to_discard], bytes_to_discard);
        }
        Serial::length_recv = Serial::bytes_available;
        recv();
    }

    if (Serial::bytes_recv == response_length)
    {
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                orient[j][i] = (float)convert2Short(&Serial::buf_recv[1 + i * 2]) / MSTRAIN_CONVERSION_FACTOR_ORIENTATION;
            }
        }
    }
} // end orientation()


/*------------------------------------------------------------------------------
 * void vectors()
 * Asks for vectors from IMU.
 *----------------------------------------------------------------------------*/

void MS3dmgCore::vectors()
{
    // Declare variables.
    char cmd = 0;
    int bytes_to_discard = 0;
    int response_length = 0;
    Serial::buf_send = &cmd;

    if (gyro_stab)
    {
        cmd = MSTRAIN_GYRO_STAB_VECTORS;
        response_length = (int)MSTRAIN_LENGTH_02;
    }
    else
    {
        cmd = MSTRAIN_INST_VECTORS;
        response_length = (int)MSTRAIN_LENGTH_03;
    }

    // Send request to and receive data from IMU.
    Serial::length_send = 1;
    send();

    if (Serial::bytes_sent > 0)
    {
        // Make sure we don't read too many bytes and overrun the buffer.
        usleep(MSTRAIN_SERIAL_DELAY);
        getBytesAvailable();
        if (bytes_available >= SERIAL_MAX_DATA)
        {
            bytes_available = SERIAL_MAX_DATA - 1;
        }
        if (bytes_available + strlen(buf_recv) >= SERIAL_MAX_DATA)
        {
            bytes_to_discard = bytes_available + strlen(buf_recv) - SERIAL_MAX_DATA - 1;
            memmove(buf_recv, &buf_recv[bytes_to_discard], bytes_to_discard);
        }
        Serial::length_recv = Serial::bytes_available;
        recv();
    }

    if (Serial::bytes_recv == response_length)
    {
        for (int i = 0; i < 3; i++)
        {
            mag[i]      = (float)convert2Short(&Serial::buf_recv[1 + i * 2])  / MSTRAIN_CONVERSION_FACTOR_MAG;
            accel[i]    = (float)convert2Short(&Serial::buf_recv[7 + i * 2])  / MSTRAIN_CONVERSION_FACTOR_ACCEL;
            ang_rate[i] = (float)convert2Short(&Serial::buf_recv[13 + i * 2]) / MSTRAIN_CONVERSION_FACTOR_ANG_RATE;
        }
    }
} // end vectors()


/*------------------------------------------------------------------------------
 * void eulerAngles()
 * Asks for Euler angles from IMU.
 *----------------------------------------------------------------------------*/

void MS3dmgCore::eulerAngles()
{
    // Declare variables.
    int response_length = (int)MSTRAIN_LENGTH_0E;
    int bytes_to_discard = 0;
    char cmd = 0;
    Serial::buf_send = &cmd;

    // Send request to and receive data from IMU.
    cmd = (char)MSTRAIN_GYRO_STAB_EULER_ANGLES;
    Serial::length_send = 1;
    send();

    if (Serial::bytes_sent > 0)
    {
        // Make sure we don't read too many bytes and overrun the buffer.
        usleep(MSTRAIN_SERIAL_DELAY);
        getBytesAvailable();
        if (bytes_available >= SERIAL_MAX_DATA)
        {
            bytes_available = SERIAL_MAX_DATA - 1;
        }
        if (bytes_available + strlen(buf_recv) >= SERIAL_MAX_DATA)
        {
            bytes_to_discard = bytes_available + strlen(buf_recv) - SERIAL_MAX_DATA - 1;
            memmove(buf_recv, &buf_recv[bytes_to_discard], bytes_to_discard);
        }
        Serial::length_recv = Serial::bytes_available;
        recv();
    }

    if (Serial::bytes_recv == response_length)
    {
        // Convert bytes to short ints.
        roll  = convert2Short(&Serial::buf_recv[1]) * MSTRAIN_CONVERSION_FACTOR_EULER_ANGLES;
        pitch = convert2Short(&Serial::buf_recv[3]) * MSTRAIN_CONVERSION_FACTOR_EULER_ANGLES;
        yaw   = convert2Short(&Serial::buf_recv[5]) * MSTRAIN_CONVERSION_FACTOR_EULER_ANGLES;
    }
} // end eulerAngles()


/*------------------------------------------------------------------------------
 * void quaternions()
 * Asks for quaternions from IMU.
 *----------------------------------------------------------------------------*/

void MS3dmgCore::quaternions()
{
    // Declare variables.
    int response_length = 0;
    char cmd = 0;
    Serial::buf_send = &cmd;

    // Set the command.
    if (gyro_stab)
    {
        cmd = MSTRAIN_GYRO_STAB_QUAT;
        response_length = (int)MSTRAIN_LENGTH_05;
    }
    else
    {
        cmd = MSTRAIN_INST_QUAT;
        response_length = (int)MSTRAIN_LENGTH_04;
    }

    // Send request to and receive data from IMU.
    Serial::length_send = 1;
    send();
    if (Serial::bytes_sent > 0)
    {
        // Make sure we don't read too many bytes and overrun the buffer.
        usleep(MSTRAIN_SERIAL_DELAY);
        getBytesAvailable();
        if (bytes_available >= SERIAL_MAX_DATA)
        {
            bytes_available = SERIAL_MAX_DATA - 1;
        }
        if (bytes_available + strlen(buf_recv) >= SERIAL_MAX_DATA)
        {
            int bytes_to_discard = bytes_available + strlen(buf_recv) - SERIAL_MAX_DATA - 1;
            memmove(buf_recv, &buf_recv[bytes_to_discard], bytes_to_discard);
        }
        Serial::length_recv = Serial::bytes_available;
        recv();
    }

    if (bytes_recv == response_length)
    {
        for (int i = 0; i < 4; i++)
        {
            quat[i] = (float)convert2Short(&Serial::buf_recv[1 + i * 2]) / MSTRAIN_CONVERSION_FACTOR_QUATERNION;
        }
    }
} // end quaternions()


/*------------------------------------------------------------------------------
 * void quaternionsVectors()
 * Asks for quaternions and vectors from IMU.
 *----------------------------------------------------------------------------*/

void MS3dmgCore::quaternionsVectors()
{
    // Declare variables.
    int response_length = (int)MSTRAIN_LENGTH_0C;
    int bytes_to_discard = 0;
    char cmd = 0;

    // Send request to and receive data from IMU.
    Serial::buf_send = &cmd;
    cmd = MSTRAIN_GYRO_STAB_QUAT_VECTORS;
    Serial::length_send = 1;
    send();

    if (bytes_sent > 0)
    {
        // Make sure we don't read too many bytes and overrun the buffer.
        usleep(MSTRAIN_SERIAL_DELAY);
        getBytesAvailable();
        if (bytes_available >= SERIAL_MAX_DATA)
        {
            bytes_available = SERIAL_MAX_DATA - 1;
        }
        if (bytes_available + strlen(buf_recv) >= SERIAL_MAX_DATA)
        {
            bytes_to_discard = bytes_available + strlen(buf_recv) - SERIAL_MAX_DATA - 1;
            memmove(buf_recv, &buf_recv[bytes_to_discard], bytes_to_discard);
        }
        Serial::length_recv = Serial::bytes_available;
        recv();
    }

    if (bytes_recv == response_length)
    {
        for (int i = 0; i < 4; i++)
        {
            quat[i] = (float)convert2Short(&Serial::buf_recv[1 + i * 2]) / MSTRAIN_CONVERSION_FACTOR_QUATERNION;
        }

        for (int i = 0; i < 3; i++)
        {
            mag[i]      = (float)convert2Short(&Serial::buf_recv[9 + i * 2])  / MSTRAIN_CONVERSION_FACTOR_MAG;
            accel[i]    = (float)convert2Short(&Serial::buf_recv[15 + i * 2]) / MSTRAIN_CONVERSION_FACTOR_ACCEL;
            ang_rate[i] = (float)convert2Short(&Serial::buf_recv[21 + i * 2]) / MSTRAIN_CONVERSION_FACTOR_ANG_RATE;
        }
    }
} // end quaternionsVectors()


/*------------------------------------------------------------------------------
 * void eulerVectors()
 * Asks for Euler Angles and vectors from IMU.
 *----------------------------------------------------------------------------*/

void MS3dmgCore::eulerVectors()
{
    // Declare variables.
    int bytes_to_discard = 0;
    char cmd = 0;
    Serial::buf_send = &cmd;

    // Send request to and receive data from IMU.
    if (gyro_stab)
    {
        cmd = MSTRAIN_GYRO_STAB_EULER_VECTORS;
    }
    else
    {
        cmd = MSTRAIN_INST_EULER_ANGLES;
    }
    Serial::length_send = 1;
    send();

    if (Serial::bytes_sent > 0)
    {
        // Make sure we don't read too many bytes and overrun the buffer.
        usleep(MSTRAIN_SERIAL_DELAY);
        getBytesAvailable();
        if (bytes_available >= SERIAL_MAX_DATA)
        {
            bytes_available = SERIAL_MAX_DATA - 1;
        }
        if (bytes_available + strlen(buf_recv) >= SERIAL_MAX_DATA)
        {
            bytes_to_discard = bytes_available + strlen(buf_recv) - SERIAL_MAX_DATA - 1;
            memmove(buf_recv, &buf_recv[bytes_to_discard], bytes_to_discard);
        }
        Serial::length_recv = Serial::bytes_available;
        recv();
    }

    // Convert bytes to short ints and normalize Euler angles.
    roll  = convert2Short(&Serial::buf_recv[1]) * MSTRAIN_CONVERSION_FACTOR_EULER_ANGLES;
    pitch = convert2Short(&Serial::buf_recv[3]) * MSTRAIN_CONVERSION_FACTOR_EULER_ANGLES;
    yaw   = convert2Short(&Serial::buf_recv[5]) * MSTRAIN_CONVERSION_FACTOR_EULER_ANGLES;

    // Normalize acceleration and angular velocity values.
    for (int i = 0; i < 3; i++)
    {
        accel[i]    /= MSTRAIN_CONVERSION_FACTOR_ACCEL;
        ang_rate[i] /= MSTRAIN_CONVERSION_FACTOR_ANG_RATE;
    }
} // end eulerVectors()


/*------------------------------------------------------------------------------
 * int convert2Int()
 * Convert two adjacent bytes to an integer.
 *----------------------------------------------------------------------------*/

int MS3dmgCore::convert2Int(char *buffer)
{
    int retval = (buffer[0] & LSB_MASK) * 256 + (buffer[1] & LSB_MASK);

    return retval;
} // end convert2Int()


/*------------------------------------------------------------------------------
 * short convert2Short()
 * Convert two adjacent bytes to a short.
 *----------------------------------------------------------------------------*/

short MS3dmgCore::convert2Short(char *buffer)
{
    short retval = (buffer[0] & LSB_MASK) * 256 + (buffer[1] & LSB_MASK);

    return retval;
} // end convert2Short()
