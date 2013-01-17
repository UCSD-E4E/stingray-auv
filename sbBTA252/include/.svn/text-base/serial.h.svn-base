/**
 *  \file serial.h
 *  \brief Handles setting up serial connections and serial I/O.
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

#ifndef SR_SERIAL_H
#define SR_SERIAL_H

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <string>
#include <string.h>
#include <iostream>

using std::string;


/******************************
 *
 * #defines
 *
 *****************************/

/** @name Maximum number of serial ports to check for available data. */
#ifndef SERIAL_MAX_PORTS
#define SERIAL_MAX_PORTS 10
#endif // SERIAL_MAX_PORTS

/** @name Timeout in [s] to check for available serial ports. */
#ifndef SERIAL_TIMEOUT_SECS
#define SERIAL_TIMEOUT_SECS 0
#endif // SERIAL_TIMEOUT_SECS

/** @name Timeout in [us] to check for available serial ports. */
#ifndef SERIAL_TIMEOUT_USECS
#define SERIAL_TIMEOUT_USECS 1
#endif // SERIAL_TIMEOUT_USECS

/** @name The maximum amount of serial data that can be stored. */
#ifndef SERIAL_MAX_DATA
#define SERIAL_MAX_DATA 65535
#endif // SERIAL_MAX_DATA

#ifndef SERIAL_EXTRA_DELAY_LENGTH
#define SERIAL_EXTRA_DELAY_LENGTH 40000
#endif // SERIAL_EXTRA_DELAY_LENGTH


/******************************
 *
 * Classes
 *
 *****************************/

class Serial
{
public:
    //! Constructor.
    //! \param setup_baud The baud rate that the serial port should be opened at.
    //! \param setup_portname The name of the serial port.
    Serial(string _portname, int _baud);

    //! Destructor.
    ~Serial();

    //! Opens a serial port. If SERIAL_DEBUG is defined then open system call
    //! errors are printed to screen.
    virtual void setup();

    //! Writes commands to serial port. If SERIAL_DEBUG is defined then write
    //! system call errors are printed to screen.
    void send();

    //! Get data from the serial port. If SERIAL_DEBUG is defined then read system
    //! call errors are printed to screen.
    void recv();

    //! Checks to see how many bytes are available to read on the serial stack. If
    //! SERIAL_DEBUG is defined then the ioctl system call error is printed to the
    //! screen.
    void getBytesAvailable();

    //! The file descriptor to use for the port. Used when setting up the port
    //! and for sending/receiving data on the port.
    int fd;

    //! The name of the port to open.
    string portname;

    //! The baud rate to use with the port.
    int baud;

    //! The length of the message to send.
    int length_send;

    //! The expected length of the message to receive.
    int length_recv;

    //! The data to be sent out of the port.
    char *buf_send;

    //! The data to received on the port.
    char buf_recv[SERIAL_MAX_DATA];

    //! Number of bytes sent.
    int bytes_sent;

    //! Number of bytes received.
    int bytes_recv;

    //! Number of bytes available in serial port buffer.
    int bytes_available;
};

#endif // SR_SERIAL_H
