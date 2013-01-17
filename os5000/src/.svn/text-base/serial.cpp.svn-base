/*------------------------------------------------------------------------------
 *  Title:        serial.cpp
 *  Description:  Handles setting up serial connections and serial I/O.
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

#include "serial.h"

/*------------------------------------------------------------------------------
 * Serial()
 * Constructor.
 *----------------------------------------------------------------------------*/

Serial::Serial(string _portname, int _baud)
{
    baud     = _baud;
    portname = _portname;
    setup();
} // end Serial()


/*------------------------------------------------------------------------------
 * ~Serial()
 * Destructor.
 *----------------------------------------------------------------------------*/

Serial::~Serial()
{
} // end ~Serial()


/*------------------------------------------------------------------------------
 * void setup()
 * Sets up a serial port for communications.
 *----------------------------------------------------------------------------*/

void Serial::setup()
{
    // Declare variables.
    int status = -1;
    struct termios options;

    // Open port.
    fd = open(portname.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 1)
    {
        perror("open");
    }

    // Get the current options for the port.
    tcgetattr(fd, &options);

    // Set the baud rate.
    switch (baud)
    {
    case 1200:
        cfsetispeed(&options, B1200);
        cfsetospeed(&options, B1200);
        break;

    case 2400:
        cfsetispeed(&options, B2400);
        cfsetospeed(&options, B2400);
        break;

    case 4800:
        cfsetispeed(&options, B4800);
        cfsetospeed(&options, B4800);
        break;

    case 9600:
        cfsetispeed(&options, B9600);
        cfsetospeed(&options, B9600);
        break;

    case 19200:
        cfsetispeed(&options, B19200);
        cfsetospeed(&options, B19200);
        break;

    case 38400:
        cfsetispeed(&options, B38400);
        cfsetospeed(&options, B38400);
        break;

    case 57600:
        cfsetispeed(&options, B57600);
        cfsetospeed(&options, B57600);
        break;

    case 115200:
        cfsetispeed(&options, B115200);
        cfsetospeed(&options, B115200);
        break;

    case 230400:
        cfsetispeed(&options, B230400);
        cfsetospeed(&options, B230400);
        break;

    default: //Bad baud rate passed.
        close(fd);
        fd = -2;
        fprintf(stderr, "Serial::%s(). Bad baud rate passed.\n", __FUNCTION__);
    }

    // Set the number of data bits.
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    // Set the number of stop bits to 1.
    options.c_cflag &= ~CSTOPB;

    // Set parity to none.
    options.c_cflag &= ~PARENB;

    // Set for non-canonical (raw processing, no echo, etc.).
    options.c_iflag = IGNPAR; // ignore parity check
    options.c_oflag = 0; // raw output
    options.c_lflag = 0; // raw input

    // Time-Outs -- won't work with NDELAY option in the call to open.
    options.c_cc[VMIN]  = 0;  // Block reading until RX x characers. If x = 0, it is non-blocking.
    options.c_cc[VTIME] = 10; // Inter-Character Timer -- i.e. timeout= x*.1 s.

    // Set local mode and enable the receiver.
    options.c_cflag |= (CLOCAL | CREAD);

    // Purge the serial port buffer.
    tcflush(fd, TCIFLUSH);

    // Set the new options for the port.
    status = tcsetattr(fd, TCSANOW, &options);
    if (status != 0)
    {
        fprintf(stderr, "%s(). Failed to set up options.\n", __FUNCTION__);
        fd = -3;
    }
} // end setup()


/*------------------------------------------------------------------------------
 * void recv()
 * Get data from the serial port.
 *----------------------------------------------------------------------------*/

void Serial::recv()
{
    // Declare variables.
    int port_count = 0;
    struct timeval timeout;
    fd_set serial_fds;
    memset(&buf_recv, 0, sizeof(buf_recv));

    // Set the timeout values for receiving data so the port doesn't block.
    timeout.tv_sec = SERIAL_TIMEOUT_SECS;
    timeout.tv_usec = SERIAL_TIMEOUT_USECS;

    // Add the current port to the list of serial ports.
    FD_ZERO(&serial_fds);
    FD_SET(fd, &serial_fds);

    // Set up the port so it can be read from.
    port_count = select(SERIAL_MAX_PORTS, &serial_fds, NULL, NULL, &timeout);
    if ((port_count == 0) || (!FD_ISSET(fd, &serial_fds)))
    {
        bytes_recv = -1;
    }

    // Read the data from the port.
    bytes_recv = read(fd, buf_recv, length_recv);
    if (bytes_recv == -1)
    {
        perror("read");
    }
} // end recv()


/*------------------------------------------------------------------------------
 * void send()
 * Writes commands to serial port.
 *----------------------------------------------------------------------------*/

void Serial::send()
{
    // Send data out on the serial port.
    bytes_sent = write(fd, buf_send, length_send);

    // Flush the data in the port buffer.
    tcdrain(fd);

    if (bytes_sent < 0)
    {
        perror("write");
    }
} // end send()


/*------------------------------------------------------------------------------
 * void getBytesAvailable()
 * Checks to see how many bytes are available to read on the serial stack.
 *----------------------------------------------------------------------------*/

void Serial::getBytesAvailable()
{
    // Get the number of bytes in the serial port buffer.
    if (ioctl(fd, FIONREAD, &bytes_available) == -1)
    {
        perror("ioctl");
    }
} // end getBytesAvailable()
