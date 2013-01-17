/*------------------------------------------------------------------------------
 *  Title:        os5000_core.cpp
 *  Description:  sending and receiving data with the Ocean Server compass. The
 * 		   		  API is described in the document
 *         		  OS5000_Compass_Manual.pdf. See
 *         		  <a href="http://www.ocean-server.com/download/OS5000_Compass_Manual.pdf">Ocean Server</a>
 *         		  for more details.
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

#include "os5000_core.h"

/*------------------------------------------------------------------------------
 * Compass()
 * Constructor.
 *----------------------------------------------------------------------------*/

Compass::Compass(string _portname, int _baud, int _rate, int _init_time) : Serial::Serial(_portname, _baud)
{
    // Initialize variables.
    baud      = _baud;
    init_time = _init_time;
    portname  = _portname;
    rate      = _rate;

    // Start a new timer.
    timer = new Timing(init_time);

    // Set up the compass.
    setup();
} // end Compass()


/*------------------------------------------------------------------------------
 * ~Compass()
 * Destructor.
 *----------------------------------------------------------------------------*/

Compass::~Compass()
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
    delete timer;
} // end ~Compass()


/*------------------------------------------------------------------------------
 * void setup()
 * Initializes communications with the Ocean Server compass. Sets up a file
 * descriptor for further communications.
 *----------------------------------------------------------------------------*/

void Compass::setup()
{
    // Declare variables.
    compass_initialized = false;

    // Check if the compass serial port was opened correctly.
    if (Serial::fd > 0)
    {
        // Set the message rate to control how often the compass will send out updates.
        setRate();

        // Initialize the timer.
        timer->set();

        // Check for valid compass data until the timer expires or valid data is found.
        while (!timer->checkExpired())
        {
            // Try to initialize the compass to make sure that we are actually getting valid data from it.
            init();
            if (compass_initialized)
            {
                break;
            }
        }

        // No valid compass data found so simulate compass data in the main loop.
        if (!compass_initialized)
        {
            // Seed the random number generator to use when simulating data.
            srand((unsigned int)time(NULL));
            Serial::fd = -1;
        }
    }
    else
    {
        // Seed the random number generator to use when simulating data.
        srand((unsigned int)time(NULL));
    }

    // Set the file descriptor.
    fd = Serial::fd;
} // end setup()


/*------------------------------------------------------------------------------
 * void getData()
 * Get orientation data from compass.
 *----------------------------------------------------------------------------*/

void Compass::getData()
{
    // Declare variables.
    int bytes_to_discard = 0;
    found_complete_message = false;

    // Get the number of bytes available on the serial port.
    getBytesAvailable();

    // Make sure we don't read too many bytes and overrun the buffer.
    if (bytes_available >= SERIAL_MAX_DATA)
    {
        bytes_available = SERIAL_MAX_DATA - 1;
    }
    if (bytes_available + strlen(buf_recv) >= SERIAL_MAX_DATA)
    {
        bytes_to_discard = bytes_available + strlen(buf_recv) - SERIAL_MAX_DATA - 1;
        memmove(buf_recv, &buf_recv[bytes_to_discard], bytes_to_discard);
    }

    // Read data off the serial port.
    if (bytes_available > 0)
    {
        recv();
        // Look for entire message.
        findMsg();
        if (found_complete_message)
        {
            // Parse the message.
            parseMsg();
        }
    }
} // end getData()


/*------------------------------------------------------------------------------
 * void setRate()
 * Set the rate at which messages are sent by the compass.
 *----------------------------------------------------------------------------*/

void Compass::setRate()
{
    // Declare variables.
    char cmd = 0;
    Serial::buf_send = &cmd;

    // send command.
    if (Serial::fd > 0)
    {
        // send out escape and R commands.
        cmd = OS5000_CMD_ESC;
        Serial::length_send = 1;
        send();
        usleep(OS5000_SERIAL_DELAY);
        cmd = OS5000_CMD_SET_RATE;
        length_send = 1;
        send();
        usleep(OS5000_SERIAL_DELAY);

        // Compute the rate command.
        if (rate <= 0)
        {
            cmd = '0';
            Serial::length_send = 1;
            send();
            usleep(OS5000_SERIAL_DELAY);
        }
        else if (rate >= 40)
        {
            cmd = '4';
            Serial::length_send = 1;
            send();
            usleep(OS5000_SERIAL_DELAY);
            cmd = '0';
            Serial::length_send = 1;
            send();
            usleep(OS5000_SERIAL_DELAY);
        }
        else if (rate >= 30)
        {
            cmd = '3';
            Serial::length_send = 1;
            send();
            usleep(OS5000_SERIAL_DELAY);
            cmd = '0';
            Serial::length_send = 1;
            send();
            usleep(OS5000_SERIAL_DELAY);
        }
        else if (rate >= 20)
        {
            cmd = '2';
            Serial::length_send = 1;
            send();
            usleep(OS5000_SERIAL_DELAY);
            cmd = '0';
            Serial::length_send = 1;
            send();
            usleep(OS5000_SERIAL_DELAY);
        }
        else if (rate >= 10)
        {
            cmd = '1';
            Serial::length_send = 1;
            send();
            usleep(OS5000_SERIAL_DELAY);
            cmd = rate + 38;
            Serial::length_send = 1;
            send();
            usleep(OS5000_SERIAL_DELAY);
        }
        else
        {
            cmd = rate + 48;
            Serial::length_send = 1;
            send();
            usleep(OS5000_SERIAL_DELAY);
        }
        cmd = OS5000_CMD_END;
        length_send = 1;
        send();
        usleep(OS5000_SERIAL_DELAY);
    }
} // end setRate()


/*------------------------------------------------------------------------------
 * void zeroDepth()
 * Zero the depth pressure sensor. This would be at the surface for an
 * underwater device. It set’s zero based on the current ATM pressure.
 *----------------------------------------------------------------------------*/

void Compass::zeroDepth()
{
	// Declare variables.
    char cmd = 0;
    Serial::buf_send = &cmd;

    // send command.
    if (Serial::fd > 0)
    {
        // send out escape and R commands.
        cmd = OS5000_CMD_ESC;
        Serial::length_send = 1;
        send();
        usleep(OS5000_SERIAL_DELAY);
        
        // send out zero depth command.
        cmd = OS5000_CMD_ZERO_DEPTH;
        length_send = 1;
        send();
        usleep(OS5000_SERIAL_DELAY);

		// send out the ending command.
		cmd = OS5000_CMD_END;
        length_send = 1;
        send();
        usleep(OS5000_SERIAL_DELAY);
        
	}
} // end zeroDepth()

/*------------------------------------------------------------------------------
 * void findMsg()
 * Searches a buffer looking for the start and end sequences.
 *----------------------------------------------------------------------------*/

void Compass::findMsg()
{
    // Declare variables.
    bool found_start = false;
    int i = 0;
    int msg_start_location = 0;
    found_complete_message = false;

    // Look for start character.
    for (i = 0; i < Serial::length_recv; i++)
    {
        if (found_start)
        {
            if (Serial::buf_recv[i] == OS5000_RESPONSE_END)
            {
                found_complete_message = true;
                found_start = false;
                // Place the last complete message at the beginning of the buffer and throw away any data before that.
                memmove(Serial::buf_recv, Serial::buf_recv + msg_start_location, Serial::length_recv - msg_start_location);
            }
        }
        // Look for start sequence. Don't assume that there is an end response before another start response.
        if (Serial::buf_recv[i] == OS5000_RESPONSE_START)
        {
            found_start = true;
            msg_start_location = i;
        }
    }
} // end findMsg()


/*------------------------------------------------------------------------------
 * void parseMsg()
 * Parses a message for compass data.
 *----------------------------------------------------------------------------*/

void Compass::parseMsg()
{
    // Declare variables.
    int i = 0;
    char *token;
    char *saveptr;
    char *delim = (char *)OS5000_DELIM;
    char *tokens[OS5000_STRING_SIZE];

    // Initialize variables.
    memset(tokens, 0, sizeof(tokens));

    // Split the incoming buffer up into tokens based on the delimiting characters.
    tokens[0] = strtok_r(Serial::buf_recv, delim, &saveptr);
    for (i = 1; i < Serial::length_recv; i++)
    {
        token = strtok_r(NULL, delim, &saveptr);
        if (token == NULL)
        {
            break;
        }
        tokens[i] = token;
    }

    // Store Euler angles, temperature, depth values.
    if (tokens[0] != NULL)
    {
        yaw = atof(tokens[0]);
    }
    if (tokens[1] != NULL)
    {
        pitch = atof(tokens[1]);
    }
    if (tokens[2] != NULL)
    {
        roll = atof(tokens[2]);
    }
    if (tokens[3] != NULL)
    {
        temperature = atof(tokens[3]);
    }
    if (tokens[4] != NULL)
    {
        depth = atof(tokens[4]);
    }
} // end parseMsg()


/*------------------------------------------------------------------------------
 * void simulateData()
 * Simulates compass data when no compass is available.
 *----------------------------------------------------------------------------*/

void Compass::simulateData()
{
    pitch        = pitch + rand() / (float)RAND_MAX;
    roll         = roll  + rand() / (float)RAND_MAX;
    yaw          = yaw   + rand() / (float)RAND_MAX;
    temperature  = 0     + rand() / (float)RAND_MAX;
    depth        = depth + rand() / (float)RAND_MAX;

    return;
} // end simulateData()


/*------------------------------------------------------------------------------
 * void init()
 * Looks for valid data from the compass.
 *----------------------------------------------------------------------------*/

void Compass::init()
{
    // Initialize variables.
    compass_initialized = false;

    // Get the number of bytes available on the serial port.
    tcflush(Serial::fd, TCIFLUSH);
    usleep(OS5000_SERIAL_DELAY);
    getBytesAvailable();

    // Read data off the serial port.
    if (Serial::bytes_available > 0)
    {
        Serial::length_recv = Serial::bytes_available;
        recv();
        // Look for entire message.
        findMsg();
        if (found_complete_message)
        {
            // Parse the message.
            parseMsg();
            compass_initialized = true;
        }
    }
} // end init()
