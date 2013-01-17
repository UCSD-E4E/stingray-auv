/**
 *  \file timing.h
 *  \brief Functions for timers and checking to see if an amount of
 * 		   time has passed.
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

#include <stdio.h>
#include <time.h>
#include <sys/time.h>

#ifndef SR_TIMING_H
#define SR_TIMING_H

/******************************
 *
 * #defines
 *
 *****************************/

#ifndef TIMING_RETURN_VALS
#define TIMING_SUCCESS			1
#define TIMING_ERROR			0
#endif // TIMING_RETURN_VALS


/******************************
 *
 * Classes
 *
 *****************************/

class Timing
{
public:
    //! Constructor.
    //! \param setup_baud The baud rate that the serial port should be opened at.
    //! \param setup_portname The name of the serial port.
    Timing(float _period);

    //! Destructor.
    ~Timing();

    //! Initializes a timer.
    void setup();

    //! Checks to see if time has elapsed.
    //! \return true if period has passed, else false.
    bool checkExpired();

    //! Sets a timer to the current system time.
    void set();

    //! Computes elapsed time for a given timer.
    //! \return The elapsed time in microseconds.
    int getDt();

    //! Get the time elapsed for a given timer in seconds.
    //! \return The time elapsed in seconds as a float.
    float getDtS();

    //! Time in microseconds.
    int us;

    //! Time in seconds.
    int s;

    //! Period before timer expires, in seconds.
    float period;

    //! Whether timer has expired or not.
    bool expired;
};

#endif /* SR_TIMING_H */
