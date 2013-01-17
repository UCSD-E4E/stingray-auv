/*
 *
 *  Copyright (c) 2004-2005  Warren Jasper <wjasper@tx.ncsu.edu>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */


#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <asm/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

#include "pmd.h"
#include "usb-1208LS.h"

enum Mode {Differential, SingleEnded};

/* configures digital port */
void usbDConfigPort_USB1208LS(HIDInterface* hid, __u8 port, __u8 direction)
{
    struct t_report
    {
        __u8 cmd;
        __u8 port;
        __u8 direction;
        __u8 pad[5];
    } report;

    report.cmd = DCONFIG;
    report.port = port;
    report.direction = direction;

    PMD_SendOutputReport(hid, 0, (__u8*) &report, sizeof(report), LS_DELAY);
}

/* reads digital port  */
void usbDIn_USB1208LS(HIDInterface* hid, __u8 port, __u8* din_value)
{
    struct t_report
    {
        __u8 cmd;
        __u8 port;
        __u8 direction;
        __u8 pad[5];
    } report;

    struct t_in
    {
        __u8 value;
        __u8 pad[7];
    } in;

    report.cmd = DIN;
    report.port = port;

    PMD_SendOutputReport(hid, 0, (__u8*) &report, sizeof(report), LS_DELAY);
    PMD_GetInputReport(hid, 0, (__u8 *) &in, sizeof(in), LS_DELAY);
    *din_value = in.value;
}

/* writes digital port */
void usbDOut_USB1208LS(HIDInterface* hid, __u8 port, __u8 value)
{
    struct t_report
    {
        __u8 cmd;
        __u8 port;
        __u8 value;
        __u8 pad[5];
    } report;

    report.cmd = DOUT;
    report.port = port;
    report.value = value;

    PMD_SendOutputReport(hid, 0, (__u8*) &report, sizeof(report), LS_DELAY);
}

/* reads digital port bit */
__u8 usbDBitIn_USB1208LS(HIDInterface* hid, __u8 port, __u8 bit)
{
    struct t_report
    {
        __u8 cmd;
        __u8 port;
        __u8 bit;
        __u8 pad[5];
    } report;

    struct t_in
    {
        __u8 value;
        __u8 pad[7];
    } in;

    report.cmd = DBIT_IN;
    report.port = port;
    report.bit = bit;

    PMD_SendOutputReport(hid, 0, (__u8*) &report, sizeof(report), LS_DELAY);
    PMD_GetInputReport(hid, 0, (__u8*) &in, sizeof(in), LS_DELAY);

    return in.value;
}

/* writes digital port bit */
void usbDBitOut_USB1208LS(HIDInterface* hid, __u8 port, __u8 bit, __u8 value)
{
    struct t_report
    {
        __u8 cmd;
        __u8 port;
        __u8 bit;
        __u8 value;
        __u8 pad[4];
    } report;

    report.cmd = DBIT_OUT;
    report.port = port;
    report.bit = bit;
    report.value = value;

    PMD_SendOutputReport(hid, 0, (__u8*) &report, sizeof(report), LS_DELAY);
}

/* reads from analog in */
signed short usbAIn_USB1208LS(HIDInterface* hid, __u8 channel, __u8 range)
{
    enum Mode mode;

    struct t_report
    {
        __u8 cmd;
        __u8 channel;
        __u8 range;
        __u8 pad[5];
    } report;

    struct t_ain
    {
        __u8 lo_byte;
        __u8 hi_byte;
        __u8 pad[6];
    } ain;

    __s16 value;

    report.cmd = AIN;
    report.channel = channel;
    report.range = range;

    if ( range == SE_10_00V )
    {
        mode = SingleEnded;
    }
    else
    {
        mode = Differential;
    }

    if (channel > 3 && mode == Differential )
    {
        printf("usbAIN: channel out of range for differential mode.\n");
        return -1;
    }

    if (channel > 7 && mode == SingleEnded )
    {
        printf("usbAIN: channel out of range for single ended mode.\n");
        return -1;
    }

    PMD_SendOutputReport(hid, 0, (__u8*) &report, sizeof(report), LS_DELAY);
    PMD_GetInputReport(hid, 0, (__u8 *) &ain, sizeof(ain), LS_DELAY);

    if ( mode == Differential )
    {
        /* the data is a 2's compliment signed 12 bit number */
        value = (ain.hi_byte << 8) | (ain.lo_byte << 4);
        value /= (1 << 4);
    }
    else
    {
        /* the data is a  11 bit number signed offset */
        value = (ain.hi_byte << 4) | (0x0f & ain.lo_byte);
        value -= 0x400;
    }
    return value;
}

/* writes to analog out */
void usbAOut_USB1208LS(HIDInterface* hid, __u8 channel, __u16 value)
{
    struct t_report
    {
        __u8 cmd;
        __u8 channel;
        __u8 value_low;
        __u8 value_hi;
        __u8 pad[4];
    } report;

    report.cmd = AOUT;
    report.channel = channel;
    report.value_low = (__u8) (value & 0xff);
    report.value_hi = (__u8) ((value >> 8) & 0xff);
    PMD_SendOutputReport(hid, 0, (__u8*) &report, sizeof(report), LS_DELAY);
}

void usbAInScan_USB1208LS(HIDInterface* hid, __u16 count, int rate, __u8 low_channel, __u8 high_channel, __u8 options, __s16 value[], __u8 gainLoadQueue[])
{
    int i, idx;
    int scans;
    __u16 scan_index;
    __u16 actual_scan_index;
    __u8 chanCount;
    __u8 chanLoadQueue[8];

    struct t_out
    {
        __u8 cmd;
        __u8 lo_count;
        __u8 hi_count;
        __u8 timer_preload;
        __u8 timer_prescale;
        __u8 options;
        __u8 extra[2];
    } out;

    struct t_feature_report
    {
        __u8 data[96];
        __u8 error;
        __u8 readAddress[2];
        __u8 writeAddress[2];
        __u8 scanIndex[2];
        __u8 extra;
    } feature_report;

    __u8 buffer[9] = { 0, 0, 0, 0, 0, 0, 0, 0};

    int timerMult;
    __u8 timerVal;
    __u8 prescale;
    __u8 preload;
    __u8 setupTime;

    usbAInStop_USB1208LS(hid);   // just to make sure.

    if ((100 <= rate) && (rate < 200))         // Select 256:1 prescaler
    {
        prescale = 7;
        setupTime = 0;
    }
    else if ((200 <= rate) && (rate < 400))   // Select 128:1 prescaler
    {
        prescale = 6;
        setupTime = 0;
    }
    else if ((400 <= rate) && (rate < 800))   // Select 64:1 prescaler
    {
        prescale = 5;
        setupTime = 0;
    }
    else if ((800 <= rate) && (rate < 1500))   // Select 32:1 prescaler
    {
        prescale = 4;
        setupTime = 1;
    }
    else if ((1500 <= rate) && (rate < 3000))   // Select 16:1 prescaler
    {
        prescale = 3;
        setupTime = 3;
    }
    else if ((3000 <= rate) && (rate < 6000))   // Select 8:1 prescaler
    {
        prescale = 2;
        setupTime = 6;
    }
    else if ((6000 <= rate) && (rate < 8192))   // Select 4:1 prescaler
    {
        prescale = 1;
        setupTime = 10;
    }
    else
    {
        printf("usbAInScan_USB1208LS: sampling rate out of range.\n");
        return;
    }

    timerMult = 1 << (prescale + 1);
    timerVal = (__u8) ((256 - (MINILAB_CLOCK / (rate * timerMult))) + 0.5);
    preload = timerVal + setupTime;

    /* set up gain queue */
    chanCount = high_channel - low_channel + 1;
    for ( i = 0; i < chanCount; i++ )
    {
        chanLoadQueue[i] = low_channel + i;
    }
    usbAInLoadQueue_USB1208LS(hid, chanCount, chanLoadQueue, gainLoadQueue);

    out.cmd = AIN_SCAN;
    out.lo_count = count & 0xff;
    out.hi_count = (count >> 8) & 0xff;
    out.timer_preload = preload;
    out.timer_prescale = prescale;
    out.options = options;

    PMD_SendOutputReport(hid, 0, (__u8*) &out, sizeof(out), LS_DELAY);

    /*
       If in external trigger mode, then wait for the device to send back
       notice that the trigger has been received, then startup the acquisition
    */

    if ( options & AIN_TRIGGER )
    {
        while ( buffer[1] != 0xC3 )    // wait until external trigger received
        {
            buffer[0] = 0;
            PMD_GetInputReport(hid, 0, (__u8  *) &buffer, sizeof(buffer), LS_DELAY*10);
        }
    }

    /*
      In Burst Mode, wait for End of Block acquisition flag (0xA5)
    */

    if ( options & AIN_BURST_MODE )
    {
        buffer[1] = 0;
        while ( buffer[1] != 0xA5 )    // wait until external trigger received
        {
            buffer[0] = 0;
            PMD_GetInputReport(hid, 0, (__u8  *) &buffer, sizeof(buffer), LS_DELAY);
        }
    }

    /*
      Retrieve the AInScan Response
      GET HID Feature Report to collect the data buffer.  Each buffer will be 105
      bytes long.  The first bye will contain the record number and can be ignored.
      The following 96 bytes will represent 64 samples of data.
    */

    feature_report.scanIndex[0] = 0xff;
    feature_report.scanIndex[1] = 0xff;
    scan_index = 0;
    idx = 0;
    for ( scans = 0; scans < count / 65 + 1; scans++ )
    {
        memset(&feature_report, 0xbeef, sizeof(feature_report));
        do
        {
            PMD_GetFeatureReport(hid, 0, (__u8 *) &feature_report, sizeof(feature_report), 2000);
            actual_scan_index = (__u16) (feature_report.scanIndex[0] | feature_report.scanIndex[1] << 8);
        }
        while ( scan_index != actual_scan_index );
        scan_index++;
        printf("Completed scan %d  error = %d\n", scan_index, feature_report.error);
        for ( i = 0; i < 96; i += 3, idx += 2 )
        {
            printf("data[%d] = %#x  data[%d] = %#x\n", i, feature_report.data[i], i + 1, feature_report.data[i+1]);
            value[idx] = feature_report.data[i] | ((feature_report.data[i+1] << 4) & 0x0f00);
            value[idx + 1] = feature_report.data[i+2] | ((feature_report.data[i+1] << 8) & 0x0f00);
        }
    }
}

void usbAInLoadQueue_USB1208LS(HIDInterface* hid, __u8 chanCount, __u8 chanLoadQueue[], __u8 gainLoadQueue[])
{
    int i;
    struct t_report
    {
        __u8 cmd;
        __u8 count;
        __u8 gains[6];
    } report;

    report.cmd = ALOAD_QUEUE;
    report.count = chanCount;   // can be 1, 2, 4, or 8
    chanCount = (chanCount == 8) ? 6 : chanCount;
    for ( i = 0; i < chanCount; i++ )
    {
        report.gains[i] = (chanLoadQueue[i] & 0x7) | gainLoadQueue[i] | 0x80;
    }
    PMD_SendOutputReport(hid, 0, (__u8*) &report, sizeof(report), LS_DELAY);

    // Configure the rest of the channels (channel 6 and 7 )
    if ( report.count == 8 )
    {
        report.count = 0x2;
        for ( i = 6; i < 8; i++ )
        {
            report.gains[6-i] = (chanLoadQueue[i] & 0x7) | gainLoadQueue[i] | 0x80;
        }
        PMD_SendOutputReport(hid, 0, (__u8*) &report, sizeof(report), LS_DELAY);
    }
}

void usbAInStop_USB1208LS(HIDInterface* hid)
{
    struct t_report
    {
        __u8 cmd;
        __u8 pad[7];
    } report;

    report.cmd = AIN_STOP;

    PMD_SendOutputReport(hid, 0, (__u8*) &report, sizeof(report), LS_DELAY);
}

/* Initialize the counter */
void usbInitCounter_USB1208LS(HIDInterface* hid)
{
    struct t_report
    {
        __u8 cmd;
        __u8 pad[7];
    } report;

    report.cmd = CINIT;
    PMD_SendOutputReport(hid, 0, (__u8*) &report, sizeof(report), LS_DELAY);
}

__u32 usbReadCounter_USB1208LS(HIDInterface* hid)
{
    struct t_report
    {
        __u8 cmd;
        __u8 pad[7];
    } report;

    struct t_in
    {
        __u32 count;
        __u8 pad[4];
    } in;

    report.cmd = CIN;

    PMD_SendOutputReport(hid, 0, (__u8*) &report, sizeof(report), LS_DELAY);
    PMD_GetInputReport(hid, 0, (__u8  *) &in, sizeof(in), LS_DELAY);

    return in.count;
}

void usbReadMemory_USB1208LS(HIDInterface* hid, __u16 address, __u8 *buffer, __u8 count)
{
    struct t_report
    {
        __u8 cmd;
        __u8 address_low;
        __u8 address_hi;
        __u8 count;
        __u8 pad[4];
    } report;

    __u8 data[8];

    if (count > 8) count = 8;     // max count is 8.

    report.cmd = MEM_READ;
    report.address_low = (__u8) (address & 0xff);  // low byte
    report.address_hi = (__u8) (address >> 0x8);  // high byte
    report.count = count;

    PMD_SendOutputReport(hid, 0, (__u8*) &report, sizeof(report), LS_DELAY);
    PMD_GetInputReport(hid, 0, data, sizeof(data), LS_DELAY);
    memcpy(buffer, data, count);
}

/* blinks the LED of USB device */
void usbBlink_USB1208LS(HIDInterface* hid)
{
    struct t_report
    {
        __u8 cmd;
        __u8 pad[7];
    } report;

    report.cmd = BLINK_LED;
    PMD_SendOutputReport(hid, 0, (__u8*) &report, sizeof(report), LS_DELAY);
}

/* resets the USB device */
void usbReset_USB1208LS(HIDInterface* hid)
{
    struct t_report
    {
        __u8 cmd;
        __u8 pad[7];
    } report;

    report.cmd = RESET;
    PMD_SendOutputReport(hid, 0, (__u8*) &report, sizeof(report), LS_DELAY);
}

/* configure external triger */
void usbSetTrigger_USB1208LS(HIDInterface* hid, __u8 type, __u8 chan)
{
    struct t_report
    {
        __u8 cmd;
        __u8 type;
        __u8 chan;
        __u8 pad[5];
    } report;

    report.cmd = SET_TRIGGER;
    report.type = type;
    report.chan = chan;

    PMD_SendOutputReport(hid, 0, (__u8*) &report, sizeof(report), LS_DELAY);
}

__u8 usbGetID_USB1208LS(HIDInterface* hid)
{
    struct t_report
    {
        __u8 cmd;
        __u8 pad[7];
    } report;

    struct t_in
    {
        __u8 id;
        __u8 pad[7];
    } in;

    report.cmd = GET_ID;

    PMD_SendOutputReport(hid, 0, (__u8*) &report, sizeof(report), LS_DELAY);
    PMD_GetInputReport(hid, 0, (__u8*) &in, sizeof(in), LS_DELAY);

    return in.id;
}

void usbSetID_USB1208LS(HIDInterface* hid, __u8 id)
{
    struct t_report
    {
        __u8 cmd;
        __u8 id;
        __u8 pad[6];
    } report;

    report.cmd = SET_ID;
    report.id = id;

    PMD_SendOutputReport(hid, 0, (__u8*) &report, sizeof(report), LS_DELAY);
}

float volts_LS( const int gain, const signed short num )
{
    float volt = 0.0;

    switch ( gain )
    {
    case SE_10_00V:
        volt = num * 10.0 / 0x3ff;
        break;
    case BP_20_00V:
        volt = num * 20.0 / 0x7ff;
        break;
    case BP_10_00V:
        volt = num * 10.0 / 0x7ff;
        break;
    case BP_5_00V:
        volt = num * 5.0 / 0x7ff;
        break;
    case BP_4_00V:
        volt = num * 4.0 / 0x7ff;
        break;
    case BP_2_50V:
        volt = num * 2.5 / 0x7ff;
        break;
    case BP_2_00V:
        volt = num * 2.0 / 0x7ff;
        break;
    case BP_1_25V:
        volt = num * 1.25 / 0x7ff;
        break;
    case BP_1_00V:
        volt = num * 1.0 / 0x7ff;
        break;
    }
    return volt;
}
