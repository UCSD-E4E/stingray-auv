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

#include "usb1208ls_node.h"
/*------------------------------------------------------------------------------
 * Usb1208ls_node()
 * Constructor.
 *----------------------------------------------------------------------------*/

Usb1208ls_node::Usb1208ls_node()
{
    this->loadTorque = 1023; //for testing, change to zero when publisher is created
    this->primePowerStartStop = false;
    this->hid = 0x0;

    //Note channel 0 = pin 13 on the board, channel 1 = pin 14
    this->input = 0;
    this->pin = 0;
    this->channel = 0;
    this->gain = 0;


} // end Usb1208ls_node()


/*------------------------------------------------------------------------------
 * ~Usb1208ls_node()
 * Destructor.
 *----------------------------------------------------------------------------*/

Usb1208ls_node::~Usb1208ls_node()
{
} // end ~Usb1208ls_node()

void Usb1208ls_node::LoadTorqueControlCallback(const stateEstimator::LoadTorque::ConstPtr& msg)
{
    loadTorque = (__u16)msg->loadTorque;

}

void Usb1208ls_node::PrimePowerControlCallback(const stateEstimator::PrimePowerStartStop::ConstPtr& msg)
{
    primePowerStartStop = msg->primePowerStartStop;

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "usb1208ls_node");
    ros::NodeHandle n;
    ros::NodeHandle private_node_handle_("~");
    ros::Rate r(20);

    //SETUP CALLBACKS!!!

    // Declare variables.
    int interface;



    // Instantiate Class
    Usb1208ls_node *usb1208ls_node;
    usb1208ls_node = new Usb1208ls_node();

    //Set up USB Connection
    usb1208ls_node->ret = hid_init();
    if (usb1208ls_node->ret != HID_RET_SUCCESS)
    {
        fprintf(stderr, "hid_init failed with return code %d\n", usb1208ls_node->ret);
        return -1;
    }

    if ((interface = PMD_Find_Interface(&usb1208ls_node->hid, 0, USB1208LS_PID)) < 0)
    {
        fprintf(stderr, "USB 1208LS not found.\n");
        exit(1);
    }
    else
    {
        printf("USB 208LS Device is found! interface = %d\n", interface);
    }

    /* config mask 0x01 means all inputs */
    usbDConfigPort_USB1208LS(usb1208ls_node->hid, DIO_PORTB, DIO_DIR_IN);
    usbDConfigPort_USB1208LS(usb1208ls_node->hid, DIO_PORTA, DIO_DIR_OUT);
    usbDOut_USB1208LS(usb1208ls_node->hid, DIO_PORTA, 0x0);
    usbDOut_USB1208LS(usb1208ls_node->hid, DIO_PORTA, 0x0);

    ros::Subscriber loadTorqueControl_sub = n.subscribe("loadTorqueControlData", 1000, &Usb1208ls_node::LoadTorqueControlCallback, usb1208ls_node);

    // Main loop.
    while (n.ok())
    {
        if (usb1208ls_node->loadTorque > 0x3ff)
        {
            usb1208ls_node->loadTorque = 0x3ff;
        }
        usbAOut_USB1208LS(usb1208ls_node->hid, usb1208ls_node->channel, usb1208ls_node->loadTorque);

        //Put in Digital IO for starting diesel engine

        ros::spinOnce();
        r.sleep();
    }

    return 0;
} // end main()

