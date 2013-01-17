#!/usr/bin/env python

#
#  Title:        transdec_state_machine.py
#  Description:  State machine for AUVSI competition at Transdec test pool.
#

#
#      Copyright (c) 2010 <iBotics -- www.sdibotics.org>
#      All rights reserved.
#
#      Redistribution and use in source and binary forms, with or without
#      modification, are permitted provided that the following conditions are
#      met:
#
#      * Redistributions of source code must retain the above copyright
#        notice, this list of conditions and the following disclaimer.
#      * Redistributions in binary form must reproduce the above
#        copyright notice, this list of conditions and the following disclaimer
#        in the documentation and/or other materials provided with the
#        distribution.
#      * Neither the name of the Stingray, iBotics nor the names of its
#        contributors may be used to endorse or promote products derived from
#        this software without specific prior written permission.
#
#      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#      "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#      LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#      A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#      OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#      SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#      LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#      DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#      THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#      (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#      OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

# Import required Python code.
import roslib
roslib.load_manifest('transdec')
import rospy
import sys
import smach
import smach_ros

# Gate task.
class Gate(smach.State):
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        # State machine stuff.
        smach.State.__init__(self, outcomes=['success', 'fail', 'found', 'lost', 'not found'])

    # Execute function is where all the state transitions occur.
    def execute(self, userdata):
        rospy.loginfo('Executing state GATE')
        return 'success'

# Pipe task.
class Pipe(smach.State):
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        # State machine stuff.
        smach.State.__init__(self, outcomes=['success', 'fail', 'found', 'centered', 'aligned'])

    # Execute function is where all the state transitions occur.
    def execute(self, userdata):
        rospy.loginfo('Executing state PIPE')
        return 'success'

# Buoy task.
class Buoy(smach.State):
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        # State machine stuff.
        smach.State.__init__(self, outcomes=['success', 'fail',
                                             'detect green', 'detect yellow', 'detect red',
                                             'touch green',  'touch yellow',  'touch red'])

    # Execute function is where all the state transitions occur.
    def execute(self, userdata):
        rospy.loginfo('Executing state BUOY')
        return 'success'

# Hedge task.
class Hedge(smach.State):
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        # State machine stuff.
        smach.State.__init__(self, outcomes=['success', 'fail', 'detect', 'align', 'center'])

    # Execute function is where all the state transitions occur.
    def execute(self, userdata):
        rospy.loginfo('Executing state HEDGE')
        return 'success'

# Boxes task.
class Boxes(smach.State):
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        # State machine stuff.
        smach.State.__init__(self, outcomes=['success', 'fail', 'center', 'align', 'depth', 'drop'])

    # Execute function is where all the state transitions occur.
    def execute(self, userdata):
        rospy.loginfo('Executing state BOXES')
        return 'success'

# Suitcase task.
class Suitcase(smach.State):
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        # State machine stuff.
        smach.State.__init__(self, outcomes=['success', 'fail', 'center', 'align', 'depth', 'grab'])

    # Execute function is where all the state transitions occur.
    def execute(self, userdata):
        rospy.loginfo('Executing state SUITCASE')
        return 'success'

# Surface task.
class Surface(smach.State):
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        # State machine stuff.
        smach.State.__init__(self, outcomes=['success', 'fail', 'depth'])

    # Execute function is where all the state transitions occur.
    def execute(self, userdata):
        rospy.loginfo('Executing state SURFACE')
        return 'success'

# Main function.    
def main():
    # Initialize the node and name it.
    rospy.init_node('transdec_state_machine')

    # Create a Smach state machine.
    sm = smach.StateMachine(outcomes = ['lose', 'win'])

    # Open the container.
    with sm:
        # Add states to the container.
        smach.StateMachine.add('GATE', Gate(),
                               transitions = {'success':'PIPE1',
                                              'fail':'lose',
                                              'found':'BUOY',
                                              'not found':'lose',
                                              'lost':'lose'})

        smach.StateMachine.add('PIPE1', Pipe(),
                               transitions = {'success':'BUOY',
                                              'fail':'GATE',
                                              'found':'GATE',
                                              'centered':'GATE',
                                              'aligned':'GATE'})

        smach.StateMachine.add('BUOY', Buoy(),
                               transitions = {'success':'PIPE2',
                                              'fail':'GATE',
                                              'detect green':'GATE',
                                              'detect yellow':'GATE',
                                              'detect red':'GATE',
                                              'touch green':'GATE',
                                              'touch yellow':'GATE',
                                              'touch red':'GATE'})

        smach.StateMachine.add('PIPE2', Pipe(),
                               transitions = {'success':'HEDGE',
                                              'fail':'GATE',
                                              'found':'GATE',
                                              'centered':'GATE',
                                              'aligned':'GATE'})

        smach.StateMachine.add('HEDGE', Hedge(),
                               transitions = {'success':'PIPE3',
                                              'fail':'GATE',
                                              'detect':'GATE',
                                              'align':'GATE',
                                              'center':'GATE'})

        smach.StateMachine.add('PIPE3', Pipe(),
                               transitions = {'success':'BOXES',
                                              'fail':'GATE',
                                              'found':'GATE',
                                              'centered':'GATE',
                                              'aligned':'GATE'})

        smach.StateMachine.add('BOXES', Boxes(),
                               transitions = {'success':'PIPE4',
                                              'fail':'GATE',
                                              'center':'GATE',
                                              'align':'GATE',
                                              'depth':'GATE',
                                              'drop':'GATE'})

        smach.StateMachine.add('PIPE4', Pipe(),
                               transitions = {'success':'SUITCASE',
                                              'fail':'GATE',
                                              'found':'GATE',
                                              'centered':'GATE',
                                              'aligned':'GATE'})

        smach.StateMachine.add('SUITCASE', Suitcase(),
                               transitions = {'success':'SURFACE',
                                              'fail':'GATE',
                                              'center':'GATE',
                                              'align':'GATE',
                                              'depth':'GATE',
                                              'grab':'GATE'})

        smach.StateMachine.add('SURFACE', Surface(),
                               transitions = {'success':'win',
                                              'fail':'GATE',
                                              'depth':'GATE'})

    # Execute the Smach plan.
    outcome = sm.execute()

    # Create an introspection server so a SM viewer can be used.
    sis = smach_ros.IntrospectionServer('stingray', sm, '/SM_TOP')
    sis.start()

    # Wait for a signal to stop the application.
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
