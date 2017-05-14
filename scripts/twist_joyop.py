#!/usr/bin/env python
# -*- coding: utf-8 -*-

#  Software License Agreement (BSD 3-Clause License)
#
#  Copyright (c) 2017, George Kouros
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#  * Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
#  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
#  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
#  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
#  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

'''
twist_joyop.py:
    A ros joystick teleoperation script for Twist compatible robots
'''

__author__ = 'George Kouros'
__license__ = 'BSD-3'
__maintainer__ = 'George Kouros'
__email__ = 'george.kouros.ece@gmail.com'

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import sys

class TwistJoyop:

    def __init__(self, args):
        self.cmd_topic = '/cmd_vel'
        if len(args) == 0:
            max_linvel = 0.5
            max_angvel = 1.0
        elif len(args) == 1:
            max_linvel = float(args[0])
            max_angvel = float(args[0])
        elif len(args) >= 2:
            max_linvel = float(args[0])
            max_angvel = float(args[1])
            if len(args) > 2:
                self.cmd_topic = args[2]

        self.linvel_range = [-float(max_linvel), float(max_linvel)]
        self.angvel_range = [-float(max_angvel), float(max_angvel)]

        self.linvel = 0
        self.angvel = 0
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.twist_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=1)
        rospy.Timer(rospy.Duration(1.0/5.0), self.pub_callback, oneshot=False)

    def joy_callback(self, joy_msg):
        self.linvel = joy_msg.axes[2] * self.max_linvel;
        self.angvel = joy_msg.axes[3] * self.max_angvel;


    def pub_callback(self, event):
        twist = Twist()
        twist.linear.x = self.linvel
        twist.angular.z = self.angvel
        self.twist_pub.publish(twist)
        self.print_state()

    def print_state(self):
        sys.stderr.write('\x1b[2J\x1b[H')
        rospy.loginfo('\x1b[1M\r************* twist_keyop.py ***************')
        rospy.loginfo('\x1b[1M\rUse the arrows to change linvel and angvel')
        rospy.loginfo('\x1b[1M\rUse space and tab to clear linvel and angvel')
        rospy.loginfo('\x1b[1M\rPublishing to: ' + self.cmd_topic)
        rospy.loginfo('\x1b[1M\rlinvel range: ' + str(self.linvel_range))
        rospy.loginfo('\x1b[1M\rangvel range: ' + str(self.angvel_range))
        rospy.loginfo('\x1b[1M\rPress <ctrl-c> to exit')
        rospy.loginfo('\x1b[1M\r********************************************')
        rospy.loginfo('\x1b[1M\r'
                '\033[34;1mlinvel: \033[32;1m%0.2f m/s, '
                '\033[34;1mSteer Angle: \033[32;1m%0.2f rad\033[0m',
                self.linvel, self.angvel)
        rospy.loginfo('\x1b[1M\r********************************************')

    def finalize(self):
        rospy.loginfo('\x1b[1M\rBraking and exiting...')
        self.settings = termios.tcgetattr(sys.stdin)
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        self.twist_pub.publish(twist)
        sys.exit()

if __name__ == '__main__':
    rospy.init_node('twist_joyop_node')
    joyop = TwistJoyop(sys.argv[1:len(sys.argv)])
    rospy.loginfo(str(sys.argv))
    rospy.spin()
