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
twist_keyop.py:
    A ros keyboard teleoperation script for Twist compatible robots
'''

__author__ = 'George Kouros'
__license__ = 'BSD-3'
__maintainer__ = 'George Kouros'
__email__ = 'george.kouros.ece@gmail.com'

import roslib
import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty
import thread
from numpy import clip

control_keys = {
    'up'    : '\x41',
    'down'  : '\x42',
    'right' : '\x43',
    'left'  : '\x44',
    'space' : '\x20',
    'tab'   : '\x09'}

key_bindings = {
    '\x41' : ( 1.0 , 0.0),
    '\x42' : (-1.0 , 0.0),
    '\x43' : ( 0.0 ,-1.0),
    '\x44' : ( 0.0 , 1.0),
    '\x20' : ( 0.0 , 0.0),
    '\x09' : ( 0.0 , 0.0)}


class TwistKeyop:

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

        for key in key_bindings:
            key_bindings[key] = (key_bindings[key][0] * float(max_linvel) / 5,
                    key_bindings[key][1] * float(max_angvel) / 5)

        self.linvel = 0
        self.angvel = 0
        self.twist_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=1)
        rospy.Timer(rospy.Duration(1.0/5.0), self.pub_callback, oneshot=False)
        self.print_state()
        self.key_loop()

    def pub_callback(self, event):
        twist = Twist()
        twist.linear.x = self.linvel
        twist.angular.z = self.angvel
        self.twist_pub.publish(twist)

    def print_state(self):
        sys.stderr.write('\x1b[2J\x1b[H')
        rospy.loginfo('\x1b[1M\r************* twist_keyop.py ***************')
        rospy.loginfo('\x1b[1M\rUse the arrows to change linvel and angvel')
        rospy.loginfo('\x1b[1M\rUse space and tab to clear linvel and angvel')
        rospy.loginfo('\x1b[1M\rPublishing to: ' + self.cmd_topic)
        rospy.loginfo('\x1b[1M\rlinvel range: ' + str(self.linvel_range))
        rospy.loginfo('\x1b[1M\rangvel range: ' + str(self.angvel_range))
        rospy.loginfo('\x1b[1M\rPress <ctrl-c> or <q> to exit')
        rospy.loginfo('\x1b[1M\r********************************************')
        rospy.loginfo('\x1b[1M\r'
                '\033[34;1mlinvel: \033[32;1m%0.2f m/s, '
                '\033[34;1mangvel: \033[32;1m%0.2f rad\033[0m',
                self.linvel, self.angvel)
        rospy.loginfo('\x1b[1M\r********************************************')

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key


    def key_loop(self):
        self.settings = termios.tcgetattr(sys.stdin)
        while 1:
            key = self.get_key()
            if key in key_bindings.keys():
                if key == control_keys['space']:
                    self.linvel = 0.0
                elif key == control_keys['tab']:
                    self.angvel = 0.0
                else:
                    self.linvel = self.linvel + key_bindings[key][0]
                    self.angvel = \
                            self.angvel + key_bindings[key][1]
                    self.linvel = clip(
                            self.linvel, self.linvel_range[0], self.linvel_range[1])
                    self.angvel = clip(
                            self.angvel,
                            self.angvel_range[0],
                            self.angvel_range[1])
                self.print_state()
            elif key == '\x03' or key == '\x71':  # ctr-c or q
                break
            else:
                continue
        self.finalize()

    def finalize(self):
        rospy.loginfo('\x1b[1M\rBraking and exiting...')
        self.settings = termios.tcgetattr(sys.stdin)
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        self.twist_pub.publish(twist)
        sys.exit()

if __name__ == '__main__':
    rospy.init_node('twist_keyop_node')
    keyop = TwistKeyop(sys.argv[1:len(sys.argv)])
