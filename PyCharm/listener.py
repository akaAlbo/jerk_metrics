#!/usr/bin/env python
'''
@source: http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29
'''

# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published
## to the 'chatter' topic

import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import numpy as np


def callback(data):
    global data_list
    global A_listener
    # rospy.loginfo(rospy.get_caller_id() + 'pos x: %s', data.pose.pose.position.x)
    # rospy.loginfo(rospy.get_caller_id() + 'pos y: %s', data.pose.pose.position.y)
    # rospy.loginfo(rospy.get_caller_id() + 'header stamp: %s', data.header.stamp)
    # rospy.loginfo(rospy.get_caller_id() + 'header seq: %s', data.header.seq)
    # rospy.loginfo(rospy.get_caller_id() + 'twist linear x: %s', data.twist.twist.linear.x)
    # rospy.loginfo(rospy.get_caller_id() + 'twist linear y: %s', data.twist.twist.linear.y)
    # rospy.loginfo(rospy.get_caller_id() + 'twist angular z:  %s', data.twist.twist.angular.z)
    data_list = [data.header.seq,
                 data.header.stamp,
                 data.twist.twist.linear.x,
                 data.twist.twist.linear.y,
                 data.twist.twist.angular.z,
                 data.pose.pose.position.x,
                 data.pose.pose.position.y]

    # append data to array
    A_listener = np.append(A_listener, [[data_list[0], data_list[1], data_list[2], data_list[3],
                                        data_list[4], data_list[5], data_list[6]]], axis=0)

    # print 'Pos x: {} [float]'.format(data.pose.pose.position.x)
    print A_listener.shape
    # print 'Pos x: {} [Array]'.format(A_listener[-1, 0])


def return_array(self):
    return A_listener


def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    global A_listener
    # create array for further use
    A_listener = np.ones([1, 7], dtype=np.double)

    # multiple options for Odometry msg type
    # Odometry.pose.pose.position.x
    # Odometry.pose.pose.position.y
    # Odometry.header.stamp
    # Odometry.header.seq
    # Odometry.twist.twist.linear.x
    # Odometry.twist.twist.linear.y
    # Odometry.twist.twist.angular.z
    rospy.Subscriber('/base/odometry_controller/odometry', Odometry, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
