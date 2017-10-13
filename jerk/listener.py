#!/usr/bin/env python
'''
@author: flg-ma
@attention: listener for given topic
@contact: albus.marcel@gmail.com (Marcel Albus)
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


import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import numpy as np
import time
from bcolors import TerminalColors as tc


class Sentence:
    def __init__(self):
        self.sentence = []
        self.sentence.append("Collecting data...")
        self.sentence.append("Waiting to finish...")
        self.sentence.append("Taking over the world...")
        self.sentence.append("Planting a tree...")
        self.sentence.append("Polluting the ocean...")
        self.sentence.append("Spinning up the hamster...")
        self.sentence.append("Shovelling coal into the server...")
        self.sentence.append("Programming the flux capacitor...")
        self.sentence.append("Hum something loud while others stare...")
        self.sentence.append("Take a moment to sign up for our lovely prizes...")
        self.sentence.append("Don\'t think of purple hippos...")
        self.sentence.append("Dig on the \'X\' for buried treasure... ARRR!")
        self.sentence.append(
            "Do you suffer from ADHD? Me neith- oh look a bunny... What was I doing again? Oh, right. Here we go.")
        self.sentence.append("Testing data on Timmy... ... ... We\'re going to need another Timmy.")
        self.sentence.append(
            "The last time I tried this the monkey didn't survive. Let's hope it works better this time.")
        self.sentence.append("Warming up Large Hadron Collider...")
        self.sentence.append("checking the gravitational constant in your locale...")

    def spin(self):
        return self.sentence[np.random.randint(0, len(self.sentence))]


class NodeListener:
    def __init__(self, topic='/base/odometry_controller/odometry'):
        self.topic = topic
        self.start_time = time.time()
        self.stop_time = None
        # create array for further use
        self.A_listener = np.ones([0, 8], dtype=np.float64)
        self.s = Sentence()

    def callback(self, data):
        # global data_list
        # rospy.loginfo(rospy.get_caller_id() + 'pos x: %s', data.pose.pose.position.x)
        # rospy.loginfo(rospy.get_caller_id() + 'pos y: %s', data.pose.pose.position.y)
        # rospy.loginfo(rospy.get_caller_id() + 'header stamp: %s', data.header.stamp)
        # rospy.loginfo(rospy.get_caller_id() + 'header seq: %s', data.header.seq)
        # rospy.loginfo(rospy.get_caller_id() + 'twist linear x: %s', data.twist.twist.linear.x)
        # rospy.loginfo(rospy.get_caller_id() + 'twist linear y: %s', data.twist.twist.linear.y)
        # rospy.loginfo(rospy.get_caller_id() + 'twist angular z:  %s', data.twist.twist.angular.z)
        data_list = [-1,  # no %time in odometry message
                     float(data.header.seq),
                     # precision of header.stamp: 3.3f (nsecs doesn't provide more than milliseconds
                     # example: sec: 121 [s], nsecs: 702000000 [ns] --> 121.702 [s]
                     (float(data.header.stamp.secs) * 10 ** 9 + float(data.header.stamp.nsecs)) * 10 ** -9,
                     float(data.twist.twist.linear.x),
                     float(data.twist.twist.linear.y),
                     float(data.twist.twist.angular.z),
                     float(data.pose.pose.position.x),
                     float(data.pose.pose.position.y)]

        # append data to array
        self.A_listener = np.append(self.A_listener, [[data_list[0], data_list[1], data_list[2], data_list[3],
                                                       data_list[4], data_list[5], data_list[6], data_list[7]]],
                                    axis=0)

        self.start_time = time.time()
        if self.A_listener.shape[0] in xrange(0, 100000, 25):
            print str(self.A_listener.shape) + ' ' + self.s.spin()

    def return_array(self):
        # deletes first row of array, because first row is only 1
        # return np.delete(A_listener, 0, 0)
        return self.A_listener

    def listener(self):
        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('listener', anonymous=True)

        print tc.OKBLUE + '=' * 98 + tc.ENDC
        print tc.OKBLUE + 'Use keyboard interrupt to kill \'listener.py\' after collecting sufficient data ' \
                          'to start evaluating!' + tc.ENDC
        print tc.OKBLUE + '=' * 98 + tc.ENDC

        # multiple options for Odometry msg type
        # Odometry.pose.pose.position.x
        # Odometry.pose.pose.position.y
        # Odometry.header.stamp
        # Odometry.header.seq
        # Odometry.twist.twist.linear.x
        # Odometry.twist.twist.linear.y
        # Odometry.twist.twist.angular.z
        rospy.Subscriber(self.topic, Odometry, self.callback)
        # while not rospy.is_shutdown():
        #     # check if idle time is too long and then shutdown node
        #     if time.time() - self.start_time >= 3:
        #         rospy.signal_shutdown('idle time too long')
        #     print 'Idle Time: %.2f' % (time.time() - self.start_time)
        #     rospy.sleep(0.25)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    if __name__ == '__main__':
        listener()
