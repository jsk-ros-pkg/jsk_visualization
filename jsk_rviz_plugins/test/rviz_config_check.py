#!/usr/bin/env python
###############################################################################
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, Kei Okada
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
###############################################################################

"""
Integration test node that checks if the rviz are working.
see example below

<test name="rviz_config_check"
      test-name="rviz_config_check"
      pkg="jsk_rviz_plugins" type="rviz_config_check.py">
  <rosparam>
    wait_time : 5
    topics:
      - name: another topic name
        timeout: timeout for the topic
  </rosparam>
</test>

Author: Kei Okada <kei.okada@gmail.com>
"""

import os
import sys
import time
import unittest

import rospy
import rosnode
import rosgraph
import rosservice


PKG = 'jsk_rviz_plugins'
NAME = 'rviz_config_check'


class RvizConfigCheck(unittest.TestCase):
    def __init__(self, *args):
        super(self.__class__, self).__init__(*args)
        rospy.init_node(NAME)
        # scrape rosparam
        # check rviz arive at least test_duration
        self.test_duration = float(rospy.get_param('~test_duration', 5.))
        # topics to check
        self.topics = {}
        params = rospy.get_param('~topics', [])
        for param in params:
            if 'name' not in param:
                self.fail("'name' field in rosparam is required but not specified.")
            topic = {'timeout': 10, 'type': None}  # this is not used
            topic.update(param)
            self.topics[topic['name']] = topic

    def setUp(self):
        # warn on /use_sim_time is true
        use_sim_time = rospy.get_param('/use_sim_time', False)
        t_start = time.time()
        while not rospy.is_shutdown() and \
                use_sim_time and (rospy.Time.now() == rospy.Time(0)):
            rospy.logwarn_throttle(
                1, '/use_sim_time is specified and rostime is 0, /clock is published?')
            if time.time() - t_start > 10:
                self.fail('Timed out (10s) of /clock publication.')
            # must use time.sleep because /clock isn't yet published, so rospy.sleep hangs.
            time.sleep(0.1)

        # Use `rosservice find rviz/SendFilePath` to get rviz node name
        rviz_service_names = []
        while not rospy.is_shutdown() and len(rviz_service_names) == 0:
            rviz_service_names = rosservice.rosservice_find('rviz/SendFilePath')
            if len(rviz_service_names) == 0:
                rospy.logwarn("[{}] Waiting rviz service (rviz/SendFilePath) for {:.3f} sec".format(rospy.get_name(), time.time() - t_start))
                # rviz/SendFilePath only available >melodic
                if os.environ['ROS_DISTRO'] < 'melodic':
                    rviz_service_names = rosservice.rosservice_find('std_srvs/Empty')
            time.sleep(0.1)

        # check if
        rospy.logwarn("[{}] Found rviz service names {}".format(rospy.get_name(), rviz_service_names))
        rviz_node_names = set(map(rosservice.get_service_node, rviz_service_names))
        rospy.logwarn("[{}] Found rviz node names {}".format(rospy.get_name(), rviz_node_names))
        if len(rviz_node_names) == 0:
            rospy.logerr("[{}] Could not find rviz nodes".format(rospy.get_name()))
            raise AssertionError

        self.rviz_node_name = list(rviz_node_names)[0]

    def test_rviz_exists(self):
        rospy.logwarn("[{}] Check rviz node exists {}".format(rospy.get_name(), self.rviz_node_name))
        subs = []
        t_start = time.time()
        while not rospy.is_shutdown():
            t_now = time.time()
            t_elapsed = t_now - t_start
            if t_elapsed > self.test_duration:
                break

            # info = rosnode.rosnode_info(self.rviz_node_name) does not return values, so we need to expand functions
            node_api = rosnode.rosnode_ping(self.rviz_node_name, max_count=1, skip_cache=True)
            if not node_api:
                rospy.logerr("[{}] Could not find rviz node api on rosmaster".format(rospy.get_name()))
                raise AssertionError

            rospy.logwarn("[{}] {:.3f} Rviz node found at {}".format(rospy.get_name(), t_elapsed, node_api))

            # check if topic exists
            master = rosgraph.Master('/rosnode')

            state = master.getSystemState()
            subs.extend(sorted([t for t, l in state[1] if self.rviz_node_name in l]))
            subs = list(set(subs))
            rospy.logwarn('[{}] rviz subscribes {}'.format(rospy.get_name(), subs))
            time.sleep(0.5)

        for topic in self.topics:
            if topic in subs:
                rospy.logwarn('[{}] rviz subscribes {}'.format(rospy.get_name(), topic))
            else:
                rospy.logerr('[{}] rviz did not subscribes {}'.format(rospy.get_name(), topic))
                raise AssertionError

        rospy.logwarn("[{}] rviz keep alive for {}[sec] and found {}".format(rospy.get_name(), self.test_duration, self.topics))
        self.assertTrue(True, "check {}/rviz alives".format(rospy.get_name()))


if __name__ == '__main__':
    import rostest
    rostest.run(PKG, NAME, RvizConfigCheck, sys.argv)
