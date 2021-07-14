#!/usr/bin/env python
#
# Copyright 2015, 2016 Thomas Timm Andersen (original version)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy
import os
import sys

from hand_arm_cbt.traj_planner import TrajPlanner


def main():
    try:
        rospy.init_node("plan_trajectories", anonymous=True, disable_signals=True)

        node = TrajPlanner()
        node.plan_all()

        

    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__':
    if len(sys.argv) == 1 or len(sys.argv)==3:
        main()
    else:
        print('Use roslaunch and the associated "*.launch" file for this script')
