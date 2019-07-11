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

import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
import std_msgs.msg
import pressure_controller_ros.msg
from math import pi
import numpy as np

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

    
arm_client = None






def get_position():

    joint_states = rospy.wait_for_message("joint_states", JointState)
    joints_pos = joint_states.position
    print(joints_pos)
    print(np.rad2deg(joints_pos).tolist())


   
def main():
    global arm_client

    try:
        rospy.init_node("get_pos", anonymous=True, disable_signals=True)

        r=rospy.Rate(100)
        ur_script_pub = rospy.Publisher('/ur_driver/URScript', std_msgs.msg.String, queue_size=10)
        ur_script_pub.publish('set robotmode freedrive')
           
        parameters = rospy.get_param(None)
        index = str(parameters).find('prefix')
        if (index > 0):
            prefix = str(parameters)[index+len("prefix': '"):(index+len("prefix': '")+str(parameters)[index+len("prefix': '"):-1].find("'"))]
            for i, name in enumerate(JOINT_NAMES):
                JOINT_NAMES[i] = prefix + name
        
        while True:
            get_position()
            r.sleep()

        

    except KeyboardInterrupt:
        ur_script_pub.publish('set robotmode run')
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': main()
