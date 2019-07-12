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
import yaml
import os
import sys

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

    
class Teacher:
    def __init__(self, traj_file):
        self.ur_script_pub = rospy.Publisher('/ur_driver/URScript', std_msgs.msg.String, queue_size=10)
        self.r=rospy.Rate(100)

        self.traj_file = traj_file

        if self.traj_file is not None:
            self.joint_traj = []
        


    def set_freedrive_mode(self, enable_teach_mode=False):
            if enable_teach_mode:
                # send URScript command that will enable teach mode
                
                # get current position
                try:
                    while not rospy.is_shutdown():
                        self.ur_script_pub.publish('def myProg():\n\twhile (True):\n\t\tfreedrive_mode()\n\t\tsync()\n\tend\nend\n')
                        self.get_pos()
                        self.r.sleep()
                except KeyboardInterrupt:
                    self.ur_script_pub.publish('def myProg():\n\twhile (True):\n\t\tend_freedrive_mode()\n\t\tsleep(0.5)\n\tend\nend\n')
            else:
                # send URScript command that will disable teach mode
                self.ur_script_pub.publish('def myProg():\n\twhile (True):\n\t\tend_freedrive_mode()\n\t\tsleep(0.5)\n\tend\nend\n')
        



    def get_pos(self):
        joint_states = rospy.wait_for_message("joint_states", JointState)
        joints_pos = joint_states.position
        joint_pos_deg = np.rad2deg(joints_pos).tolist()

        out = {}
        out['joints_pos'] = list(joint_states.position)
        out['joints_vel'] = list(joint_states.velocity)
        out['time'] = joint_states.header.stamp.to_sec()

        print (["{0:0.2f}".format(i) for i in joint_pos_deg])

        if self.traj_file is not None:
            self.joint_traj.append(out)


    def save_file(self):
        print('saving file')
        if self.traj_file is not None:
            curr_path=os.path.dirname(os.path.abspath(__file__))
            outFile=os.path.join(curr_path,"..","trajectories","arm",self.traj_file+".yaml")


            dirname = os.path.dirname(outFile)
            if not os.path.exists(dirname):
                os.makedirs(dirname)

            
            with open(outFile, 'w') as f:
                yaml.dump(self.joint_traj, f, default_flow_style=None)




   
def main():
    try:
        rospy.init_node("get_pos", anonymous=True, disable_signals=True)
        file_name = rospy.get_param("~out_file_name",None)
        teach=Teacher("Testing")   
        teach.set_freedrive_mode(True)
        teach.save_file()

    except KeyboardInterrupt:
        
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': main()
