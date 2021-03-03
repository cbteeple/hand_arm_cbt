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

from __future__ import print_function

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
from pynput.keyboard import Key, Listener

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

    
class Teacher:
    def __init__(self, traj_file):
        self.ur_script_pub = rospy.Publisher('/ur_driver/URScript', std_msgs.msg.String, queue_size=10)
        self.r=rospy.Rate(20)
        self.toggle_record = False

        self.traj_file = traj_file

        if self.traj_file is not None:
            self.joint_traj = []
        else:
            print("Starting Teach Mode, but not saving a file")            

        


    def set_freedrive_mode(self, enable_teach_mode=False):
            if enable_teach_mode:
                # send URScript command that will enable teach mode
                
                # get current position
                try:
                    print("Starting Freedrive Mode: You can now move the robot by hand!")
                    while not rospy.is_shutdown():
                        self.ur_script_pub.publish('def myProg():\n\twhile (True):\n\t\tfreedrive_mode()\n\t\tsync()\n\tend\nend\n')

                        if self.toggle_record:
                            self.get_pos()

                        self.r.sleep()
                except KeyboardInterrupt:
                    print("\nExiting Freedrive Mode: The robot is now locked in place.")
                    self.ur_script_pub.publish('def myProg():\n\twhile (True):\n\t\tend_freedrive_mode()\n\t\tsleep(0.5)\n\tend\nend\n')
            else:
                # send URScript command that will disable teach mode
                print("\nExiting Freedrive Mode: The robot is now locked in place.")
                self.ur_script_pub.publish('def myProg():\n\twhile (True):\n\t\tend_freedrive_mode()\n\t\tsleep(0.5)\n\tend\nend\n')
        



    def get_pos(self,time=None):
        joint_states = rospy.wait_for_message("joint_states", JointState)
        joints_pos = joint_states.position
        joint_pos_deg = np.rad2deg(joints_pos).tolist()

        out = {}
        out['joints_pos'] = list(joint_states.position)
        out['joints_vel'] = list(joint_states.velocity)

        if time is None:
            out['time'] = joint_states.header.stamp.to_sec()
        else:
            print(len(self.joint_traj))
            if len(self.joint_traj) ==0:
                out['time'] = 0.0
            else:
                out['time'] = self.joint_traj[-1]['time']+time

        #print('\r'+"LIVE TRAJECTORY FOLLOWER: Uploading Trajectory, %0.1f"%((idx+1)/float(len_traj)*100) +'%' +" complete", end='')

        print (["{0:0.2f}".format(i) for i in joint_pos_deg])



        if self.traj_file is not None:
            self.joint_traj.append(out)


    def save_file(self):
        if self.traj_file is not None:
            curr_path=os.path.dirname(os.path.abspath(__file__))
            outFile_rel=os.path.join("trajectories","arm",self.traj_file+".yaml")
            outFile=os.path.join(curr_path,"..",outFile_rel)

            print("Saving Trajectory to file: '%s'"%(outFile_rel))


            dirname = os.path.dirname(outFile)
            if not os.path.exists(dirname):
                os.makedirs(dirname)

            
            with open(outFile, 'w') as f:
                yaml.dump(self.joint_traj, f, default_flow_style=None)


    def on_press(self, key):
        pass


    def on_release(self, key):
        global restartFlag
        if key == Key.space:
            self.get_pos(time=1.5)

        elif key == Key.shift:
            self.toggle_record = not self.toggle_record









   
def main(file_name=None):
    try:

        rospy.init_node("get_pos", anonymous=True, disable_signals=True)
        teach=Teacher(file_name)

        listener = Listener(
            on_press=teach.on_press,
            on_release=teach.on_release)
        listener.start()

        teach.set_freedrive_mode(True)


        teach.save_file()

    except KeyboardInterrupt:
        
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__':
    if len(sys.argv) < 2:
        main()
    elif len(sys.argv) ==2:
        main(sys.argv[1])
    else:
        print("Usage:")
        print("\tteach.py \t\t- Enable freedrive but don't save")
        print("\tteach.py [FILENAME]\t- Save a trajectory")
