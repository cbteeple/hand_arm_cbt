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
import roslib# ; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from pressure_controller_ros.msg import *
from math import pi
import yaml
import pickle
import os
import sys
import matplotlib.pyplot as plt

from hand_arm_cbt.arm_mover import trajSender as ur_traj_sender
from hand_arm_cbt.arm_moveit import MoveItPythonInteface as ur_traj_sender_moveit


reset_time = 2.0

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

curr_path=os.path.dirname(os.path.abspath(__file__))
filepath_traj = os.path.join(curr_path,'..','..','trajectories')
filepath_out = os.path.join(curr_path,'..','..','trajectories')
filepath_config = os.path.join(curr_path,'..','..','config')

Fake = False




class TrajPlanner:
    def __init__(self, traj=None):
        if isinstance(traj,str):
            self.traj_profile = traj
        else:
            self.traj_profile = rospy.get_param(rospy.get_name()+'/traj')

        # Read the trajectory configuration file
        self.filepath = os.path.join(filepath_traj)

        if self.traj_profile.endswith(".yaml"):
            self.filepath = os.path.join(self.filepath, os.path.dirname(self.traj_profile))
            self.in_files = [os.path.basename(self.traj_profile)]
        else:
            self.filepath =   os.path.join(self.filepath,self.traj_profile)
            self.in_files = [f for f in os.listdir(self.filepath) if (os.path.isfile(os.path.join(self.filepath, f)) and f.endswith(".yaml"))]

        with open(os.path.join(self.filepath,self.in_files[0]),'r') as f:
            # use safe_load instead of load
            self.traj_config = yaml.safe_load(f)

            self.sequence = self.traj_config['sequence']
            self.setup = self.sequence['setup']
            self.arm_segs = self.traj_config['arm']
            self.settings = self.traj_config.get('settings',{})
            f.close()

        # Create the arm objects
        if self.settings.get('use_arm',True):
            if self.setup['arm_traj_space'] == 'cartesian':
                self.arm_sender = ur_traj_sender_moveit(joint_names=JOINT_NAMES, non_excecuting=False)

                #configure the planners based on the config file
                self.arm_sender.config_planner(os.path.join(filepath_config,'moveit_config.yaml'))
            
            elif self.setup['arm_traj_space'] == 'joint':
                self.arm_sender = None
            else:
                print('Nonstandard arm trajectory space')
                self.arm_sender = None
                #raise        


    def plan_all(self):
        for config_file in self.in_files:
            with open(os.path.join(self.filepath,config_file),'r') as f:
                # use safe_load instead of load
                self.traj_config = yaml.safe_load(f)

                self.sequence = self.traj_config['sequence']
                self.setup = self.sequence['setup']
                self.arm_segs = self.traj_config['arm']
                self.settings = self.traj_config.get('settings',{})
                f.close()

            if self.settings.get('use_arm',True):
                self.plan_segments()
            
            self.save_plan(os.path.join(self.filepath,config_file))


    def plan_segments(self):
        # build the trajectories
        self.planned_segments = {}
        for segment_name in self.arm_segs:
            print(segment_name)
            if self.arm_sender is None:
                self.planned_segments[segment_name] = self.arm_segs[segment_name]
            else:
                self.planned_segments[segment_name] = self.arm_sender.convert_traj(self.arm_sender.build_traj(self.arm_segs[segment_name]))

   


    def save_plan(self, in_file_name):
        if self.settings.get('use_arm',True):
            self.traj_config['arm'] = self.planned_segments
            self.traj_config['sequence']['setup']['arm_traj_space'] = 'joint-planned'
        
        else:
            self.traj_config['arm'] = {}

        if not self.settings.get('use_hand',True):
            self.traj_config['hand'] = {}

        if not self.settings.get('use_servo',True):
            self.traj_config['servo'] = {}

        out_file =  in_file_name.replace('.yaml',".traj")

        if not os.path.exists(os.path.dirname(out_file)):
            try:
                os.makedirs(os.path.dirname(out_file))
            except OSError as exc: # Guard against race condition
                if exc.errno != errno.EEXIST:
                    raise

      
        with open(out_file, 'w+') as f:
            pickle.dump(self.traj_config, f)
        #with open(out_file+'y', 'w+') as f:
        #    yaml.dump(self.traj_config, f, default_flow_style=None)

           
  


def main():
    try:
        rospy.init_node("plan_trajectories", anonymous=True, disable_signals=True)

        node = TrajPlanner()
        node.load_trajectory()
        node.plan_all()

        

    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__':
    if len(sys.argv) == 1 or len(sys.argv)==3:
        main()
    else:
        print('Use roslaunch and the associated "*.launch" file for this script')
