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
from pressure_controller_ros.msg import *
from math import pi
import yaml
import pickle
import os
import sys
import matplotlib.pyplot as plt

from pressure_controller_ros.live_traj_new import trajSender as pneu_traj_sender
from hand_arm_cbt.arm_mover import trajSender as ur_traj_sender
from hand_arm_cbt.arm_moveit import MoveItPythonInteface as ur_traj_sender_moveit


reset_time = 2.0

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

curr_path=os.path.dirname(os.path.abspath(__file__))
filepath_traj = os.path.join(curr_path,'..','trajectories')
filepath_config = os.path.join(curr_path,'..','config')






class pickPlace:
    def __init__(self):
        self.traj_profile = rospy.get_param(rospy.get_name()+'/traj_profile')
        self.speed_factor = rospy.get_param(rospy.get_name()+'/speed_factor',1.0)
        self.num_reps = rospy.get_param(rospy.get_name()+'/num_reps',1)
        self.replan = rospy.get_param(rospy.get_name()+'/replan',False)

        self.use_arm = rospy.get_param(rospy.get_name()+'/use_arm',True)
        self.use_hand = rospy.get_param(rospy.get_name()+'/use_hand',True)
        self.fake = rospy.get_param(rospy.get_name()+'/fake',False)

        

        # Read the trajectory configuration file
        self.filepath = os.path.join(filepath_traj)

        config_file =   os.path.join(self.filepath,self.traj_profile+'_planned.traj')
        if (not os.path.exists(config_file)) or self.replan:
            self.replan = True
            config_file =   os.path.join(self.filepath,self.traj_profile+'.yaml')
            print('Planning trajectory from scratch')

        with open(config_file,'r') as f:

            if config_file.endswith('.traj'):
                self.traj_config = pickle.load(f)
            else:
                self.traj_config = yaml.safe_load(f)

            self.operations = self.traj_config['sequence']
            f.close()


        setup = self.operations['setup']
        
        # Create the arm objects
        if self.use_arm:
            if (setup['arm_traj_space']== 'joint') or (setup['arm_traj_space']== 'joint-planned'):
                self.arm_sender = ur_traj_sender(JOINT_NAMES, self.speed_factor)

            elif setup['arm_traj_space'] == 'cartesian':
                self.arm_sender = ur_traj_sender_moveit(JOINT_NAMES)

                #configure the planners based on the config file
                self.arm_sender.config_planner(os.path.join(filepath_config,'moveit_config.yaml'))
            else:
                print('Nonstandard arm trajectory space')
                self.arm_sender = None
                raise
        else:
            print('Not using arm - Arm moves will be skipped')

        # Create the pneumatic hand object
        if self.use_hand:
            if setup['hand_traj_space'] == 'pressure':
                self.hand_sender = pneu_traj_sender(self.speed_factor)
            else:
                print('Nonstandard hand trajectory space')
                raise
        else:
            print('Not using hand - Hand moves will be skipped')


    def get_sequence(self):
        # get the trajectories
        self.operation_sequence = []

        for move in self.operations['operations']:
            out = {}
            out['arm']  = None
            out['hand'] = None
            if move['arm']:
                 out['arm'] = self.traj_config['arm'].get(move['arm'], None)

            if move['hand']:
                out['hand'] = self.traj_config['hand'].get(move['hand'], None)

            self.operation_sequence.append(out)



    def go_to_start(self):
        try:
            # Start the trajectories
            if self.use_hand:
                self.hand_sender.go_to_start(self.operation_sequence[0]['hand'], reset_time, blocking=False)
            if self.use_arm:
                self.arm_sender.go_to_start(self.operation_sequence[0]['arm'], reset_time, blocking=False)

            # Wait for the traj to finish
            if self.use_hand:
                self.hand_sender.traj_client.wait_for_result()
            if self.use_arm:
                self.arm_sender.traj_client.wait_for_result()
        except KeyboardInterrupt:
            raise



    def plan_sequence(self):

        self.operation_plans = []

        if self.replan:            
            for movement in self.operation_sequence:
                out = {}
                out['arm']  = None
                out['hand'] = None
                if movement['arm'] is not None:
                    if self.use_arm:
                        out['arm'] = self.arm_sender.build_traj(movement['arm'])
                    else:
                        out['arm'] = None

                if movement['hand'] is not None:
                    if self.use_hand:
                        out['hand'] = self.hand_sender.build_traj(movement['hand'])
                    else:
                        out['hand'] = None

                self.operation_plans.append(out)

        else:
            for movement in self.operation_sequence:
                out = {}
                out['arm']  = None
                out['hand'] = None
                if movement['arm'] is not None:
                    if self.use_arm:
                        out['arm'] = movement['arm']
                    else:
                        out['arm'] = None
                if movement['hand'] is not None:
                    if self.use_hand:
                        out['hand'] = self.hand_sender.build_traj(movement['hand'])
                    else:
                        out['hand'] = None

                self.operation_plans.append(out)




    def excecute_sequence(self):
        try:
            for plan in self.operation_plans:
                if (plan['hand'] is not None) and (self.use_hand):
                    self.hand_sender.execute_traj(plan['hand'], blocking=False)

                if (plan['arm'] is not None) and (self.use_arm):
                    if not self.fake:
                        self.arm_sender.execute_traj(plan['arm'], blocking=False)
                    else:
                        self.arm_sender.display_trajectory(plan['arm'])

                if self.use_hand:
                    self.hand_sender.traj_client.wait_for_result()
                if self.use_arm:
                    self.arm_sender.traj_client.wait_for_result()

         except KeyboardInterrupt:
            raise

            
        
    def rep_sequence(self):
        try:
            for idx in range(self.num_reps):
                inp = raw_input("Execute Action? (Press ENTER)")
                self.excecute_sequence()
                self.go_to_start()

        except KeyboardInterrupt:
            if self.use_hand:
                self.hand_sender.shutdown()
            raise

   


def main(file_name=None):
    try:
        rospy.init_node("pick_and_place", anonymous=True, disable_signals=True)

        node = pickPlace()
        node.get_sequence()

        # Go to the start
        inp = raw_input("Move to Starting Position? (Press ENTER)")
        node.go_to_start()
        node.plan_sequence()

        # Excecute the trajectory the desired number of times
        node.rep_sequence()
        

    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__':
    if len(sys.argv) < 2 or len(sys.argv) > 2:
        main()
    elif len(sys.argv) ==2:
        main(sys.argv[1])
    else:
        print("Usage:")
        print("\tpick_place.py [FILENAME]\t- Replay a trajectory")
