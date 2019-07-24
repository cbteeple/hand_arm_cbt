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
import os
import sys
import matplotlib.pyplot as plt

from pressure_controller_ros.live_traj_new import trajSender as pneu_traj_sender
from hand_arm_cbt.arm_mover import trajSender as ur_traj_sender


reset_time = 1.0

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

curr_path=os.path.dirname(os.path.abspath(__file__))
filepath_default = os.path.join(curr_path,'..','trajectories')
    
   
def main(file_name=None):
    try:
        rospy.init_node("pick_and_place", anonymous=True, disable_signals=True)

        traj_profile = rospy.get_param(rospy.get_name()+'/traj_profile')
        speed_factor = rospy.get_param(rospy.get_name()+'/speed_factor',1.0)
        num_reps = rospy.get_param(rospy.get_name()+'/num_reps',1)


        operations = []

        # Set up the operations that the ahand and arm will do

        # Move to object
        operations.append({'arm': '_pre' , 'hand': '_startup'})

        # Grasp
        operations.append({'arm': None , 'hand': '_grasp'})

        # Move Object to target
        operations.append({'arm': '_move' , 'hand': None})

        # Release
        operations.append({'arm': None , 'hand': '_release'})

        #Move Object to target
        operations.append({'arm': '_post' , 'hand': None})



        # Create the arm object
        arm_sender = ur_traj_sender(JOINT_NAMES, speed_factor)

        # Create the pneumatic hand object
        hand_sender = pneu_traj_sender(speed_factor)



        # Build the trajectories
        operation_sequence = []

        filepath = os.path.join(filepath_default,traj_profile)

        for move in operations:
            out = {}
            out['arm']  = None
            out['hand'] = None
            if move['arm'] is not None:
                 out['arm'] = arm_sender.load_trajectory(traj_profile+move['arm'], filepath)

            if move['hand'] is not None:
                out['hand'] = hand_sender.load_trajectory(traj_profile+move['hand'], filepath)


            operation_sequence.append(out)





        inp = raw_input("Move to Starting Position? (Press ENTER)")
        hand_sender.go_to_start(operation_sequence[0]['hand'], reset_time)
        arm_sender.go_to_start(operation_sequence[0]['arm'], reset_time)

        #Excecute the trajectory the desired number of times
        for idx in range(num_reps):
            inp = raw_input("Execute Action? (Press ENTER)")
            for movement in operation_sequence:
                if movement['hand'] is not None:
                    hand_sender.execute_traj(movement['hand'])

                if movement['arm'] is not None:
                    arm_sender.execute_traj(movement['arm'])

        hand_sender.shutdown()

    except KeyboardInterrupt:
        hand_sender.shutdown()
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
