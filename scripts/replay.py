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
import pressure_controller_ros.msg
from math import pi
import yaml
import os
import sys
import matplotlib.pyplot as plt

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

scaler = 1.0
reset_time = 2.0
    
arm_client = None
trajIn = []

g = FollowJointTrajectoryGoal()
g.trajectory = JointTrajectory()


def get_traj(traj_file):
    global trajIn
    if traj_file is not None:
        curr_path=os.path.dirname(os.path.abspath(__file__))
        inFile_rel=os.path.join("trajectories","arm",traj_file+".yaml")
        inFile=os.path.join(curr_path,"..",inFile_rel)
        print("Reading Trajectory from file: '%s'"%(inFile_rel))


        with open(inFile,'r') as f:
            # use safe_load instead of load
            trajIn = yaml.safe_load(f)
            f.close()
    else:
        raise("No filename given")


def build_traj():
    global g
    global trajIn

    g.trajectory.joint_names = JOINT_NAMES
    joint_states = rospy.wait_for_message("joint_states", JointState)
    joints_pos = joint_states.position

    g.trajectory.points= []

    time_offset = trajIn[0]['time'] -reset_time

    curr_pt = JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0))
    g.trajectory.points.append(curr_pt)


    traj_pts=[]
    traj_time=[]

    for point in trajIn:
        curr_pt = JointTrajectoryPoint(positions=point['joints_pos'], velocities=point['joints_vel'], time_from_start=rospy.Duration((point['time']-time_offset)*scaler))
        g.trajectory.points.append(curr_pt)
        traj_pts.append(point['joints_pos'])
        traj_time.append((point['time']-time_offset)*scaler)



def safe_stop():
    g.trajectory.joint_names = JOINT_NAMES
    joint_states = rospy.wait_for_message("joint_states", JointState)
    joints_pos = joint_states.position

    g.trajectory.points= []

    curr_pt = JointTrajectoryPoint(positions=joint_states.position, velocities=[0]*6, time_from_start=rospy.Duration(0.0))
    g.trajectory.points.append(curr_pt)
    curr_pt = JointTrajectoryPoint(positions=joint_states.position, velocities=[0]*6, time_from_start=rospy.Duration(1.0))
    g.trajectory.points.append(curr_pt)

    arm_client.send_goal(g)



def move1():
    global joints_pos
    global trajIn
    global g
    try:
        build_traj()
        arm_client.send_goal(g)
        arm_client.wait_for_result()



    except KeyboardInterrupt:
        arm_client.cancel_goal()
        safe_stop()
        raise
    except:
        raise


   
def main(file_name=None):
    global arm_client
    global trajIn
    try:
        rospy.init_node("test_move", anonymous=True, disable_signals=True)
        arm_client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for servers..."
        arm_client.wait_for_server()
        print "Connected to servers"
        parameters = rospy.get_param(None)
        index = str(parameters).find('prefix')
        if (index > 0):
            prefix = str(parameters)[index+len("prefix': '"):(index+len("prefix': '")+str(parameters)[index+len("prefix': '"):-1].find("'"))]
            for i, name in enumerate(JOINT_NAMES):
                JOINT_NAMES[i] = prefix + name

        get_traj(file_name)
        print "Please make sure that your robot can move freely between these poses before proceeding!"
        inp = raw_input("Continue? y/n: ")[0]
        if (inp == 'y'):
            for idx in range(1):
                move1()
        else:
            print "Halting program"
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
