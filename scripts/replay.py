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

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

time_scaler = 1.0
reset_time = 4.0
    
arm_client = None
arm_trajIn = []

g = FollowJointTrajectoryGoal()
g.trajectory = JointTrajectory()
g.trajectory.joint_names = JOINT_NAMES


def get_traj(traj_file):
    global arm_trajIn
    if traj_file is not None:
        curr_path=os.path.dirname(os.path.abspath(__file__))
        inFile_rel=os.path.join("trajectories","arm",traj_file+".yaml")
        inFile=os.path.join(curr_path,"..",inFile_rel)
        print("Reading Trajectory from file: '%s'"%(inFile_rel))


        with open(inFile,'r') as f:
            # use safe_load instead of load
            arm_trajIn = yaml.safe_load(f)
            f.close()
    else:
        raise("No filename given")



def go_to_start():
    global g
    global arm_trajIn

    g.trajectory.points= []
    
    joint_states = rospy.wait_for_message("joint_states", JointState)
    

    curr_pt = JointTrajectoryPoint(positions=joint_states.position, velocities=[0]*6, time_from_start=rospy.Duration(0.0))
    g.trajectory.points.append(curr_pt)
    curr_pt = JointTrajectoryPoint(positions=arm_trajIn[0]['joints_pos'], velocities=arm_trajIn[0]['joints_vel'], time_from_start=rospy.Duration(reset_time))
    g.trajectory.points.append(curr_pt)



def build_traj():
    global g
    global arm_trajIn

    joint_states = rospy.wait_for_message("joint_states", JointState)
    joints_pos = joint_states.position

    g.trajectory.points= []

    time_offset = arm_trajIn[0]['time']

    curr_pt = JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0))
    g.trajectory.points.append(curr_pt)


    traj_pts=[]
    traj_time=[]

    for point in arm_trajIn:
        curr_pt = JointTrajectoryPoint(positions=point['joints_pos'], velocities=point['joints_vel'], time_from_start=rospy.Duration((point['time']-time_offset)*time_scaler))
        g.trajectory.points.append(curr_pt)
        traj_pts.append(point['joints_pos'])
        traj_time.append((point['time']-time_offset)*time_scaler)

    point=arm_trajIn[-1]
    curr_pt = JointTrajectoryPoint(positions=point['joints_pos'], velocities=point['joints_vel'], time_from_start=rospy.Duration((point['time']-time_offset)*time_scaler +2.0))
    g.trajectory.points.append(curr_pt)
    traj_pts.append(point['joints_pos'])
    traj_time.append((point['time']-time_offset)*time_scaler)



def safe_stop():
    joint_states = rospy.wait_for_message("joint_states", JointState)
    joints_pos = joint_states.position

    g.trajectory.points= []

    curr_pt = JointTrajectoryPoint(positions=joint_states.position, velocities=[0]*6, time_from_start=rospy.Duration(0.0))
    g.trajectory.points.append(curr_pt)
    curr_pt = JointTrajectoryPoint(positions=joint_states.position, velocities=[0]*6, time_from_start=rospy.Duration(1.0))
    g.trajectory.points.append(curr_pt)

    arm_client.send_goal(g)



def execute_traj():
    global joints_pos
    global arm_trajIn
    global g
    try:
        arm_client.send_goal(g)
        arm_client.wait_for_result()

    except KeyboardInterrupt:
        arm_client.cancel_goal()
        safe_stop()
        raise
    except:
        raise


   
def main(file_name=None, scaler=None):
    global time_scaler
    if scaler is not None and scaler!=0.0:
        time_scaler = 1.0/scaler
    global arm_client
    global arm_trajIn
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
        inp = raw_input("Move to Starting Position? y/n: ")[0]
        if (inp == 'y'):
            go_to_start()
            execute_traj()

        inp = raw_input("Execute Trajectory? y/n: ")[0]
        if (inp == 'y'):
            for idx in range(1):
                build_traj()
                execute_traj()
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
    elif len(sys.argv) ==3:
        main(sys.argv[1], float(sys.argv[2]))
    else:
        print("Usage:")
        print("\tteach.py \t\t- Enable freedrive but don't save")
        print("\tteach.py [FILENAME]\t- Save a trajectory")
