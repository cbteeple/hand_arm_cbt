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
import numpy as np

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
zero_pos = [0,-90,0,-90,0,0]
home_pos = [75, -70, 65, 0, 90, -90]


scaler = 1.0
    
arm_client = None
hand_client = None



def move_home():
    global joints_pos
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    try:
        move_to(home_pos,g)
        #zero_hand()

    except KeyboardInterrupt:
        arm_client.cancel_goal()
        #zero_hand()
        raise
    except:
        raise



def move_zero():
    global joints_pos
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    try:
        move_to(zero_pos,g)
        #zero_hand()

    except KeyboardInterrupt:
        arm_client.cancel_goal()
        #zero_hand()
        raise
    except:
        raise



def zero_hand():
    hand_goal = pressure_controller_ros.msg.RunGoal(traj=False, data=False, wait_for_finish = False)
    hand_client.send_goal(hand_goal)
    hand_client.wait_for_result()



def move_to(new_pos,g):
    new_pos_rad = np.deg2rad(new_pos).tolist()
    joint_states = rospy.wait_for_message("joint_states", JointState)
    joints_pos = joint_states.position
    g.trajectory.points = [
        JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
        JointTrajectoryPoint(positions=new_pos_rad, velocities=[0]*6, time_from_start=rospy.Duration(2.0*scaler))]

    arm_client.send_goal(g)
    arm_client.wait_for_result()


   
def main():
    global arm_client
    global hand_client
    try:
        if len(sys.argv)==2:
            position_type=int(sys.argv[1])
        
        rospy.init_node("test_move", anonymous=True, disable_signals=True)
        arm_client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
        #hand_client = actionlib.SimpleActionClient('pre_built_traj', pressure_controller_ros.msg.RunAction)
        print "Waiting for servers..."
        arm_client.wait_for_server()
        #hand_client.wait_for_server()
        print "Connected to servers"
        parameters = rospy.get_param(None)
        index = str(parameters).find('prefix')
        if (index > 0):
            prefix = str(parameters)[index+len("prefix': '"):(index+len("prefix': '")+str(parameters)[index+len("prefix': '"):-1].find("'"))]
            for i, name in enumerate(JOINT_NAMES):
                JOINT_NAMES[i] = prefix + name
        if position_type==0:
            move_zero()
        elif position_type==1:
            move_home()

    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': main()
