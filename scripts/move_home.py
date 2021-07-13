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
import yaml
import os

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
zero_pos = [0,-90,0,-90,0,0]
home_pos = [i*180.0/pi for i in [1.5707,-1.3090,1.8383,-0.5236,1.5709,-1.5712]]

curr_path=os.path.dirname(os.path.abspath(__file__))
pose_config = os.path.join(curr_path,"../config/arm_poses.yaml")


scaler = 1.0

    
import thread
import threading

def raw_input_with_timeout(prompt, timeout=30.0):
    print prompt,    
    timer = threading.Timer(timeout, thread.interrupt_main)
    astring = None
    try:
        timer.start()
        astring = raw_input(prompt)
    except KeyboardInterrupt:
        pass
    timer.cancel()
    return astring



class PoseHandler():
    def __init__(self, pose_config, joint_names):
        self.pose_config = pose_config
        self.JOINT_NAMES = joint_names

        self.arm_client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
        self.ur_script_pub = rospy.Publisher('/ur_driver/URScript', std_msgs.msg.String, queue_size=10)
        #hand_client = actionlib.SimpleActionClient('pre_built_traj', pressure_controller_ros.msg.RunAction)
        print "Waiting for servers..."
        self.arm_client.wait_for_server()
        #hand_client.wait_for_server()
        print "Connected to servers"

        self.r=rospy.Rate(2)


    def _get_pose(self):
        joint_states = rospy.wait_for_message("joint_states", JointState)
        joints_pos = list(joint_states.position)
        return joints_pos


    def _get_saved_poses(self):
        if os.path.isfile(self.pose_config):
            with open(self.pose_config, 'r') as f:
                saved_poses = yaml.safe_load(f)
        else:
            saved_poses={}

        return saved_poses

    def _save_poses(self, poses):
        filename = self.pose_config
        dirname = os.path.dirname(filename)
        if not os.path.exists(dirname):
            os.makedirs(dirname)
        
        with open(self.pose_config, 'w+') as f:
            yaml.dump(poses,f,default_flow_style=None)


    def get_pose_by_name(self, pose_name):
        saved_poses = self._get_saved_poses()
        return saved_poses[pose_name]


    def set_pose_by_name(self, pose_name, pose):
        if pose_name=='home' or pose_name=='zero':
            response = raw_input('Are you sure you want to overwrite the "%s" pose? (y/[N])'%(pose_name))
            print(response)
            if response.lower() == 'y':
                pass

            else:
                print('Aborting pose set')
                return False

        saved_poses = self._get_saved_poses()
        saved_poses[pose_name] = pose
        self._save_poses(saved_poses)
        return True


    def move_to_pose(self, pose_name):
        pose = self.get_pose_by_name(pose_name)
        self.move_to(pose)


    def set_pose(self, pose_name):
        self.set_freedrive_mode(True)

        new_pose = {}
        new_pose['joints'] = self._get_pose()
        new_pose['units']  = 'radians'
        response = self.set_pose_by_name(pose_name, new_pose)
        print('Pose "%s" saved:'%(pose_name))
        print(new_pose)



    def move_to(self, new_pos):
        joints = new_pos['joints']
        units = new_pos['units']

        if units == 'degrees':
            new_pos_rad = np.deg2rad(joints).tolist()
        else:
            new_pos_rad = joints

        joints_pos = self._get_pose()
        g = FollowJointTrajectoryGoal()
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = JOINT_NAMES
        g.trajectory.points = [
            JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
            JointTrajectoryPoint(positions=new_pos_rad, velocities=[0]*6, time_from_start=rospy.Duration(2.0*scaler))]

        try:
            self.arm_client.send_goal(g)
            self.arm_client.wait_for_result()

        except KeyboardInterrupt:
            self.arm_client.cancel_goal()
            raise
        except:
            raise


    def set_freedrive_mode(self, enable_teach_mode=False):
        if enable_teach_mode:
            # send URScript command that will enable teach mode
            
            # get current position
            try:
                print("Starting Freedrive Mode: You can now move the robot by hand!")
                #while not rospy.is_shutdown():
                self.ur_script_pub.publish('def myProg():\n\twhile (True):\n\t\tfreedrive_mode()\n\t\tsync()\n\tend\nend\n')
                response = raw_input('Press ENTER to save')
                self.ur_script_pub.publish('def myProg():\n\twhile (True):\n\t\tend_freedrive_mode()\n\t\tsleep(0.5)\n\tend\nend\n')

            except KeyboardInterrupt:
                print("\nExiting Freedrive Mode: The robot is now locked in place.")
                self.ur_script_pub.publish('def myProg():\n\twhile (True):\n\t\tend_freedrive_mode()\n\t\tsleep(0.5)\n\tend\nend\n')
        else:
            # send URScript command that will disable teach mode
            print("\nExiting Freedrive Mode: The robot is now locked in place.")
            self.ur_script_pub.publish('def myProg():\n\twhile (True):\n\t\tend_freedrive_mode()\n\t\tsleep(0.5)\n\tend\nend\n')




   
def main():
    try:
        if len(sys.argv)==2:
            cmd_type = 'go'
            position_type=str(sys.argv[1])

        elif len(sys.argv)==3:
            cmd_type=str(sys.argv[1])
            position_type=str(sys.argv[2])
        
        rospy.init_node("test_move", anonymous=True, disable_signals=True)
        parameters = rospy.get_param(None)
        index = str(parameters).find('prefix')
        if (index > 0):
            prefix = str(parameters)[index+len("prefix': '"):(index+len("prefix': '")+str(parameters)[index+len("prefix': '"):-1].find("'"))]
            for i, name in enumerate(JOINT_NAMES):
                JOINT_NAMES[i] = prefix + name

        pose_handler = PoseHandler(pose_config, JOINT_NAMES)
        
        if cmd_type == 'go':
            pose_handler.move_to_pose(position_type)
        
        if cmd_type == 'set':
            pose_handler.set_pose(position_type)

    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__':
    main()