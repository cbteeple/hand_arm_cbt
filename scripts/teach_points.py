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
import roslib; roslib.load_manifest('ur_driver');
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest
from sensor_msgs.msg import JointState
import std_msgs.msg
import pressure_controller_ros.msg
from math import pi
import numpy as np
import yaml
import os
import sys
from pynput.keyboard import Key, KeyCode, Listener

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

import inspect


class Teacher:
    def __init__(self, traj_file, point_type='joint', hand_type='pressure' ):
        self.ur_script_pub = rospy.Publisher('/ur_driver/URScript', std_msgs.msg.String, queue_size=10)
        self.r=rospy.Rate(20)
        self.toggle_record = False

        self.point_type = point_type
        self.hand_type  = hand_type

        self.traj_file = traj_file

        if self.traj_file is None:
            print("Starting Teach Mode, but not saving a file")

        self.compute_fk = rospy.ServiceProxy('/compute_fk', GetPositionFK)    
        self.compute_fk.wait_for_service()
        print("got kinematics service")

        self.get_settings()
        print("Got Settings")

        self.curr_segment = 0
        self.curr_segment_name= "segment_%03d"%(self.curr_segment)
        self.sequence = []
        self.sequence.append({'arm': self.curr_segment_name, 'hand':False})

        self.joint_traj = {}
        self.joint_traj[self.curr_segment_name]=[]

        self.use_hand=False
        self.last_time = None


    def get_settings(self):
        self.hand={}
        if 'robotiq' in self.hand_type:
            robotiq_width = float(raw_input("Enter grasping width (0.0 - 0.085, default=0.0 m): ") or "0.0")
            robotiq_speed = float(raw_input("Enter grasping speed (0.01 - 0.10, default=0.10 m/s): ") or "0.10")
            robotiq_force = int(raw_input("Enter grasping force (0, 200, default=0): ") or "0")

            from robotiq_trajectory_control.robotiq_2f_trajectory import trajSender as robotiq_traj_sender
            self.hand_sender = robotiq_traj_sender(1.0)

            import robotiq_trajectory_control.msg as hand_msg
            self.grasp_traj_send = hand_msg.TrajectoryGoal()
            self.grasp_traj_send.trajectory = hand_msg.Trajectory()
            self.grasp_traj_send.trajectory.points.append(hand_msg.TrajectoryPoint(pos=robotiq_width, speed=robotiq_speed, force=robotiq_force, time_from_start=rospy.Duration.from_sec(0.5)))

            self.release_traj_send = hand_msg.TrajectoryGoal()
            self.release_traj_send.trajectory = hand_msg.Trajectory()
            self.release_traj_send.trajectory.points.append(hand_msg.TrajectoryPoint(pos=0.085, speed=robotiq_speed, force=robotiq_force, time_from_start=rospy.Duration.from_sec(0.5)))

            self.grasp_traj = [ [0.5, robotiq_width, robotiq_speed, robotiq_force ] ]
            self.release_traj = [ [0.5, 0.085, robotiq_speed, robotiq_force ] ]

            self.hand['grasp'] = self.grasp_traj
            self.hand['release'] = self.release_traj
            self.hand['startup'] = self.release_traj[0]

            self.hand_sender.go_to_start(self.release_traj, reset_time=2.0, blocking=False)


        elif 'pressure' in self.hand_type:
            g_press = float(raw_input("Enter grasping pressure (default=15.0 psi): ") or "15.0")
            i_press = float(raw_input("Enter idle pressure (default=0.0 psi): ") or "0.0")
            g_time = float(raw_input("Enter grasping time (default=1.5 sec): ") or "1.5")

            from pressure_controller_ros.live_traj_new import trajSender as pneu_traj_sender
            self.hand_sender = pneu_traj_sender(1.0)
        
            num_channels = sum(rospy.get_param('/config/num_channels'))
            
            import pressure_controller_ros.msg as hand_msg
            self.grasp_traj = [ [0.0] + [i_press]*num_channels, [g_time] + [g_press]*num_channels ]
            self.release_traj = [ [0.0] + [g_press]*num_channels, [g_time] + [i_press]*num_channels ]
            
            self.grasp_traj_send = hand_msg.PressureTrajectoryGoal()
            self.grasp_traj_send.trajectory = hand_msg.PressureTrajectory()
            self.grasp_traj_send.trajectory.points.append(hand_msg.PressureTrajectoryPoint(pressures=self.grasp_traj[0][1:], time_from_start=self.grasp_traj[0][0]))
            self.grasp_traj_send.trajectory.points.append(hand_msg.PressureTrajectoryPoint(pressures=self.grasp_traj[1][1:], time_from_start=self.grasp_traj[0][1]))
            
            self.release_traj_send = hand_msg.PressureTrajectoryGoal()
            self.release_traj_send.trajectory = hand_msg.PressureTrajectory()
            self.release_traj_send.trajectory.points.append(hand_msg.PressureTrajectoryPoint(pressures=self.release_traj[0][1:], time_from_start=self.release_traj[0][0]))
            self.release_traj_send.trajectory.points.append(hand_msg.PressureTrajectoryPoint(pressures=self.release_traj[1][1:], time_from_start=self.release_traj[0][1]))

            self.hand['grasp'] = self.grasp_traj
            self.hand['release'] = self.release_traj
            self.hand['startup'] = self.grasp_traj[0]

            self.hand_sender.go_to_start(self.grasp_traj, 2.0, blocking=False)

        else:
            self.hand_sender = None


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
        

    def grasp(self, time=None):
        curr_jp = self.joint_traj[self.curr_segment_name][-1]
        self.sequence.append({'arm': False, 'hand':'grasp'})
        self.curr_segment +=1
        self.curr_segment_name= "segment_%03d"%(self.curr_segment)
        self.sequence.append({'arm': self.curr_segment_name, 'hand':False})

        self.joint_traj[self.curr_segment_name]=[]
        self.joint_traj[self.curr_segment_name].append(curr_jp)

        try: 
            self.hand_sender.execute_traj(self.grasp_traj_send)
        except:
            raise

        self.use_hand=True


    def release(self, time=None):
        curr_jp = self.joint_traj[self.curr_segment_name][-1]
        self.sequence.append({'arm': False, 'hand':'release'})
        self.curr_segment +=1
        self.curr_segment_name= "segment_%03d"%(self.curr_segment)
        self.sequence.append({'arm': self.curr_segment_name, 'hand':False})

        self.joint_traj[self.curr_segment_name]=[]
        self.joint_traj[self.curr_segment_name].append(curr_jp)

        try:
            self.hand_sender.execute_traj(self.release_traj_send)
        except:
            raise

        self.use_hand=True
        


    def get_pos(self,time=None):
        joint_states = rospy.wait_for_message("joint_states", JointState)

        if self.last_time is None:
            self.last_time = joint_states.header.stamp.to_sec()

        if self.point_type == 'cartesian':
            req = GetPositionFKRequest()
            req.header.frame_id = 'world'
            req.fk_link_names = ['tcp']
            req.robot_state.joint_state = joint_states

            pose = self.compute_fk.call(req)
            out = {}
            pos = pose.pose_stamped[0].pose.position
            ori = pose.pose_stamped[0].pose.orientation
            out['position'] = [pos.x, pos.y, pos.z]
            out['orientation'] = [ori.x, ori.y, ori.z, ori.w]

            print('pos',out['position'])
            print('ori',out['orientation'])
        
        else:
            joints_pos = joint_states.position
            joint_pos_deg = np.rad2deg(joints_pos).tolist()
            print (["{0:0.2f}".format(i) for i in joint_pos_deg])

            out = {}
            out['joints_pos'] = list(joint_states.position)
            out['joints_vel'] = list(joint_states.velocity)

        if time is None:
            stream_time = joint_states.header.stamp.to_sec() - self.last_time
            if len(self.joint_traj[self.curr_segment_name]) ==0:
                out['time'] = stream_time
            else:
                out['time'] = self.joint_traj[self.curr_segment_name][-1]['time'] + stream_time
            self.last_time = joint_states.header.stamp.to_sec()
        else:
            print(len(self.joint_traj[self.curr_segment_name]))
            if len(self.joint_traj[self.curr_segment_name]) ==0:
                out['time'] = 0.0
            else:
                out['time'] = self.joint_traj[self.curr_segment_name][-1]['time']+time


        #print('\r'+"LIVE TRAJECTORY FOLLOWER: Uploading Trajectory, %0.1f"%((idx+1)/float(len_traj)*100) +'%' +" complete", end='')


        if self.traj_file is not None:
            self.joint_traj[self.curr_segment_name].append(out)


    def save_file(self):
        if self.traj_file is not None:
            curr_path=os.path.dirname(os.path.abspath(__file__))
            outFile_rel=os.path.join("trajectories","arm",self.traj_file,"trajectory.yaml")
            outFile=os.path.join(curr_path,"..",outFile_rel)

            print("Saving Trajectory to file: '%s'"%(outFile_rel))


            dirname = os.path.dirname(outFile)
            if not os.path.exists(dirname):
                os.makedirs(dirname)

            out = {}
            out['arm'] = self.joint_traj
            out['arm']['pre'] = [out['arm']["segment_%03d"%(0)][0]]

            out['hand'] = self.hand
            out['sequence'] = {}
            out['sequence']['setup']={'arm_traj_space': self.point_type, 'hand_traj_space': self.hand_type}
            out['sequence']['startup']={'arm': 'pre', 'hand': 'startup'}
            out['sequence']['operations'] = self.sequence
            out['settings'] = {'boomerang': True, 'reset_object': False, 'type': 'single', 'use_arm': True, 'use_hand': self.use_hand}
            with open(outFile, 'w') as f:
                yaml.dump(out, f, default_flow_style=None)


    def on_press(self, key):
        pass


    def on_release(self, key):
        global restartFlag
        if key == Key.space:
            self.get_pos(time=1.5)

        if key == KeyCode(char='g'):
            self.get_pos(time=1.5)
            self.grasp(time=1.5)
        if key == KeyCode(char='r'):
            self.get_pos(time=1.5)
            self.release(time=1.5)

        elif key == Key.shift:
            self.toggle_record = not self.toggle_record
            self.last_time = None









   
def main(file_name=None, flags=""):
    try:
        flags=str(flags)

        rospy.init_node("get_pos", anonymous=True, disable_signals=True)

        if "cartesian" in flags:
            point_type='cartesian'
            print("Using Cartesian Mode")
        else:
            point_type = 'joint'

        if "robotiq" in flags:
            hand_type='robotiq'
            print("Using Robotiq Hand")
        else:
            hand_type = 'pressure'

        
        teach=Teacher(file_name, point_type, hand_type)


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
    elif len(sys.argv) ==3:
        main(sys.argv[1],sys.argv[2])
    else:
        print("Usage:")
        print("\tteach.py \t\t- Enable freedrive but don't save")
        print("\tteach.py [FILENAME]\t- Save a trajectory")
