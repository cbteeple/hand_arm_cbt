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
from simple_ur_move.cartesian_trajectory_handler import CartesianTrajectoryHandler
from simple_ur_move.joint_trajectory_handler import JointTrajectoryHandler
import rospkg
from math import pi
import yaml
import os
import sys
import copy
import threading


JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']


filepath_default = os.path.join('..','trajectories')
filepath_config = os.path.join(rospkg.RosPack().get_path('hand_arm'), 'config')


class trajSender:
    def __init__(self, speed_factor=1.0, traj_type='joint', debug=False):
        self.filepath_default = filepath_default
        self.speed_multiplier = 1.0/speed_factor
        self.reset_time = 10.0
        self.debug = debug
            
        # Load up the trajectory handler
        if traj_type == 'cartesian':
            self.traj_handler = CartesianTrajectoryHandler(name="", controller="", debug=self.debug
            self.traj_handler.load_config('arm_cartesian_config.yaml', filepath_config)
            self.traj_handler.set_initialize_time(self.reset_time)
        
        else:
            self.traj_handler = JointTrajectoryHandler(name="", controller="", debug=self.debug
            self.traj_handler.load_config('arm_joint_config.yaml', filepath_config)
            self.get_joint_names(JOINT_NAMES)
        

        self.traj_handler.set_initialize_time(self.reset_time)
        self.traj_handler.speed_factor(speed_factor)

        self.traj_client = self.traj_handler.trajectory_client

        
        self.arm_trajIn = []
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                            DisplayTrajectory,
                                                            queue_size=20)


    def get_joint_names(self, joint_names):
        parameters = rospy.get_param(None)
        index = str(parameters).find('prefix')
        if (index > 0):
            prefix = str(parameters)[index+len("prefix': '"):(index+len("prefix': '")+str(parameters)[index+len("prefix': '"):-1].find("'"))]
            for i, name in enumerate(joint_names):
                joint_names[i] = prefix + name

        self.traj_handler.joint_names = joint_names


    def build_traj(self, arm_trajIn = None):

        if isinstance(arm_trajIn, list):
            if "positions" in arm_trajIn[0].keys()
                traj= arm_trajIn
            else:
                traj= []
                time_offset = arm_trajIn[0]['time']

                for point in arm_trajIn:
                    curr_pt = {'positions': point['joints_pos'],
                            'velocities': point['joints_vel'],
                            'time': point['time']-time_offset}
                    traj.append(curr_pt)
        else:
            traj= arm_trajIn.trajectory

        goal = self.traj_handler.build_goal(traj)

        return goal


    def go_to_start(self, goal, reset_time, blocking=True):

        self.traj_handler.set_initialize_time(reset_time)
        self.traj_handler.go_to_point(goal.trajectory.points[0])


    def safe_stop(self):
        self.traj_client.cancel_all_goals()


    def display_trajectory(self, plan):
        #print(plan)
        traj = plan.trajectory
        joints = traj.joint_names 
        #print(traj)

        init_joints = JointState()
        init_joints.name = joints
        init_joints.position = traj.points[0].positions
        init_joints.velocity = traj.points[0].velocities
        init_joints.effort = traj.points[0].effort

        joint_traj = RobotTrajectory()
        joint_traj.joint_trajectory = plan.trajectory

        init_state = RobotState()
        init_state.joint_state = init_joints

        display_trajectory = DisplayTrajectory()
        display_trajectory.trajectory_start = init_state
        display_trajectory.trajectory.append(joint_traj)
        # Publish
        self.display_trajectory_publisher.publish(display_trajectory)
        rospy.sleep(3)




    def execute_traj(self, goal, blocking=True):
        try:
            self.traj_handler.run_trajectory(goal, blocking=False, perform_init=False)

            if blocking:
                self.traj_client.wait_for_result()
            else:
                pass

        except KeyboardInterrupt:
            self.shutdown()
            raise
        except:
            raise


    def shutdown(self):
        self.traj_client.cancel_goal()
        self.safe_stop()


   
def main(file_name=None):
    try:
        rospy.init_node("pick_and_place", anonymous=True, disable_signals=True)

        node = trajSender()

        traj_built = node.load_trajectory(file_name)

        inp = raw_input("Move to Starting Position? y/n: ")[0]
        if (inp == 'y'):
            node.go_to_start(traj_built)


        inp = raw_input("Execute Trajectory? y/n: ")[0]
        if (inp == 'y'):
            for idx in range(1):
                node.build_traj()
                node.execute_traj()
        else:
            print "Halting program"
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__':
    if len(sys.argv) ==2:
        main(sys.argv[1])
    else:
        print("Usage:")
        print("\tpick_place.py [FILENAME]\t- Replay a trajectory")
