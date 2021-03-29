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
import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
import rospy
import actionlib
#from control_msgs.msg import *
#from trajectory_msgs.msg import *
#from sensor_msgs.msg import JointState
from hand_arm.msg import *
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
from math import pi
import yaml
import os
import sys
import copy



filepath_default = os.path.join('..','trajectories')


class trajSender:
    def __init__(self, joint_names=JOINT_NAMES, speed_factor=1.0):
        self.filepath_default = filepath_default
        self.speed_multiplier = 1.0/speed_factor
        self.reset_time = 10.0
            
        # Load up the hand client
        self.traj_client = actionlib.SimpleActionClient('/robotiq_trajectory', PressureTrajectoryAction)
        self.traj_client.wait_for_server()
          


    def load_trajectory(self, traj_file, filepath=None):

        if filepath is None:
                # Use the default file path if we don't give it one explicitly
                filepath = self.filepath_default


        if traj_file is not None:
            inFile=os.path.join(filepath,'arm',traj_file+".yaml")

            with open(inFile,'r') as f:
                # use safe_load instead of load
                arm_trajIn = yaml.safe_load(f)
                f.close()

            # Build the trajectory into a ROS message
            return arm_trajIn

        else:
            raise("No filename given")




    def build_traj(self, arm_trajIn = None):

        if type(arm_trajIn) is type(self.goal_blank):
            goal= arm_trajIn

            waypoints = arm_trajIn.trajectory.points

            for idx, waypoint in enumerate(waypoints):
                waypoint.time_from_start = waypoint.time_from_start*self.speed_multiplier

                vels=[]
                for vel in waypoint.velocities:
                    vels.append(vel/self.speed_multiplier)

                waypoint.velocities = tuple(vels)



        else:

            joint_states = rospy.wait_for_message("joint_states", JointState)
            joints_pos = joint_states.position


            goal = copy.deepcopy(self.goal_blank)

            goal.trajectory.points= []
            time_offset = arm_trajIn[0]['time']


            for point in arm_trajIn:
                curr_pt = JointTrajectoryPoint(positions=point['joints_pos'], velocities=point['joints_vel'], time_from_start=rospy.Duration((point['time']-time_offset)*self.speed_multiplier))
                goal.trajectory.points.append(curr_pt)

            curr_pt = JointTrajectoryPoint(positions=arm_trajIn[-1]['joints_pos'], velocities=arm_trajIn[-1]['joints_vel'], time_from_start=rospy.Duration((arm_trajIn[-1]['time']-time_offset+0.25)*self.speed_multiplier))
            goal.trajectory.points.append(curr_pt)

        return goal



    def go_to_start(self, goal, reset_time, blocking=True):

        goal_tmp = copy.deepcopy(self.goal_blank)
                
        joint_states = rospy.wait_for_message("joint_states", JointState)
        
        curr_pt = JointTrajectoryPoint(positions=joint_states.position, velocities=[0]*6, time_from_start=rospy.Duration(0.0))
        goal_tmp.trajectory.points.append(curr_pt)

        curr_pt =JointTrajectoryPoint( positions=goal.trajectory.points[0].positions,
                                        velocities=[0]*6,
                                        time_from_start=rospy.Duration(reset_time) )

        goal_tmp.trajectory.points.append(curr_pt)

        self.execute_traj( goal_tmp, blocking)



    def safe_stop(self):
        joint_states = rospy.wait_for_message("joint_states", JointState)

        goal_stop = copy.deepcopy(self.goal_blank)

        goal_stop.trajectory.points= []

        curr_pt = JointTrajectoryPoint(positions=joint_states.position, velocities=[0]*6, time_from_start=rospy.Duration(0.0))
        goal_stop.trajectory.points.append(curr_pt)
        curr_pt = JointTrajectoryPoint(positions=joint_states.position, velocities=[0]*6, time_from_start=rospy.Duration(1.0))
        goal_stop.trajectory.points.append(curr_pt)

        self.traj_client.send_goal(goal_stop)






    def execute_traj(self, goal, blocking=True):
        try:
            self.traj_client.send_goal(goal)

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
