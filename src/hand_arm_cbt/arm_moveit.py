#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

# MODIFICATIONS: Clark Teeple


import os
import sys
import copy
import rospy
import time
import actionlib
import yaml
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from math import pi
from std_msgs.msg import String
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from moveit_commander.conversions import pose_to_list
## END_SUB_TUTORIAL

filepath_default = os.path.join('..','trajectories')


JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']



def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True


class MoveItPythonInteface(object):
    """MoveGroupPythonIntefaceTutorial"""
    def __init__(self, joint_names=JOINT_NAMES, non_excecuting = True):
        super(MoveItPythonInteface, self).__init__()

        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                                                                     moveit_msgs.msg.DisplayTrajectory,
                                                                                                     queue_size=20)



        # Load up the arm client
        if non_excecuting:
            #self.traj_client = actionlib.SimpleActionClient('/move_group', moveit_msgs.msg.MoveGroupAction)
            self.traj_client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
            print("Waiting for servers...")
            self.traj_client.wait_for_server()
            print ("Connected to servers")
        else:
            self.traj_client = None


        # We can get the name of the reference frame for this robot:
        self.planning_frame = self.move_group.get_planning_frame()
        print("============ Planning frame: %s" % self.planning_frame)

        self.eef_link = self.move_group.get_end_effector_link()
        print ("============ End effector link: %s" % self.eef_link)

        self.group_names = self.robot.get_group_names()
        print("============ Available Planning Groups:")
        print(self.robot.get_group_names())


        rospy.sleep(2)
        
        self.add_world_collisions()


        # Misc variables
        self.box_name = ''
        self.last_state = None


        self.goal_blank = FollowJointTrajectoryGoal()
        self.goal_blank.trajectory = JointTrajectory()
        self.get_joint_names(joint_names)

        


    def config_planner(self, config_file):

        with open(config_file,'r') as f:
            # use safe_load instead of load
            movit_config = yaml.safe_load(f)
            f.close()

        print(movit_config)

        self.move_group.set_planner_id(movit_config['planner_id'])
        self.move_group.set_planning_time(movit_config['planning_time'])
        self.move_group.allow_replanning(True)
        self.move_group.set_num_planning_attempts(movit_config.get('num_planning_attempts',1))



    def get_joint_names(self, joint_names):
        parameters = rospy.get_param(None)
        index = str(parameters).find('prefix')
        if (index > 0):
            prefix = str(parameters)[index+len("prefix': '"):(index+len("prefix': '")+str(parameters)[index+len("prefix': '"):-1].find("'"))]
            for i, name in enumerate(joint_names):
                joint_names[i] = prefix + name

        self.goal_blank.trajectory.joint_names = joint_names





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
        waypoints = []

        wpose = geometry_msgs.msg.Pose()

        
        for point in arm_trajIn:
            wpose.position.x =    point['position'][0]
            wpose.position.y =    point['position'][1]
            wpose.position.z =    point['position'][2]
            wpose.orientation.x = point['orientation'][0]
            wpose.orientation.y = point['orientation'][1]
            wpose.orientation.z = point['orientation'][2]
            wpose.orientation.w = point['orientation'][3]
            waypoints.append(copy.deepcopy(wpose))

        plan, fraction = self.plan_cartesian_path(waypoints, from_last=True)

        return plan



    def go_to_start(self, goal, reset_time, blocking=True):

        wpose = geometry_msgs.msg.Pose()

        wpose.position.x =    goal[0]['position'][0]
        wpose.position.y =    goal[0]['position'][1]
        wpose.position.z =    goal[0]['position'][2]
        wpose.orientation.x = goal[0]['orientation'][0]
        wpose.orientation.y = goal[0]['orientation'][1]
        wpose.orientation.z = goal[0]['orientation'][2]
        wpose.orientation.w = goal[0]['orientation'][3]

        self.move_group.set_start_state_to_current_state()
        self.move_group.set_joint_value_target(wpose)
        plan = self.move_group.plan()

        self.execute_traj(plan,blocking)
     
                       

    def go_to_joint_state(self, joint_goal_in = None):
        # We can get the joint values from the group and adjust some of the values:
        if joint_goal_in is None:
                joint_goal = self.move_group.get_current_joint_values()
                joint_goal[0] = 0
                joint_goal[1] = -pi/2
                joint_goal[2] = 0
                joint_goal[3] = -pi/2
                joint_goal[4] = 0
                joint_goal[5] = 0
        else:
                joint_goal=joint_goal_in

        print(joint_goal)
        print(self.move_group.get_current_joint_values())


        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()

        # For testing:
        current_joints = self.move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)



    def plan_pose_goal(self):
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 0.0
        pose_goal.position.x = 0.4
        pose_goal.position.y = 0.3
        pose_goal.position.z = 0.45

        self.move_group.set_pose_target(pose_goal)
        plan = self.move_group.plan()

        print(plan)
        return plan




    def go_to_pose_goal(self, pose_goal):
        if not pose_goal:
            pose_goal = geometry_msgs.msg.Pose()
            pose_goal.orientation.w = 0.0
            pose_goal.position.x = 0.4
            pose_goal.position.y = 0.3
            pose_goal.position.z = 0.45


        self.move_group.clear_pose_targets()
        self.move_group.set_pose_target(pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        plan = self.move_group.go(wait=True)
        


        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.move_group.clear_pose_targets()

        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        #return all_close(pose_goal, current_pose, 0.01)





    def plan_cartesian_path(self, waypoints, from_last = False):     
        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.


        if from_last:
            if self.last_state:
                self.move_group.set_start_state(self.last_state)
            else:
                self.move_group.set_start_state_to_current_state()
        else:
            self.move_group.set_start_state_to_current_state()
        
        (plan, fraction) = self.move_group.compute_cartesian_path(
                                                                             waypoints,   # waypoints to follow
                                                                             0.01,        # eef_step
                                                                             0.0)         # jump_threshold

        self.last_state = self.robot.get_current_state()
        self.last_state.joint_state.position=plan.joint_trajectory.points[-1].positions
        self.last_state.joint_state.velocity=[0]*6
        
        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

        ## END_SUB_TUTORIAL


    def display_trajectory(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        ## BEGIN_SUB_TUTORIAL display_trajectory
        ##
        ## Displaying a Trajectory
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
        ## group.plan() method does this automatically so this is not that useful
        ## here (it just displays the same trajectory again):
        ##
        ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        ## We populate the trajectory_start with our current robot state to copy over
        ## any AttachedCollisionObjects and add our plan to the trajectory.
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory);

        ## END_SUB_TUTORIAL


    def execute_traj_moveit(self, plan, blocking=False):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.

        ## BEGIN_SUB_TUTORIAL execute_plan
        ##
        ## Executing a Plan
        ## ^^^^^^^^^^^^^^^^
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:


        # TODO  TODO  TODO  TODO  TODO  TODO  TODO  TODO  TODO
        # ====================================================
        # TODO: replace first point with initial pose (or prepend it)
        # ====================================================


        self.move_group.execute(plan, wait=False)

        ## **Note:** The robot's current joint state must be within some tolerance of the
        ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
        ## END_SUB_TUTORIAL


    def convert_traj(self, plan):
        goal = copy.deepcopy(self.goal_blank)
        goal.trajectory.points = copy.deepcopy(plan.joint_trajectory.points)

        curr_pt = JointTrajectoryPoint( positions=plan.joint_trajectory.points[-1].positions,
                                        velocities=[0]*6,
                                        time_from_start = goal.trajectory.points[-1].time_from_start + rospy.Duration(0.5) )


        goal.trajectory.points.append(copy.deepcopy(curr_pt))

        return goal




    def execute_traj(self, goal_in, blocking=False):

        if self.traj_client is None:
            print("You cannot call execute_traj with the 'use_traj_client' option set True")
            raise


        if type(goal_in) is moveit_msgs.RobotTrajectory:
            goal = convert_traj(goal_in)

        if type(goal_in) is FollowJointTrajectoryGoal:
            goal = goal_in


        try:
            self.traj_client.send_goal(goal)

            if blocking:
                self.traj_client.wait_for_result()
            else:
                pass

        except KeyboardInterrupt:
            self.traj_client.cancel_goal()
            self.safe_stop()
            raise
        except:
            raise



    def wait_for_state_update(self, box_name=None, box_is_attached=False, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        scene = self.scene

        if box_name is not None:
                box_is_known = True
        else:
                box_is_known = False

        ## BEGIN_SUB_TUTORIAL wait_for_scene_update
        ##
        ## Ensuring Collision Updates Are Receieved
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## If the Python node dies before publishing a collision object update message, the message
        ## could get lost and the box will not appear. To ensure that the updates are
        ## made, we wait until we see the changes reflected in the
        ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
        ## For the purpose of this tutorial, we call this function after adding,
        ## removing, attaching or detaching an object in the planning scene. We then wait
        ## until the updates have been made or ``timeout`` seconds have passed
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
        ## END_SUB_TUTORIAL




    def add_world_collisions(self, timeout=4):

        # Add the table
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = self.robot.get_planning_frame()
        box_pose.pose.orientation.w = 10.0
        box_pose.pose.position.z = -0.25 # place it at the floor
        self.scene.add_box('table', box_pose, size=(2.5, 2.5, 0.5))
        self.wait_for_state_update(box_name='table', timeout=timeout)

        # Add the right-side wall
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = self.robot.get_planning_frame()
        box_pose.pose.orientation.x = 0.5
        box_pose.pose.orientation.y = 0.5
        box_pose.pose.orientation.z = 0.5
        box_pose.pose.orientation.w = 0.5
        box_pose.pose.position.z = 0.5 # slightly above the end effector
        box_pose.pose.position.x = -0.93 # slightly above the end effector
        self.scene.add_box('right-wall', box_pose, size=(2.5, 2.5, 0.005))
        self.wait_for_state_update(box_name='right-wall', timeout=timeout)

        # Add the back wall
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = self.robot.get_planning_frame()
        box_pose.pose.orientation.x = 0.5
        box_pose.pose.orientation.y = 0.5
        box_pose.pose.orientation.z = 0.5
        box_pose.pose.orientation.w = -0.5
        box_pose.pose.position.z = 0.5 # slightly above the end effector
        box_pose.pose.position.y = -0.70 # slightly above the end effector
        self.scene.add_box('back-wall', box_pose, size=(2.5, 2.5, 0.005))
        self.wait_for_state_update(box_name='back-wall', timeout=timeout)

        return True


 

def main():
    try:
        rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
        print ""
        print "----------------------------------------------------------"
        print "Welcome to the MoveIt MoveGroup Python Interface Tutorial"
        print "----------------------------------------------------------"
        print "Press Ctrl-D to exit at any time"
        print ""
        print "============ Press `Enter` to begin the tutorial by setting up the moveit_commander ..."
        raw_input()
        tutorial = MoveItPythonInteface()

        print "============ Press `Enter` to execute a movement using a joint state goal ..."
        raw_input()
        tutorial.go_to_joint_state()

        print "============ Press `Enter` to plan a movement using a pose goal ..."
        raw_input()
        tutorial.plan_pose_goal()

        print "============ Press `Enter` to execute a movement using a pose goal ..."
        raw_input()
        tutorial.go_to_pose_goal()

        print "============ Press `Enter` to plan and display a Cartesian path ..."
        raw_input()
        cartesian_plan, fraction = tutorial.plan_cartesian_path(scale=1.0)

        print "============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ..."
        raw_input()
        tutorial.display_trajectory(cartesian_plan)

        print "============ Press `Enter` to execute a saved path ..."
        raw_input()
        tutorial.execute_plan(cartesian_plan)


        print "============ Python tutorial demo complete!"
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        raise

if __name__ == '__main__':
    main()


