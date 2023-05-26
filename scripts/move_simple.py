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
import roslib#; roslib.load_manifest('ur_driver')
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
import pdb


## from  test_move
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from controller_manager_msgs.srv import SwitchControllerRequest, SwitchController
from controller_manager_msgs.srv import LoadControllerRequest, LoadController
from controller_manager_msgs.srv import ListControllers, ListControllersRequest
import geometry_msgs.msg as geometry_msgs
from cartesian_control_msgs.msg import (
    FollowCartesianTrajectoryAction,
    FollowCartesianTrajectoryGoal,
    CartesianTrajectoryPoint,
)
# All of those controllers can be used to execute joint-based trajectories.
# The scaled versions should be preferred over the non-scaled versions.
JOINT_TRAJECTORY_CONTROLLERS = [
    "scaled_pos_joint_traj_controller",
    "scaled_vel_joint_traj_controller",
    "pos_joint_traj_controller",
    "vel_joint_traj_controller",
    "forward_joint_traj_controller",
]

# All of those controllers can be used to execute Cartesian trajectories.
# The scaled versions should be preferred over the non-scaled versions.
CARTESIAN_TRAJECTORY_CONTROLLERS = [
    "pose_based_cartesian_traj_controller",
    "joint_based_cartesian_traj_controller",
    "forward_cartesian_traj_controller",
]
# We'll have to make sure that none of these controllers are running, as they will
# be conflicting with the joint trajectory controllers
CONFLICTING_CONTROLLERS = ["joint_group_vel_controller", "twist_controller"]

##




JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

curr_path=os.path.dirname(os.path.abspath(__file__))
pose_config = os.path.join(curr_path,"../config/arm_poses.yaml")



class PoseHandler():
    """
    Handle simple motions between saved poses
    """
    def __init__(self, pose_config, joint_names):
        """
        Parameters
        ----------
        pose_config : str
            The filepath to the pose config file (default='../config/arm_poses.yaml')
        joint_names : list(str)
            A list of the robot's joint names
        """
        self.pose_config = pose_config
        self.JOINT_NAMES = joint_names
        self.topics_connected = False
        self.move_time = 2.0
        self.ur_script_pub = None
        

        ## From test move
        timeout = rospy.Duration(5)
        self.switch_srv = rospy.ServiceProxy(
            "controller_manager/switch_controller", SwitchController
        )
        self.load_srv = rospy.ServiceProxy("controller_manager/load_controller", LoadController)
        self.list_srv = rospy.ServiceProxy("controller_manager/list_controllers", ListControllers)
        try:
            self.switch_srv.wait_for_service(timeout.to_sec())
        except rospy.exceptions.ROSException as err:
            rospy.logerr("Could not reach controller switch service. Msg: {}".format(err))
            sys.exit(-1)

        self.joint_trajectory_controller = JOINT_TRAJECTORY_CONTROLLERS[0]
        self.cartesian_trajectory_controller = CARTESIAN_TRAJECTORY_CONTROLLERS[0]
        ## end from test move

    def switch_controller(self, target_controller):
        """Activates the desired controller and stops all others from the predefined list above"""
        other_controllers = (
            JOINT_TRAJECTORY_CONTROLLERS
            + CARTESIAN_TRAJECTORY_CONTROLLERS
            + CONFLICTING_CONTROLLERS
        )

        other_controllers.remove(target_controller)

        srv = ListControllersRequest()
        response = self.list_srv(srv)
        for controller in response.controller:
            if controller.name == target_controller and controller.state == "running":
                return

        srv = LoadControllerRequest()
        srv.name = target_controller
        self.load_srv(srv)

        srv = SwitchControllerRequest()
        srv.stop_controllers = other_controllers
        srv.start_controllers = [target_controller]
        srv.strictness = SwitchControllerRequest.BEST_EFFORT
        self.switch_srv(srv)

    def _connect_topics(self):
        """
        Connect to the arm control action client and the ur_script topic
        """
        try:
            if not self.topics_connected:
                # pdb.set_trace()
                # make sure the correct controller is loaded and activated
                self.switch_controller(self.joint_trajectory_controller)
        
                self.arm_client = actionlib.SimpleActionClient("{}/follow_joint_trajectory".format(self.joint_trajectory_controller), FollowJointTrajectoryAction)
                # pdb.set_trace()
                self.ur_script_pub = rospy.Publisher('script_command', std_msgs.msg.String, queue_size=10)
                #hand_client = actionlib.SimpleActionClient('pre_built_traj', pressure_controller_ros.msg.RunAction)
                print "Waiting for servers..."
                timeout = rospy.Duration(5)
                if not self.arm_client.wait_for_server(timeout):
                    rospy.logerr("Coud not reach controller action server.")
                    sys.exit(-1)

                #hand_client.wait_for_server()
                print "Connected to servers"

                self.r=rospy.Rate(2)
                self.topics_connected = True
        except:
            self.topics_connected = False


    def set_default_move_time(self, time):
        """
        Set the default move time (in seconds)

        Parameters
        ----------
        time : float
            The desired duration of arm motions
        """
        if isinstance(time, float):
            if time>=0:
                self.move_time = time
                return True
        
        return False


    def _get_pose(self):
        """
        Get the current pose of the arm from the "joint_states" topic
        """
        joint_states = rospy.wait_for_message("joint_states", JointState)
        joints_pos = list(joint_states.position)
        return joints_pos


    def _get_saved_poses(self):
        """
        Load saved poses from the pose config file
        """
        if os.path.isfile(self.pose_config):
            with open(self.pose_config, 'r') as f:
                saved_poses = yaml.safe_load(f)
        else:
            saved_poses={}

        return saved_poses


    def _save_poses(self, poses):
        """
        Save a dictionary of poses to the pose config file
        """
        filename = self.pose_config
        dirname = os.path.dirname(filename)
        if not os.path.exists(dirname):
            os.makedirs(dirname)
        
        with open(self.pose_config, 'w+') as f:
            yaml.dump(poses,f,default_flow_style=None)


    def _list_pose(self, pose, pose_name, verbose=True):
        """
        Print out details of a single pose to the console

        Parameters
        ----------
        pose : dict
            A pose to print
        pose_name : str
            The name of the pose
        verbose : bool, optional
            Decide whether to include a verbose printout
        """

        if verbose:
            print(pose_name+':')
            print('    units: '+pose['units'])
            print('    Pose: ['+' '.join([str(pos) for pos in pose['joints']])+']')
            print("")
        else:
            print(pose_name)


    def _list_poses(self, verbose=False):
        """
        Print out details of all saved poses to the console

        Parameters
        ----------
        verbose : bool, optional
            Decide whether to include a verbose printout for each pose
        """
        saved_poses = self._get_saved_poses()
        for pose_name in sorted(saved_poses):
            self._list_pose(saved_poses[pose_name], pose_name, verbose)
        print('')


    def list_poses(self, verbose=False):
        """
        Print out details of all saved poses to the console cleanly

        Parameters
        ----------
        verbose : bool, optional
            Decide whether to include a verbose printout for each pose
        """
        print("")
        print("SAVED ARM POSES:")
        self._list_poses(verbose)


    def get_pose_by_name(self, pose_name):
        """
        Get a pose definition from the saved poses by name

        Parameters
        ----------
        pose_name : str
            The name of the pose to get (must be a valid dictionary key)
        """
        saved_poses = self._get_saved_poses()
        return saved_poses.get(pose_name, None)


    def set_pose_by_name(self, pose_name, pose):
        """
        Add a pose definition into the list of saved poses. If the pose already exists,
        the user is prompted whether to replace it with the new pose or abort.

        Parameters
        ----------
        pose_name : str
            The name of the pose to save (must be a valid dictionary key)
        pose : dict
            The pose to save 
        """
        saved_poses = self._get_saved_poses()

        if pose_name in list(saved_poses.keys()):
            response = raw_input('Are you sure you want to overwrite the pose: "%s"? ([Y]/n)'%(pose_name))
            if response.lower() == 'n':
                print('Aborting pose set')
                return False

        
        saved_poses[pose_name] = pose
        self._save_poses(saved_poses)
        return True


    def move_to_pose(self, pose_name, time=None):
        """
        Move the arm to a saved pose 

        Parameters
        ----------
        pose_name : str
            The name of the pose to move to (must be a valid dictionary key)
        time : float, optional
            The time duration of the move. (default = 2.0 sec)
        """
        # pdb.set_trace()
        pose = self.get_pose_by_name(pose_name)

        print(pose)
        pdb.set_trace()
        if pose is not None:
            self.move_to(pose, time)
        else:
            print("")
            print('ERROR: Pose with name "%s" not found in saved poses'%(pose_name))
            print("")
            print("VALID SAVED POSES ARE: ")
            self._list_poses(verbose=False)


    def set_pose(self, pose_name, from_freedrive=True):
        """
        Add a pose definition into the list of saved poses.

        Parameters
        ----------
        pose_name : str
            The name of the pose to save (must be a valid dictionary key)
        from_freedrive : bool, optional
            Enable freedrive mode before saving the pose. This allows users to
            move the arm to a pose, then save it. (default=True)
        """
        if from_freedrive:
            self.run_freedrive_mode()

        new_pose = {}
        new_pose['joints'] = self._get_pose()
        print("GOT POSE: ", new_pose['joints'])
        new_pose['units']  = 'radians'
        response = self.set_pose_by_name(pose_name, new_pose)
        if response:
            print("")
            print('POSE SAVED:')
            self._list_pose(new_pose, pose_name, verbose=True)


    def move_to(self, pose, time=None):
        """
        Move the arm from its current pose to a desired pose. This just moves
        directly in joint space, so beware of strange behaviors. 

        Parameters
        ----------
        pose : dict
            The pose definition to move to.
        time : float, optional
            The time duration of the move. (default = 2.0 sec)
        """
        time_duration = self.move_time
        if isinstance(time, float):
            if time>=0:
                time_duration = time


        self._connect_topics()
        joints = pose['joints']
        units = pose['units']

        if units == 'degrees':
            pose_rad = np.deg2rad(joints).tolist()
        else:
            pose_rad = joints

        joints_pos = self._get_pose()
        g = FollowJointTrajectoryGoal()
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = JOINT_NAMES
        g.trajectory.points = [
            JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
            JointTrajectoryPoint(positions=pose_rad, velocities=[0]*6, time_from_start=rospy.Duration(time_duration)),
            JointTrajectoryPoint(positions=pose_rad, velocities=[0]*6, time_from_start=rospy.Duration(time_duration+1.0))]

        try:
            self.arm_client.send_goal(g)
            self.arm_client.wait_for_result()

        except KeyboardInterrupt:
            self.arm_client.cancel_goal()
            raise
        except:
            raise


    def run_freedrive_mode(self):
        """
        Enable freeddrive mode, wait until the user is finished moving,
        then lock the arm's position.
        """
        self._connect_topics()
        # send URScript command that will enable teach mode
        try:
            print("Starting Freedrive Mode: You can now move the robot by hand!")
            #while not rospy.is_shutdown():
            self.ur_script_pub.publish('def myProg():\n\twhile (True):\n\t\tfreedrive_mode()\n\t\tsync()\n\tend\nend\n')
            response = raw_input('Press ENTER to save')
            print("\nExiting Freedrive Mode: The robot is now locked in place.")
            self.ur_script_pub.publish('def myProg():\n\twhile (True):\n\t\tend_freedrive_mode()\n\t\tsleep(0.5)\n\tend\nend\n')

        except KeyboardInterrupt:
            print("\nExiting Freedrive Mode: The robot is now locked in place.")
            self.ur_script_pub.publish('def myProg():\n\twhile (True):\n\t\tend_freedrive_mode()\n\t\tsleep(0.5)\n\tend\nend\n')




   
def main():
    """
    If this file is run using rosrun, interpret user inputs and execute.
    """
    try:
        if len(sys.argv)==2:
            cmd_type=str(sys.argv[1])
            position_name = None

        elif len(sys.argv)==3:
            cmd_type=str(sys.argv[1])
            position_name=str(sys.argv[2])
        
        rospy.init_node("test_move", anonymous=True, disable_signals=True)
        parameters = rospy.get_param(None)
        index = str(parameters).find('prefix')
        if (index > 0):
            prefix = str(parameters)[index+len("prefix': '"):(index+len("prefix': '")+str(parameters)[index+len("prefix': '"):-1].find("'"))]
            for i, name in enumerate(JOINT_NAMES):
                JOINT_NAMES[i] = prefix + name

        pose_handler = PoseHandler(pose_config, JOINT_NAMES)
        
        if cmd_type == 'goto' and position_name is not None:
            pose_handler.move_to_pose(position_name)
        
        elif cmd_type == 'set' and position_name is not None:
            pose_handler.set_pose(position_name)
        
        elif cmd_type == 'setnow' and position_name is not None:
            pose_handler.set_pose(position_name, from_freedrive=False)

        elif cmd_type == 'listnames' and position_name is None:
            pose_handler.list_poses()
        
        elif cmd_type == 'list' and position_name is None:
            pose_handler.list_poses(verbose=True)

    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__':
    main()