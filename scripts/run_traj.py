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
import pickle
import os
import sys
import errno
import matplotlib.pyplot as plt
from datetime import datetime

from pressure_controller_ros.live_traj_new import trajSender as pneu_traj_sender
from hand_arm_cbt.arm_mover import trajSender as ur_traj_sender
from robotiq_trajectory_control.robotiq_2f_trajectory import trajSender as robotiq_traj_sender
from hand_arm_cbt.arm_moveit import MoveItPythonInteface as ur_traj_sender_moveit
import rosbag_recorder.srv as rbr
import video_recorder.srv as vrec


reset_time = 2.0

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

curr_path=os.path.dirname(os.path.abspath(__file__))
filepath_traj = os.path.join(curr_path,'..','trajectories')
filepath_config = os.path.join(curr_path,'..','config')

#save_data_folder = 'Documents/data'

save_data_folder = '/media/armando/LaCie/robot_data'



class TrajRunner:
    def __init__(self):
        self.traj_profile = rospy.get_param(rospy.get_name()+'/traj_profile')
        self.speed_factor = rospy.get_param(rospy.get_name()+'/speed_factor',1.0)
        self.num_reps = rospy.get_param(rospy.get_name()+'/num_reps',1)
        self.replan = rospy.get_param(rospy.get_name()+'/replan',False)
        self.starting_index = rospy.get_param(rospy.get_name()+'/start',0)
        self.fake = rospy.get_param(rospy.get_name()+'/fake',False)
        self.out_id = rospy.get_param(rospy.get_name()+'/out_id',"")
        self.use_camera = rospy.get_param(rospy.get_name()+'/use_camera',"")

        self.use_arm = rospy.get_param(rospy.get_name()+'/use_arm',True)
        self.use_hand = rospy.get_param(rospy.get_name()+'/use_hand',True)
        self.use_servo = rospy.get_param(rospy.get_name()+'/use_servo',False)
        self.use_tags = rospy.get_param(rospy.get_name()+'/use_tags',True)
        self.save_data = rospy.get_param(rospy.get_name()+'/save_data',False)
        self.use_checklist = rospy.get_param(rospy.get_name()+'/use_checklist',True)
        self.DEBUG  = rospy.get_param(rospy.get_name()+'/debug',False)
        self.saving_now = False
        self.curr_file = None
        self.curr_rep  = None
        

        # Read the trajectory configuration file
        self.filepath = os.path.join(filepath_traj)

        if self.traj_profile.endswith(".traj"):
            self.filepath = os.path.join(self.filepath,os.path.dirname(self.traj_profile))
            self.in_files=[os.path.basename(self.traj_profile)]

        else:
            self.filepath =   os.path.join(self.filepath,self.traj_profile)
            self.in_files = [f for f in os.listdir(self.filepath) if (os.path.isfile(os.path.join(self.filepath, f)) and f.endswith(".traj"))]
            self.in_files.sort()
            self.in_files=self.in_files[self.starting_index:]
            
            if self.DEBUG:
                print("STARTING AT POSITION: %d"%(self.starting_index))

        self.save_data_folder = self.get_save_locations(os.path.join(filepath_config,'save_config.yaml'))
        self.save_folder_curr = None


        config_file =   os.path.join(self.filepath,self.in_files[0])

        with open(config_file,'r') as f:

            if config_file.endswith('.traj'):
                self.traj_config = pickle.load(f)
            else:
                self.traj_config = yaml.safe_load(f)

            self.operations = self.traj_config['sequence']
            f.close()


        setup = self.operations['setup']

        # Create the arm objects
        if self.use_arm:
            if (setup['arm_traj_space']== 'joint') or (setup['arm_traj_space']== 'joint-planned'):
                self.arm_sender = ur_traj_sender(JOINT_NAMES, self.speed_factor)

            elif setup['arm_traj_space'] == 'cartesian':
                self.arm_sender = ur_traj_sender_moveit(JOINT_NAMES)

                #configure the planners based on the config file
                self.arm_sender.config_planner(os.path.join(filepath_config,'moveit_config.yaml'))
            else:
                if self.DEBUG:
                    print('Nonstandard arm trajectory space')
                self.arm_sender = None
                raise
        else:
            if self.DEBUG:
                print('Not using arm - Arm moves will be skipped')

        # Create the pneumatic hand object
        if self.use_hand:
            if setup['hand_traj_space'] == 'pressure':
                self.hand_sender = pneu_traj_sender(self.speed_factor)
                self.hand_sender.DEBUG=self.DEBUG
            elif setup['hand_traj_space'] == 'robotiq':
                if self.DEBUG:
                    print("getting for robotiq hand traj server")
                self.hand_sender = robotiq_traj_sender(self.speed_factor)
                self.hand_sender.DEBUG=self.DEBUG
                if self.DEBUG:
                    print("waiting for robotiq hand traj server")
            else:
                if self.DEBUG:
                    print('Nonstandard hand trajectory space')
                raise
        else:
            if self.DEBUG:
                print('Not using hand - Hand moves will be skipped')

        # Create the pneumatic hand object
        if self.use_servo:
                self.servo_sender = pneu_traj_sender(self.speed_factor, name='servo')
        else:
            if self.DEBUG:
                print('Not using servo - Servo moves will be skipped')


        if self.save_data:
            self.createOutFolder(self.traj_profile)


        # create the checklist topic
        if self.use_checklist:
            if self.save_folder_curr is None:
                self.createOutFolder(self.traj_profile)

            self.createSuccessFile('success')
            self.checklist_pub = rospy.Publisher('/trial_success', TrialSuccess, queue_size=10)
            


    def get_save_locations(self, config_filename):
        # Get save locations
        with open(config_filename,'r') as f:
            # use safe_load instead of load
            save_config = yaml.safe_load(f)
        
        # Try the user-specified directory
        data_folder_out = save_config.get('save_folder',None)
        directory_exists = True

        if data_folder_out is not None:
            if not os.path.exists(data_folder_out):
                try:
                    os.makedirs(data_folder_out)
                    directory_exists = True
                except OSError as e:
                    if e.errno != errno.EEXIST:
                        directory_exists = False

        if not directory_exists:
            if self.DEBUG:
                print('SAVE SETUP: Save directory did not exist. Trying default save directory')
            data_folder_out = save_config.get('save_folder_default',None)
            if data_folder_out is not None:
                if not os.path.exists(data_folder_out):
                    try:
                        os.makedirs(data_folder_out)
                        directory_exists = True
                    except OSError as e:
                        if e.errno != errno.EEXIST:
                            directory_exists = False
            
                else:
                    directory_exists = True

        
        if not directory_exists:
            print('ERROR: Please use an existing data directory in save_config.yaml')
            raise
        else:
            print('SAVE SETUP: Saving in: %s'%(data_folder_out))
        
        return(data_folder_out)
        

    def start_saving(self):
        rospy.wait_for_service('rosbag_recorder/record_topics')

        if self.use_camera:
            rospy.wait_for_service('video_recorder/record')

        # generate the topic list
        topic_list = []
        if self.use_checklist:
            topic_list.extend(['/trial_success'])

        if self.use_arm:
            topic_list.extend(['/joint_states','/wrench','/tool_velocity'])
        if self.use_hand:
            topic_list.extend(['/pressure_control/echo','/pressure_control/pressure_data'])
        if self.use_servo:
            topic_list.extend(['/servo/echo','/servo/pressure_data'])
        
        if self.use_tags:
            topic_list.extend(['/tag_detections'])

        try:
            service = rospy.ServiceProxy('rosbag_recorder/record_topics', rbr.RecordTopics)
            response = service(self.out_filename, topic_list)

            if self.use_camera:
                service = rospy.ServiceProxy('video_recorder/record', vrec.RecordVideo)
                response = service(self.out_filename.replace('.bag','.mp4'))

            self.saving_now = True
            return response.success
            
        except rospy.ServiceException, e:
            if self.DEBUG:
                print "Service call failed: %s"%e


    def stop_saving(self):
        try:
            service = rospy.ServiceProxy('rosbag_recorder/stop_recording', rbr.StopRecording)
            response = service(self.out_filename)

            if self.use_camera:
                service = rospy.ServiceProxy('video_recorder/stop_recording', vrec.StopRecording)
                response = service(self.out_filename.replace('.bag','.mp4'))
            
            self.saving_now = False
            return response.success
        except rospy.ServiceException, e:
            if self.DEBUG:
                print "Service call failed: %s"%e



    def createOutFolder(self,filename):
        now = datetime.now()
        print(filename)
        folder, file = os.path.split(filename)
        self.save_folder_curr = os.path.join(folder, self.out_id, file+'_'+ now.strftime("%Y%m%d_%H%M%S"))

        dirname = os.path.abspath(os.path.join(os.path.expanduser('~'),self.save_data_folder,self.save_folder_curr))
        if not os.path.exists(dirname):
            os.makedirs(dirname)

        


    def createOutFile(self,filename=None):

        if filename is None:
            #print(self.curr_file)
            #print(self.curr_rep)
            if (self.curr_file is None) or (self.curr_rep is None):
                return None

            filename = '%s_rep%04d'%(self.curr_file.replace('.traj',''),self.curr_rep)

        outFile=os.path.abspath(os.path.join(os.path.expanduser('~'),self.save_data_folder,self.save_folder_curr,filename))
        #print(outFile)

        return "%s.bag" % (outFile)



    def createSuccessFile(self,filename=None):

        if not filename:
            #print(self.curr_file)
            #print(self.curr_rep)
            if (self.curr_file is None) or (self.curr_rep is None):
                return None

            filename = '%s_rep%04d'%(self.curr_file.replace('.traj',''),self.curr_rep)

        outFile=os.path.abspath(os.path.join(os.path.expanduser('~'),self.save_data_folder,self.save_folder_curr,filename))
        #print(outFile)

        self.success_filename = "%s.suc" % (outFile)

        with open(self.success_filename,'w') as f:
            f.write("Trial Name,Rep,Success"+'\n');




    def run_multiple(self):

        #try:

        for file in self.in_files:
            self.curr_file = file
            print('\n'+file)
            config_file =   os.path.join(self.filepath,file)

            with open(config_file,'r') as f:
                self.traj_config = pickle.load(f)
            self.operations = self.traj_config['sequence']
            self.settings = self.traj_config['settings']

            self.get_sequence()

            # Go to the start
            #inp = raw_input("Move to Starting Position? (Press ENTER)")
            self.go_to_start()
            self.plan_sequence()

            # Excecute the trajectory the desired number of times
            self.rep_sequence(wait_before_each = False)
                

        #except KeyboardInterrupt:
        #    self.shutdown()
        #    rospy.signal_shutdown("KeyboardInterrupt")
        #    raise



    def get_sequence(self):
        # get the trajectories
        self.operation_sequence = []

        for move in self.operations['operations']:
            out = {}
            out['arm']  = None
            out['hand'] = None
            out['servo'] = None
            if move['arm']:
                 out['arm'] = self.traj_config['arm'].get(move['arm'], None)

            if move['hand']:
                out['hand'] = self.traj_config['hand'].get(move['hand'], None)

            if move.get('servo',False) and self.traj_config.get('servo',False):
                out['servo'] = self.traj_config['servo'].get(move['servo'], None)

            self.operation_sequence.append(out)


    def go_to_start(self):
        print('Go To Start')
        # Start the trajectories
        if self.use_hand:
            self.hand_sender.go_to_start(self.operation_sequence[0]['hand'], reset_time, blocking=False)
        if self.use_servo:
            self.servo_sender.go_to_start(self.operation_sequence[0]['servo'], reset_time, blocking=False)
        if self.use_arm:
            self.arm_sender.go_to_start(self.operation_sequence[0]['arm'], reset_time, blocking=False)

        # Wait for the traj to finish
        if self.use_hand:
            self.hand_sender.traj_client.wait_for_result()
        if self.use_servo:
            self.servo_sender.traj_client.wait_for_result()
        if self.use_arm:
            self.arm_sender.traj_client.wait_for_result()



    def plan_sequence(self):

        self.operation_plans = []

        if self.replan:            
            for movement in self.operation_sequence:
                out = {}
                out['arm']  = None
                out['hand'] = None
                out['servo'] = None
                if movement['arm'] is not None:
                    if self.use_arm:
                        out['arm'] = self.arm_sender.build_traj(movement['arm'])
                    else:
                        out['arm'] = None

                if movement['hand'] is not None:
                    if self.use_hand:
                        out['hand'] = self.hand_sender.build_traj(movement['hand'])
                    else:
                        out['hand'] = None

                if movement['servo'] is not None:
                    if self.use_servo:
                        out['servo'] = self.servo_sender.build_traj(movement['servo'])
                    else:
                        out['servo'] = None

                self.operation_plans.append(out)

        else:
            for movement in self.operation_sequence:
                out = {}
                out['arm']  = None
                out['hand'] = None
                out['servo'] = None
                if movement['arm'] is not None:
                    if self.use_arm:
                        out['arm'] = self.arm_sender.build_traj(movement['arm'])
                    else:
                        out['arm'] = None
                if movement['hand'] is not None:
                    if self.use_hand:
                        out['hand'] = self.hand_sender.build_traj(movement['hand'])
                    else:
                        out['hand'] = None

                if movement['servo'] is not None:
                    if self.use_servo:
                        out['servo'] = self.servo_sender.build_traj(movement['servo'])
                    else:
                        out['servo'] = None

                self.operation_plans.append(out)




    def excecute_sequence(self):
        print('Excecute Sequence')
        for plan in self.operation_plans:
            if (plan['hand'] is not None) and (self.use_hand):
                self.hand_sender.execute_traj(plan['hand'], blocking=False)

            if (plan['servo'] is not None) and (self.use_servo):
                self.servo_sender.execute_traj(plan['servo'], blocking=False)

            if (plan['arm'] is not None) and (self.use_arm):
                if not self.fake:
                    self.arm_sender.execute_traj(plan['arm'], blocking=False)
                else:
                    self.arm_sender.display_trajectory(plan['arm'])

            if self.use_hand:
                self.hand_sender.traj_client.wait_for_result()
            if self.use_servo:
                self.servo_sender.traj_client.wait_for_result()
            if self.use_arm:
                self.arm_sender.traj_client.wait_for_result()

            
        
    def rep_sequence(self, wait_before_each = True):
        for idx in range(self.num_reps):
            self.curr_rep=idx
            print('REP: %d'%(idx))
            if wait_before_each:
                inp = raw_input("Start? (Press ENTER)")

            if self.save_data:
                self.out_filename=self.createOutFile()
                self.start_saving()

            self.excecute_sequence()

            
            if self.save_data:
                self.stop_saving()

            # get user input about the success or failure of the trial
            if self.use_checklist:
                inp=None
                while (type(inp) != int):
                    inp = input("Was the trial successful? (0,1,2...9) ")

                self.mark_success(inp)

            if not self.settings.get('reset_object', False):
                self.go_to_start()



    def mark_success(self,success):
        
        print("Trial Marked: %d"%(success))
        out = success
        out_str ="%s,%d,%d\n"%(self.curr_file.replace('.traj',''), self.curr_rep, out)
        with open(self.success_filename,'a') as f:
            f.write(out_str)


    def shutdown(self, hard=False):
        if self.save_data:
            self.stop_saving()
            print('-Stopped data logger')

        if self.use_hand:
            print('Stopping hand controller')
            self.hand_sender.shutdown(reset_pressures='resting')
            #self.hand_sender.shutdown(reset_pressures=[2,0])
            print('-Stopped hand controller')
        
        if self.use_servo:
            print('Stopping servo controller')
            self.servo_sender.shutdown(reset_pressures='resting')
            #self.hand_sender.shutdown(reset_pressures=[2,0])
            print('-Stopped servo controller')

        if hard:
            if self.use_arm:
                self.arm_sender.shutdown()
                print('-Stopped arm controller (hard)')

   


def main(file_name=None):
    try:
        rospy.init_node("trajectory_runner", anonymous=True, disable_signals=True)

        node = TrajRunner()
        node.run_multiple()
        node.shutdown()

    except KeyboardInterrupt:
        print("Shutting Down Top-Level Node")
        node.shutdown()
        print("Shutting Down ROS")
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__':
    if len(sys.argv) < 2 or len(sys.argv) > 2:
        main()
    elif len(sys.argv) ==2:
        main(sys.argv[1])
    else:
        print("Usage:")
        print("\ttrajectory_runner.py [FILENAME]\t- Run a trajectory")
