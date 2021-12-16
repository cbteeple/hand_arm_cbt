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

import rospy
from math import pi
import yaml
import os
import shutil
import errno
import sys
import matplotlib.pyplot as plt
import numpy as np
import copy
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from pressure_controller_skills.build_skills import SkillBuilder




curr_path=os.path.dirname(os.path.abspath(__file__))
filepath_traj = os.path.join(curr_path,'..','traj_setup')
filepath_out = os.path.join(curr_path,'..','trajectories')








class pickPlaceBuild:
    def __init__(self):
        self.traj_profile = rospy.get_param(rospy.get_name()+'/traj_profile')

        

        # Read the trajectory configuration file
        self.filepath = os.path.join(filepath_traj)
        self.filepath_out_dir = os.path.join(filepath_out,self.traj_profile)
        self.config_file =   os.path.join(self.filepath,self.traj_profile+'.yaml')

        with open(self.config_file,'r') as f:
            # use safe_load instead of load
            self.config = yaml.safe_load(f)
            f.close()

        self.reset_object = self.config['settings'].get('reset_object', False)
        self.pattern_type = self.config['settings']['type']

        self.use_servo = self.config['settings'].get('use_servo',False)

        self.traj_patterned = []

        self.max_p = self.config['hand'].get("max_pressure", np.inf)
        self.min_p  = self.config['hand'].get("min_pressure", -np.inf)
        self.hand_type=self.config['hand'].get('type','pressure_controlled')

        skill_context = rospy.get_param('/config_node/profile_name',[])
        self.skill_builder = SkillBuilder(context=skill_context,skill_package='hand_arm', skill_folder='hand_skills')



    def create_traj_pattern(self):

        # Make a grid if that's what the file says to do
        if self.pattern_type == 'grid':
            grid_settings = self.config['arm']['grid']

            num_pts = grid_settings['num_pts']
            dims = grid_settings['dims']

            x = np.linspace(-dims[0]/2, dims[0]/2, num_pts[0])
            y = np.linspace(-dims[1]/2, dims[1]/2, num_pts[1])
            z = np.linspace(-dims[2]/2, dims[2]/2, num_pts[2])

            x1,x2,x3 = np.meshgrid(x,y,z)

            x1 = x1.flatten()
            x2 = x2.flatten()
            x3 = x3.flatten()

            self.perturbations = np.vstack((x1,x2,x3)).transpose().tolist()

            rel_dims = grid_settings.get('release_dims',None)

            if rel_dims is not None:
                x = np.linspace(-rel_dims[0]/2, rel_dims[0]/2, num_pts[0])
                y = np.linspace(-rel_dims[1]/2, rel_dims[1]/2, num_pts[1])
                z = np.linspace(-rel_dims[2]/2, rel_dims[2]/2, num_pts[2])

                x1,x2,x3 = np.meshgrid(x,y,z)

                x1 = x1.flatten()
                x2 = x2.flatten()
                x3 = x3.flatten()

            self.release_perturbations = np.vstack((x1,x2,x3)).transpose().tolist()

        else:
            self.perturbations = [0,0,0]
            self.release_perturbations = [0,0,0]






    def build_traj(self):
        self.create_traj_pattern()

        self.trajectory_built = {}
        self.build_sequence()
        self.build_grasp('hand')
        if self.use_servo:
            self.build_grasp('servo')

        base_config = copy.deepcopy(self.config)

        perturb_num = 0

        self.prep_directory(self.filepath_out_dir)

        if self.pattern_type == 'grid':
            shift_initial_pose = self.config['arm']['grid'].get('affects_initial_pose',True)
            shift_release_pose = self.config['arm']['grid'].get('affects_release_pose',True)

            for entry, release_entry in zip(self.perturbations, self.release_perturbations):
                
                # Update with current perturbation
                curr_config = copy.deepcopy(base_config)

                if shift_initial_pose:
                    if isinstance(curr_config['arm']['initial_pose'], list):
                        for row_idx, row in enumerate(curr_config['arm']['initial_pose']):
                            for axis_idx, axis in enumerate(row['position']):
                                curr_config['arm']['initial_pose'][row_idx]['position'][axis_idx] = axis + entry[axis_idx]

                    else:
                        for idx, axis in enumerate(curr_config['arm']['initial_pose']['position']):
                            curr_config['arm']['initial_pose']['position'][idx] = axis + entry[idx]



                if isinstance(curr_config['arm']['grasp_pose'], list):
                    for row_idx, row in enumerate(curr_config['arm']['grasp_pose']):
                        for axis_idx, axis in enumerate(row['position']):
                            curr_config['arm']['grasp_pose'][row_idx]['position'][axis_idx] = axis + entry[axis_idx]

                else:
                    for idx, axis in enumerate(curr_config['arm']['grasp_pose']['position']):
                        curr_config['arm']['grasp_pose']['position'][idx] = axis + entry[idx]


                if shift_release_pose:
                    if isinstance(curr_config['arm']['release_pose'], list):
                        for row_idx, row in enumerate(curr_config['arm']['release_pose']):
                            for axis_idx, axis in enumerate(row['position']):
                                curr_config['arm']['release_pose'][row_idx]['position'][axis_idx] = axis + entry[axis_idx]
                    else:
                        for idx, axis in enumerate(curr_config['arm']['release_pose']['position']):
                            curr_config['arm']['release_pose']['position'][idx] = axis + release_entry[idx]

                self.build_moves(curr_config)

                out_file =   os.path.join(self.filepath_out_dir,"pos_"+"%04d"%(perturb_num)+'.yaml')

                self.save_traj(out_file)

                perturb_num+=1

        else:
            self.build_moves(base_config)
            out_file =   os.path.join(self.filepath_out_dir,"pos_"+"%04d"%(0)+'.yaml')
            self.save_traj(out_file)


    def prep_directory(self, out_dir):
        if not os.path.exists(out_dir):
            try:
                os.makedirs(out_dir)
            except OSError as exc: # Guard against race condition
                if exc.errno != errno.EEXIST:
                    raise

        else:
            #Remove all existing files
            filelist = [ f for f in os.listdir(out_dir)]
            for f in filelist:
                os.remove(os.path.join(out_dir, f))



    def save_traj(self, out_file):

        self.trajectory_built['settings'] = self.config['settings']

        with open(out_file, 'w+') as f:
            f.write('# This file was automatically generated by the hand_arm package\n# (https://github.com/cbteeple/hand_arm_cbt)\n# DO NOT EDIT THIS FILE DIRECTLY UNLESS YOU KNOW WHAT YOU ARE DOING!\n')
            yaml.dump(self.trajectory_built, f, default_flow_style=None)



    def build_grasping_sequence(self, grasp_hand_when, grasp_servo_when):
        grasping_move = [{'arm': False, 'hand': False, 'servo': False},
                         {'arm': False, 'hand': False, 'servo': False},
                         {'arm': False, 'hand': False, 'servo': False}]

        if grasp_hand_when == "during" and grasp_servo_when == "during" :
            grasping_move[0]['arm'] = 'grasp_move'
            grasping_move[0]['hand'] = 'grasp'
            grasping_move[0]['servo'] = 'grasp'

        else: #release after by default
            grasping_move[1]['arm'] = 'grasp_move'
            if grasp_hand_when == "before":
                grasping_move[0]['hand'] = 'grasp'
            elif grasp_hand_when == "during":
                grasping_move[1]['hand'] = 'grasp'
            else:
                grasping_move[2]['hand'] = 'grasp'

            if grasp_servo_when == "before":
                grasping_move[0]['servo'] = 'grasp'
            elif grasp_servo_when == "during":
                grasping_move[1]['servo'] = 'grasp'
            else:
                grasping_move[2]['servo'] = 'grasp'

        return grasping_move



    def build_sequence(self):
        # get the trajectories
        
        sequence = {}

        sequence['setup'] = {}
        sequence['setup']['arm_traj_space']  = 'cartesian'
        if 'robotiq' in self.hand_type:
            sequence['setup']['hand_traj_space'] = 'robotiq'
        elif 'dynamixel' in self.hand_type:
            sequence['setup']['hand_traj_space'] = 'dynamixel'
        else:
            sequence['setup']['hand_traj_space'] = 'pressure'

        sequence['startup'] = {}
        sequence['startup']['arm'] = 'pre'
        sequence['startup']['hand'] = 'startup'
        sequence['startup']['servo'] = 'startup'

        sequence['operations'] = []
        ops = sequence['operations']

        ops.append({'arm': 'pre', 'hand': 'startup', 'servo': 'startup'})

        grasp_hand_when = self.config['hand'].get('grasp_start','after')
        grasp_servo_when = self.config.get('servo',{'grasp_start':'after'})
        grasp_servo_when = grasp_servo_when.get('grasp_start','after')

        grasping_move = self.build_grasping_sequence(grasp_hand_when,grasp_servo_when)

        ops.extend(grasping_move)

        #if self.config['hand'].get('grasp_start','after') == "during" and self.config['servo'].get('grasp_start','after') == "during":
        #    ops.append({'arm': 'grasp_move', 'hand': 'grasp', 'servo': 'grasp'})
        #elif self.config['hand'].get('grasp_start','after') == "during" and self.config['servo'].get('grasp_start','after') != "during":
        #    ops.append({'arm': 'grasp_move', 'hand': 'grasp', 'servo': None})
        #    ops.append({'arm': None, 'hand': None, 'servo': 'grasp'})
        #elif self.config['hand'].get('grasp_start','after') != "during" and self.config['servo'].get('grasp_start','after') == "during":
        #    ops.append({'arm': 'grasp_move', 'hand': None, 'servo': 'grasp'})
        #    ops.append({'arm': None, 'hand': 'grasp', 'servo': None})
        #else:
        #    ops.append({'arm': 'grasp_move', 'hand': False, 'servo': False})
        #    ops.append({'arm': False, 'hand': 'grasp', 'servo': 'grasp'})
        
        ops.append({'arm': 'move', 'hand': False, 'servo': False})

        if self.config['arm']['pickup']['type'] == 'to_pose':
            ops.append({'arm': 'manip_before', 'hand': False, 'servo': False})

        hand_manip = self.config['hand'].get('manip_sequence', None)
        arm_manip = self.config['arm'].get('manip_sequence', None)
        if hand_manip and arm_manip:
            ops.append({'arm': 'manip_seq_before', 'hand': False, 'servo': False})
            ops.append({'arm': 'manip', 'hand': 'manip', 'servo': False})
            ops.append({'arm': 'manip_seq_after', 'hand': False, 'servo': False})
        elif hand_manip:
            ops.append({'arm': False, 'hand': 'manip', 'servo': False})
        elif arm_manip:
            ops.append({'arm': 'manip_seq_before', 'hand': False, 'servo': False})
            ops.extend(self.extra_manip_sequence)
            ops.append({'arm': 'manip', 'hand': False, 'servo': False})
            ops.append({'arm': 'manip_seq_after', 'hand': False, 'servo': False})
        
        if self.config['arm']['pickup']['type'] == 'to_pose':
            ops.append({'arm': 'manip_after', 'hand': False, 'servo': False})

        release_when = self.config['hand'].get('release_start','after')
        release_servo = self.config.get('servo',{'release_start':'after'})
        release_servo= release_servo.get('release_start','after')
        rel_move =      [{'arm': False, 'hand': False, 'servo': False},
                         {'arm': False, 'hand': False, 'servo': False},
                         {'arm': False, 'hand': False, 'servo': False}]
        if release_when == "during" and release_servo == "during" :
            rel_move[0]['arm'] = 'release_move'
            rel_move[0]['hand'] = 'release'
            rel_move[0]['servo'] = 'release'

        else: #release after by default
            rel_move[1]['arm'] = 'release_move'
            if release_when == "before":
                rel_move[0]['hand'] = 'release'
            else:
                rel_move[2]['hand'] = 'release'

            if release_servo == "before":
                rel_move[0]['servo'] = 'release'
            else:
                rel_move[2]['servo'] = 'release'

        ops.extend(rel_move)
        ops.append({'arm': 'post', 'hand': False,'servo': False})

        if self.reset_object:
            ops.append({'arm': 'post_inv', 'hand': False,'servo': False})
            rel_move_inv=copy.deepcopy(list(reversed(rel_move)))
            grasping_move_inv=copy.deepcopy(list(reversed(grasping_move)))

            grasping_move_inv = self.build_grasping_sequence('before','before')

            for item in rel_move_inv:
                if item['arm'] == 'release_move':
                    item['arm'] = 'release_move_inv'

                if item['hand'] == 'release':
                    item['hand'] = 'grasp'

            for item in grasping_move_inv:
                if item['arm'] == 'grasp_move':
                    item['arm'] = 'grasp_move_inv'

                if item['hand'] == 'grasp':
                    item['hand'] = 'release'

            ops.extend(rel_move_inv)

            if self.config['arm']['pickup']['type'] == 'to_pose':
                ops.append({'arm': 'manip_after_inv', 'hand': False, 'servo': False})

            if self.config['hand'].get('manip_sequence',False):
                ops.append({'arm': False, 'hand': 'manip', 'servo': False})

            if self.config['arm']['pickup']['type'] == 'to_pose':
                ops.append({'arm': 'manip_before_inv', 'hand': False, 'servo': False})

            ops.append({'arm': 'move_inv', 'hand': False, 'servo': False})
            
            ops.extend(grasping_move_inv)

            ops.append({'arm': 'pre_inv', 'hand': 'startup_inv', 'servo': 'startup'})

        #print(ops)

        self.trajectory_built['sequence'] = sequence


    def dict_to_list(self, dict, keys):
        out = []
        for key in keys:
            val = dict.get(key, None)
            if val is not None:
                val = float(val)
            out.append(val)
        print(out)
        return out



    def build_grasp(self, channel='hand'):
        hand_moves = {}


        wait_before = self.config[channel].get('wait_before_grasp',0.0)
        grasp_duration = self.config[channel].get('grasp_time',0.0)
        wait_after  = self.config[channel].get('wait_after_grasp',0.0)

        if 'robotiq' in self.hand_type or 'dynamixel' in self.hand_type:
            self.keylist = ['pos','speed','force', 'continuous']
            idle  = self.config[channel]['idle_pos']
            grasp = self.config[channel]['grasp_pos']
            initial = self.config[channel].get('initial_pos')
            act_kwd = 'pos'

            hand_moves['grasp']= []
            if wait_before>0.0:
                hand_moves['grasp'].append(self.build_pressure_vec(initial, 0.0))

            grasp_sequence = self.config[channel].get('grasp_sequence',None)
            if grasp_sequence is not None:
                grasp_duration = self.config[channel]['grasp_sequence']['sequence'][-1]['time']

                for row in self.config[channel]['grasp_sequence']['sequence']:
                    new_row = self.config[channel][row[act_kwd]]
                    hand_moves['grasp'].append(self.build_pressure_vec(new_row, wait_before+row['time']))

                grasp_end= self.config[channel][self.config[channel]['grasp_sequence']['sequence'][-1][act_kwd]]
            
            else:
                hand_moves['grasp'].append(self.build_pressure_vec(grasp, wait_before+grasp_duration))
                hand_moves['grasp'].append(self.build_pressure_vec(grasp, wait_before+grasp_duration+wait_after))
                grasp_end= grasp
        
        else:
            act_kwd = 'pressure'
            

        if 'robotiq' not in self.hand_type and 'dynamixel' not in self.hand_type:
            # Build the grasping move
            grasp_sequence = self.config[channel].get('grasp_sequence',None)

            if grasp_sequence is not None:
                grasp_type=grasp_sequence.get('type', None)

                if grasp_type == 'skill':
                    grasp_skills = grasp_sequence.get('sequence', [])

                    if len(grasp_skills)>0:
                        grasp_traj = self.build_skill_sequence(grasp_skills)

                        if wait_before >0:
                            hand_moves['grasp'] =  [self.build_pressure_vec(grasp_traj[0]['pressure'], grasp_traj[0]['time'])]
                            grasp_traj = self.skill_builder.adjust_points(grasp_traj,wait_before)
                        
                        else:
                            hand_moves['grasp'] =  []
                        
                        # Convert trajectory to correct format
                        for row in grasp_traj:
                            hand_moves['grasp'].append(self.build_pressure_vec(row['pressure'], row['time']))

                        grasp_duration = grasp_traj[-1]['time']
                        grasp_end      = grasp_traj[-1]['pressure']    
                
                elif grasp_type == 'trajectory':
                    grasp_duration = self.config[channel]['grasp_sequence']['sequence'][-1]['time']

                    for row in self.config[channel]['grasp_sequence']['sequence']:
                        new_row = self.config[channel][row[act_kwd]]
                        hand_moves['grasp'].append(self.build_pressure_vec(new_row, wait_before+row['time']))

                    grasp_end= self.config[channel][self.config[channel]['grasp_sequence']['sequence'][-1][act_kwd]]

                else:
                    grasp_duration = self.config[channel]['grasp_sequence'][-1]['time']

                    for row in self.config[channel]['grasp_sequence']:
                        new_row = self.config[channel][row[act_kwd]]
                        hand_moves['grasp'].append(self.build_pressure_vec(new_row, wait_before+row['time']))

                    grasp_end= self.config[channel][self.config[channel]['grasp_sequence'][-1][act_kwd]]
                
                
            else:
                idle  = [ float(x) for x in self.config[channel]['idle_pressure'] ]
                grasp = [ float(x) for x in self.config[channel]['grasp_pressure'] ]
                initial = [ float(x) for x in self.config[channel].get('initial_pressure',idle) ]

                hand_moves['grasp']= [self.build_pressure_vec(initial, 0.0)]
                hand_moves['grasp'].append(self.build_pressure_vec(initial, wait_before))
                hand_moves['grasp'].append(self.build_pressure_vec(grasp, wait_before+grasp_duration))

                grasp_end= grasp

            if wait_after >0:
                hand_moves['grasp'].append(self.build_pressure_vec(grasp_end, wait_before+grasp_duration+wait_after))

            # Build the manipulation move
            manip_sequence = self.config[channel].get('manip_sequence',None)

            manip_duration = 0.0
            if manip_sequence is not None:
                manip_type=manip_sequence.get('type', None)
                hand_moves['manip'] = []

                if manip_type == 'skill':
                    manip_skills = manip_sequence.get('sequence', [])

                    if len(manip_skills)>0:
                        manip_traj = self.build_skill_sequence(manip_skills)
                        # Convert trajectory to correct format
                        for row in manip_traj:
                            hand_moves['manip'].append(self.build_pressure_vec(row['pressure'], row['time']))
                            
                        manip_duration = manip_traj[-1]['time']
                        grasp_end      = manip_traj[-1]['pressure']

                else:
                    num_reps=int(self.config[channel].get('manip_repeat',1))

                    manip_duration = self.config[channel]['manip_sequence'][-1]['time']

                    first_point = self.config[channel]['manip_sequence'][0]
                    self.config[channel]['manip_sequence'].pop(0)

                    hand_moves['manip'].append(self.build_pressure_vec(self.config[channel][first_point[act_kwd]], first_point['time']))
                    for rep in range(num_reps):
                        for row in self.config[channel]['manip_sequence']:
                            hand_moves['manip'].append(self.build_pressure_vec(self.config[channel][row[act_kwd]], rep*manip_duration+row['time']))    

                    grasp_end = self.config[channel][row[act_kwd]]  


        wait_before = self.config[channel].get('wait_before_release',0.0)
        grasp_duration = self.config[channel].get('release_time',0.0)
        wait_after  = self.config[channel].get('wait_after_release',0.0)

        # Build the release move
        if 'robotiq' in self.hand_type or 'dynamixel' in self.hand_type:
            # Build release move
            hand_moves['release']=[]
            if wait_before>0.0:
                hand_moves['release'].append(self.build_pressure_vec(grasp_end, wait_before))
            
            hand_moves['release'].append(self.build_pressure_vec(idle, wait_before+grasp_duration))
            hand_moves['release'].append(self.build_pressure_vec(idle, wait_before+grasp_duration+wait_after))
            # Build the startup move
            hand_moves['startup'] = [self.build_pressure_vec(initial, 0.0), 
                                    self.build_pressure_vec(initial, 0.0)]

        else:
            release_sequence = self.config[channel].get('release_sequence',None)

            if release_sequence is not None:
                release_type=release_sequence.get('type', None)
                hand_moves['release'] = []

                if release_type == 'skill':
                    release_skills = release_sequence.get('sequence', [])

                    if len(release_skills)>0:
                        
                        release_traj = self.build_skill_sequence(release_skills)

                        if wait_before >0:
                            hand_moves['release'] =  [self.build_pressure_vec(release_traj[0]['pressure'], release_traj[0]['time'])]
                            release_traj = self.skill_builder.adjust_points(release_traj,wait_before)

                        # Convert trajectory to correct format
                        for row in release_traj:
                            hand_moves['release'].append(self.build_pressure_vec(row['pressure'], row['time']))

                        if wait_after >0:
                            hand_moves['release'].append(self.build_pressure_vec(row['pressure'], wait_before+row['time']+wait_after))
                            

            else:
                idle  = [ float(x) for x in self.config[channel]['idle_pressure'] ]           

                hand_moves['release']= [self.build_pressure_vec(grasp_end, 0.0), 
                                        self.build_pressure_vec(grasp_end, wait_before),
                                        self.build_pressure_vec(idle, wait_before+grasp_duration), 
                                        self.build_pressure_vec(idle, wait_before+grasp_duration+wait_after)]

            # Build the startup move
            idle  = [ float(x) for x in self.config[channel]['idle_pressure'] ]  
            initial = [ float(x) for x in self.config[channel].get('initial_pressure',idle) ]
            hand_moves['startup'] = [self.build_pressure_vec(initial, 0.0), 
                                    self.build_pressure_vec(initial, 0.0)]


        if self.reset_object:
            #hand_moves['release_inv'] = self.invert_hand_move(hand_moves['release'])
            #hand_moves['grasp_inv'] = self.invert_hand_move(hand_moves['grasp'])
            hand_moves['startup_inv'] = self.invert_hand_move(hand_moves['startup'])


        self.trajectory_built[channel] = hand_moves


    # Invert a grasping move
    def invert_hand_move(self, move):
        end_time = move[-1][0]
        move_inv = copy.deepcopy(list(reversed(move)))
        for waypoint in move_inv:
            waypoint[0] = end_time - waypoint[0]
        
        return move_inv


    # Build a sequence of skills
    def build_skill_sequence(self, skill_sequence):
        skill_traj = []
        last_time = 0

        # Chain together all skills in the skill sequence
        for row in skill_sequence:
            skill_curr = self.build_skill(row['skill'], row['vars'], row['times'], row['main_repeat'])

            if len(skill_traj)>0:
                last_time = skill_traj[-1]['time']+0.001
            
            skill_traj.extend(self.skill_builder.adjust_points(skill_curr, last_time))

        return skill_traj


    # Build skills
    def build_skill(self, filename, vars=None, times=None, main_repeat=None):
        try:
            self.skill_builder.load_skill(filename)
            self.skill_builder.generate_skill(vars, times)

            skill_flat = self.skill_builder.get_skill_flattened(main_repeat = main_repeat)

            return skill_flat

        except ValueError:
            raise      


    def build_pressure_vec(self,pressures, time):
        # Coerce pressures
        pressure_too_high = False
        pressure_too_low = False
        if isinstance(self.max_p, list):
            if len(pressures) != len(self.max_p):
                raise ValueError("Max pressure list needs to be the same length as the pressure vectors")
            else:
                max_p = self.max_p
        else:
            max_p = [self.max_p]*len(pressures)

        if isinstance(self.min_p, list):
            if len(pressures) != len(self.min_p):
                raise ValueError("Min pressure list needs to be the same length as the pressure vectors")
            else:
                min_p = self.min_p
        else:
            min_p = [self.min_p]*len(pressures)

        if ('robotiq' in self.hand_type or 'dynamixel' in self.hand_type):
            pressure_out=self.dict_to_list(pressures,self.keylist)

        else:
            pressure_out = []
            for idx, pressure in enumerate(pressures):
                if pressure > max_p[idx]:
                    pressure_out.append(max_p[idx])
                    pressure_too_high = True
                elif pressure < min_p[idx]:
                    pressure_out.append(min_p[idx])
                    pressure_too_low = True
                else:
                    pressure_out.append(pressure)
                

        if pressure_too_high:
            print("Some pressures were too high - pressures coerced")
            print(pressures)
            print(pressure_out)

        if pressure_too_low:
            print("Some pressures were too low - pressures coerced")
            print(pressures)
            print(pressure_out)

        out = copy.deepcopy(pressure_out)
        out.insert(0,time)
        return out



    def build_moves(self, config):

        arm_moves = {}

        # Build the grasp move.
        # If the object is a list, then use all the entries
        if isinstance(config['arm']['grasp_pose'], list):
            arm_moves['pre'] = [copy.deepcopy(config['arm']['initial_pose']),
                                copy.deepcopy(config['arm']['grasp_pose'][0])   ]

            grasp_end_pose = copy.deepcopy(config['arm']['grasp_pose'][-1])

            arm_moves['grasp_move'] = []
            for row in config['arm']['grasp_pose']:
                arm_moves['grasp_move'].append(copy.deepcopy(row))

        else:
            arm_moves['pre'] = [copy.deepcopy(config['arm']['initial_pose']),
                                copy.deepcopy(config['arm']['grasp_pose'])   ]

            arm_moves['grasp_move'] = [copy.deepcopy(config['arm']['grasp_pose']),
                                       copy.deepcopy(config['arm']['grasp_pose']) ]

            grasp_end_pose = copy.deepcopy(config['arm']['grasp_pose'])


        # Build the pickup/release move
        if isinstance(config['arm']['release_pose'], list):
            arm_moves['release_move'] = []
            for row in config['arm']['release_pose']:
                arm_moves['release_move'].append(copy.deepcopy(row))
            
            release_start = copy.deepcopy(config['arm']['release_pose'][0])
            release_end = copy.deepcopy(config['arm']['release_pose'][-1])
        else:
            arm_moves['release_move'] = [copy.deepcopy(config['arm']['release_pose']),
                                       copy.deepcopy(config['arm']['release_pose']) ]
            
            release_start = copy.deepcopy(config['arm']['release_pose'])
            release_end = copy.deepcopy(config['arm']['release_pose'])



        pickup = config['arm'].get('pickup', None)

        # Package args if they are defined inline (for backward compatibillity)
        if pickup.get('args', None) is None:
            pickup['args']={}
            for key in pickup:
                if key=='type':
                    continue
                else:
                    pickup['args'][key] = copy.deepcopy(pickup[key])
                    #del pickup[key]

        # Check if the pickup is reasonable
        if not self.validate_pickup(pickup):
            return
        
        # Generate a "square" pickup sequence
        if pickup['type'] == 'square':
            above_grasp = copy.deepcopy(grasp_end_pose)
            above_grasp['position'][2] = above_grasp['position'][2] + pickup['args']['height']

            above_release = copy.deepcopy(release_start)
            above_release['position'][2] = above_release['position'][2] + pickup['args']['height']

            arm_moves['move'] = [copy.deepcopy(grasp_end_pose),
                                above_grasp,
                                above_release,
                                release_start]

        # Generate a "triangle" pickup sequence
        elif pickup['type'] == 'triangle':
            above_grasp = copy.deepcopy(grasp_end_pose)
            above_release = copy.deepcopy(release_start)
            for axis_idx, item in enumerate(above_grasp['position']):
                above_grasp['position'][axis_idx] = ((1-pickup['args']['percent'])*above_grasp['position'][axis_idx] + (pickup['args']['percent'])*above_release['position'][axis_idx])

            above_grasp['position'][2] = above_grasp['position'][2] + pickup['args']['height']

            arm_moves['move'] = [copy.deepcopy(grasp_end_pose),
                                above_grasp,
                                release_start ]

        # Generate a "to-pose" sequence
        elif pickup['type'] == 'to_pose':
            poses = pickup.get('args', None)

            if isinstance(poses, dict):
                pose_to_use_before = poses['before']
                pose_to_use_after =  poses['after']
            elif pose is not None:
                pose_to_use_before = poses
                pose_to_use_after =  poses

            # Get the manip pose and turn it into a list if needed
            print(pose_to_use_before)
            print(pose_to_use_after)
            manip_pose_before = config['arm'][pose_to_use_before]
            manip_pose_after = config['arm'][pose_to_use_after]
            if not isinstance(manip_pose_before, list):
                manip_pose_before = [manip_pose_before]
            if not isinstance(manip_pose_after, list):
                manip_pose_after = [manip_pose_after]

            arm_moves['move'] = [copy.deepcopy(grasp_end_pose),
                                 copy.deepcopy(grasp_end_pose) ]

            arm_moves['manip_before'] = [copy.deepcopy(grasp_end_pose)]
            arm_moves['manip_before'].extend(copy.deepcopy(manip_pose_before))
            arm_moves['manip_after'] =  copy.deepcopy([manip_pose_before[-1]])
            arm_moves['manip_after'].extend(copy.deepcopy(manip_pose_after))
            arm_moves['manip_after'].append(copy.deepcopy(release_start))


        else:
            print('Pickup type not implemented yet: Doing square instead')

            above_grasp = copy.deepcopy(grasp_end_pose)
            above_grasp['position'][2] = above_grasp['position'][2] + pickup['args']['height']

            above_release = copy.deepcopy(release_start)
            above_release['position'][2] = above_release['position'][2] + pickup['args']['height']

            arm_moves['move'] = [copy.deepcopy(grasp_end_pose),
                                above_grasp,
                                above_release,
                                release_start ]
   
        # Build the manipulation sequence
        arm_manip_sequence = config['arm'].get('manip_sequence', None)

        if arm_manip_sequence is not None:
            manip_type=arm_manip_sequence['type']
            manip_args=arm_manip_sequence['args']

            if manip_type == 'trajectory':
                manip_traj = manip_args['trajectory']
                manip_sequence = manip_args.get('sequence',None)
                manip_moves = manip_args.get('moves',None)

                manip_file =   os.path.join(os.path.dirname(self.config_file),manip_traj)
                with open(manip_file,'r') as f:
                    # use safe_load instead of load
                    manip_config = yaml.safe_load(f)
                
                # If we provide a list of moves to import, handle that
                if isinstance(manip_sequence,list):
                    manip_move = []
                    keylist = list(manip_config.keys())

                    for move in manip_sequence:
                        if not (move in keylist):
                            raise ValueError("Arm manipulation sequence contains moves that are not defined")

                    # Make one long move
                    for move in manip_sequence:
                        manip_move.extend(manip_config[move])

                    # add this move to the manipulation moves
                    arm_moves['manip'] = manip_move
                    arm_moves['manip_seq_before'] = [manip_move[0]]
                    arm_moves['manip_seq_after'] = [manip_move[-1]]

                # NEW FEATURE HERE!
                elif isinstance(manip_sequence,str):
                    sequence = manip_config.get(manip_sequence,'sequence')
                    moves = manip_config.get(manip_moves,'arm')

                    self.extra_manip_sequence = sequence

                    for key in moves:
                        arm_moves[key] = moves[key]

                    

            if arm_moves.get('manip_before',None) is not None:
                arm_moves['manip_seq_before'].insert(0, arm_moves['manip_before'][-1])
            
            if arm_moves.get('manip_after',None) is not None:
                arm_moves['manip_seq_after'].append(arm_moves['manip_after'][0])
                    


        # Build the post-release move.
        arm_moves['post'] = [release_end,
                            copy.deepcopy(config['arm']['final_pose'])   ]


        if self.reset_object:
            arm_moves['release_move_inv'] = copy.deepcopy(list(reversed(arm_moves['release_move'])))
            arm_moves['move_inv'] = copy.deepcopy(list(reversed(arm_moves['move'])))

            if arm_moves.get('manip_before'):
                arm_moves['manip_before_inv'] = copy.deepcopy(list(reversed(arm_moves['manip_before'])))
                arm_moves['manip_after_inv'] = copy.deepcopy(list(reversed(arm_moves['manip_after'])))


            arm_moves['grasp_move_inv'] = copy.deepcopy(list(reversed(arm_moves['grasp_move'])))
            arm_moves['pre_inv'] = copy.deepcopy(list(reversed(arm_moves['pre'])))
            arm_moves['post_inv'] = copy.deepcopy(list(reversed(arm_moves['post'])))

        # Convert euler angles to quaterions if needed
        orientation_type = config['arm'].get('orientation_type')
        if orientation_type is not None:
            for key in arm_moves:
                curr_move = arm_moves[key]
                print(key)
                for entry in curr_move:
                    if len(entry['orientation'])==3:
                        if orientation_type == 'degrees':
                            entry_to_use = np.deg2rad(entry['orientation']).tolist()
                        else:
                            entry_to_use = copy.deepcopy(entry['orientation'])
                        ori_quat = quaternion_from_euler(entry_to_use[0],entry_to_use[1],entry_to_use[2])
                        entry['orientation'] = ori_quat.tolist()          


        self.trajectory_built['arm'] = arm_moves


    # Validate a pickup definition
    def validate_pickup(self, pickup):
        ok=True

        if pickup.get('percent', None) is not None:
            if pickup['args']['percent'] >1 or pickup['args']['percent'] <0:
                ok = False

        return ok
        
   


def main(file_name=None):
    try:
        rospy.init_node("pick_and_place_builder", anonymous=True, disable_signals=True)

        node = pickPlaceBuild()
        node.build_traj()

      

    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__':
    if len(sys.argv) < 2 or len(sys.argv) > 2:
        main()
    elif len(sys.argv) ==2:
        main(sys.argv[1])
    else:
        print("Usage:")
        print("\tpick_place_build.py [FILENAME]\t- Build a pick-and-place action")
