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




curr_path=os.path.dirname(os.path.abspath(__file__))
filepath_traj = os.path.join(curr_path,'..','traj_setup')
filepath_out = os.path.join(curr_path,'..','trajectories')








class pickPlaceBuild:
    def __init__(self):
        self.traj_profile = rospy.get_param(rospy.get_name()+'/traj_profile')

        

        # Read the trajectory configuration file
        self.filepath = os.path.join(filepath_traj)
        self.filepath_out_dir = os.path.join(filepath_out,self.traj_profile)
        config_file =   os.path.join(self.filepath,self.traj_profile+'.yaml')

        with open(config_file,'r') as f:
            # use safe_load instead of load
            self.config = yaml.safe_load(f)
            f.close()

        self.boomerang = self.config['settings']['boomerang']
        self.reset_object = self.config['settings'].get('reset_object', False)
        self.pattern_type = self.config['settings']['type']

        self.use_servo = self.config['settings'].get('use_servo',False)

        self.traj_patterned = []

        self.max_p = self.config['hand'].get("max_pressure", np.inf)
        self.min_p  = self.config['hand'].get("min_pressure", -np.inf)
        self.hand_type=self.config['hand'].get('type','pressure_controlled')



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
            shift_release_pose = self.config['arm']['grid'].get('affects_release_pose',True)

            for entry, release_entry in zip(self.perturbations, self.release_perturbations):
                
                # Update with current perturbation
                curr_config = copy.deepcopy(base_config)

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

        if self.config['hand'].get('manip_sequence',False):
            ops.append({'arm': False, 'hand': 'manip', 'servo': False})

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
            ops.append({'arm': 'move_inv', 'hand': False, 'servo': False})
            ops.extend(grasping_move_inv)

            ops.append({'arm': 'pre_inv', 'hand': 'startup', 'servo': 'startup'})

        print(ops)




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


        if 'robotiq' in self.hand_type:
            self.keylist = ['pos','speed','force']
            idle  = self.config[channel]['idle_pos']
            grasp = self.config[channel]['grasp_pos']
            initial = self.config[channel].get('initial_pos')
            act_kwd = 'pos'


        else:
            idle  = [ float(x) for x in self.config[channel]['idle_pressure'] ]
            grasp = [ float(x) for x in self.config[channel]['grasp_pressure'] ]
            initial = [ float(x) for x in self.config[channel].get('initial_pressure',idle) ]
            act_kwd = 'pressure'


        wait_before = self.config[channel].get('wait_before_grasp',0.0)
        grasp_duration = self.config[channel].get('grasp_time',0.0)
        wait_after  = self.config[channel].get('wait_after_grasp',0.0)


        if 'robotiq' in self.hand_type:
            hand_moves['grasp']= []
            if wait_before>0.0:
                hand_moves['grasp'].append(self.build_pressure_vec(initial, 0.0))
            hand_moves['grasp'].append(self.build_pressure_vec(grasp, wait_before+grasp_duration))
            hand_moves['grasp'].append(self.build_pressure_vec(grasp, wait_before+grasp_duration+wait_after))
            grasp_end= grasp

        else:
            hand_moves['grasp']= [self.build_pressure_vec(initial, 0.0)]

            if self.config[channel].get('grasp_sequence',False):
                grasp_duration = self.config[channel]['grasp_sequence'][-1]['time']

                for row in self.config[channel]['grasp_sequence']:
                    new_row = self.config[channel][row[act_kwd]]
                    hand_moves['grasp'].append(self.build_pressure_vec(new_row, wait_before+row['time']))

                grasp_end= self.config[channel][self.config[channel]['grasp_sequence'][-1][act_kwd]]
                
            else:
                hand_moves['grasp'].append(self.build_pressure_vec(initial, wait_before))
                hand_moves['grasp'].append(self.build_pressure_vec(grasp, wait_before+grasp_duration))

                grasp_end= grasp

            hand_moves['grasp'].append(self.build_pressure_vec(grasp_end, wait_before+grasp_duration+wait_after))


        manip_duration = 0.0
        if self.config[channel].get('manip_sequence',False):
            num_reps=int(self.config[channel].get('manip_repeat',1))

            manip_duration = self.config[channel]['manip_sequence'][-1]['time']

            first_point = self.config[channel]['manip_sequence'][0]
            self.config[channel]['manip_sequence'].pop(0)

            hand_moves['manip'] = []
            hand_moves['manip'].append(self.build_pressure_vec(self.config[channel][first_point[act_kwd]], first_point['time']))
            for rep in range(num_reps):
                for row in self.config[channel]['manip_sequence']:
                    hand_moves['manip'].append(self.build_pressure_vec(self.config[channel][row[act_kwd]], rep*manip_duration+row['time']))    

            grasp_end = self.config[channel][row[act_kwd]]  


        wait_before = self.config[channel].get('wait_before_release',0.0)
        grasp_duration = self.config[channel].get('release_time',0.0)
        wait_after  = self.config[channel].get('wait_after_release',0.0)

        hand_moves['release']= [self.build_pressure_vec(grasp_end, 0.0), 
                                self.build_pressure_vec(grasp_end, wait_before),
                                self.build_pressure_vec(idle, wait_before+grasp_duration), 
                                self.build_pressure_vec(idle, wait_before+grasp_duration+wait_after)]

        hand_moves['startup'] = [self.build_pressure_vec(initial, 0.0), 
                                 self.build_pressure_vec(initial, 0.0)]


        if self.reset_object:
            hand_moves['release_inv']= [
                                self.build_pressure_vec(idle, 0.0),
                                self.build_pressure_vec(idle, wait_before),
                                self.build_pressure_vec(grasp_end, wait_before+grasp_duration),
                                self.build_pressure_vec(grasp_end, wait_before+grasp_duration+wait_after)]

            end_time = hand_moves['grasp'][-1][0]
            grasp_inv = copy.deepcopy(list(reversed(hand_moves['grasp'])))
            for waypoint in grasp_inv:
                waypoint[0] = end_time - waypoint[0]
            
            hand_moves['grasp_inv'] = grasp_inv

            
            hand_moves['startup_inv'] = copy.deepcopy(list(reversed(hand_moves['startup'])))


        self.trajectory_built[channel] = hand_moves

    

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

        if ('robotiq' in self.hand_type):
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

        if pickup is None:
            pickup = {}
            pickup['height']  = config['arm']['pickup_height']
            pickup['type']    = config['arm']['pickup_type']
            pickup['percent'] = config['arm']['pickup_percent']

        if not self.validate_pickup(pickup):
            return
        
        if pickup['type'] == 'square':
            above_grasp = copy.deepcopy(grasp_end_pose)
            above_grasp['position'][2] = above_grasp['position'][2] + pickup['height']

            above_release = copy.deepcopy(release_start)
            above_release['position'][2] = above_release['position'][2] + pickup['height']

            arm_moves['move'] = [copy.deepcopy(grasp_end_pose),
                                above_grasp,
                                above_release,
                                release_start]

        elif pickup['type'] == 'triangle':
            above_grasp = copy.deepcopy(grasp_end_pose)
            above_release = copy.deepcopy(release_start)
            for axis_idx, item in enumerate(above_grasp['position']):
                above_grasp['position'][axis_idx] = ((1-pickup['percent'])*above_grasp['position'][axis_idx] + (pickup['percent'])*above_release['position'][axis_idx])

            above_grasp['position'][2] = above_grasp['position'][2] + pickup['height']

            arm_moves['move'] = [copy.deepcopy(grasp_end_pose),
                                above_grasp,
                                release_start ]

        else:
            print('Pickup type not implemented yet: Doing square instead')

            above_grasp = copy.deepcopy(grasp_end_pose)
            above_grasp['position'][2] = above_grasp['position'][2] + pickup['height']

            above_release = copy.deepcopy(release_start)
            above_release['position'][2] = above_release['position'][2] + pickup['height']

            arm_moves['move'] = [copy.deepcopy(grasp_end_pose),
                                above_grasp,
                                above_release,
                                release_start ]
   

        # Build the post-release move.
        arm_moves['post'] = [release_end,
                            copy.deepcopy(config['arm']['final_pose'])   ]


        if self.reset_object:
            arm_moves['release_move_inv'] = copy.deepcopy(list(reversed(arm_moves['release_move'])))
            arm_moves['move_inv'] = copy.deepcopy(list(reversed(arm_moves['move'])))
            arm_moves['grasp_move_inv'] = copy.deepcopy(list(reversed(arm_moves['grasp_move'])))
            arm_moves['pre_inv'] = copy.deepcopy(list(reversed(arm_moves['pre'])))
            arm_moves['post_inv'] = copy.deepcopy(list(reversed(arm_moves['post'])))

        # Convert euler angles to quaterions if needed
        orientation_type = config['arm'].get('orientation_type')
        if orientation_type is not None:
            for key in arm_moves:
                curr_move = arm_moves[key]
                for entry in curr_move:
                    if len(entry['orientation'])==3:
                        if orientation_type == 'degrees':
                            entry_to_use = np.deg2rad(entry['orientation']).tolist()
                        else:
                            entry_to_use = copy.deepcopy(entry['orientation'])
                        ori_quat = quaternion_from_euler(entry_to_use[0],entry_to_use[1],entry_to_use[2])
                        entry['orientation'] = ori_quat.tolist()          


        self.trajectory_built['arm'] = arm_moves


    def validate_pickup(self, pickup):
        ok=True

        if pickup['percent'] >1 or pickup['percent'] <0:
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
