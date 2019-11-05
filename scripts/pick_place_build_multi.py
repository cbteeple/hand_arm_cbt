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
        self.pattern_type = self.config['settings']['type']

        self.traj_patterned = []



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

        else:
            self.perturbations = [0,0,0]






    def build_traj(self):
        self.create_traj_pattern()

        self.trajectory_built = {}
        self.build_sequence()
        self.build_grasp()
        base_config = copy.deepcopy(self.config)

        perturb_num = 0

        self.prep_directory(self.filepath_out_dir)

        for entry in self.perturbations:
            
            # Update with current perturbation
            curr_config = copy.deepcopy(base_config)

            for idx, axis in enumerate(curr_config['arm']['grasp_pose']['position']):
                curr_config['arm']['grasp_pose']['position'][idx] = axis + entry[idx]

            for idx, axis in enumerate(curr_config['arm']['release_pose']['position']):
                curr_config['arm']['release_pose']['position'][idx] = axis + entry[idx]

            self.build_moves(curr_config)

            out_file =   os.path.join(self.filepath_out_dir,"pos_"+"%04d"%(perturb_num)+'.yaml')

            self.save_traj(out_file)

            perturb_num+=1


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

        with open(out_file, 'w+') as f:
            yaml.dump(self.trajectory_built, f, default_flow_style=None)



    def build_sequence(self):
        # get the trajectories
        
        sequence = {}

        sequence['setup'] = {}
        sequence['setup']['arm_traj_space']  = 'cartesian'
        sequence['setup']['hand_traj_space'] = 'pressure'

        sequence['startup'] = {}
        sequence['startup']['arm'] = 'pre'
        sequence['startup']['hand'] = 'startup'

        sequence['operations'] = []
        ops = sequence['operations']

        ops.append({'arm': 'pre', 'hand': 'startup'})
        ops.append({'arm': False, 'hand': 'grasp'})
        ops.append({'arm': 'move', 'hand': False})
        ops.append({'arm': False, 'hand': 'release'})
        ops.append({'arm': 'post', 'hand': False})


        self.trajectory_built['sequence'] = sequence



    def build_grasp(self):
        pass

        hand_moves = {}

        idle  = [ float(x) for x in self.config['hand']['idle_pressure'] ]

        grasp = [ float(x) for x in self.config['hand']['grasp_pressure'] ]
        
        hand_moves['grasp']= [self.build_pressure_vec(idle, 0.0), 
                              self.build_pressure_vec(grasp, self.config['hand']['grasp_time']), 
                              self.build_pressure_vec(grasp, self.config['hand']['grasp_time'] + self.config['hand']['wait_after_grasp'])]

        hand_moves['release'] = [self.build_pressure_vec(grasp, 0.0), 
                                 self.build_pressure_vec(idle, self.config['hand']['release_time']), 
                                 self.build_pressure_vec(idle, self.config['hand']['release_time'] + self.config['hand']['wait_after_release'])]

        hand_moves['startup'] = [self.build_pressure_vec(idle, 0.0), 
                                 self.build_pressure_vec(idle, 0.0)]


        self.trajectory_built['hand'] = hand_moves

    

    def build_pressure_vec(self,pressures, time):
        out = copy.deepcopy(pressures)
        out.insert(0,time)
        return out



    def build_moves(self, config):

        arm_moves = {}

        pickup_height = config['arm']['pickup_height']

        above_grasp = copy.deepcopy(config['arm']['grasp_pose'])
        above_grasp['position'][2] = above_grasp['position'][2] + pickup_height

        above_release = copy.deepcopy(config['arm']['release_pose'])
        above_release['position'][2] = above_release['position'][2] + pickup_height



        arm_moves['pre'] = [copy.deepcopy(config['arm']['initial_pose']),
                            copy.deepcopy(config['arm']['grasp_pose'])   ]

        arm_moves['move'] = [copy.deepcopy(config['arm']['grasp_pose']),
                            above_grasp,
                            above_release,
                            copy.deepcopy(config['arm']['release_pose']) ]

        arm_moves['post'] = [copy.deepcopy(config['arm']['release_pose']),
                            copy.deepcopy(config['arm']['final_pose'])   ]

        self.trajectory_built['arm'] = arm_moves

   


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
