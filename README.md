# hand_arm_cbt
A top-level package to coordinate a robot with a soft pneumatic hand.


## Dependencies
### Hardware:
- A pressure control system running my [custom firmware](https://github.com/cbteeple/pressure_controller)
- A robot arm (currently tested only with a UR5e)
- A desktop computer running Linux (currently tested only in [Ubuntu 18.04](https://ubuntu.com/download/desktop))

### Software:
- [ROS Melodic](http://wiki.ros.org/melodic/Installation)
	- My [pressure_control_cbt](https://github.com/cbteeple/pressure_control_cbt) package for ROS
	- My version of [rosbag_recorder](https://github.com/cbteeple/rosbag-recorder) for ROS
	- The [ur_modern_driver](https://github.com/plusone-robotics/ur_modern_driver/tree/add-e-series-support) ROS package with e-series support (by plusone robotics)
	- The [bond](https://github.com/ros/bond_core) package for ROS
- Various python libraries:
	- [scipy](https://www.scipy.org/) (`pip install scipy`)
	- [numpy](https://www.numpy.org/) (`pip install numpy`)
	- [numbers](https://docs.python.org/2/library/numbers.html) (`pip install numbers`)
	- [matplotlib](https://matplotlib.org/) (`pip install matplotlib`)
	- [pynput](https://pypi.org/project/pynput/) (`pip install pynput`)
	- [yaml](https://pyyaml.org/wiki/PyYAMLDocumentation) (`pip install pyyaml`)
	
## Installation
1. Add this package to your `workspace/src` folder.
2. Run `catkin_make` to enable the custom python modules in this package to work


## Usage
**Unless specified, all commands assume you want to run a command using BOTH the arm and a hand. If you want to run on only one device, see the _"Run trajectories on only one device"_ section below.**

### Prerequisits
Before you can control the robot and hand, you first need to start some ROS servers:
- Start the robot control server
	- `roslaunch ur_modern_driver ur5e_bringup.launch limited:=true robot_ip:=192.168.1.2`
- Start the hand control server (based on [pressure_control_ros](https://github.com/cbteeple/pressure_control_cbt) package)
	- `roslaunch hand_arm hand_bringup.launch profile:=anthro7`
- Start MoveIt! (If you're building trajectories in cartesian space)
	- With the real robot
		- `roslaunch ur5_e_moveit_config ur5_e_moveit_planning_execution.launch limited:=false`
		- `roslaunch ur5_e_moveit_config moveit_rviz.launch config:=true `
	- With a simulated robot
		- `roslaunch ur_e_gazebo ur5e.launch`
		- `roslaunch ur5_e_moveit_config ur5_e_moveit_planning_execution.launch sim:=true`
		- `roslaunch ur5_e_moveit_config moveit_rviz.launch config:=true`


### Move to specified joint positions:
Move to zero:
`rosrun hand_arm move_home.py go 0`

Move to home:
`rosrun hand_arm move_home.py go 1`

Move to some other position:
`rosrun hand_arm move_home.py go [POSITION_NUMBER]`

Set a position (*all but the zero position can be set*):
`rosrun hand_arm move_home.py set [POSITION_NUMBER]`


### Teach the robot:
When using teach mode, the robot will be put into freedrive mode, enabling you to push it around.

- Enable freedrive mode without saving anything:
	- `rosrun hand_arm teach.py`

- Teach the entire trajectory from start to finish 
	- `rosrun hand_arm teach.py [FILENAME]`

- Teach individual waypoints in a trajectory:
	- `rosrun hand_arm teach_points.py [FILENAME]`
	- In this mode, use the space bar to save the current position as a point in the trajectory.

- Replay a trajectory:
	- `rosrun hand_arm replay.py [FILENAME]`


### Set up motion routines:

1. Create a yaml file similar to the ones in "trajectories"

2. Set up arm trajectories
	- Joint Space
		- In *sequence* >> *setup*, change the *arm_traj_space* to "*joint*"
		- In *arm*, set up each move segment by naming it
		- Use a list of joint positions and times to create joint trajectories for each move segment

	- Cartesian Space (End effector poses)
		- In *sequence* >> *setup*, change the *arm_traj_space* to "*cartesian*"
		- In *arm*, set up each move segment by naming it
		- Use a list of end effector positions and orientations to create pose waypoints for each move segment

3. Set up hand trajectories
	- In *sequence* >> *setup*, set the *hand_traj_space* to "*pressure*"
	- In *hand*, set up each move segment by naming it
	- Use a list of trajectory points in the following form: 
		- `[time, p1, p2, ..., pn ]` where time is in seconds and *p1* - *pn* are pressures in psi

4. Set up the motion sequence
	- Each line of the sequence should have an *arm* and *hand* entry. If no trajectory segment should be used, set it to *false*
	- In *sequence* >> *startup*, set the startup trajectory segments for each device
	- In *sequence* >> *operations*, set the sequence of trajectory segments to use. These should be the exact names of segments you entered before.


### Do pick-and-place actions:

#### Cartesian Space
You can set up pick-and-place routine using cartesian poses, then use MoveIt! to do the IK and motion planning.

_This requires that you bring up the robot and start MoveIt! (See above)_

- Build a routine
	- Create a yaml file similar to the ones in "traj_setup"
	- Set the poses and grasping settings you want to use.
	- `roslaunch hand_arm pick-place-build.launch traj:=[FILENAME]` This command automatically builds a trajectory usig the format described above.
	- `roslaunch hand_arm pick-place-build-multi.launch traj:=[FILENAME]` Build a family of trajectories

- Plan a routine
	- `roslaunch hand_arm pick-place-plan.launch traj:=[FILENAME]`
	- `roslaunch hand_arm pick-place-plan-multi.launch traj:=[FILENAME]` Build a family of trajectories
	- This command uses MoveIt! to plan a trajectory based on poses, then saves the resulting joint space trajectory.

- Run a planned routine
	- `roslaunch hand_arm pick-place-run.launch traj:=[FILENAME] reps:=[# REPS]`
	- `roslaunch hand_arm pick-place-run-multi.launch traj:=[FILENAME] reps:=[# REPS]`


- Run a live routine (this replans, but doesn't save)
	- `roslaunch hand_arm pick-place-run.launch traj:=[FILENAME] reps:=[# REPS] replan:=true`


#### Joint Space:
You can set up pick-and-place routine using joint configurations directly. 
- Build an action automatically
	- Follow the steps in the "Cartesian" section to build a pick-and-place trajectory using end effector poses

- Build an routine manually
	- Create a yaml file similar to the ones in "trajectories"
	- In *sequence* >> *setup*, change the *arm_traj_space* to "*joint*"


- Do pick and place
	- `roslaunch hand_arm pick-place-run.launch traj:=pick_front speed_factor:=1.0 reps:=20`


### Run trajectories on only one device
#### Any Trajectory
- Arm Only
	- `roslaunch hand_arm arm-traj.launch traj:=[FILENAME] reps:=[# REPS]`
- Hand Only
	- `roslaunch pressure_controller_ros load_traj.launch profile:=example/planar2seg_demo`
	- `roslaunch pressure_controller_ros run_traj.launch`

#### Pick-and-place actions
- Arm Only
	- `roslaunch hand_arm pick-place-run.launch hand:=false traj:=[FILENAME] reps:=[# REPS]`
- Hand Only
	- `roslaunch hand_arm pick-place-run.launch arm:=false traj:=[FILENAME] reps:=[# REPS]`
