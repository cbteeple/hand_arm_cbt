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
	- The [ur_modern_driver](https://github.com/plusone-robotics/ur_modern_driver/tree/add-e-series-support) ROS package with e-series support (by plusone robotics)
	- The [bond](https://github.com/ros/bond_core) package for ROS
- Various python libraries:
	- [scipy](https://www.scipy.org/) (`pip install scipy`)
	- [numpy](https://www.numpy.org/) (`pip install numpy`)
	- [numbers](https://docs.python.org/2/library/numbers.html) (`pip install numbers`)
	- [matplotlib](https://matplotlib.org/) (`pip install matplotlib`)
	- [pynput](https://pypi.org/project/pynput/) (`pip install pynput`)
	- [yaml](https://pyyaml.org/wiki/PyYAMLDocumentation) (`pip install pyyaml`)
	
## How to Install
1. Add this package to your `workspace/src` folder.
2. Run `catkin_make` to enable the custom python modules in this package to work


## How To Use
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
	- `rosrun hand_arm teach.py [INSERT FILENAME]`

- Teach individual waypoints in a trajectory:
	- `rosrun hand_arm teach_points.py [INSERT FILENAME]`
	- In this mode, use the space bar to save the current position as a point in the trajectory.

- Replay a trajectory:
	- `rosrun hand_arm replay.py [INSERT FILENAME]`
	
- Do pick and place:
	- `roslaunch hand_arm pick-and-place.launch traj_profile:=pick_front speed_factor:=1.0 num_reps:=20`



