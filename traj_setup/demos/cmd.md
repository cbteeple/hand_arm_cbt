bash bringup-hw.sh hid1 anthro8

bash bringup-planning.sh ur5e_with_soft_hand
bash pick-place-build-plan.sh demos/pick_place_back_forth
bash pick-place-build-plan.sh demos/pick_place_back_forth_up
bash pick-place-build-plan.sh demos/pick_place_back_forth_rot
bash pick-place-build-plan.sh demos/pick_place_back_forth_rotgait
bash pick-place-build-plan.sh demos/pick_place_back_forth_rotgait2
bash pick-place-build-plan.sh demos/pick_place_back_forth_square
bash pick-place-build-plan.sh demos/write_name

# 1 rep = ~15 seconds
roslaunch hand_arm pick-place-run-multi.launch  config:=arm_hand.yaml  id:=TEST_DEMO  traj:=demos/pick_place_back_forth  use_checklist:=false  save:=false  reps:=1

roslaunch hand_arm pick-place-run-multi.launch  config:=arm_hand.yaml  id:=TEST_DEMO  traj:=demos/pick_place_back_forth_up  use_checklist:=false  save:=false  reps:=4

roslaunch hand_arm pick-place-run-multi.launch  config:=arm_hand.yaml  id:=TEST_DEMO  traj:=demos/pick_place_back_forth_rot  use_checklist:=false  save:=false  reps:=1

roslaunch hand_arm pick-place-run-multi.launch  config:=arm_hand.yaml  id:=TEST_DEMO  traj:=demos/pick_place_back_forth_rotgait  use_checklist:=false  save:=false  reps:=1
roslaunch hand_arm pick-place-run-multi.launch  config:=arm_hand.yaml  id:=TEST_DEMO  traj:=demos/pick_place_back_forth_rotgait2  use_checklist:=false  save:=false  reps:=1

roslaunch hand_arm pick-place-run-multi.launch  config:=arm_hand.yaml  id:=TEST_DEMO  traj:=demos/pick_place_back_forth_square  use_checklist:=false  save:=false  reps:=1

roslaunch hand_arm pick-place-run-multi.launch  config:=arm_hand.yaml  id:=TEST_DEMO  traj:=demos/write_name  use_checklist:=false  save:=false  reps:=1 speed_factor:=0.5

roslaunch hand_arm pick-place-run-multi.launch  config:=arm_hand.yaml  id:=TEST_DEMO  traj:=demos/pick_place_back_forth_up  use_checklist:=false  save:=false  reps:=1 fake:=true


# 60 rep = ~15 minutes
roslaunch hand_arm pick-place-run-multi.launch  config:=arm_hand.yaml  id:=TEST_DEMO  traj:=demos/pick_place_back_forth  use_checklist:=false  reps:=60

roslaunch hand_arm pick-place-run-multi.launch  config:=arm_hand.yaml  id:=TEST_DEMO  traj:=demos/pick_place_back_forth_up  use_checklist:=false  save:=false  reps:=60

# new EE
rosrun xacro xacro -o ur5e_with_soft_hand.urdf ur5e_with_soft_hand.xacro
roslaunch ur5e_with_soft_hand display.launch

bash bringup-planning.sh ur5e_with_soft_hand



# running it
bash bringup-hw.sh hid1 anthro8
bash pick-place-build-plan.sh demos/pick_place_back_forth_ee

roslaunch hand_arm pick-place-run-multi.launch  config:=arm_hand.yaml  id:=TEST_DEMO  traj:=demos/pick_place_back_forth_ee  use_checklist:=false  reps:=1
roslaunch hand_arm pick-place-run-multi.launch  config:=arm_hand.yaml  id:=TEST_DEMO  traj:=demos/pick_place_back_forth_ee  use_checklist:=false  reps:=60


# 16 Channel pressure controller
bash bringup-hand-april.sh hid2 anthro16
bash bringup-hand-april.sh hid3 anthro16

bash pick-place-build-plan.sh demos/8fing_zrot
roslaunch hand_arm pick-place-run-multi.launch config:=hand_only_tracking.yaml id:=TEST_COMMS traj:=demos/8fing_zrot reps:=1 save:=false debug:=true



