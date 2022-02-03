# Robotiq Gripper
bash bringup-hw-robotiq.sh /dev/ttyACM0

bash bringup-planning.sh ur5e_with_robotiq


bash pick-place-build-plan.sh rethi/insert_filter
bash pick-place-build-plan.sh rethi/insert_press
bash pick-place-build-plan.sh rethi/insert_press_tall



roslaunch hand_arm run-traj.launch  config:=arm_hand.yaml  id:=TEST_DEMO  traj:=rethi/insert_filter  use_checklist:=false save:=false reps:=1 start:=0
roslaunch hand_arm run-traj.launch  config:=arm_hand.yaml  id:=TEST_DEMO  traj:=rethi/insert_press  use_checklist:=false save:=false reps:=1 start:=0
roslaunch hand_arm run-traj.launch  config:=arm_hand.yaml  id:=TEST_DEMO  traj:=rethi/insert_press_tall  use_checklist:=false save:=false reps:=1 start:=0


# Remove the filter from the housing (and set on the table)

## Push the handle up
bash pick-place-build-plan.sh rethi/push_handle_up
roslaunch hand_arm run-traj.launch  config:=arm_hand.yaml  id:=TEST_DEMO  traj:=rethi/push_handle_up  use_checklist:=false save:=false reps:=1 start:=0 speed_factor:=0.5


# Insert the filter into the housing

## Initial Insertion
bash pick-place-build-plan.sh rethi/initial_insert
roslaunch hand_arm run-traj.launch  config:=arm_hand.yaml  id:=TEST_DEMO  traj:=rethi/initial_insert  use_checklist:=false save:=true reps:=1 start:=0 speed_factor:=0.5

## Slide in
bash pick-place-build-plan.sh rethi/slide_in
roslaunch hand_arm run-traj.launch  config:=arm_hand.yaml  id:=TEST_DEMO  traj:=rethi/slide_in  use_checklist:=false save:=false reps:=1 start:=0 speed_factor:=0.5

## use handle
bash pick-place-build-plan.sh rethi/use_handle
roslaunch hand_arm run-traj.launch  config:=arm_hand.yaml  id:=TEST_DEMO  traj:=rethi/use_handle  use_checklist:=false save:=false reps:=1 start:=0 speed_factor:=0.5




# Testing Angular Deflections

## XZ with pivot
bash pick-place-build-plan.sh rethi/initial_insert_0deg
bash pick-place-build-plan.sh rethi/initial_insert_2deg
bash pick-place-build-plan.sh rethi/initial_insert_4deg
bash pick-place-build-plan.sh rethi/initial_insert_6deg

roslaunch hand_arm run-traj.launch  config:=arm_hand.yaml  id:=angle_offsets  traj:=rethi/initial_insert_0deg  use_checklist:=false reps:=1 speed_factor:=0.5
roslaunch hand_arm run-traj.launch  config:=arm_hand.yaml  id:=angle_offsets  traj:=rethi/initial_insert_2deg  use_checklist:=false reps:=1 speed_factor:=0.5
roslaunch hand_arm run-traj.launch  config:=arm_hand.yaml  id:=angle_offsets  traj:=rethi/initial_insert_4deg  use_checklist:=false reps:=1 speed_factor:=0.5
roslaunch hand_arm run-traj.launch  config:=arm_hand.yaml  id:=angle_offsets  traj:=rethi/initial_insert_6deg  use_checklist:=false reps:=1 speed_factor:=0.5

roslaunch hand_arm run-traj.launch  config:=arm_hand.yaml  id:=angle_offsets_soft  traj:=rethi/initial_insert_0deg  use_checklist:=false reps:=1 speed_factor:=0.5
roslaunch hand_arm run-traj.launch  config:=arm_hand.yaml  id:=angle_offsets_soft  traj:=rethi/initial_insert_2deg  use_checklist:=false reps:=1 speed_factor:=0.5
roslaunch hand_arm run-traj.launch  config:=arm_hand.yaml  id:=angle_offsets_soft  traj:=rethi/initial_insert_4deg  use_checklist:=false reps:=1 speed_factor:=0.5
roslaunch hand_arm run-traj.launch  config:=arm_hand.yaml  id:=angle_offsets_soft  traj:=rethi/initial_insert_6deg  use_checklist:=false reps:=1 speed_factor:=0.5



## XZ no pivot (orthogonal)
bash pick-place-build-plan.sh rethi/initial_insert_0deg_noslip
bash pick-place-build-plan.sh rethi/initial_insert_2deg_noslip
bash pick-place-build-plan.sh rethi/initial_insert_4deg_noslip
bash pick-place-build-plan.sh rethi/initial_insert_6deg_noslip


roslaunch hand_arm run-traj.launch  config:=arm_hand.yaml  id:=angle_offsets  traj:=rethi/initial_insert_0deg_noslip  use_checklist:=false reps:=1 speed_factor:=0.5
roslaunch hand_arm run-traj.launch  config:=arm_hand.yaml  id:=angle_offsets  traj:=rethi/initial_insert_2deg_noslip  use_checklist:=false reps:=1 speed_factor:=0.5
roslaunch hand_arm run-traj.launch  config:=arm_hand.yaml  id:=angle_offsets  traj:=rethi/initial_insert_4deg_noslip  use_checklist:=false reps:=1 speed_factor:=0.5
roslaunch hand_arm run-traj.launch  config:=arm_hand.yaml  id:=angle_offsets  traj:=rethi/initial_insert_6deg_noslip  use_checklist:=false reps:=1 speed_factor:=0.5

roslaunch hand_arm run-traj.launch  config:=arm_hand.yaml  id:=angle_offsets_soft  traj:=rethi/initial_insert_0deg_noslip  use_checklist:=false reps:=1 speed_factor:=0.5
roslaunch hand_arm run-traj.launch  config:=arm_hand.yaml  id:=angle_offsets_soft  traj:=rethi/initial_insert_2deg_noslip  use_checklist:=false reps:=1 speed_factor:=0.5
roslaunch hand_arm run-traj.launch  config:=arm_hand.yaml  id:=angle_offsets_soft  traj:=rethi/initial_insert_4deg_noslip  use_checklist:=false reps:=1 speed_factor:=0.5
roslaunch hand_arm run-traj.launch  config:=arm_hand.yaml  id:=angle_offsets_soft  traj:=rethi/initial_insert_6deg_noslip  use_checklist:=false reps:=1 speed_factor:=0.5





# Angular Deflections with balanced handle


## XZ with pivot
bash pick-place-build-plan.sh rethi/initial_insert_com/0deg
bash pick-place-build-plan.sh rethi/initial_insert_com/2deg
bash pick-place-build-plan.sh rethi/initial_insert_com/4deg
bash pick-place-build-plan.sh rethi/initial_insert_com/6deg

### Rigid
roslaunch hand_arm run-traj.launch  config:=arm_hand.yaml  id:=angle_offsets  traj:=rethi/initial_insert_com/0deg  use_checklist:=false reps:=1 speed_factor:=0.5
roslaunch hand_arm run-traj.launch  config:=arm_hand.yaml  id:=angle_offsets  traj:=rethi/initial_insert_com/2deg  use_checklist:=false reps:=1 speed_factor:=0.5
roslaunch hand_arm run-traj.launch  config:=arm_hand.yaml  id:=angle_offsets  traj:=rethi/initial_insert_com/4deg  use_checklist:=false reps:=1 speed_factor:=0.5
roslaunch hand_arm run-traj.launch  config:=arm_hand.yaml  id:=angle_offsets  traj:=rethi/initial_insert_com/6deg  use_checklist:=false reps:=1 speed_factor:=0.5

### Soft
roslaunch hand_arm run-traj.launch  config:=arm_hand.yaml  id:=angle_offsets_soft  traj:=rethi/initial_insert_com/0deg  use_checklist:=false reps:=1 speed_factor:=0.5
roslaunch hand_arm run-traj.launch  config:=arm_hand.yaml  id:=angle_offsets_soft  traj:=rethi/initial_insert_com/2deg  use_checklist:=false reps:=1 speed_factor:=0.5
roslaunch hand_arm run-traj.launch  config:=arm_hand.yaml  id:=angle_offsets_soft  traj:=rethi/initial_insert_com/4deg  use_checklist:=false reps:=1 speed_factor:=0.5
roslaunch hand_arm run-traj.launch  config:=arm_hand.yaml  id:=angle_offsets_soft  traj:=rethi/initial_insert_com/6deg  use_checklist:=false reps:=1 speed_factor:=0.5



## XZ no pivot (orthogonal)
bash pick-place-build-plan.sh rethi/initial_insert_com/0deg_noslip
bash pick-place-build-plan.sh rethi/initial_insert_com/2deg_noslip
bash pick-place-build-plan.sh rethi/initial_insert_com/4deg_noslip
bash pick-place-build-plan.sh rethi/initial_insert_com/6deg_noslip

### Rigid
roslaunch hand_arm run-traj.launch  config:=arm_hand.yaml  id:=angle_offsets  traj:=rethi/initial_insert_com/0deg_noslip  use_checklist:=false reps:=1 speed_factor:=0.5
roslaunch hand_arm run-traj.launch  config:=arm_hand.yaml  id:=angle_offsets  traj:=rethi/initial_insert_com/2deg_noslip  use_checklist:=false reps:=1 speed_factor:=0.5
roslaunch hand_arm run-traj.launch  config:=arm_hand.yaml  id:=angle_offsets  traj:=rethi/initial_insert_com/4deg_noslip  use_checklist:=false reps:=1 speed_factor:=0.5
roslaunch hand_arm run-traj.launch  config:=arm_hand.yaml  id:=angle_offsets  traj:=rethi/initial_insert_com/6deg_noslip  use_checklist:=false reps:=1 speed_factor:=0.5

### Soft
roslaunch hand_arm run-traj.launch  config:=arm_hand.yaml  id:=angle_offsets_soft  traj:=rethi/initial_insert_com/0deg_noslip  use_checklist:=false reps:=1 speed_factor:=0.5
roslaunch hand_arm run-traj.launch  config:=arm_hand.yaml  id:=angle_offsets_soft  traj:=rethi/initial_insert_com/2deg_noslip  use_checklist:=false reps:=1 speed_factor:=0.5
roslaunch hand_arm run-traj.launch  config:=arm_hand.yaml  id:=angle_offsets_soft  traj:=rethi/initial_insert_com/4deg_noslip  use_checklist:=false reps:=1 speed_factor:=0.5
roslaunch hand_arm run-traj.launch  config:=arm_hand.yaml  id:=angle_offsets_soft  traj:=rethi/initial_insert_com/6deg_noslip  use_checklist:=false reps:=1 speed_factor:=0.5