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
roslaunch hand_arm run-traj.launch  config:=arm_hand.yaml  id:=TEST_DEMO  traj:=rethi/initial_insert  use_checklist:=false save:=false reps:=1 start:=0 speed_factor:=0.5

## Slide in
bash pick-place-build-plan.sh rethi/slide_in
roslaunch hand_arm run-traj.launch  config:=arm_hand.yaml  id:=TEST_DEMO  traj:=rethi/slide_in  use_checklist:=false save:=false reps:=1 start:=0 speed_factor:=0.5

## use handle
bash pick-place-build-plan.sh rethi/use_handle
roslaunch hand_arm run-traj.launch  config:=arm_hand.yaml  id:=TEST_DEMO  traj:=rethi/use_handle  use_checklist:=false save:=false reps:=1 start:=0 speed_factor:=0.5