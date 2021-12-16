# Robotiq Gripper
bash bringup-hw-robotiq.sh /dev/ttyACM0

bash bringup-planning.sh ur5e_with_robotiq


bash pick-place-build-plan.sh rethi/insert_filter


roslaunch hand_arm run-traj.launch  config:=arm_hand.yaml  id:=TEST_DEMO  traj:=rethi/insert_filter  use_checklist:=false save:=false reps:=1