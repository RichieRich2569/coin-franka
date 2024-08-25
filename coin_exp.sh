#!/bin/bash

# Define the robot IP and other fixed parameters
ROBOT_IP="192.168.0.4"
LOAD_GRIPPER="false"
ROBOT="panda"
TOTAL_TRIALS=190

cd ~/catkin_ws

echo "Moving robot to start..."
roslaunch franka_coin_controllers coin_move_to_start.launch \
    robot_ip:=$ROBOT_IP load_gripper:=$LOAD_GRIPPER robot:=$ROBOT

# Wait for 2 seconds
echo "Waiting for 2 seconds..."
sleep 2

# Loop from trial 1 to TOTAL_TRIALS
for trial in $(seq 182 $TOTAL_TRIALS)
do
  # Launch the spontaneous controller with the current trial number
  echo "Launching COIN Controller for trial $trial..."
  roslaunch franka_coin_controllers coin_spontaneous_controller.launch \
    robot_ip:=$ROBOT_IP load_gripper:=$LOAD_GRIPPER robot:=$ROBOT trial:=$trial

  # Wait for 2 seconds
  echo "Waiting for 2 seconds..."
  sleep 2
  
  
  # Launch the move_to_start controller
  echo "Moving robot to start..."
  roslaunch franka_coin_controllers coin_move_to_start.launch \
    robot_ip:=$ROBOT_IP load_gripper:=$LOAD_GRIPPER robot:=$ROBOT

  

  # Wait for 2 seconds
  echo "Waiting for 2 seconds..."
  sleep 2
done

echo "All trials completed."
