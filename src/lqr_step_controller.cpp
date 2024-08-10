// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_coin_controllers/lqr_step_controller.h>

#include <array>
#include <cmath>
#include <memory>
#include <string>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace franka_coin_controllers {

bool LQRStepController::init(hardware_interface::RobotHW* robot_hardware,
                                              ros::NodeHandle& node_handle) {
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("LQRStepController: Could not get parameter arm_id");
    return false;
  }

  velocity_cartesian_interface_ =
      robot_hardware->get<franka_hw::FrankaVelocityCartesianInterface>();
  if (velocity_cartesian_interface_ == nullptr) {
    ROS_ERROR(
        "LQRStepController: Could not get Cartesian velocity interface from "
        "hardware");
    return false;
  }
  try {
    velocity_cartesian_handle_ = std::make_unique<franka_hw::FrankaCartesianVelocityHandle>(
        velocity_cartesian_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "LQRStepController: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "LQRStepController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "LQRStepController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("LQRStepController: Could not get state interface from hardware");
    return false;
  }

  try {
    state_handle_ = state_interface->getHandle(arm_id + "_robot");

    std::array<double, 7> q_start{{0, 0, 0, -7 * M_PI_4 / 2, 0, 7 * M_PI_4 / 2, 0}};
    for (size_t i = 0; i < q_start.size(); i++) {
      if (std::abs(state_handle_.getRobotState().q_d[i] - q_start[i]) > 0.1) {
        ROS_ERROR_STREAM(
            "LQRStepController: Robot is not in the expected starting position "
            "for running this example. Run `roslaunch franka_coin_controllers "
            "coin_move_to_start.launch robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` "
            "first.");
        return false;
      }
    }
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "LQRStepController: Exception getting state handle: " << e.what());
    return false;
  }

  state_k_.setZero();

  return true;
}

void LQRStepController::starting(const ros::Time& /* time */) {
  elapsed_time_ = ros::Duration(0.0);
  franka::RobotState initial_state = state_handle_->getRobotState();
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector)

  // convert to eigen
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());

  // get initial state
  Eigen::Vector3d position_k = initial_transform.translation();
  Eigen::Matrix<double, 6, 1> velocity_k = jacobian * dq;

  state_k_.segment<2>(0) = position_k.head<2>();
  state_k_.segment<2>(2) = velocity_k.head<2>();

}

void LQRStepController::update(const ros::Time& /* time */,
                                                const ros::Duration& period) {
  elapsed_time_ += period;
  franka::RobotState initial_state = state_handle_->getRobotState();
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector)

  // convert to eigen
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());

  double time_max = 5.0;
  double v_max = 0.5; // Limit for Panda is 1.7 m/s

  // get new state
  Eigen::Vector3d position_k = initial_transform.translation();
  Eigen::Matrix<double, 6, 1> velocity_k = jacobian * dq;

  state_k_.segment<2>(0) = position_k.head<2>();
  state_k_.segment<2>(2) = velocity_k.head<2>();
  state_k_(4) = state_k_(4) + (position_k(0) - 1.0)*period; // Track reference of 1
  state_k_(5) = position_k(0) - 1.0;

  // calculate LQR gain

  // Find appropriate velocities


  double angle = M_PI / 4.0;
  double cycle = std::floor(
      pow(-1.0, (elapsed_time_.toSec() - std::fmod(elapsed_time_.toSec(), time_max)) / time_max));
  double v = cycle * v_max / 2.0 * (1.0 - std::cos(2.0 * M_PI / time_max * elapsed_time_.toSec()));
  double v_x = std::cos(angle) * v;
  double v_z = -std::sin(angle) * v;
  std::array<double, 6> command = {{v_x, 0.0, v_z, 0.0, 0.0, 0.0}};
  velocity_cartesian_handle_->setCommand(command);
}

void LQRStepController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

}  // namespace franka_coin_controllers

PLUGINLIB_EXPORT_CLASS(franka_coin_controllers::LQRStepController,
                       controller_interface::ControllerBase)
