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

  auto* model_interface = robot_hardware->get<franka_hw::FrankaModelInterface>();
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
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));

    std::array<double, 7> q_start{{0, 0, 0, -7 * M_PI_4 / 2, 0, 7 * M_PI_4 / 2, 0}};
    for (size_t i = 0; i < q_start.size(); i++) {
      if (std::abs(state_handle_->getRobotState().q_d[i] - q_start[i]) > 0.1) {
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

  // Define state space matrices A, B (continuous) and LQR Q and R
  A_ << 0, 0, 1, 0, 0, 0,
        0, 0, 0, 1, 0, 0,
        0, 0, 0, -15, 0, 0,
        0, 0, 15, 0, 0, 0,
        1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0;

  B_ << 0, 0, 0, 0,
        0, 0, 0, 0,
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, -1, 0,
        0, 0, 0, -1;

  Eigen::VectorXd q(6) ;
  q << 1, 4, 0.01, 0.01, 4, 4;
  Q_ = q.array().sqrt().matrix().asDiagonal();

  Eigen::VectorXd r(4) ;
  r << 0.01, 0.01, 1, 1;
  R_ = r.array().sqrt().matrix().asDiagonal();

  pos_init_.setZero();
  state_k_.setZero();
  K_.setZero();

  K_ << 0.1136, 1.4062, 1.2142, -0.0168, 0.0085, 0.0597,
       -1.4062, 0.1136, 0.0168, 1.2142, -0.0597, 0.0085,
       0, 0, 0, 0, 0, 0,
       0, 0, 0, 0, 0, 0; 

  return true;
}

void LQRStepController::starting(const ros::Time& /* time */) {
  elapsed_time_ = ros::Duration(0.0);
  franka::RobotState initial_state = state_handle_->getRobotState();

  // convert to eigen
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  // get initial position
  Eigen::Vector3d position_k = initial_transform.translation();
  pos_init_ = position_k;

}

void LQRStepController::update(const ros::Time& /* time */,
                                                const ros::Duration& period) {
  elapsed_time_ += period;
  franka::RobotState robot_state = state_handle_->getRobotState();
  //std::array<double, 42> jacobian_array =
  //    model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

  // convert to eigen
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  // Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  // Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());

  double time_max = 5.0;
  double v_max = 0.5; // Limit for Panda is 1.7 m/s
  double dt = period.toSec();

  // get new state
  Eigen::Vector3d position_k = transform.translation() - pos_init_;
  // Eigen::Matrix<double, 6, 1> velocity_k = jacobian * dq;

  // state_k_.segment<2>(0) = position_k.head<2>();
  // state_k_.segment<2>(2) = velocity_k.head<2>();

  // Find discrete matrices - depending on period
  Eigen::Matrix<double, 6, 6> A = Eigen::Matrix<double, 6, 6 >::Identity(6,6) + A_*dt;
  Eigen::Matrix<double, 6, 4> B = B_*dt;

  // Find appropriate velocities
  Eigen::Matrix<double, 4, 1> u_k = - K_ * state_k_;
  u_k(2) = 0.2; // Set reference
  Eigen::Matrix<double, 6, 1> state_new = A*state_k_ + B*u_k;

  // Velocity can only be changed by a maximum amount
  state_new(2) = std::min(std::max(state_new(2), -v_max), v_max);
  state_new(3) = std::min(std::max(state_new(3), -v_max), v_max);
  // state_new = state_k_ + (state_new - state_k_)*0.0001;

  // For now just output given velocities - Adapt this in the command below
  if (elapsed_time_.toSec() < 1000) {
    // std::cout << "vx: " << round(1000*state_new(2))/1000 << " m/s, vy: " << round(1000*state_new(3))/1000 << " m/s." << std::endl;
    // std::cout << "delta v: " << sqrt(pow(state_new(0)-state_k_(0),2) + pow(state_new(1)-state_k_(1),2)) << std::endl;
    // std::cout << "x: " << round(1000*state_new(0))/1000 << " m, y: " << round(1000*state_new(1))/1000 << " m" << std::endl;
    // std::cout << "accel: " << sqrt(pow(state_new(0)-state_k_(0),2) + pow(state_new(1)-state_k_(1),2))/dt << std::endl;
    std::cout << "x: " << round(1000*position_k(0))/1000 << " m, y: " << round(1000*position_k(1))/1000 << " m" << std::endl;
  }

  double v_x = state_new(2);
  double v_y = state_new(3);
  state_k_ = state_new;

  std::array<double, 6> command = {{v_x, v_y, 0.0, 0.0, 0.0, 0.0}};
  velocity_cartesian_handle_->setCommand(command);
}

void LQRStepController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

bool LQRStepController::solveRicattiD(const Eigen::MatrixXd &Ad,
                            const Eigen::MatrixXd &Bd, const Eigen::MatrixXd &Q,
                            const Eigen::MatrixXd &R, Eigen::MatrixXd &P,
                            const double &tolerance,
                            const int iter_max) {
  P = Q; // initialize

  Eigen::MatrixXd P_next;

  Eigen::MatrixXd AdT = Ad.transpose();
  Eigen::MatrixXd BdT = Bd.transpose();
  Eigen::MatrixXd Rinv = R.inverse();

  double diff;
  for (int i = 0; i < iter_max; ++i) {
    // -- discrete solver --
    P_next = AdT * P * Ad -
             AdT * P * Bd * (R + BdT * P * Bd).inverse() * BdT * P * Ad + Q;

    diff = fabs((P_next - P).maxCoeff());
    P = P_next;
    if (diff < tolerance) {
      // std::cout << "iteration mumber = " << i << std::endl;
      return true;
    }
  }
  return false; // over iteration limit
}

}  // namespace franka_coin_controllers

PLUGINLIB_EXPORT_CLASS(franka_coin_controllers::LQRStepController,
                       controller_interface::ControllerBase)
