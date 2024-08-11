// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <string>
#include <Eigen/Dense>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

namespace franka_coin_controllers {

class LQRStepController : public controller_interface::MultiInterfaceController<
                                               franka_hw::FrankaModelInterface,
                                               franka_hw::FrankaVelocityCartesianInterface,
                                               franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void starting(const ros::Time&) override;
  void stopping(const ros::Time&) override;

 private:
  // Ricatti Solver for LQR
  bool solveRicattiD(const Eigen::MatrixXd &Ad,
                            const Eigen::MatrixXd &Bd, const Eigen::MatrixXd &Q,
                            const Eigen::MatrixXd &R, Eigen::MatrixXd &P,
                            const double &tolerance = 1.E-5,
                            const int iter_max = 100000);
  
  franka_hw::FrankaVelocityCartesianInterface* velocity_cartesian_interface_;
  std::unique_ptr<franka_hw::FrankaCartesianVelocityHandle> velocity_cartesian_handle_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  ros::Duration elapsed_time_;

  Eigen::Matrix<double, 6, 1> state_k_; // Current state vector for LQR controller with Integral Control
  Eigen::Matrix<double, 6, 6> A_; // Matrix A in state-space
  Eigen::Matrix<double, 6, 4> B_; // Matrix B in state-space
  Eigen::Matrix<double, 6, 6> Q_; // Matrix Q in LQR design
  Eigen::Matrix<double, 4, 4> R_; // Matrix R in LQR design
};

}  // namespace franka_coin_controllers
