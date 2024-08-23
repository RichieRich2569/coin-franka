// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_coin_controllers/coin_spontaneous_controller.h>

#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <fstream>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace franka_coin_controllers {

bool COINSpontaneousController::init(hardware_interface::RobotHW* robot_hardware,
                                              ros::NodeHandle& node_handle) {
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("COINSpontaneousController: Could not get parameter arm_id");
    return false;
  }

  velocity_cartesian_interface_ =
      robot_hardware->get<franka_hw::FrankaVelocityCartesianInterface>();
  if (velocity_cartesian_interface_ == nullptr) {
    ROS_ERROR(
        "COINSpontaneousController: Could not get Cartesian velocity interface from "
        "hardware");
    return false;
  }
  try {
    velocity_cartesian_handle_ = std::make_unique<franka_hw::FrankaCartesianVelocityHandle>(
        velocity_cartesian_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "COINSpontaneousController: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  auto* model_interface = robot_hardware->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "COINSpontaneousController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "COINSpontaneousController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("COINSpontaneousController: Could not get state interface from hardware");
    return false;
  }

  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));

    std::array<double, 7> q_start{{0, 0, 0, -7 * M_PI_4 / 2, 0, 7 * M_PI_4 / 2, 0}};
    for (size_t i = 0; i < q_start.size(); i++) {
      if (std::abs(state_handle_->getRobotState().q_d[i] - q_start[i]) > 0.1) {
        ROS_ERROR_STREAM(
            "COINSpontaneousController: Robot is not in the expected starting position "
            "for running this example. Run `roslaunch franka_coin_controllers "
            "coin_move_to_start.launch robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` "
            "first.");
        return false;
      }
    }
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "COINSpontaneousController: Exception getting state handle: " << e.what());
    return false;
  }

  // Obtain current trial
  int trial_param;
  // Retrieve the parameter value from the parameter server
  if (!node_handle.getParam("trial_param", trial_param)) {
      ROS_WARN("COINSpontaneousController: Parameter 'trial' not set. Using default value.");
      trial_param = 1; // default value
  }
  ROS_INFO("Beginning Trial: %d", trial_param);

  // Load coin values
  Eigen::Matrix<double, 340, 18> G;
  if (!loadCOINestimates(G)) {
     ROS_ERROR("COINSpontaneousController: Could not load COIN values");
  } else {
    ROS_INFO("COIN Estimates successfully loaded from file");
  }

  g_estimates_ = G.row(trial_param-1);

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
  q << 1, 0.001, 1, 1, 100, 10;
  Q_ = q.array().matrix().asDiagonal();

  Eigen::VectorXd r(4) ;
  r << 0.001, 0.001, 1000, 1000;
  R_ = r.array().matrix().asDiagonal();

  pos_init_.setZero();
  state_k_.setZero();
  K_.setZero();
  x_goal_ = 0.2; // Goal set to 0.2m in front of robot's initial pose.

  // K_ << 142.79, 33.06, 35.21, -1.18, 281.56, 38.82,
  //      -63.56, 77.98, -0.24, 33.94, -121.43, 89.92,
  //      0, 0, 0, 0, 0, 0,
  //      0, 0, 0, 0, 0, 0;
  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(A_.rows(),A_.cols());
  Eigen::MatrixXd A = 0.001*A_ + Eigen::MatrixXd::Identity(A_.rows(),A_.cols());
  Eigen::MatrixXd B = 0.001*B_;

  solveRicattiD(A,B,Q_,R_,P);

  K_ = (R_.inverse() * B.transpose() * P);
  K_.bottomRows(2).setZero();

  return true;
}

void COINSpontaneousController::starting(const ros::Time& /* time */) {
  elapsed_time_ = ros::Duration(0.0);
  franka::RobotState initial_state = state_handle_->getRobotState();

  // convert to eigen
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  // get initial state
  pos_init_ = initial_transform.translation();

  // Find discrete matrices - depending on period
  // double dt = 0.001; // Period yet unknown, but on average from 1 KHz communication.
  // Eigen::Matrix<double, 6, 6> A = Eigen::Matrix<double, 6, 6 >::Identity(6,6) + A_*dt;
  // Eigen::Matrix<double, 6, 4> B = B_*dt;

  // Calculate LQR gain
  // Eigen::MatrixXd P = Eigen::Matrix<double, 6, 6>::Zero();
  // solveRicattiD(A,B,Q_,R_,P);
  // K_ =  (R_.inverse() * B_.transpose() * P);
  // K_.bottomRows(2).setZero();

}

void COINSpontaneousController::update(const ros::Time& /* time */,
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

  // Measure true position of robot to update state.
  Eigen::Vector3d position_k = (transform.translation() - pos_init_)/x_goal_; // position scaled to controller model
  state_k_(0) = position_k(0);
  state_k_(1) = position_k(1);

  // measure covariance /////////////////
  // static Eigen::Matrix<double,6,1> state_km1 = state_k_;
  // static Eigen::Matrix<double,6,1> mu_k = Eigen::Matrix<double,6,1>::Zero();
  // static Eigen::Matrix<double,6,6> S_k = Eigen::Matrix<double, 6, 6>::Zero();
  // static int k = 0;
  // Eigen::Matrix<double,6,1> y_k; y_k << position_k(0), position_k(1), 
  //                                       state_k_(2), state_k_(3),
  //                                       state_km1(4)+dt*(position_k(0)-1), 
  //                                       state_km1(5) + dt*position_k(1);
  // Eigen::Matrix<double,6,6> VTV = (y_k-state_k_)*(y_k-state_k_).transpose();
  // S_k = k/(k+1)*S_k + VTV;
  // mu_k = k/(k+1)*mu_k + (y_k-state_k_);
  // if (k < 1000) {
  //   std::cout << "k: " << k << "\n" << "res_x: " << y_k(0) - state_k_(0) << ", res_y: " << y_k(1) - state_k_(1) << std::endl;
  // }
  // k++;
  ///////////////////////////////////////



  // Find discrete matrices - depending on period
  Eigen::Matrix<double, 6, 6> A = Eigen::Matrix<double, 6, 6 >::Identity(6,6) + A_*dt;
  Eigen::Matrix<double, 6, 4> B = B_*dt;

  // Find appropriate velocities
  Eigen::Matrix<double, 4, 1> u_k = - K_ * state_k_;
  u_k(2) = 1; // Set reference (y-reference automatically set to zero with K definition)
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
    // std::cout << "x: " << round(1000*position_k(0))/1000 << " m, y: " << round(1000*position_k(1))/1000 << " m" << std::endl;
  }

  double v_x = x_goal_*state_new(2); // Scale is adjusted (unit step in control, x_goal_ in front of robot.)
  double v_y = x_goal_*state_new(3);
  //state_km1 = state_k_;
  state_k_ = state_new;

  std::array<double, 6> command = {{v_x, v_y, 0.0, 0.0, 0.0, 0.0}}; 
  velocity_cartesian_handle_->setCommand(command);
}

void COINSpontaneousController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

void COINSpontaneousController::solveRicattiD(const Eigen::MatrixXd &A,
                            const Eigen::MatrixXd &B, const Eigen::MatrixXd &Q,
                            const Eigen::MatrixXd &R, Eigen::MatrixXd &P,
                            const double &tolerance,
                            const int iter_max,
                            const bool &verbose) {
  
    P = Q;
    Eigen::MatrixXd G = B * R.inverse() * B.transpose();
    Eigen::MatrixXd Ak = A;
    Eigen::MatrixXd Ak_next;
    Eigen::MatrixXd P_next;
    Eigen::MatrixXd G_next;
    int m = A.rows();
    int n = A.cols();
    double diff;
    if (verbose) {
        std::cout << "Iter " << "      " << " Error " <<  std::endl;
        std::cout << "-----------------" << std::endl << std::endl;
    }
    for (int i=0; i < iter_max; ++i) {
        Ak_next = Ak * (Eigen::MatrixXd::Identity(m, n) + G * P).inverse()*Ak;
        G_next = G + Ak*(Eigen::MatrixXd::Identity(m, n) + G * P).inverse()*G*Ak.transpose();
        P_next = P + Ak.transpose()*P*(Eigen::MatrixXd::Identity(m, n) + G * P).inverse()*Ak;

        diff = fabs((P_next - P).maxCoeff());
        P = P_next;
        Ak = Ak_next;
        G = G_next;
        if (verbose) {
            std::cout << i << "         " << diff << std::endl;
        }
        if (diff < tolerance) {
        return;
        }
    }
}

bool COINSpontaneousController::loadCOINestimates(Eigen::Matrix<double, 340, 18> &A) {   
    Eigen::MatrixXd matrix(1,340*18);

    std::ifstream file("/home/richard/catkin_ws/src/coin-franka/coin/g_estimates.bin", std::ios::binary); // NOT a relative PATH
    if (file.is_open()) {
        file.read(reinterpret_cast<char*>(matrix.data()), sizeof(double) * 340 * 18);
        file.close();
    } else {
        return false;
    }

    // Reshape matrix
    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> reshapedMatrix(matrix.data(), 340, 18);
    A = reshapedMatrix;
    
    return true;
}

} // namespace franka_coin_controllers

PLUGINLIB_EXPORT_CLASS(franka_coin_controllers::COINSpontaneousController,
                       controller_interface::ControllerBase)