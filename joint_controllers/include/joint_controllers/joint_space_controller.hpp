// Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef JOINT_SPACE_TISD_CONTROLLER_HPP_
#define JOINT_SPACE_TISD_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/component_parser.hpp>
#include <hardware_interface/actuator_interface.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include "joint_controllers/joint_controllers_params.hpp"
#include <joint_controllers/interfaces.hpp>

namespace joint_controllers
{

class JointSpacePositionController : public controller_interface::ControllerInterface
{
public:
  JointSpacePositionController();

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  controller_interface::return_type
  update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  void setPositionCb(std_msgs::msg::Float64MultiArray::ConstSharedPtr msg);

  void interpolate(double t, const Eigen::VectorXd &q_meas);

  virtual void updateParams();
  std::pair<Eigen::VectorXd, Eigen::VectorXd> getActualState() const;

protected:
  template<typename T>
  using InterfaceReferences =
    std::vector<std::vector<std::reference_wrapper<T>>>;
  InterfaceReferences<hardware_interface::LoanedStateInterface>
  joint_state_interfaces_;
  joint_controllers::Params params_;
  std::vector<std::vector<std::string>> state_interface_names_;
  std::shared_ptr<joint_controllers::ParamListener> param_listener_;

  std::vector<std::string> joint_names_;
  pinocchio::Model model_;
  double v_scaling_;
  std::vector<std::string> joint_command_names_;
  std::map<std::string, int> jnt_id_;
  std::map<std::string, int> jnt_command_id_;
  rclcpp::Duration dt_;
  bool first_iter_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_curr_vel;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_curr_pos;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_curr_current;

  Eigen::VectorXd vel_min_;
  Eigen::VectorXd vel_max_;

  double kp_;
  double ki_;
  double lpf_alpha_;
  double integral_clamp_;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
    joint_cmd_sub_;
  double t_acc_ = 0.0;
  double t_flat_ = 0.0;
  double t_curr_ = 0.0;
  double v_max = 2.5;

  Eigen::VectorXd q_start_; // configuration from which the servoing started
  Eigen::VectorXd q_end_; // reference set by the user
  Eigen::VectorXd q_openloop_; // q that would result from the perfect application of the commands
  Eigen::VectorXd q_filtered_; // q filtered using a loz pass exponential filter
  Eigen::VectorXd q_desired_; // soft target, before Franka clamping
  Eigen::VectorXd q_cmd_; // command sent to the hardware interface
  Eigen::VectorXd integral_error_;
  Eigen::VectorXd last_commanded_position_;
  Eigen::VectorXd last_commanded_velocity_;   
  Eigen::VectorXd last_commanded_acceleration_; 
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_current_cmd;

  Eigen::VectorXd accel_max_;
  Eigen::VectorXd jerk_max_;

};
} // namespace joint_controllers

#endif // JOINT_SPACE_TISD_CONTROLLER_HPP_
