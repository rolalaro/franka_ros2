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

#ifndef JOINT_SPACE_VEL_TSID_CONTROLLER_HPP_
#define JOINT_SPACE_VEL_TSID_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/actuator_interface.hpp>
#include <hardware_interface/component_parser.hpp>
#include <joint_controllers/interfaces.hpp>
#include <joint_controllers/joint_controllers_params.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

namespace joint_controllers
{

class JointSpaceVelocityController
  : public controller_interface::ControllerInterface
{
public:
  JointSpaceVelocityController();

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type
  update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  void DefaultVelocityTasks();
  virtual void updateParams();
  std::pair<Eigen::VectorXd, Eigen::VectorXd> getActualState() const;

  void setVelocityCb(std_msgs::msg::Float64MultiArray::ConstSharedPtr msg);

private:

  template<typename T>
  using InterfaceReferences =
    std::vector<std::vector<std::reference_wrapper<T>>>;
  void check_limit_reached(
    const Eigen::VectorXd & q, const Eigen::VectorXd & v);

  Eigen::VectorXd create_velocity_ramp(const Eigen::VectorXd &vel_current, const Eigen::VectorXd & vel_cmd, Eigen::VectorXd &accel_current, const Eigen::VectorXd &accel_max, const Eigen::VectorXd &jerk_max) const;
    
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
  bool joint_limit_reached_;
  pinocchio::Data data_;
  Eigen::VectorXd q_cmd_;
  Eigen::VectorXd v_com_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_curr_vel_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_curr_pos_;
  Eigen::VectorXd q_int_;
  Eigen::VectorXd v_int_;
  Eigen::VectorXd q_min_, q_max_;
  Eigen::VectorXd q_prev_;
  Eigen::VectorXd v_max_;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
    joint_cmd_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_current_cmd_;
  Eigen::VectorXd velocity_cmd_;

  Eigen::VectorXd accel_max_;
  Eigen::VectorXd accel_current_;
  Eigen::VectorXd jerk_max_;

};
} // namespace joint_controllers

#endif // JOINT_SPACE_VEL_TSID_CONTROLLER_HPP_
