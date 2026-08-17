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

#include "joint_controllers/tsid_position_control.hpp"
#include <pluginlib/class_list_macros.hpp>
#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

using namespace controller_interface;

namespace joint_controllers
{
using std::placeholders::_1;

TsidPositionControl::TsidPositionControl()
: controller_interface::ControllerInterface(),
  dt_(0, 0)
{
}

controller_interface::CallbackReturn TsidPositionControl::on_init()
{
  try {
    param_listener_ = std::make_shared<joint_controllers::ParamListener>(get_node());

    if (!param_listener_) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize ParamListener.");
      return controller_interface::CallbackReturn::ERROR;
    }
    params_ = param_listener_->get_params();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Exception thrown during controller's init: %s",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TsidPositionControl::on_configure(
  const rclcpp_lifecycle::State & /*prev_state*/)
{
  // Check if parameters were taken correctly
  if (!param_listener_) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }

  param_listener_->refresh_dynamic_parameters();
  params_ = param_listener_->get_params();

  // Check if the actuator names are not empty
  if (params_.joint_state_names.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "The joints name cannot be empty");
    return controller_interface::CallbackReturn::ERROR;
  } else {
    joint_names_ = params_.joint_state_names;
  }

  // Check if command joint names are not empty
  if (params_.joint_command_names.empty() ) {
    RCLCPP_INFO(
      get_node()->get_logger(), "The joint command names is empty. Joint state will be used");
    joint_command_names_ = params_.joint_state_names;
  } else {
    joint_command_names_.resize(params_.joint_command_names.size());
    for (size_t i = 0; i < params_.joint_command_names.size(); i++) {
      size_t start = params_.joint_command_names[i].find("/");
      if (start != std::string::npos) {
        auto joint = params_.joint_command_names[i].substr(start + 1);
        joint_command_names_[i] = joint;
      } else {
        joint_command_names_[i] = params_.joint_command_names[i];
      }
    }
  }

  // Create the state and command interfaces
  state_interfaces_.reserve(3 * joint_names_.size());
  command_interfaces_.reserve(joint_command_names_.size());
  joint_state_interfaces_.resize(joint_names_.size());
  state_interface_names_.resize(joint_names_.size());

  // Creating a map between index and joint
  int idx = 0;

  Interfaces pos_iface = Interfaces::position;
  Interfaces vel_iface = Interfaces::velocity;
  Interfaces eff_iface = Interfaces::effort;

  for (const auto & joint : joint_names_) {
    jnt_id_.insert(std::make_pair(joint, idx));

    joint_state_interfaces_[idx].reserve(3);

    // Create the state interfaces
    state_interface_names_[idx].resize(3);
    state_interface_names_[idx][pos_iface._value] = joint + "/" + pos_iface._to_string();
    state_interface_names_[idx][vel_iface._value] = joint + "/" + vel_iface._to_string();
    state_interface_names_[idx][eff_iface._value] = joint + "/" + eff_iface._to_string();

    idx++;
  }

  idx = 0;

  // Creating a map for command joint
  for (const auto & joint : joint_command_names_) {
    jnt_command_id_.insert(std::make_pair(joint, idx));
    idx++;
  }

  // Creating model in pinocchio
  pinocchio::urdf::buildModelFromXML(
    this->get_robot_description(), pinocchio::JointModelFreeFlyer(), model_);

  RCLCPP_INFO(get_node()->get_logger(), "Model has been built, it has %d joints", model_.njoints);

  std::vector<pinocchio::JointIndex> joints_to_lock;

  for (auto & name : model_.names) {
    if (name != "universe" && name != "root_joint" &&
      std::find(
        joint_names_.begin(), joint_names_.end(),
        name) == joint_names_.end())
    {
      joints_to_lock.push_back(model_.getJointId(name));
      RCLCPP_INFO(get_node()->get_logger(), "Lock joint %s: ", name.c_str());
    }
  }

  /* Removing the unused joints from the model*/
  model_ =
    buildReducedModel(model_, joints_to_lock, pinocchio::neutral(model_));

  for (auto joint : model_.names) {
    RCLCPP_INFO(get_node()->get_logger(), "Joint name: %s", joint.c_str());
  }

  // Getting control period
  dt_ = rclcpp::Duration(std::chrono::duration<double, std::milli>(1e3 / this->get_update_rate()));
  RCLCPP_INFO(get_node()->get_logger(), "Control period: %f", dt_.seconds());

  publisher_curr_pos =
    get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("joint_cmd_pos", 10);

  v_scaling_ = params_.velocity_scaling;
  kp_ = params_.Kp;
  ki_ = params_.Ki;
  lpf_alpha_ = params_.lpf_alpha;
  integral_clamp_ = params_.integral_clamp;
  vel_max_ =
    v_scaling_ * model_.upperVelocityLimit.tail(model_.nv - 6);

  return controller_interface::CallbackReturn::SUCCESS;
}

InterfaceConfiguration TsidPositionControl::state_interface_configuration()
const
{
  std::vector<std::string> state_interfaces_config_names;

  for (const auto & joint: joint_names_) {
    for (int i = 0; i < 3; i++) {
      state_interfaces_config_names.push_back(state_interface_names_[jnt_id_.at(joint)][i]);
    }
  }
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL, state_interfaces_config_names};
}

controller_interface::InterfaceConfiguration
TsidPositionControl::command_interface_configuration() const
{
  std::vector<std::string> command_interfaces_config_names;
  for (const auto & joint : params_.joint_command_names) {
    const auto full_name = joint + "/" + params_.interface_name;
    command_interfaces_config_names.push_back(full_name);
  }

  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    command_interfaces_config_names};
}
controller_interface::CallbackReturn TsidPositionControl::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Check if all the command interfaces are in the actuator interface
  for (const auto & joint : joint_names_) {
    if (
      !controller_interface::get_ordered_interfaces(
        state_interfaces_, state_interface_names_[jnt_id_[joint]],
        std::string(""),
        joint_state_interfaces_[jnt_id_[joint]]))
    {
      RCLCPP_ERROR(
        this->get_node()->get_logger(), "Expected %zu state interfaces, got %zu",
        state_interface_names_[jnt_id_[joint]].size(),
        joint_state_interfaces_[jnt_id_[joint]].size());
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  for (const auto & joint : joint_names_) {
    RCLCPP_INFO(
      this->get_node()->get_logger(), "Joint %s position: %f",
      joint.c_str(), joint_state_interfaces_[jnt_id_[joint]][0].get().get_optional().value());
  }

  // Taking initial position from the joint state interfaces
  Eigen::VectorXd q0 = Eigen::VectorXd::Zero(model_.nq);
  Eigen::VectorXd v0 = Eigen::VectorXd::Zero(model_.nv);

  std::pair<Eigen::VectorXd, Eigen::VectorXd> state = getActualState();

  q0 = state.first;
  // q_prev_ = q0.tail(model_.nq - 7);

  // formulation_->computeProblemData(0.0, q0, v0);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TsidPositionControl::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{ // reset command interfaces
  release_interfaces();
  for (auto joint : params_.joint_state_names) {
    joint_state_interfaces_[jnt_id_[joint]].clear();
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

std::pair<Eigen::VectorXd, Eigen::VectorXd> TsidPositionControl::getActualState() const
{
  Eigen::VectorXd q = Eigen::VectorXd::Zero(model_.nq);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(model_.nv);

  for (const auto & joint : joint_names_) {
    q.tail(model_.nq - 7)[model_.getJointId(joint) - 2] =
      joint_state_interfaces_[jnt_id_.at(joint)][Interfaces::position].get().get_optional().value();
    v.tail(model_.nv - 6)[model_.getJointId(joint) - 2] =
      joint_state_interfaces_[jnt_id_.at(joint)][Interfaces::velocity].get().get_optional().value();
  }

  return std::make_pair(q, v);
}

void TsidPositionControl::updateParams()
{
  if (param_listener_->is_old(params_)) {
    params_ = param_listener_->get_params();
    RCLCPP_INFO(get_node()->get_logger(), "Updating parameters");


    v_scaling_ = params_.velocity_scaling;
    vel_max_ =
      v_scaling_ * model_.velocityLimit.tail(model_.nv - 6);

    kp_ = params_.Kp;
    ki_ = params_.Ki;
    lpf_alpha_ = params_.lpf_alpha;
    integral_clamp_ = params_.integral_clamp;

    for (const auto & joint : joint_names_) {
      if (std::find(
          joint_command_names_.begin(), joint_command_names_.end(),
          joint) == joint_command_names_.end())
      {
        vel_max_[model_.getJointId(joint) - 2] = 0;
      }
    }

  }
}

}  // namespace dynamic_tsid_controller
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  joint_controllers::TsidPositionControl,
  controller_interface::ControllerInterface)
