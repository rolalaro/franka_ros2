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

#include "joint_controllers/joint_space_controller.hpp"
#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <pluginlib/class_list_macros.hpp>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>


#include <franka/rate_limiting.h>

using namespace controller_interface;

namespace joint_controllers
{
using std::placeholders::_1;

JointSpacePositionController::JointSpacePositionController()
: controller_interface::ControllerInterface(),
  dt_(0, 0) {}

controller_interface::CallbackReturn JointSpacePositionController::on_init()
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


controller_interface::CallbackReturn JointSpacePositionController::on_configure(
  const rclcpp_lifecycle::State & prev_state)
{
  q_start_ = Eigen::VectorXd::Zero(params_.joint_command_names.size());
  q_end_ = Eigen::VectorXd::Zero(params_.joint_command_names.size());
  q_cmd_ = Eigen::VectorXd::Zero(params_.joint_command_names.size());
  q_openloop_ = Eigen::VectorXd::Zero(params_.joint_command_names.size());
  q_filtered_ = Eigen::VectorXd::Zero(params_.joint_command_names.size());
  q_desired_ = Eigen::VectorXd::Zero(params_.joint_command_names.size());
  q_cmd_ = Eigen::VectorXd::Zero(params_.joint_command_names.size());
  integral_error_ = Eigen::VectorXd::Zero(params_.joint_command_names.size());
  last_commanded_position_ = Eigen::VectorXd::Zero(params_.joint_command_names.size());
  last_commanded_velocity_ = Eigen::VectorXd::Zero(params_.joint_command_names.size());   
  last_commanded_acceleration_ = Eigen::VectorXd::Zero(params_.joint_command_names.size());  

  std::string controller_name = get_node()->get_name();
  // Position command
  joint_cmd_sub_ =
    get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    controller_name + "/joint_position_cmd", 1,
    std::bind(&JointSpacePositionController::setPositionCb, this, _1));

  // Publisher for current command
  publisher_current_cmd =
    get_node()->create_publisher<sensor_msgs::msg::JointState>(
    controller_name + "/joint_position_cmd_curr", 1);

  // Initializing containers
  vel_max_ = Eigen::VectorXd::Zero(params_.joint_command_names.size());
  vel_min_ = Eigen::VectorXd::Zero(params_.joint_command_names.size());
  accel_max_ = Eigen::VectorXd::Zero(params_.joint_command_names.size());
  jerk_max_ = Eigen::VectorXd::Zero(params_.joint_command_names.size());

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
  vel_min_ =
    -v_scaling_ * model_.lowerVelocityLimit.tail(model_.nv - 6);

  for(const auto & joint : joint_names_) {
    int joint_id = jnt_id_[joint];
    if(joint.find(std::to_string(joint_id + 1)) == std::string::npos) throw; // Ensure mapping of the order of the joints names and Franka library limits
    accel_max_[joint_id] = franka::kMaxJointAcceleration[joint_id];
    jerk_max_[joint_id] = franka::kMaxJointJerk[joint_id];
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointSpacePositionController::on_activate(
  const rclcpp_lifecycle::State & previous_state)
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
  std::pair<Eigen::VectorXd, Eigen::VectorXd> state = getActualState();

  q_start_ = state.first.tail(params_.joint_command_names.size());

  q_end_ = q_start_;

  first_iter_ = true;

  return controller_interface::CallbackReturn::SUCCESS;
}

InterfaceConfiguration JointSpacePositionController::state_interface_configuration()
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
JointSpacePositionController::command_interface_configuration() const
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

controller_interface::CallbackReturn JointSpacePositionController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{ // reset command interfaces
  release_interfaces();
  for (auto joint : params_.joint_state_names) {
    joint_state_interfaces_[jnt_id_[joint]].clear();
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

std::pair<Eigen::VectorXd, Eigen::VectorXd> JointSpacePositionController::getActualState() const
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

void JointSpacePositionController::updateParams()
{
  if (param_listener_->is_old(params_)) {
    params_ = param_listener_->get_params();
    RCLCPP_INFO(get_node()->get_logger(), "Updating parameters");


    v_scaling_ = params_.velocity_scaling;
    vel_max_ =
      v_scaling_ * model_.upperVelocityLimit.tail(model_.nv - 6);
    vel_min_ =
      -v_scaling_ * model_.lowerVelocityLimit.tail(model_.nv - 6);

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
        vel_min_[model_.getJointId(joint) - 2] = 0;
      }
    }

  }
}

controller_interface::return_type JointSpacePositionController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  t_curr_ = t_curr_ + dt_.seconds();

  updateParams();

  std::pair<Eigen::VectorXd, Eigen::VectorXd> state = getActualState();
  state.first[6] = 1.0;
  Eigen::VectorXd &q = state.first /* , &v = state.second */;

  interpolate(t_curr_, q.tail(7)); // compute the velocity and the position

  /// BEGIN compute_problem_and_set_command(state.first, state.second); //q and v
  
  // Setting the command to the joint command interfaces
  for (const auto & joint : joint_command_names_) {
    if (!command_interfaces_[jnt_command_id_[joint]].set_value(
        q_cmd_[model_.getJointId(joint) - 2])) // Minus 2 to remove "universe" and "root_joint" joints
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Failed to set command for joint %s",
        joint.c_str());
    }
  }

  std_msgs::msg::Float64MultiArray pub_pos;
  for (Eigen::Index i = 0; i < q_cmd_.size(); i++) {
    pub_pos.data.push_back(q_cmd_[i]);
  }

  publisher_curr_pos->publish(pub_pos);
  /// END compute_problem_and_set_command(state.first, state.second); //q and v

  //Publishing the current command
  auto message = sensor_msgs::msg::JointState();
  message.header.stamp = get_node()->get_clock()->now();

  message.name = joint_command_names_;
  message.position.resize(joint_command_names_.size());
  message.velocity.resize(joint_command_names_.size());
  message.effort.resize(joint_command_names_.size());

  for (size_t i = 0; i < joint_command_names_.size(); i++) {
    message.position[i] = q_end_[i];
    message.velocity[i] = 0.0;
    message.effort[i] = 0.0;
  }

  publisher_current_cmd->publish(message);

  return controller_interface::return_type::OK;
}

void JointSpacePositionController::setPositionCb(
  std_msgs::msg::Float64MultiArray::ConstSharedPtr msg)
{
  if (msg->data.size() != params_.joint_command_names.size()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Received joint position command with incorrect size");
    return;
  }

  auto upper_limits = model_.upperPositionLimit.tail(model_.nv - 6);
  auto lower_limits = model_.lowerPositionLimit.tail(model_.nv - 6);

  for (auto joint : joint_command_names_) {

    if (msg->data[jnt_command_id_[joint]] >
      upper_limits[model_.getJointId(joint) - 2] ||
      msg->data[jnt_command_id_[joint]] <
      lower_limits[model_.getJointId(joint) - 2])
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Joint %s command out of boundaries! The motion will not be "
        "performed!",
        joint.c_str());
      return;
    }
  }

  // Setting the reference
  Eigen::VectorXd ref(params_.joint_command_names.size());

  for (size_t i = 0; i < params_.joint_command_names.size(); i++) {
    ref[i] = msg->data[i];
  }

  q_start_ = getActualState().first.tail(params_.joint_command_names.size());
  q_end_ = ref;

  t_curr_ = 0.;
  integral_error_ = Eigen::VectorXd::Zero(q_cmd_.size());
  first_iter_ = true;

}

void JointSpacePositionController::interpolate(double t_curr, const Eigen::VectorXd &q_meas)
{
  if (q_end_.isApprox(q_start_,1e-5)) {
    q_cmd_ = q_end_;
    q_openloop_ = q_end_;
    return;
  }

  double dt = dt_.seconds();
  if (first_iter_) {
    dt = 0.;
    q_start_ = q_meas;
    q_filtered_ = q_meas;
    last_commanded_position_ = q_meas; // or position_start_
    last_commanded_velocity_ = Eigen::VectorXd::Zero(q_cmd_.size()); 
    last_commanded_acceleration_ = Eigen::VectorXd::Zero(q_cmd_.size()); 
    first_iter_ = false;
  }


  int maxDiffIndex = 0;
  double maxDiff = std::abs(q_end_[0] - q_start_[0]);

  for (Eigen::Index i = 1; i < q_start_.size(); ++i) {
    double currentDiff = std::abs(q_end_[i] - q_start_[i]);
    if (currentDiff > maxDiff) {
      maxDiff = currentDiff;
      maxDiffIndex = i;
    }
  }
  double a_max = accel_max_[maxDiffIndex];
  double v_max = vel_max_[maxDiffIndex];
  t_acc_ = v_max / a_max;
  t_flat_ =
    (std::abs(q_end_[maxDiffIndex] - q_start_[maxDiffIndex]) - v_max * t_acc_) /
    v_max;

  double s = 0;
  // double s_dot = 0;

  // Computation of a trapezoidal trajectory
  for (auto joint : params_.joint_command_names) {
    int joint_id = jnt_id_[joint];
    double accel = accel_max_[joint_id];
    double vel = vel_max_[joint_id];
    if (t_curr < t_acc_) {
      s = 0.5 * accel * t_curr * t_curr;
      // s_dot = accel * t_curr;
    } else if (t_curr >= t_acc_ && t_curr < t_acc_ + t_flat_) {
      s = vel *
        (std::abs(q_end_[joint_id] - q_start_[joint_id])) /
        (std::abs(q_end_[maxDiffIndex] - q_start_[maxDiffIndex])) *
        (t_curr - t_acc_ / 2 );
      // s_dot = vel * (std::abs(q_end_[joint_id] - q_start_[joint_id])) /
      //   (std::abs(q_end_[maxDiffIndex] - q_start_[maxDiffIndex]));
    } else if (t_curr >= t_acc_ + t_flat_ && t_curr < t_flat_ + 2 * t_acc_) {
      s = std::abs(q_end_[joint_id] - q_start_[joint_id]) -
        0.5 * accel *
        (t_flat_ + 2 * t_acc_ - t_curr) *
        (t_flat_ + 2 * t_acc_ - t_curr);
      // s_dot = vel * (std::abs(q_end_[joint_id] - q_start_[joint_id])) /
      //   (std::abs(q_end_[maxDiffIndex] - q_start_[maxDiffIndex])) - accel *
      //   (t_curr - t_flat_ - t_acc_);
    } else {
      s = std::abs(q_end_[joint_id] - q_start_[joint_id]);
      // s_dot = 0;
    }
    
    double sign = copysign(1.0, q_end_[joint_id] - q_start_[joint_id]);
    q_openloop_[joint_id] = q_start_[joint_id] + sign * s;
    
    // ---- feedback trim ----
    // 1) low-pass the noisy measurement (cheap alternative to a Kalman filter;
    //    we don't need a velocity estimate, so a state observer is overkill)
    q_filtered_[joint_id] =
      lpf_alpha_ * q_meas[joint_id] + (1.0 - lpf_alpha_) * q_filtered_[joint_id];

    // 2) tracking error against the feedforward reference
    double error = q_openloop_[joint_id] - q_filtered_[joint_id];

    // 3) integral with anti-windup clamp
    if (dt > 0.0) {
      integral_error_[joint_id] =
        std::clamp(integral_error_[joint_id] + error * dt, -integral_clamp_, integral_clamp_);
    }

    double correction = kp_ * error + ki_ * integral_error_[joint_id];

    // 4) apply correction to the commanded position, then hard-clamp to
    //    [start, end] so overshoot past the target is structurally impossible
    double lo = std::min(q_start_[joint_id], q_end_[joint_id]);
    double hi = std::max(q_start_[joint_id], q_end_[joint_id]);
    q_desired_[joint_id] =
      std::clamp(q_openloop_[joint_id] + correction, lo, hi);
  }

  // Rate-limit per joint using libfranka's helper (one call per joint, or use franka_hw wrapper if it batches)
  for (auto joint : params_.joint_command_names) {
    int joint_id = jnt_id_[joint];
    double limited = franka::limitRate(
      vel_max_[joint_id], vel_min_[joint_id], accel_max_[joint_id], jerk_max_[joint_id],
      q_desired_[joint_id],
      last_commanded_position_[joint_id],
      last_commanded_velocity_[joint_id],
      last_commanded_acceleration_[joint_id]);

    double new_vel = (limited - last_commanded_position_[joint_id]) / dt_.seconds();
    double new_acc = (new_vel - last_commanded_velocity_[joint_id]) / dt_.seconds();

    q_cmd_[joint_id] = limited;
    last_commanded_acceleration_[joint_id] = new_acc;
    last_commanded_velocity_[joint_id] = new_vel;
    last_commanded_position_[joint_id] = limited;
  }
  RCLCPP_INFO_STREAM(get_node()->get_logger(), "vel_max_ = " << vel_max_.transpose());
  RCLCPP_INFO_STREAM(get_node()->get_logger(), "vel_min_ = " << vel_min_.transpose());
  RCLCPP_INFO_STREAM(get_node()->get_logger(), "accel_max_ = " << accel_max_.transpose());
  RCLCPP_INFO_STREAM(get_node()->get_logger(), "jerk_max_ = " << jerk_max_.transpose());
  RCLCPP_INFO_STREAM(get_node()->get_logger(),"q_meas(" <<  t_curr << ") = " << q_meas.transpose());
  RCLCPP_INFO_STREAM(get_node()->get_logger(),"q_start_(" <<  t_curr << ") = " << q_start_.transpose());
  RCLCPP_INFO_STREAM(get_node()->get_logger(),"q_end_(" <<  t_curr << ") = " << q_end_.transpose());
  RCLCPP_INFO_STREAM(get_node()->get_logger(),"q_cmd_(" <<  t_curr << ") = " << q_cmd_.transpose());
  RCLCPP_INFO_STREAM(get_node()->get_logger(),"last_commanded_velocity_(" <<  t_curr << ") = " << last_commanded_velocity_.transpose());
  RCLCPP_INFO_STREAM(get_node()->get_logger(),"last_commanded_acceleration_(" <<  t_curr << ") = " << last_commanded_acceleration_.transpose());
}

} // namespace joint_controllers
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  joint_controllers::JointSpacePositionController,
  controller_interface::ControllerInterface)
