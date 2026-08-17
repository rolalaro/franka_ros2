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


#include <franka/rate_limiting.h>

using namespace controller_interface;

namespace joint_controllers
{
using std::placeholders::_1;

JointSpacePositionController::JointSpacePositionController()
: joint_controllers::TsidPositionControl() {}

controller_interface::CallbackReturn JointSpacePositionController::on_configure(
  const rclcpp_lifecycle::State & prev_state)
{
  position_start_ = Eigen::VectorXd::Zero(params_.joint_command_names.size());
  position_end_ = Eigen::VectorXd::Zero(params_.joint_command_names.size());
  position_curr_ = Eigen::VectorXd::Zero(params_.joint_command_names.size());
  vel_curr_ = Eigen::VectorXd::Zero(params_.joint_command_names.size());

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

  accel_max_ = Eigen::VectorXd::Zero(params_.joint_command_names.size());
  jerk_max_ = Eigen::VectorXd::Zero(params_.joint_command_names.size());
  for(int i = 0;i < accel_max_.size(); ++i) {
    accel_max_[i] = franka::kMaxJointAcceleration[i];
    jerk_max_[i] = franka::kMaxJointJerk[i];
  }

  return TsidPositionControl::on_configure(prev_state);
}

controller_interface::CallbackReturn JointSpacePositionController::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  auto result = TsidPositionControl::on_activate(previous_state);
  if (result != controller_interface::CallbackReturn::SUCCESS) {
    return result; // Propagate error if the base configuration fails
  }

  std::pair<Eigen::VectorXd, Eigen::VectorXd> state = getActualState();

  position_start_ = state.first.tail(params_.joint_command_names.size());

  position_end_ = position_start_;

  first_tsid_iter_ = true;

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type JointSpacePositionController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  t_curr_ = t_curr_ + dt_.seconds();

  TsidPositionControl::updateParams();

  std::pair<Eigen::VectorXd, Eigen::VectorXd> state = getActualState();
  state.first[6] = 1.0;
  Eigen::VectorXd &q = state.first, &v = state.second;

  interpolate(t_curr_); // compute the velocity and the position

  /// BEGIN compute_problem_and_set_command(state.first, state.second); //q and v
  if (first_tsid_iter_) {
    q_int_ = q;
    v_int_ = v;
    first_tsid_iter_ = false;
  }

  v_int_.head(6) = Eigen::VectorXd::Zero(6);
  v_int_.tail(model_.nv - 6) = vel_curr_;
  // Computing the problem data
  Eigen::VectorXd q_cmd;

  // Integrating velocity to get position
  RCLCPP_INFO_STREAM(get_node()->get_logger(),"q_int_(" << t_curr_ << ") = " << q_int_.transpose());
  q_int_ = pinocchio::integrate(
    model_, q_int_, v_int_ * dt_.seconds());

  q_cmd = q_int_.tail(model_.nq - 7);
  RCLCPP_INFO_STREAM(get_node()->get_logger(),"v_int_(" << t_curr_ << ") = " << v_int_.transpose());
  RCLCPP_INFO_STREAM(get_node()->get_logger(),"q_cmd(" << t_curr_ << ") = " << q_cmd.transpose());

  // Setting the command to the joint command interfaces
  for (const auto & joint : joint_command_names_) {
    if (!command_interfaces_[jnt_command_id_[joint]].set_value(
        q_cmd[model_.getJointId(joint) - 2]))
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Failed to set command for joint %s",
        joint.c_str());
    }
  }
  q_prev_ = q.tail(model_.nq - 7);

  std_msgs::msg::Float64MultiArray pub_pos;
  for (Eigen::Index i = 0; i < q_cmd.size(); i++) {
    pub_pos.data.push_back(q_cmd[i]);
  }

  publisher_curr_pos->publish(pub_pos);

  q_prev_ = q.tail(model_.nq - 7);
  /// END compute_problem_and_set_command(state.first, state.second); //q and v

  //Publishing the current command
  auto message = sensor_msgs::msg::JointState();
  message.header.stamp = get_node()->get_clock()->now();

  message.name = joint_command_names_;
  message.position.resize(joint_command_names_.size());
  message.velocity.resize(joint_command_names_.size());
  message.effort.resize(joint_command_names_.size());

  for (size_t i = 0; i < joint_command_names_.size(); i++) {
    message.position[i] = position_end_[i];
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

  position_start_ = getActualState().first.tail(params_.joint_command_names.size());
  position_end_ = ref;

  t_curr_ = 0;
  first_tsid_iter_ = true;

}

void JointSpacePositionController::interpolate(double t_curr)
{
  if (position_end_ == position_start_) {
    position_curr_ = position_end_;
    vel_curr_ = Eigen::Zero(params_.joint_command_names.size());
    return;
  }


  int maxDiffIndex = 0;
  double maxDiff = std::abs(position_end_[0] - position_start_[0]);

  for (Eigen::Index i = 1; i < position_start_.size(); ++i) {
    double currentDiff = std::abs(position_end_[i] - position_start_[i]);
    if (currentDiff > maxDiff) {
      maxDiff = currentDiff;
      maxDiffIndex = i;
    }
  }

  double s = 0;
  double s_dot = 0;

  // Computation of a trapezoidal trajectory
  for (auto joint : params_.joint_command_names) {
    int joint_id = jnt_id_[joint];
    double a_max = accel_max_[joint_id];
    double v_max_ = params_.velocity_scaling * v_max;
    t_acc_ = v_max_ / a_max;
    t_flat_ =
      (std::abs(position_end_[maxDiffIndex] - position_start_[maxDiffIndex]) - v_max_ * t_acc_) /
      v_max_;
    if (t_curr < t_acc_) {
      s = 0.5 * a_max * t_curr * t_curr;
      s_dot = a_max * t_curr;
    } else if (t_curr >= t_acc_ && t_curr < t_acc_ + t_flat_) {
      s = v_max_ *
        (std::abs(position_end_[joint_id] - position_start_[joint_id])) /
        (std::abs(position_end_[maxDiffIndex] - position_start_[maxDiffIndex])) *
        (t_curr - t_acc_ / 2 );
      s_dot = v_max_ * (std::abs(position_end_[joint_id] - position_start_[joint_id])) /
        (std::abs(position_end_[maxDiffIndex] - position_start_[maxDiffIndex]));
    } else if (t_curr >= t_acc_ + t_flat_ && t_curr < t_flat_ + 2 * t_acc_) {
      s = std::abs(position_end_[joint_id] - position_start_[joint_id]) -
        0.5 * a_max *
        (t_flat_ + 2 * t_acc_ - t_curr) *
        (t_flat_ + 2 * t_acc_ - t_curr);
      s_dot = v_max_ * (std::abs(position_end_[joint_id] - position_start_[joint_id])) /
        (std::abs(position_end_[maxDiffIndex] - position_start_[maxDiffIndex])) - a_max *
        (t_curr - t_flat_ - t_acc_);
    } else {
      s = std::abs(position_end_[joint_id] - position_start_[joint_id]);
      s_dot = 0;
    }
    position_curr_[joint_id] = position_start_[joint_id] +
      copysign(1, position_end_[joint_id] - position_start_[joint_id]) * s;
    vel_curr_[joint_id] =
      copysign(1, position_end_[joint_id] - position_start_[joint_id]) * s_dot;
  }

  RCLCPP_INFO_STREAM(get_node()->get_logger(),"position_start_(" <<  t_curr << ") = " << position_start_.transpose());
  RCLCPP_INFO_STREAM(get_node()->get_logger(),"position_end_(" <<  t_curr << ") = " << position_end_.transpose());
  RCLCPP_INFO_STREAM(get_node()->get_logger(),"position_curr_(" <<  t_curr << ") = " << position_curr_.transpose());
  RCLCPP_INFO_STREAM(get_node()->get_logger(),"vel_curr_(" <<  t_curr << ") = " << vel_curr_.transpose());
}

} // namespace joint_controllers
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  joint_controllers::JointSpacePositionController,
  controller_interface::ControllerInterface)
