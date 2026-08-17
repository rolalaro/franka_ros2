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

#include "joint_controllers/joint_space_vel_controller.hpp"
#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <pluginlib/class_list_macros.hpp>

#include <franka/rate_limiting.h>

using namespace controller_interface;

namespace joint_controllers
{
using std::placeholders::_1;

JointSpaceVelocityController::JointSpaceVelocityController()
: joint_controllers::TsidVelocityControl() {}

controller_interface::CallbackReturn JointSpaceVelocityController::on_configure(
  const rclcpp_lifecycle::State & prev_state)
{
  auto result = TsidVelocityControl::on_configure(prev_state);
  if (result != controller_interface::CallbackReturn::SUCCESS) {
    return result; // Propagate error if the base configuration fails
  }

  // Position command
  std::string controller_name = get_node()->get_name();

  joint_cmd_sub_ =
    get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    controller_name + "/joint_velocity_cmd", 1,
    std::bind(&JointSpaceVelocityController::setVelocityCb, this, _1));

  // Publisher for current command
  publisher_current_cmd_ =
    get_node()->create_publisher<sensor_msgs::msg::JointState>(
    controller_name + "/joint_velocity_cmd_curr", 1);

  velocity_cmd_ = Eigen::VectorXd::Zero(params_.joint_command_names.size());

  accel_max_ = Eigen::VectorXd::Zero(params_.joint_command_names.size());
  jerk_max_ = Eigen::VectorXd::Zero(params_.joint_command_names.size());
  accel_current_ = Eigen::VectorXd::Zero(params_.joint_command_names.size());
  for(int i = 0;i < accel_max_.size(); ++i) {
    accel_max_[i] = franka::kMaxJointAcceleration[i];
    jerk_max_[i] = franka::kMaxJointJerk[i];
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointSpaceVelocityController::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  auto result = TsidVelocityControl::on_activate(previous_state);
  if (result != controller_interface::CallbackReturn::SUCCESS) {
    return result; // Propagate error if the base configuration fails
  }


  // Taking initial state from the joint state interfaces
  Eigen::VectorXd q0 = Eigen::VectorXd::Zero(model_.nq);
  Eigen::VectorXd v0 = Eigen::VectorXd::Zero(model_.nv);

  for (const auto & joint : params_.joint_state_names) {
    q0.tail(model_.nq - 7)[model_.getJointId(joint) - 2] =
      joint_state_interfaces_[jnt_id_[joint]][0].get().get_optional().value();
  }

  // formulation_->computeProblemData(0.0, q0, v0);

  // traj_joint_velocity_->setReference(v0.tail(model_.nv - 6));
  // task_joint_velocity_->setReference(traj_joint_velocity_->computeNext());


  return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::return_type
JointSpaceVelocityController::update(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  TsidVelocityControl::updateParams();

  std::pair<Eigen::VectorXd, Eigen::VectorXd> state = getActualState();
  state.first[6] = 1.0;  // Normalize the quaternion of the floating base
  
  /// BEGIN compute pb
  Eigen::VectorXd &q = state.first, &v = state.second;
  Eigen::VectorXd v_ = Eigen::VectorXd::Zero(model_.nv);
  v_.tail(model_.nq - 7) = (q.tail(model_.nq - 7) - q_prev_) / dt_.seconds();
  
  v_com_ = create_velocity_ramp(v_com_, velocity_cmd_, accel_current_, accel_max_, jerk_max_);

  // Integrating to check that q(t + dt) will still be in the limit if performing the command
  Eigen::VectorXd v_cmd = Eigen::VectorXd::Zero(model_.nv);
  v_cmd.tail(model_.nv - 6) = v_com_;
  q_int_ = pinocchio::integrate(
    model_, q, v_cmd * dt_.seconds());

  q_cmd_ = q_int_.tail(model_.nq - 7);

  joint_limit_reached_ = false;

  check_limit_reached(q, v);

  for (auto joint : joint_command_names_) {
    if (joint_limit_reached_) {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 5000,
        "Joint limit reached");

      if (!params_.use_sim_time) {
        if (!command_interfaces_[jnt_command_id_[joint]].set_value(
            0.0))
        {
          RCLCPP_ERROR(
            get_node()->get_logger(),
            "Failed to set command for joint %s", joint.c_str());
        }
      } else {
        if (!command_interfaces_[jnt_command_id_[joint]].set_value(
            q[model_.getJointId(joint) - 2]))
        {
          RCLCPP_ERROR(
            get_node()->get_logger(),
            "Failed to set command for joint %s", joint.c_str());
        }

      }
    } else {
      if (!params_.use_sim_time) {
        if (!command_interfaces_[jnt_command_id_[joint]].set_value(
            v_com_[model_.getJointId(joint) - 2]))
        {
          RCLCPP_ERROR(
            get_node()->get_logger(),
            "Failed to set command for joint %s", joint.c_str());
        }
      } else {
        if (!command_interfaces_[jnt_command_id_[joint]].set_value(
            q_cmd_[model_.getJointId(joint) - 2]))
        {
          RCLCPP_ERROR(
            get_node()->get_logger(),
            "Failed to set command for joint %s", joint.c_str());
        }

      }
    }
  }

  q_prev_ = q.tail(model_.nq - 7);
  std_msgs::msg::Float64MultiArray pub;

  for (Eigen::Index i = 0; i < v_.size(); i++) {
    pub.data.push_back(v_[i]);
  }
  publisher_curr_vel_->publish(pub);

  std_msgs::msg::Float64MultiArray pub_pos;
  for (Eigen::Index i = 0; i < q_cmd_.size(); i++) {
    pub_pos.data.push_back(q_cmd_[i]);
  }
  publisher_curr_pos_->publish(pub_pos);
  /// END compute pb


  //Publishing the current command
  auto message = sensor_msgs::msg::JointState();
  message.header.stamp = get_node()->get_clock()->now();

  message.name = joint_command_names_;
  message.position.resize(joint_command_names_.size());
  message.velocity.resize(joint_command_names_.size());
  message.effort.resize(joint_command_names_.size());

  for (size_t i = 0; i < joint_command_names_.size(); i++) {
    message.position[i] = 0.0;
    message.velocity[i] = velocity_cmd_[i];
    message.effort[i] = 0.0;
  }

  publisher_current_cmd_->publish(message);
  return controller_interface::return_type::OK;
}

void JointSpaceVelocityController::setVelocityCb(
  std_msgs::msg::Float64MultiArray::ConstSharedPtr msg)
{
  if (msg->data.size() != params_.joint_command_names.size()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Received joint velocity command with incorrect size");
    return;
  }

  // Setting the reference
  Eigen::VectorXd ref(params_.joint_command_names.size());

  for (size_t i = 0; i < params_.joint_command_names.size(); i++) {
    ref[i] = msg->data[i];
  }

  velocity_cmd_ = ref;
}

Eigen::VectorXd JointSpaceVelocityController::create_velocity_ramp(
    const Eigen::VectorXd &vel_current, 
    const Eigen::VectorXd & vel_cmd, 
    Eigen::VectorXd &accel_current, 
    const Eigen::VectorXd &accel_max, 
    const Eigen::VectorXd &jerk_max
  ) const
{
  assert(vel_current.size() == vel_cmd.size());
  assert(vel_current.size() == accel_current.size());
  assert(vel_current.size() == accel_max.size());
  assert(vel_current.size() == jerk_max.size());

  RCLCPP_INFO_STREAM(get_node()->get_logger(),"DEBUG vel_current (#" << vel_current.size() << ") = " << vel_current.transpose());
  RCLCPP_INFO_STREAM(get_node()->get_logger(),"DEBUG vel_cmd (#" << vel_cmd.size() << ") = " << vel_cmd.transpose());
  // RCLCPP_INFO_STREAM(get_node()->get_logger(),"DEBUG accel_current (#" << accel_current.size() << ") = " << accel_current.transpose());
  // RCLCPP_INFO_STREAM(get_node()->get_logger(),"DEBUG accel_max (#" << accel_max.size() << ") = " << accel_max.transpose());
  // RCLCPP_INFO_STREAM(get_node()->get_logger(),"DEBUG jerk_max (#" << jerk_max.size() << ") = " << jerk_max.transpose());

  const double dt = dt_.seconds();

  // 1. Acceleration that would be needed to reach vel_cmd in one step
  Eigen::VectorXd accel_desired = (vel_cmd - vel_current) / dt;

  // 2. Clamp desired acceleration to the acceleration limit
  accel_desired = accel_desired.cwiseMax(-0.95*accel_max).cwiseMin(0.95*accel_max);
  RCLCPP_INFO_STREAM(get_node()->get_logger(),"DEBUG accel_desired (#" << accel_desired.size() << ") = " << accel_desired.transpose());

  // 3. Clamp the CHANGE in acceleration (jerk) relative to current acceleration
  const Eigen::VectorXd max_daccel = 0.95 * jerk_max * dt;
  Eigen::VectorXd daccel = accel_desired - accel_current;
  daccel = daccel.cwiseMax(-max_daccel).cwiseMin(max_daccel);
  RCLCPP_INFO_STREAM(get_node()->get_logger(),"DEBUG daccel (#" << daccel.size() << ") = " << daccel.transpose());

  // 4. Update acceleration state
  accel_current = accel_current + daccel;
  // safety: also re-clamp resulting acceleration to accel_max
  accel_current = accel_current.cwiseMax(-0.95*accel_max).cwiseMin(0.95*accel_max);

  // 5. Integrate acceleration to get the new (jerk-limited) velocity
  Eigen::VectorXd vel_new = vel_current + accel_current * dt;
  RCLCPP_INFO_STREAM(get_node()->get_logger(),"DEBUG vel_new (#" << vel_new.size() << ") = " << vel_new.transpose());

  return vel_new;
}

} // namespace joint_controllers
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  joint_controllers::JointSpaceVelocityController,
  controller_interface::ControllerInterface)
