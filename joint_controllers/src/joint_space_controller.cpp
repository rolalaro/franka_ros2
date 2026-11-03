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

SCurveProfile makeSCurve(double distance,
                         double v_max,
                         double a_max,
                         double j_max)
{
  SCurveProfile profile;

  if (distance <= 0.0) {
    return profile;
  }

  if (v_max <= 0.0 || a_max <= 0.0 || j_max <= 0.0) {
    throw std::invalid_argument(
        "S-curve limits must be strictly positive.");
  }

  // ------------------------------------------------------------
  // Compute the acceleration profile required to reach a given
  // peak velocity.
  // ------------------------------------------------------------
  auto compute_for_peak_velocity =
      [&](double v_peak)
      {
        double tj;
        double ta;

        const double v_at_a_max =
            a_max * a_max / j_max;

        if (v_peak <= v_at_a_max) {
          // Jerk-limited triangular acceleration.
          tj = std::sqrt(v_peak / j_max);
          ta = 0.0;
        } else {
          // Reach a_max, then hold it.
          tj = a_max / j_max;
          ta = v_peak / a_max - tj;
        }

        // Distance travelled during the acceleration phase.
        //
        // x_acc =
        // J * tj * (ta^2 + 3*ta*tj + 2*tj^2) / 2
        //
        const double x_acc =
            j_max * tj *
            (ta * ta +
             3.0 * ta * tj +
             2.0 * tj * tj) / 2.0;

        return std::pair<double, std::pair<double, double>>{
            2.0 * x_acc,
            {tj, ta}
        };
      };

  // ------------------------------------------------------------
  // First check whether v_max can actually be reached.
  // ------------------------------------------------------------
  const auto [distance_at_vmax, times_at_vmax] =
      compute_for_peak_velocity(v_max);

  double v_peak;

  if (distance >= distance_at_vmax) {
    // We can reach v_max and have a constant velocity segment.
    v_peak = v_max;

    profile.tv =
        (distance - distance_at_vmax) / v_peak;
  } else {
    // We cannot reach v_max.
    //
    // Find the peak velocity such that the acceleration and
    // deceleration phases alone exactly consume the distance.
    double lo = 0.0;
    double hi = v_max;

    for (int iteration = 0; iteration < 100; ++iteration) {
      const double mid = 0.5 * (lo + hi);

      const auto [d_mid, unused] =
          compute_for_peak_velocity(mid);

      if (d_mid < distance) {
        lo = mid;
      } else {
        hi = mid;
      }
    }

    v_peak = 0.5 * (lo + hi);
    profile.tv = 0.0;
  }

  const auto [unused_distance, times] =
      compute_for_peak_velocity(v_peak);

  profile.v = v_peak;
  profile.tj = times.first;
  profile.ta = times.second;

  if (profile.tj > 0.0) {
    profile.j = profile.v <= a_max * a_max / j_max
                    ? profile.v /
                          (profile.tj * profile.tj)
                    : j_max;
  }

  profile.a = profile.j * profile.tj;

  return profile;
}


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
  vel_max_ =
    v_scaling_ * model_.upperVelocityLimit.tail(model_.nv - 6);

  for(const auto & joint : joint_names_) {
    int joint_id = jnt_id_[joint];
    if(joint.find(std::to_string(joint_id + 1)) == std::string::npos) throw; // Ensure mapping of the order of the joints names and Franka library limits
    accel_max_[joint_id] = franka::kMaxJointAcceleration[joint_id];
    jerk_max_[joint_id] = franka::kMaxJointJerk[joint_id];
  }

  // Prepare message containing the command sent by the user
  msg_cmd_.name = joint_command_names_;
  msg_cmd_.position.resize(joint_command_names_.size());
  msg_cmd_.velocity.resize(joint_command_names_.size());
  msg_cmd_.effort.resize(joint_command_names_.size());

  for (size_t i = 0; i < joint_command_names_.size(); i++) {
    msg_cmd_.velocity[i] = 0.0;
    msg_cmd_.effort[i] = 0.0;
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
  q_cmd_ = q_start_;
  trajectory_active_ = false;
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
  q[6] = 1.0; // To have a valid quaternion for the free-flying base
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

controller_interface::return_type JointSpacePositionController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  t_curr_ = t_curr_ + dt_.seconds();

  updateParams();

  interpolate(t_curr_); // compute the velocity and the position
  
  // Setting the command to the joint command interfaces
  for (int i = 0; i < 7; ++i) {
    if (!command_interfaces_[i].set_value(
        q_cmd_[i])) 
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Failed to set command for joint %d",
        i);
    }
  }

  std_msgs::msg::Float64MultiArray pub_pos;
  for (Eigen::Index i = 0; i < q_cmd_.size(); i++) {
    pub_pos.data.push_back(q_cmd_[i]);
  }

  publisher_curr_pos->publish(pub_pos);

  //Publishing the current command
  msg_cmd_.header.stamp = get_node()->get_clock()->now();

  for (size_t i = 0; i < joint_command_names_.size(); i++) {
    msg_cmd_.position[i] = q_end_[i];
  }

  publisher_current_cmd->publish(msg_cmd_);

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

  q_start_ = q_cmd_;
  q_end_ = ref;

  // ------------------------------------------------------------
  // Check whether this is effectively a zero-length trajectory.
  // ------------------------------------------------------------
  constexpr double position_tolerance = 1e-7;

  if ((q_end_ - q_start_).norm() < position_tolerance) {
    trajectory_active_ = false;
    q_cmd_ = q_end_;
    return;
  }

  // ------------------------------------------------------------
  // Compute normalized scalar limits.
  //
  // q_i = q_start_i + dq_i * s
  //
  // dq_i = dq_i * s_dot
  // ddq_i = dq_i * s_ddot
  // dddq_i = dq_i * s_dddot
  // ------------------------------------------------------------

  double s_vel_max = std::numeric_limits<double>::infinity();
  double s_acc_max = std::numeric_limits<double>::infinity();
  double s_jerk_max = std::numeric_limits<double>::infinity();

  for (int i = 0; i < params_.joint_command_names.size(); ++i) {
    const double dq =
        std::abs(q_end_[i] - q_start_[i]);

    if (dq < position_tolerance) {
      continue;
    }

    s_vel_max =
        std::min(s_vel_max, vel_max_[i] / dq);

    s_acc_max =
        std::min(s_acc_max, accel_max_[i] / dq);

    s_jerk_max =
        std::min(s_jerk_max, jerk_max_[i] / dq);
  }

  // ------------------------------------------------------------
  // Generate a normalized trajectory s(t): 0 -> 1.
  // ------------------------------------------------------------
  const double &conservative_factor = params_.velocity_scaling;
  s_vel_max *= conservative_factor;
  s_acc_max *= conservative_factor;
  s_jerk_max *= conservative_factor;
  s_curve_ =
      makeSCurve(
          1.0,
          s_vel_max,
          s_acc_max,
          s_jerk_max);

  t_curr_ = 0.;
  first_iter_ = true;
  trajectory_active_ = true;
}

void JointSpacePositionController::interpolate(double t_curr)
{
  if (!trajectory_active_) {
    q_cmd_ = q_end_;
    return;
  }

  const double s =
      s_curve_.position(t_curr);

  for (Eigen::Index i = 0; i < q_start_.size(); ++i) {
    q_cmd_[i] =
        q_start_[i] +
        s * (q_end_[i] - q_start_[i]);
  }

  // ------------------------------------------------------------
  // End condition.
  // ------------------------------------------------------------
  if (t_curr >= s_curve_.total_time()) {
    q_cmd_ = q_end_;
    trajectory_active_ = false;
  }
}

} // namespace joint_controllers
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  joint_controllers::JointSpacePositionController,
  controller_interface::ControllerInterface)
