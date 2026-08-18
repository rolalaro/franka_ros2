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
struct SCurveProfile {
  // Duration of the jerk-up, constant-acceleration, and jerk-down
  // portions of the acceleration phase.
  double tj{0.0};
  double ta{0.0};

  // Constant-velocity duration.
  double tv{0.0};

  // Peak values actually used by the profile.
  double j{0.0};
  double a{0.0};
  double v{0.0};

  double total_time() const {
    return 4.0 * tj + 2.0 * ta + tv;
  }

  double position(double t) const {
    if (t <= 0.0) {
      return 0.0;
    }

    const double T = total_time();

    if (t >= T) {
      return 1.0;
    }

    // ------------------------------------------------------------
    // Segment 1: +jerk
    // ------------------------------------------------------------
    if (t < tj) {
      return j * t * t * t / 6.0;
    }

    // State at end of segment 1
    const double x1 = j * tj * tj * tj / 6.0;
    const double v1 = 0.5 * j * tj * tj;
    const double a1 = j * tj;

    // ------------------------------------------------------------
    // Segment 2: constant acceleration
    // ------------------------------------------------------------
    if (t < tj + ta) {
      const double tau = t - tj;

      return x1 +
             v1 * tau +
             0.5 * a1 * tau * tau;
    }

    // State at end of segment 2
    const double x2 =
        x1 +
        v1 * ta +
        0.5 * a1 * ta * ta;

    const double v2 =
        v1 + a1 * ta;

    // ------------------------------------------------------------
    // Segment 3: -jerk
    // ------------------------------------------------------------
    if (t < 2.0 * tj + ta) {
      const double tau = t - (tj + ta);

      return x2 +
             v2 * tau +
             0.5 * a1 * tau * tau -
             j * tau * tau * tau / 6.0;
    }

    // State at end of acceleration phase
    const double x3 =
        x2 +
        v2 * tj +
        0.5 * a1 * tj * tj -
        j * tj * tj * tj / 6.0;

    // This should be the midpoint of the trajectory.
    const double t_acc = 2.0 * tj + ta;

    // ------------------------------------------------------------
    // Segment 4: constant velocity
    // ------------------------------------------------------------
    if (t < t_acc + tv) {
      const double tau = t - t_acc;
      return x3 + v * tau;
    }

    // ------------------------------------------------------------
    // Use symmetry for the second half.
    // ------------------------------------------------------------
    const double tau = T - t;

    // Distance travelled during the last tau seconds is identical
    // to the distance travelled during the first tau seconds.
    return 1.0 - position(tau);
  }
};

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

  void interpolate(double t);

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

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_curr_pos;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
    joint_cmd_sub_;
  double t_curr_ = 0.0;

  Eigen::VectorXd q_start_; // configuration from which the servoing started
  Eigen::VectorXd q_end_; // reference set by the user
  Eigen::VectorXd q_cmd_; // command sent to the hardware interface
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_current_cmd;
  sensor_msgs::msg::JointState msg_cmd_;

  Eigen::VectorXd accel_max_;
  Eigen::VectorXd jerk_max_;
  Eigen::VectorXd vel_max_;
  bool trajectory_active_ = false;
  SCurveProfile s_curve_;
};
} // namespace joint_controllers

#endif // JOINT_SPACE_TISD_CONTROLLER_HPP_
