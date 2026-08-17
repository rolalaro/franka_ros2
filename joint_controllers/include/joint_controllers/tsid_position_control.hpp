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

#ifndef TSID_POSITION_CONTROL_HPP_
#define TSID_POSITION_CONTROL_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/component_parser.hpp"
// #include "pal_utils/better_enums.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/actuator_interface.hpp>
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <rclcpp/rclcpp.hpp>
#include "joint_controllers/joint_controllers_params.hpp"
#include <joint_controllers/interfaces.hpp>
#include <unordered_map>

namespace joint_controllers
{

struct BoundingBox
{
  double x_min, x_max;
  double y_min, y_max;
  double z_min, z_max;
};
class TsidPositionControl : public controller_interface::ControllerInterface
{
public:
  TsidPositionControl();

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  controller_interface::return_type
  update(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    return controller_interface::return_type::OK;
  }

  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  void DefaultPositionTasks();
  virtual void updateParams();
  std::pair<Eigen::VectorXd, Eigen::VectorXd> getActualState() const;

  void compute_problem_and_set_command(Eigen::VectorXd q, Eigen::VectorXd v);

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
  Eigen::VectorXd position_end_;
  bool first_tsid_iter_;
  std::unordered_map<std::string, BoundingBox> bounding_boxes_;

private:

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_curr_vel;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_curr_pos;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_curr_current;

  Eigen::VectorXd q_prev_;
  Eigen::VectorXd q_int_;
  Eigen::VectorXd v_int_;


};
} // namespace joint_controllers

#endif // TSID_POSITION_CONTROL_HPP_
