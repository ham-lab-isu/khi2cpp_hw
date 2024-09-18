// Copyright 2023 ros2_control Development Team
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

// Jakob D. Hamilton, IMSE, Iowa State University 2024
// This source file defines the methods for ROS2 Control objects to interact with the hardware methods and members defined in the KRNX driver
// The KhiSystem (hardware_interface::SystemInterface) object, when utilized in main.cpp, allows ROS2 controllers (JointStateController, JointTrajectoryController, etc)
// to interface with the hardware driver.
//
// This code is adapted from Example 7 of the ROS2 Control Demo
//

#include "include/khi_cnt_interface.hpp"

#include <stddef.h>
#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

using config_type = controller_interface::interface_configuration_type;

namespace khi2cpp_hw
{
    KhiController::KhiController() : controller_interface::ControllerInterface() {}

    // --------------------------------------------------------------------------------------------
    // This method initializes the KhiController. It declares and gets parameters needed for controller initilization
    // and allocates memory that will exist throughout the life of the controller
    controller_interface::CallbackReturn KhiController::on_init()
    {
        // should have error handling ...
        joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
        command_interface_types_ =
            auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
        state_interface_types_ =
            auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);

        // assign 0's in the vector of pos/vels within the point_interp_ trajectory_msg object (member of KhiController)
        // use the joint_names_ member to assign the vector size
        point_interp_.positions.assign(joint_names_.size(), 0);
        point_interp_.velocities.assign(joint_names_.size(), 0);

        return CallbackReturn::SUCCESS;
    }
    // --------------------------------------------------------------------------------------------

    // --------------------------------------------------------------------------------------------
    // This method creates an InterfaceConfiguration object with members describing the command interfaces
    // returns a list of InterfaceConfiguration objects to indicate which command interfaces the cnt needs for operation
    // if the interface name and type are not found in the HardwareInterface (SystemInterface) object, it will fail
    controller_interface::InterfaceConfiguration KhiController::command_interface_configuration()
    const
    {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

        conf.names.reserve(joint_names_.size() * command_interface_types_.size());
        for (const auto & joint_name : joint_names_)
        {
            for (const auto & interface_type : command_interface_types_)
            {
            conf.names.push_back(joint_name + "/" + interface_type);
            }
        }

        return conf;
    }
    // --------------------------------------------------------------------------------------------

    // --------------------------------------------------------------------------------------------
    // This method creates an InterfaceConfiguration object with members describing the state interfaces
    // returns a list of InterfaceConfiguration objects to indicate which state interfaces the cnt needs for operation
    // if the interface name and type are not found in the HardwareInterface (SystemInterface) object, it will fail
    controller_interface::InterfaceConfiguration KhiController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

        conf.names.reserve(joint_names_.size() * state_interface_types_.size());
        for (const auto & joint_name : joint_names_)
        {
            for (const auto & interface_type : state_interface_types_)
            {
                conf.names.push_back(joint_name + "/" + interface_type);
            }
        }

        return conf;
    }
    // --------------------------------------------------------------------------------------------

    // --------------------------------------------------------------------------------------------
    // This method declares and gets parameters needed for controller operations
    // it sets up realtime buffers, ROS publishers and subscribers
    controller_interface::CallbackReturn KhiController::on_configure(const rclcpp_lifecycle::State &)
    {
        auto callback =
            [this](const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> traj_msg) -> void
        {
            traj_msg_external_point_ptr_.writeFromNonRT(traj_msg);
            new_msg_ = true;
        };

        joint_command_subscriber_ =
            get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "~/joint_trajectory", rclcpp::SystemDefaultsQoS(), callback);

        return CallbackReturn::SUCCESS;
    }
    // --------------------------------------------------------------------------------------------

    // --------------------------------------------------------------------------------------------
    // This method handles controller restarts and safety checks when a controller is activated.
    controller_interface::CallbackReturn KhiController::on_activate(const rclcpp_lifecycle::State &)
    {
        // clear out vectors in case of restart
        joint_position_command_interface_.clear();
        joint_velocity_command_interface_.clear();
        joint_position_state_interface_.clear();
        joint_velocity_state_interface_.clear();

        // assign command interfaces
        for (auto & interface : command_interfaces_)
        {
            command_interface_map_[interface.get_interface_name()]->push_back(interface);
        }

        // assign state interfaces
        for (auto & interface : state_interfaces_)
        {
            state_interface_map_[interface.get_interface_name()]->push_back(interface);
        }

        return CallbackReturn::SUCCESS;
    }
    // --------------------------------------------------------------------------------------------

    // --------------------------------------------------------------------------------------------
    void interpolate_point(
        const trajectory_msgs::msg::JointTrajectoryPoint & point_1,
        const trajectory_msgs::msg::JointTrajectoryPoint & point_2,
        trajectory_msgs::msg::JointTrajectoryPoint & point_interp, double delta)
    {
        for (size_t i = 0; i < point_1.positions.size(); i++)
        {
            point_interp.positions[i] = delta * point_2.positions[i] + (1.0 - delta) * point_2.positions[i];
        }
        for (size_t i = 0; i < point_1.positions.size(); i++)
        {
            point_interp.velocities[i] =
            delta * point_2.velocities[i] + (1.0 - delta) * point_2.velocities[i];
        }
    }
    // --------------------------------------------------------------------------------------------

    // --------------------------------------------------------------------------------------------
    // This method takes a trajectory message and current time and does some math to update the KhiController member values
    void interpolate_trajectory_point(
        const trajectory_msgs::msg::JointTrajectory & traj_msg, const rclcpp::Duration & cur_time,
        trajectory_msgs::msg::JointTrajectoryPoint & point_interp)
    {
        double traj_len = traj_msg.points.size();
        auto last_time = traj_msg.points[traj_len - 1].time_from_start;
        double total_time = last_time.sec + last_time.nanosec * 1E-9;

        size_t ind = cur_time.seconds() * (traj_len / total_time);
        ind = std::min(static_cast<double>(ind), traj_len - 2);
        double delta = cur_time.seconds() - ind * (total_time / traj_len);
        interpolate_point(traj_msg.points[ind], traj_msg.points[ind + 1], point_interp, delta);
    }
    // --------------------------------------------------------------------------------------------

    // --------------------------------------------------------------------------------------------
    // THIS IS THE MEAT OF THE CONTROL, and it must obey realtime control rules
    // This reads the controller inputs from the state interfaces, calculates output values,
    // and writes them to the command interfaces.
    controller_interface::return_type KhiController::update(
        const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
    {
        if (new_msg_)
        {
            trajectory_msg_ = *traj_msg_external_point_ptr_.readFromRT();
            start_time_ = time;
            new_msg_ = false;
        }

        if (trajectory_msg_ != nullptr)
        {
            interpolate_trajectory_point(*trajectory_msg_, time - start_time_, point_interp_);
            for (size_t i = 0; i < joint_position_command_interface_.size(); i++)
            {
                // for each pos command interface, set the value per the point_interp_ member
                joint_position_command_interface_[i].get().set_value(point_interp_.positions[i]);
            }
            for (size_t i = 0; i < joint_velocity_command_interface_.size(); i++)
            {
                // for each vel command interface, set the value per the point_interp_ member
                joint_velocity_command_interface_[i].get().set_value(point_interp_.velocities[i]);
            }
        }
        return controller_interface::return_type::OK;
    }
    // --------------------------------------------------------------------------------------------

    // --------------------------------------------------------------------------------------------
    // This method is used when a controller stops running.
    controller_interface::CallbackReturn KhiController::on_deactivate(const rclcpp_lifecycle::State &)
    {
        // release the controller interfaces so other controllers can access them
        release_interfaces();

        return CallbackReturn::SUCCESS;
    }
    // --------------------------------------------------------------------------------------------

    // --------------------------------------------------------------------------------------------
    // This method is used when a controller's lifecycle node is transitioning to shutting down
    controller_interface::CallbackReturn KhiController::on_cleanup(const rclcpp_lifecycle::State &)
    {
        return CallbackReturn::SUCCESS;
    }
    // --------------------------------------------------------------------------------------------

    // --------------------------------------------------------------------------------------------
    // This method is called if the managed node fails any of the state transitions.
    controller_interface::CallbackReturn KhiController::on_error(const rclcpp_lifecycle::State &)
    {
        return CallbackReturn::SUCCESS;
    }
    // --------------------------------------------------------------------------------------------

    // --------------------------------------------------------------------------------------------
    // This method is used when a controller's lifecycle node is transitioning to shutting down
    controller_interface::CallbackReturn KhiController::on_shutdown(const rclcpp_lifecycle::State &)
    {
        return CallbackReturn::SUCCESS;
    }
    // --------------------------------------------------------------------------------------------


}  // namespace khi2cpp_hw

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  khi2cpp_hw::KhiController, controller_interface::ControllerInterface)