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
// This code is adapted from Example 7 of the ROS2 Control Demos
//

#include "include/khi_hw_interface.hpp"
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "khi_krnx_driver.h"

namespace khi2cpp_hw
{
    // --------------------------------------------------------------------------------------------
    // Method called for initializing the hardware interface
    // returns a CallbackReturn (rclcpp_lifecycle) object indicating ERROR or SUCCESS
    CallbackReturn KhiSystem::on_init(const hardware_interface::HardwareInfo & info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
        {
            // if on_init(info) fails, error out
            return CallbackReturn::ERROR;
        }
        
        data_.robot_name = info_.name;
        data_.arm_num = 0;
        data_.arm[0].jt_num = 0;

        bool in_sim = true;

        // assign robot-specific data -> can this data not be grabbed from the URDF? What's our info argument?
        // robot has 6 joints and 2 interfaces
        joint_position_.assign(6, 0);
        joint_velocities_.assign(6, 0);
        joint_position_command_.assign(6, 0);
        joint_velocities_command_.assign(6, 0);

        // force sensor has 6 readings
        ft_states_.assign(6, 0);
        ft_command_.assign(6, 0);

        int jt = 0;
        for (const auto & joint : info_.joints)
        {
            // Assign HardwareInfo members to the KhiRobotData object
            data_.arm[0].name[jt] = joint.name;
            data_.arm[0].type[jt] = 0;

            // Retrieve KhiRobotArm data position and velocity; assign as members in the KhiSystem
            joint_position_[jt] = data_.arm[0].pos[jt];
            joint_velocities_[jt] = data_.arm[0].vel[jt];

            // loop through the joint (ComponentInfo) objects in the info_ member of KhiSystem (Hardware Interface) object
            for (const auto & interface : joint.state_interfaces)
            {
                // loop through the interface members of each joint member and
                // append the joint name at the end of the joint_interfaces vector for each interface
                joint_interfaces[interface.name].push_back(joint.name);
            }
            jt++;
        }

        // Allocate memory for the Krnx driver in memory; driver_ is just a pointer to the actual driver
        driver_ = new khi_robot_control::KhiRobotKrnxDriver();

        // Call the intialize member function of driver_
        if( ! driver_->initialize(cont_no_, 4, data_, in_sim )) { 
            KhiSystem::close(cont_no_);
            return CallbackReturn::ERROR;}

        // Call the "open" member function of driver_
        if ( ! driver_->open(cont_no_, "192.168.0.2", data_ )) {
            KhiSystem::close(cont_no_);
            return CallbackReturn::ERROR;}

        // Call the driver activation member function
        if ( ! driver_->activate(cont_no_, data_)) {
            KhiSystem::close(cont_no_);
            return CallbackReturn::ERROR;}

        // returns success if ... it was a success
        RCLCPP_INFO(rclcpp::get_logger("KhiSystemInterface"), "+++++++++++++ KHI-ROS Hardware Initialization SUCCESS ++++++++++++++");
        return CallbackReturn::SUCCESS;
    }
    // --------------------------------------------------------------------------------------------

    // --------------------------------------------------------------------------------------------
    // Method for reading the hardware state via a vector of StateInterface object(s); exists within the System Interface (KhiSystem)
    // returns a std::vector of State Interface objects
    std::vector<hardware_interface::StateInterface> KhiSystem::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        int ind = 0;
        for (const auto & joint_name : joint_interfaces["position"])
        {
            // loop through the joints and append the joint position to the state_interfaces vector
            state_interfaces.emplace_back(joint_name, "position", &joint_position_[ind++]);
        }

        ind = 0;
        for (const auto & joint_name : joint_interfaces["velocity"])
        {
            // loop through the joints and append the joint velocities to the state_interfaces vector
            state_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_[ind++]);
        }
        return state_interfaces;
        }
    // --------------------------------------------------------------------------------------------

    // --------------------------------------------------------------------------------------------
    // Method for reading the hardware state via a vector of CommandInterface object(s); exists within the System Interface (KhiSystem)
    // returns a std::vector of CommandInterface objects
    std::vector<hardware_interface::CommandInterface> KhiSystem::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        int ind = 0;
        for (const auto & joint_name : joint_interfaces["position"])
        {
            // loop through the joints and append the joint position to the command_interfaces vector
            command_interfaces.emplace_back(joint_name, "position", &joint_position_command_[ind++]);
        }

        ind = 0;
        for (const auto & joint_name : joint_interfaces["velocity"])
        {
            // loop through the joints and append the joint velocity to the command_interfaces vector
            command_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_command_[ind++]);
        }

            // append any sensor data to the command_interfaces vector
        //command_interfaces.emplace_back("tcp_fts_sensor", "force.x", &ft_command_[0]);
        
        return command_interfaces;
    }
    // --------------------------------------------------------------------------------------------

    // --------------------------------------------------------------------------------------------
    // Method for asking to read the hardware for state interfaces, e.g. joint encoders and sensor readings
    // returns a return_type of OK
    return_type KhiSystem::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
    {
        // TODO(pac48) set sensor_states_ values from subscriber

        std::vector<hardware_interface::StateInterface> state_interfaces;

        driver_->readData(cont_no_, data_);

        // this needs edited to get joint_position_ and joint_velocities_ from the updated data_ member after readData
        for (auto i = 0ul; i < joint_velocities_command_.size(); i++)
        {
            // loop through the joint_velocities_command_ member vector and calculate the position (double)
            // and assign that position into the joint_position_ member vector of (doubles)
            joint_velocities_[i] = joint_velocities_command_[i];
            joint_position_[i] += joint_velocities_command_[i] * period.seconds();
        }

        for (auto i = 0ul; i < joint_position_command_.size(); i++)
        {
            // loop through the joint_positions_command_ member vector
            // and assign any joint_position_command_ (double) into the joint_position_ vector of (doubles)
            joint_position_[i] = joint_position_command_[i];
        }

        RCLCPP_DEBUG(rclcpp::get_logger("KhiSystemInterface"), "Reading joint positions");

        return return_type::OK;
    }
    // --------------------------------------------------------------------------------------------

    // --------------------------------------------------------------------------------------------
    // Method for accessing the data values pointed to in the CommandInterfaces object
    // returns a return_type of OK
    return_type KhiSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
    {
        std::vector<hardware_interface::CommandInterface> cmd = export_command_interfaces();

        // assign values from cmd into data_ member; data.arm[arm_num].cmd[joint_num]
        data_.arm[0].pos[0] = cmd[0].get_value();

        driver_->writeData(cont_no_, data_);
        RCLCPP_INFO(rclcpp::get_logger("KhiSystemInterface"), "Writing joint positions: j1=%f", data_.arm[0].pos[0]);

        //client->write(data_);
        return return_type::OK;
    }
    // --------------------------------------------------------------------------------------------
 
    // --------------------------------------------------------------------------------------------
    // Method for closing the KRNX driver via hardware interface
    // returns nothing
    void KhiSystem::close(const int & cont_no_)
    {
        driver_->deactivate(cont_no_, data_);
        driver_->close(cont_no_);
        rclcpp::shutdown();
    }
    // --------------------------------------------------------------------------------------------
 
    // --------------------------------------------------------------------------------------------
    // Method for holding the KRNX driver via hardware interface
    // returns nothing
    void KhiSystem::hold(const int & cont_no_)
    {
        driver_->hold(cont_no_, data_);
    }
    // --------------------------------------------------------------------------------------------
 
    // --------------------------------------------------------------------------------------------
    // Method for updating the KRNX driver state via hardware interface
    // returns nothing
    void KhiSystem::update()
    {
        driver_->updateState(cont_no_, data_);
    }
    // --------------------------------------------------------------------------------------------
 
    // --------------------------------------------------------------------------------------------
    // Method for getting the period difference between HW and real robot
    // returns nothing
    void KhiSystem::getPeriodDiff(double& diff)
    {
        driver_->getPeriodDiff(cont_no_, diff);
    }
    // --------------------------------------------------------------------------------------------
 
    // --------------------------------------------------------------------------------------------
    // Method for updating the KRNX driver state name via hardware interface
    // returns nothing
    std::string KhiSystem::getStateName()
    {
        std::string state = driver_->getStateName(cont_no_);
        return state;
    }

}  // end of namespace khi2cpp_hw
// --------------------------------------------------------------------------------------------


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  khi2cpp_hw::KhiSystem, hardware_interface::SystemInterface)