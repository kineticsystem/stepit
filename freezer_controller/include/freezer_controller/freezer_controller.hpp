/*
 * Copyright (c) 2022, Giovanni Remigi
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <freezer_controller/visibility_control.hpp>
#include <freezer_msgs/srv/set_io.hpp>

#include <controller_interface/controller_interface.hpp>

namespace freezer_controller
{
enum CommandInterfaces
{
  DIGITAL_OUTPUTS_CMD = 0u,
  ANALOG_OUTPUTS_CMD = 18,
  IO_ASYNC_SUCCESS = 20,
  TARGET_SPEED_FRACTION_CMD = 21,
  TARGET_SPEED_FRACTION_ASYNC_SUCCESS = 22,
  RESEND_ROBOT_PROGRAM_CMD = 23,
  RESEND_ROBOT_PROGRAM_ASYNC_SUCCESS = 24,
  PAYLOAD_MASS = 25,
  PAYLOAD_COG_X = 26,
  PAYLOAD_COG_Y = 27,
  PAYLOAD_COG_Z = 28,
  PAYLOAD_ASYNC_SUCCESS = 29,
};

enum StateInterfaces
{
  DIGITAL_OUTPUTS = 0u,
  DIGITAL_INPUTS = 18,
  ANALOG_OUTPUTS = 36,
  ANALOG_INPUTS = 38,
  ANALOG_IO_TYPES = 40,
  TOOL_MODE = 44,
  TOOL_OUTPUT_VOLTAGE = 45,
  TOOL_OUTPUT_CURRENT = 46,
  TOOL_TEMPERATURE = 47,
  TOOL_ANALOG_INPUTS = 48,
  TOOL_ANALOG_IO_TYPES = 50,
  ROBOT_MODE = 52,
  ROBOT_STATUS_BITS = 53,
  SAFETY_MODE = 57,
  SAFETY_STATUS_BITS = 58,
  INITIALIZED_FLAG = 69,
  PROGRAM_RUNNING = 70,
};

/**
 * This is a plugin which is plugged into the URDF together with a configuration file.
 * It is automatically wrapped by a ROS node and exposes services and publishers to
 * write and read digital IO to and from a Freezer motherboard.
 */
class FreezerController : public controller_interface::ControllerInterface
{
public:
  /** Default constructor. */
  FreezerController();

  /**
   * Defines aliases and static functions for using the Class with shared_ptrs.
   * With such definitions, the control manager can instantiate the class with
   * auto controller_interface = FreezerController::make_shared();
   */
  RCLCPP_SHARED_PTR_DEFINITIONS(FreezerController)

  FREEZER_CONTROLLER_PUBLIC CallbackReturn on_init() override;

  FREEZER_CONTROLLER_PUBLIC CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  /**
   * @brief This method exposes all command interfaces.
   * @return Return the command interface configuration.
   */
  FREEZER_CONTROLLER_PUBLIC controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  FREEZER_CONTROLLER_PUBLIC controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  FREEZER_CONTROLLER_PUBLIC CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  FREEZER_CONTROLLER_PUBLIC CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  FREEZER_CONTROLLER_PUBLIC controller_interface::return_type update(const rclcpp::Time& time,
                                                                     const rclcpp::Duration& period) override;

private:
  // Service to set digital outputs.
  rclcpp::Service<freezer_msgs::srv::SetIO>::SharedPtr set_io_srv_;

  //  std::shared_ptr<rclcpp::Publisher<ur_msgs::msg::IOStates>> io_pub_;
  //  ur_msgs::msg::IOStates io_msg_;
  bool setIO(freezer_msgs::srv::SetIO::Request::SharedPtr req, freezer_msgs::srv::SetIO::Response::SharedPtr resp);
};
}  // namespace freezer_controller
