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

#include <stepit_hardware/request_interface.hpp>
#include <stepit_hardware/request_interface_factory.hpp>
#include <stepit_hardware/visibility_control.hpp>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace stepit_hardware
{
// Not a number variable to initialize class member variables
constexpr static auto kDoubleNaN = std::numeric_limits<double>::quiet_NaN();
constexpr static auto kUint8NaN = std::numeric_limits<uint8_t>::quiet_NaN();
/**
 * https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/writing_new_hardware_interface.html
 */
class StepitHardware : public hardware_interface::SystemInterface
{
public:
  /**
   * Default constructor.
   */
  StepitHardware();

  /**
   * Constructor with given command interface. This method is used for testing.
   * @param command_interface The interface to send commands and queries to
   * the hardware.
   */
  explicit StepitHardware(std::unique_ptr<RequestInterfaceFactory> command_interface_factory);

  /**
   * Defines aliases and static functions for using the Class with shared_ptrs.
   * With such definitions, the control manager can instantiate the class with
   * auto hardware_interface = StepitHardware::make_shared();
   */
  RCLCPP_SHARED_PTR_DEFINITIONS(StepitHardware)

  /**
   * Initialization of the hardware interface from data parsed from the
   * robot's URDF.
   * @param hardware_info Structure with data from URDF.
   * @returns CallbackReturn::SUCCESS if required data are provided and can be
   * parsed or CallbackReturn::ERROR if any error happens or data are missing.
   */
  STEPIT_HARDWARE_PUBLIC CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  /**
   * Connect to the hardware.
   * @param previous_state The previous state.
   * @returns CallbackReturn::SUCCESS if required data are provided and can be
   * parsed or CallbackReturn::ERROR if any error happens or data are missing.
   */
  STEPIT_HARDWARE_PUBLIC CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  /**
   * This method exposes position and velocity of joints for reading.
   */
  STEPIT_HARDWARE_PUBLIC std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  /**
   * This method exposes the joints targets for writing.
   */
  STEPIT_HARDWARE_PUBLIC std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  /**
   * This method is invoked when the hardware is connected.
   * @param previous_state Unconfigured, Inactive, Active or Finalized.
   * @returns CallbackReturn::SUCCESS or CallbackReturn::ERROR.
   */
  STEPIT_HARDWARE_PUBLIC CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  /**
   * This method is invoked when the hardware is disconnected.
   * @param previous_state Unconfigured, Inactive, Active or Finalized.
   * @returns CallbackReturn::SUCCESS or CallbackReturn::ERROR.
   */
  STEPIT_HARDWARE_PUBLIC CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  /**
   * Read data from the hardware.
   */
  STEPIT_HARDWARE_PUBLIC hardware_interface::return_type read(const rclcpp::Time& time,
                                                              const rclcpp::Duration& period) override;

  /**
   * Write data from to hardware.
   */
  STEPIT_HARDWARE_PUBLIC hardware_interface::return_type write(const rclcpp::Time& time,
                                                               const rclcpp::Duration& period) override;

private:
  // Internal structure to store joint states or targets.
  struct JointValue
  {
    double position = kDoubleNaN;
    double velocity = kDoubleNaN;
  };

  // Internal structure to store joint states and targets.
  struct Joint
  {
    uint8_t id = kUint8NaN;
    double acceleration = kDoubleNaN;
    double max_velocity = kDoubleNaN;
    JointValue state{};
    JointValue command{};
  };

  // Store information about current joint states and targets.
  std::vector<Joint> joints_;

  // Interface to send binary data to the hardware using the serial port.
  std::unique_ptr<RequestInterface> command_interface_;

  // Factory to create the command handler during the initialization step.
  std::unique_ptr<RequestInterfaceFactory> command_interface_factory_;
};
}  // namespace stepit_hardware
