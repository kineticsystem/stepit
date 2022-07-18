/*
 * Copyright (C) 2022 Remigi Giovanni
 * g.remigi@kineticsystem.org
 * www.kineticsystem.org
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option) any
 * later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more
 * details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, write to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#pragma once

#include <stepit_hardware/visibility_control.hpp>
#include <stepit_hardware/data_interface.hpp>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp/macros.hpp>

using hardware_interface::return_type;

namespace stepit_hardware
{
class StepitHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(StepitHardware)

  STEPIT_HARDWARE_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  /**
   * This method exposes position and velocity of joints for reading.
   */
  STEPIT_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  /**
   * This method exposes the joints targets for writing.
   */
  STEPIT_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  STEPIT_HARDWARE_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  STEPIT_HARDWARE_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  STEPIT_HARDWARE_PUBLIC
  return_type read() override;

  STEPIT_HARDWARE_PUBLIC
  return_type write() override;

private:
  // Internal structure to store joint states or targets.
  struct JointValue
  {
    double position{ 0.0 };
    double velocity{ 0.0 };
  };

  // Internal structure to store joint states and targets.
  struct Joint
  {
    JointValue state{};
    JointValue command{};
  };

  // Store joint ids.
  std::vector<uint8_t> joint_ids_;

  // Store information about current joint states and targets.
  std::vector<Joint> joints_;

  // Interface to send binary data.
  // I think we should use a CommandInterface because the DataInterface
  // is too generic: it deals with bytes sequences wrapped in data frames.
  std::unique_ptr<DataInterface> data_interface_;
};
}  // namespace stepit_hardware
