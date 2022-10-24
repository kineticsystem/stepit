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

#include <stepit_hardware/stepit_hardware.hpp>
#include <stepit_hardware/stepit_hardware_impl.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace stepit_hardware
{

// This constructor is use for testing only.
StepitHardware::StepitHardware() : impl_{ std::make_unique<StepitHardwareImpl>() }
{
}

StepitHardware::StepitHardware(std::unique_ptr<CommandInterface> command_interface)
  : impl_{ std::make_unique<StepitHardwareImpl>(std::move(command_interface)) }
{
}

StepitHardware::~StepitHardware()
{
}

hardware_interface::CallbackReturn StepitHardware::on_init(const hardware_interface::HardwareInfo& info)
{
  try
  {
    return impl_->on_init(info);
  }
  catch (const std::exception& e)
  {
    return CallbackReturn::ERROR;
  }
}

std::vector<hardware_interface::StateInterface> StepitHardware::export_state_interfaces()
{
  try
  {
    return impl_->export_state_interfaces();
  }
  catch (const std::exception& e)
  {
    return {};
  }
}

std::vector<hardware_interface::CommandInterface> StepitHardware::export_command_interfaces()
{
  try
  {
    return impl_->export_command_interfaces();
  }
  catch (const std::exception& e)
  {
    return {};
  }
}

hardware_interface::CallbackReturn StepitHardware::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  try
  {
    return impl_->on_activate(previous_state);
  }
  catch (const std::exception& e)
  {
    return CallbackReturn::ERROR;
  }
}

hardware_interface::CallbackReturn StepitHardware::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  try
  {
    return impl_->on_deactivate(previous_state);
  }
  catch (const std::exception& e)
  {
    return CallbackReturn::ERROR;
  }
}

hardware_interface::return_type StepitHardware::read(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  try
  {
    return impl_->read(time, period);
  }
  catch (const std::exception& e)
  {
    return hardware_interface::return_type::ERROR;
  }
}

hardware_interface::return_type StepitHardware::write(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  try
  {
    return impl_->write(time, period);
  }
  catch (const std::exception& e)
  {
    return hardware_interface::return_type::ERROR;
  }
}

}  // namespace stepit_hardware

PLUGINLIB_EXPORT_CLASS(stepit_hardware::StepitHardware, hardware_interface::SystemInterface)
