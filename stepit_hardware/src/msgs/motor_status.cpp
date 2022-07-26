#include <stepit_hardware/msgs/motor_status.hpp>

#include <stepit_hardware/data_utils.hpp>

namespace stepit_hardware
{

int32_t MotorStatus::position() const
{
  return position_;
}

int32_t MotorStatus::distance_to_go() const
{
  return distance_to_go_;
}

float MotorStatus::speed() const
{
  return speed_;
}
}  // namespace stepit_hardware
