#include <stepit_driver/motor_status.h>

#include <stepit_driver/data_utils.h>

namespace stepit_driver
{
void MotorStatus::read(DataBuffer& buffer)
{
  position_ = buffer.remove_int32(BufferPosition::Head);
  distance_to_go_ = buffer.remove_int32(BufferPosition::Head);
  speed_ = buffer.remove_float(BufferPosition::Head);
}

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
}  // namespace stepit_driver
