#include <stepit_hardware/data_buffer.hpp>

namespace stepit_hardware
{
DataBuffer::DataBuffer(std::size_t capacity) : buffer_{ capacity }
{
}

int DataBuffer::size() const
{
  return buffer_.size();
}

void DataBuffer::clear()
{
  buffer_.clear();
}

void DataBuffer::add_int8(const int8_t value, BufferPosition position)
{
  buffer_.add(value, position);
}

int8_t DataBuffer::remove_int8(BufferPosition position)
{
  return buffer_.remove(position);
}

// Following IEEE 754 specification a type int is always sent/received
// with Most Significant Byte (MSB) first.
void DataBuffer::add_int16(const int16_t value, BufferPosition position)
{
  const auto bytes = reinterpret_cast<const uint8_t*>(&value);
  if (position == BufferPosition::Tail)
  {
    buffer_.add(bytes[1], BufferPosition::Tail);
    buffer_.add(bytes[0], BufferPosition::Tail);
  }
  else
  {
    buffer_.add(bytes[0], BufferPosition::Head);
    buffer_.add(bytes[1], BufferPosition::Head);
  }
}

// Following IEEE 754 specification, a type int is always sent/received
// with Most Significant Byte (MSB) first.
int16_t DataBuffer::remove_int16(BufferPosition position)
{
  int value;
  auto bytes = reinterpret_cast<uint8_t*>(&value);
  if (position == BufferPosition::Head)
  {
    bytes[1] = buffer_.remove(BufferPosition::Head);
    bytes[0] = buffer_.remove(BufferPosition::Head);
  }
  else
  {
    bytes[0] = buffer_.remove(BufferPosition::Tail);
    bytes[1] = buffer_.remove(BufferPosition::Tail);
  }
  return value;
}

// Following IEEE 754 specification, a type long is always sent/received
// with Most Significant Byte (MSB) first.
void DataBuffer::add_int32(const int32_t value, BufferPosition position)
{
  const auto bytes = reinterpret_cast<const uint8_t*>(&value);
  if (position == BufferPosition::Tail)
  {
    buffer_.add(bytes[3], BufferPosition::Tail);
    buffer_.add(bytes[2], BufferPosition::Tail);
    buffer_.add(bytes[1], BufferPosition::Tail);
    buffer_.add(bytes[0], BufferPosition::Tail);
  }
  else
  {
    buffer_.add(bytes[0], BufferPosition::Head);
    buffer_.add(bytes[1], BufferPosition::Head);
    buffer_.add(bytes[2], BufferPosition::Head);
    buffer_.add(bytes[3], BufferPosition::Head);
  }
}

// Following IEEE 754 specification, a type long is always sent/received
// with Most Significant Byte (MSB) first.
int32_t DataBuffer::remove_int32(BufferPosition position)
{
  long value;
  auto bytes = reinterpret_cast<uint8_t*>(&value);
  if (position == BufferPosition::Head)
  {
    bytes[3] = buffer_.remove(BufferPosition::Head);
    bytes[2] = buffer_.remove(BufferPosition::Head);
    bytes[1] = buffer_.remove(BufferPosition::Head);
    bytes[0] = buffer_.remove(BufferPosition::Head);
  }
  else
  {
    bytes[0] = buffer_.remove(BufferPosition::Tail);
    bytes[1] = buffer_.remove(BufferPosition::Tail);
    bytes[2] = buffer_.remove(BufferPosition::Tail);
    bytes[3] = buffer_.remove(BufferPosition::Tail);
  }
  return value;
}

// Following IEEE 754 specification, a type float is always sent/received
// with Sign and Exponent first followed by the Significand in
// Most Significant Byte (MSB) first.
void DataBuffer::add_float(const float value, BufferPosition position)
{
  const auto bytes = reinterpret_cast<const uint8_t*>(&value);
  if (position == BufferPosition::Tail)
  {
    buffer_.add(bytes[3], BufferPosition::Tail);
    buffer_.add(bytes[2], BufferPosition::Tail);
    buffer_.add(bytes[1], BufferPosition::Tail);
    buffer_.add(bytes[0], BufferPosition::Tail);
  }
  else
  {
    buffer_.add(bytes[0], BufferPosition::Head);
    buffer_.add(bytes[1], BufferPosition::Head);
    buffer_.add(bytes[2], BufferPosition::Head);
    buffer_.add(bytes[3], BufferPosition::Head);
  }
}

// Following IEEE 754 specification a type float is stored in the following way:
// - Sign: 1bit
// - Exponent: 8 bits
// - Significand: 23 bits.
float DataBuffer::remove_float(BufferPosition position)
{
  float value;
  auto bytes = reinterpret_cast<uint8_t*>(&value);
  if (position == BufferPosition::Head)
  {
    bytes[3] = buffer_.remove(BufferPosition::Head);
    bytes[2] = buffer_.remove(BufferPosition::Head);
    bytes[1] = buffer_.remove(BufferPosition::Head);
    bytes[0] = buffer_.remove(BufferPosition::Head);
  }
  else
  {
    bytes[0] = buffer_.remove(BufferPosition::Tail);
    bytes[1] = buffer_.remove(BufferPosition::Tail);
    bytes[2] = buffer_.remove(BufferPosition::Tail);
    bytes[3] = buffer_.remove(BufferPosition::Tail);
  }
  return value;
}
}  // namespace stepit_hardware
