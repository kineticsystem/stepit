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

#include <gmock/gmock.h>
#include <stepit_hardware/request.h>
#include <stepit_hardware/motor_move_to_command.h>
#include <stepit_hardware/command_interface.h>
#include <stepit_hardware/serial_interface.h>
#include <mock_serial_interface.h>

namespace stepit_hardware::tests
{
TEST(command_interface, send)
{
  MockSerialInterface mock;
  CommandInterface cmd_interface{ &mock };

  EXPECT_CALL(mock, write(std::vector<uint8_t>{ {
                        0x7e,  // Delimiter
                        0x00,  // Request ID
                        0x71,  // Motor Move To command ID
                        0x01,  // Motor ID
                        0x00,  // Position (MSB)
                        0x00,  // Position
                        0x0b,  // Position
                        0xb8,  // Position (LSB)
                        0x05,  // CRC (MSB)
                        0x17,  // CRC (LSB)
                        0x7e   // Delimiter
                    } }));

  MotorMoveToCommand cmd{ 1, 3000 };
  cmd_interface.send(cmd);

}  // namespace stepit_hardware::tests

}  // namespace stepit_hardware::tests
