// Copyright 2023 Giovanni Remigi
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Giovanni Remigi nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <gtest/gtest.h>
#include <cobs_serial/crc_utils.hpp>

namespace cobs_serial::test
{
TEST(TestCrcUtils, calculate_crc)
{
  ASSERT_EQ(cobs_serial::crc_ccitt({ 0xA1, 0xB2, 0xC3, 0xD4, 0xE5, 0xF6 }), 0x3B07);
  ASSERT_EQ(cobs_serial::crc_ccitt({ 0xE2, 0x12, 0xF1, 0xFF, 0x00, 0xD2 }), 0x7071);
  ASSERT_EQ(cobs_serial::crc_ccitt({ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }), 0xFBE9);
  ASSERT_EQ(cobs_serial::crc_ccitt({ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }), 0x0000);
  ASSERT_EQ(cobs_serial::crc_ccitt({ 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39 }), 0x2189);
  ASSERT_EQ(cobs_serial::crc_ccitt({ 0x80, 0x00, 0x00, 0x03 }), 0x1ff5);
}
}  // namespace cobs_serial::test
