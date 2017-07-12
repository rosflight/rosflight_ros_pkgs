/*
 * Copyright (c) 2017 Daniel Koch, BYU MAGICC Lab.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * \file udp_board.cpp
 * \author Daniel Koch
 */

#include <rosflight_firmware/udp_board.h>

namespace rosflight_firmware
{

UDPBoard::UDPBoard(std::string bind_host, uint16_t bind_port, std::string remote_host, uint16_t remote_port)
{
}

UDPBoard::~UDPBoard()
{
}

void UDPBoard::serial_init(uint32_t baud_rate)
{
}

void UDPBoard::serial_write(uint8_t byte)
{
}

uint16_t UDPBoard::serial_bytes_available()
{
}

uint8_t UDPBoard::serial_read()
{
  if (read_queue_.empty())
    return 0;
}

void UDPBoard::async_read()
{
}

void UDPBoard::async_read_end(const boost::system::error_code &error, size_t bytes_transferred)
{
}

void UDPBoard::async_write(bool check_write_state)
{
}

void UDPBoard::async_write_end(const boost::system::error_code &error, size_t bytes_transferred)
{
}

} // namespace rosflight_firmware
