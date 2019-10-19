/* Copyright (c) 2019 James Jackson, Matt Rydalch
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef UBLOX_H
#define UBLOX_H

#include <stdint.h>
#include <fstream>
#include <functional>
#include <iostream>

#include "rosflight_utils/ubx.h"

#include "async_comm/serial.h"

namespace ublox
{
class UBLOX
{
    static constexpr int MAX_NUM_TRIES = 3;

public:
    typedef enum
    {
        NONE = 0,
        ROVER = 0b10,
        BASE = 0b11,
        RTK = 0b10,
    } rtk_type_t;

    enum
    {
        MOVING,
        STATIONARY
    };

    UBLOX(async_comm::Serial& ser);
    ~UBLOX() = default;

    // UBLOX receiver read/write
    void read_cb(const uint8_t* buf, const size_t size);

    void registerListener(ublox::UBXListener* l) { ubx_.registerListener(l); }

private:
    void poll_value();

    async_comm::Serial& ser_;

    UBX ubx_;
};
}  // namespace ublox

#endif
