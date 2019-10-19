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

#include "rosflight_utils/ublox.h"

#define DBG(...) fprintf(stderr, __VA_ARGS__)

namespace ublox
{
constexpr int UBLOX::MAX_NUM_TRIES;

UBLOX::UBLOX(async_comm::Serial &ser) : ser_(ser), ubx_(ser)
{
    ser_.register_receive_callback([this](const uint8_t* buf, size_t size) {
        this->read_cb(buf, size);
    });

    //  We have to make sure we get the version before we do anything else
    int num_tries = 0;
    while (++num_tries < MAX_NUM_TRIES && !ubx_.get_version())
    {
        printf("Unable to query version.  Trying again (%d out of %d)\n", num_tries + 1,
               MAX_NUM_TRIES);
    }

    if (num_tries == MAX_NUM_TRIES)
    {
        throw std::runtime_error("Unable to connect.  Please check your serial port configuration");
    }

    // configure the parsers/Enable Messages
    ubx_.disable_nmea();
    ubx_.set_nav_rate(200);

    ubx_.enable_message(CLASS_NAV, NAV_PVT, 1);
    ubx_.enable_message(CLASS_NAV, NAV_VELECEF, 1);
    ubx_.enable_message(CLASS_NAV, NAV_POSECEF, 1);
}

void UBLOX::read_cb(const uint8_t *buf, size_t size)
{
    for (int i = 0; i < size; i++)
    {
            ubx_.read_cb(buf[i]);
    }
}

}  // namespace ublox
