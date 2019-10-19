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

#ifndef UBX_H
#define UBX_H

#include <cstdint>
#include <functional>
#include <iostream>
#include <set>
#include <vector>

#include "rosflight_utils/ubx_defs.h"
#include "async_comm/serial.h"


namespace ublox
{
class UBXListener
{
public:
    void subscribe(const uint8_t cls, const uint8_t id)
    {
        uint16_t key = cls << 8 | id;
        subscriptions_.insert(key);
    }
    bool subscribed(const uint8_t cls, const uint8_t id) const
    {
        uint16_t key = cls << 8 | id;
        return subscriptions_.find(key) != subscriptions_.end();
    }

    virtual void got_ubx(const uint8_t cls, const uint8_t id, const UBX_message_t &) = 0;

private:
    std::set<uint16_t> subscriptions_;
};

class UBX
{
    static constexpr int TIMEOUT_MS = 1000;

public:
    UBX(async_comm::Serial &ser);

    void configure(uint8_t version,
                   uint8_t layer,
                   uint64_t cfgData,
                   uint32_t cfgDataKey,
                   uint8_t size);
    void get_configuration(uint8_t version, uint8_t layer, uint32_t cfgDataKey);
    void set_dynamic_mode();
    void enable_message(uint8_t msg_cls, uint8_t msg_id, uint8_t rate);

    void poll_value();
    void restart();
    void disable_nmea();

    bool get_version();

    bool wait_for_response();

    // This function returns true when a new message has been parsed
    bool read_cb(uint8_t byte);

    void version_cb();

    // listener handling
    void registerListener(UBXListener *listener);
    std::vector<UBXListener *> listeners_;

    bool parsing_message();

    size_t num_messages_received();

    void set_nav_rate(uint8_t period_ms);

    // Send the supplied message
    bool send_message(uint8_t msg_class, uint8_t msg_id, UBX_message_t &message, uint16_t len);

    // Main buffers for communication
    UBX_message_t out_message_;
    UBX_message_t in_message_;

    // low-level parsing functions
    bool decode_message();
    void calculate_checksum(const uint8_t msg_cls,
                            const uint8_t msg_id,
                            const uint16_t len,
                            const UBX_message_t payload,
                            uint8_t &ck_a,
                            uint8_t &ck_b) const;

    void extract_version_string(const char *str);
    void extract_module_name(const char *str);

    // Parsing State Working Memory
    uint8_t prev_byte_;
    uint16_t buffer_head_ = 0;
    bool start_message_ = false;
    bool end_message_ = false;
    bool got_ack_ = false;
    bool got_ver_ = false;
    bool got_nack_ = false;
    parse_state_t parse_state_;
    uint8_t message_class_;
    uint8_t message_type_;
    uint16_t length_;
    uint8_t ck_a_;
    uint8_t ck_b_;
    uint32_t num_errors_ = 0;
    uint32_t num_messages_received_ = 0;
    uint8_t version;  // 0 poll request, 1 poll (receiver to return config data key
                      // and value pairs)
    uint8_t layer;
    uint32_t cfgDataKey;
    uint64_t cfgData;
    uint8_t size;
    
    volatile bool new_data_;

    int major_version() const { return major_version_; }
    int minor_version() const { return minor_version_; }
    const char *module_name() const { return module_name_; }

    int major_version_;
    int minor_version_;
    char module_name_[10];

    // Serial Port
    void read(uint8_t *buf, size_t size);
    void write(const uint8_t byte);
    void write(const uint8_t *byte, const size_t size);
    async_comm::Serial &ser_;
};

}  // namespace ublox
#endif  // UBX_H
