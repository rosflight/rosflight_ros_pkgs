/* Copyright (c) 2019 James Jackson
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

#pragma once

#include <cstddef>
#include <cstdint>

namespace ublox
{
static constexpr size_t BUFFER_SIZE = 2048;

enum
{
    FIX_TYPE_NO_FIX = 0x00,
    FIX_TYPE_DEAD_RECKONING = 0x01,
    FIX_TYPE_2D = 0x02,
    FIX_TYPE_3D = 0x03,
    FIX_TYPE_GPS_AND_DEAD_RECKONING = 0x04,
    FIX_TYPE_TIME_ONLY = 0x05,
};

enum
{
    START_BYTE_1 = 0xB5,
    START_BYTE_2 = 0x62,
};

enum
{
    CLASS_NAV = 0x01,  //    Navigation Results Messages: Position, Speed, Time, Acceleration,
                       //    Heading, DOP, SVs used
    CLASS_RXM = 0x02,  //    Receiver Manager Messages: Satellite Status, RTC Status
    CLASS_INF = 0x04,  //    Information Messages: Printf-Style Messages, with IDs such as Error,
                       //    Warning, Notice
    CLASS_ACK = 0x05,  //    Ack/Nak Messages: Acknowledge or Reject messages to CFG input messages
    CLASS_CFG = 0x06,  //    Configuration Input Messages: Set Dynamic Model, Set DOP Mask, Set Baud
                       //    Rate, etc.
    CLASS_UPD = 0x09,  //    Firmware Update Messages: Memory/Flash erase/write, Reboot, Flash
                       //    identification, etc.
    CLASS_MON = 0x0A,  //    Monitoring Messages: Communication Status, CPU Load, etc...
    CLASS_AID = 0x0B,  //    AssistNow Aiding Messages: Ephemeris, Almanac, other A-GPS data input
    CLASS_TIM = 0x0D,  //    Timing Messages: Time Pulse Output, Time Mark Results
    CLASS_ESF = 0x10,  //    External Sensor Fusion Messages: External Sensor Measurements and
                       //    Status Information
    CLASS_MGA = 0x13,  //    Multiple GNSS Assistance Messages: Assistance data for various GNSS
    CLASS_LOG = 0x21,  //    Logging Messages: Log creation, deletion, info and retrieva
    CLASS_RTCM = 0XF5,  //    RTCM messages
};

enum
{
    ACK_ACK = 0x01,   // Message Acknowledged
    ACK_NACK = 0x00,  // Message Not-Acknowledged
};

enum
{
    AID_ALM = 0x30,  // Poll GPS Aiding Almanac Data
    AID_AOP = 0x33,  // AssistNow Autonomous data
    AID_EPH = 0x31,  // GPS Aiding Ephemeris Input/Output Message
    AID_HUI = 0x02,  // Poll GPS Health, UTC, ionosphere parameters
    AID_INI = 0x01,  // Aiding position, time, frequency, clock drift
};

enum
{
    CFG_ANT = 0x13,       // Get/Set Antenna Control Settings
    CFG_BATCH = 0x93,     // Get/Set Get/Set data batching configuration
    CFG_CFG = 0x09,       // Command Clear, Save and Load configurations
    CFG_DAT = 0x06,       // Get The currently defined Datum
    CFG_DGNSS = 0x70,     // Get/Set DGNSS configuration
    CFG_DOSC = 0x61,      // Get/Set Disciplined oscillator configuration
    CFG_DYNSEED = 0x85,   // Set Programming the dynamic seed for the host...
    CFG_ESRC = 0x60,      // Get/Set External synchronization source configuration
    CFG_FIXSEED = 0x84,   // Set Programming the fixed seed for host...
    CFG_GEOFENCE = 0x69,  // Get/Set Geofencing configuration
    CFG_GNSS = 0x3E,      // Get/Set GNSS system configuration
    CFG_HNR = 0x5C,       // Get/Set High Navigation Rate Settings
    CFG_INF = 0x02,       // Poll Request Poll configuration for one protocol
    CFG_ITFM = 0x39,      // Get/Set Jamming/Interference Monitor configuration
    FG_LOGFILTER = 0x47,  // Get/Set Data Logger Configuration
    CFG_MSG = 0x01,       // Poll Request Poll a message configuration
    CFG_NAV5 = 0x24,      // Get/Set Navigation Engine Settings
    CFG_NAVX5 = 0x23,     // Get/Set Navigation Engine Expert Settings
    CFG_NMEA = 0x17,      // Get/Set NMEA protocol configuration (deprecated)
    CFG_ODO = 0x1E,       // Get/Set Odometer, Low_speed COG Engine Settings
    CFG_PM2 = 0x3B,       // Get/Set Extended Power Management configuration
    CFG_PMS = 0x86,       // Get/Set Power Mode Setup
    CFG_PRT = 0x00,       // Poll Request Polls the configuration for one I/O Port
    CFG_PWR = 0x57,       // Set Put receiver in a defined power state.
    CFG_RATE = 0x08,      // Get/Set Navigation/Measurement Rate Settings
    CFG_RINV = 0x34,      // Get/Set Contents of Remote Inventory
    CFG_RST = 0x04,       // Command Reset Receiver / Clear Backup Data Structures
    CFG_RXM = 0x11,       // Get/Set RXM configuration
    CFG_SBAS = 0x16,      // Get/Set SBAS Configuration
    CFG_SMGR = 0x62,      // Get/Set Synchronization manager configuration
    CFG_TMODE2 = 0x3D,    // Get/Set Time Mode Settings 2
    CFG_TMODE3 = 0x71,    // Get/Set Time Mode Settings 3
    CFG_TP5 = 0x31,       // Poll Request Poll Time Pulse Parameters for Time Pulse 0
    CFG_TXSLOT = 0x53,    // Set TX buffer time slots configuration
    CFG_USB = 0x1B,       // Get/Set USB Configuration
    CFG_VALDEL = 0x8C,    // Deletes values corresponding to...
    CFG_VALGET = 0x8B,    // Get configuration items
    CFG_VALSET = 0x8A,    // Set values correpsonding to provided...
};

enum
{
    NAV_AOPSTATUS = 0x60,  // Periodic/Polled AssistNow Autonomous Status
    NAV_ATT = 0x05,        // Periodic/Polled Attitude Solution
    NAV_CLOCK = 0x22,      // Periodic/Polled Clock Solution
    NAV_DGPS = 0x31,       // Periodic/Polled DGPS Data Used for NAV
    NAV_DOP = 0x04,        // Periodic/Polled Dilution of precision
    NAV_EOE = 0x61,        // Periodic End Of Epoch
    NAV_GEOFENCE = 0x39,   // Periodic/Polled Geofencing status
    NAV_HPPOSECEF = 0x13,  // Periodic/Polled High Precision Position Solution in ECEF
    NAV_HPPOSLLH = 0x14,   // Periodic/Polled High Precision Geodetic Position Solution
    NAV_ODO = 0x09,        // Periodic/Polled Odometer Solution
    NAV_ORB = 0x34,        // Periodic/Polled GNSS Orbit Database Info
    NAV_POSECEF = 0x01,    // Periodic/Polled Position Solution in ECEF
    NAV_POSLLH = 0x02,     // Periodic/Polled Geodetic Position Solution
    NAV_PVT = 0x07,        // Periodic/Polled Navigation Position Velocity Time Solution
    NAV_RELPOSNED = 0x3C,  // Periodic/Polled Relative Positioning Information in NED frame
    NAV_RESETODO = 0x10,   // Command Reset odometer
    NAV_SAT = 0x35,        // Periodic/Polled Satellite Information
    NAV_SBAS = 0x32,       // Periodic/Polled SBAS Status Data
    NAV_SOL = 0x06,        // Periodic/Polled Navigation Solution Information
    NAV_STATUS = 0x03,     // Periodic/Polled Receiver Navigation Status
    NAV_SVINFO = 0x30,     // Periodic/Polled Space Vehicle Information
    NAV_SVIN = 0x3B,       // Periodic/Polled Survey-in data
    NAV_TIMEBDS = 0x24,    // Periodic/Polled BDS Time Solution
    NAV_TIMEGAL = 0x25,    // Periodic/Polled Galileo Time Solution
    NAV_TIMEGLO = 0x23,    // Periodic/Polled GLO Time Solution
    NAV_TIMEGPS = 0x20,    // Periodic/Polled GPS Time Solution
    NAV_TIMELS = 0x26,     // Periodic/Polled Leap second event information
    NAV_TIMEUTC = 0x21,    // Periodic/Polled UTC Time Solution
    NAV_VELECEF = 0x11,    // Periodic/Polled Velocity Solution in ECEF
    NAV_VELNED = 0x12,     // Periodic/Polled Velocity Solution in NED
    NAV_SIG = 0x43,        // Periodic/Polled signal information
};

enum
{
    MON_COMMS = 0x36,  // Comm port information
    MON_GNSS = 0x28,   // Information message major GNSS
    MON_HW2 = 0x0B,    // Hardware Status
    MON_HW3 = 0x37,    // HW I/O pin information
    MON_HW = 0x09,     // Hardware Status
    MON_IO = 0x02,     // I/O Subsystem Status
    MON_MSGPP = 0x06,  // Message Parse and Process Status
    MON_PATCH = 0x27,  // Information about installed...
    MON_RF = 0x38,     // RF information
    MON_RXBUF = 0x07,  // Receiver Buffer Status
    MON_RXR = 0x21,    // Receiver Status Information
    MON_TXBUF = 0x08,  // Buffer Status
    MON_VER = 0x04,    // PolledReceiver/Software Version
};

enum
{
    RXM_RAWX = 0x15,  // Multi-GNSS Raw Measurement Data
    RXM_SFRBX = 0x13  // Broadcast Navigation Data Subframe (ephemeris)
};

typedef enum
{
    START,
    GOT_START_FRAME,
    GOT_CLASS,
    GOT_MSG_ID,
    GOT_LENGTH1,
    GOT_LENGTH2,
    GOT_PAYLOAD,
    GOT_CK_A,
    GOT_CK_B,
    GOT_CK_C,
    DONE,
} parse_state_t;

typedef struct
{
    uint8_t clsID;
    uint8_t msgID;
} __attribute__((packed)) ACK_ACK_t;

typedef struct
{
    uint8_t clsID;
    uint8_t msgID;
} __attribute__((packed)) ACK_NACK_t;

typedef struct
{
    uint8_t msgClass;
    uint8_t msgID;
    uint8_t rate;
} __attribute__((packed)) CFG_MSG_t;

typedef struct
{
    enum
    {
        DYNMODE_PORTABLE = 0,
        DYNMODE_STATIONARY = 2,
        DYNMODE_PEDESTRIAN = 3,
        DYNMODE_AUTOMOTIVE = 4,
        DYNMODE_SEA = 5,
        DYNMODE_AIRBORNE_1G = 6,
        DYNMODE_AIRBORNE_2G = 7,
        DYNMODE_AIRBORNE_4G = 8
    };
    enum
    {
        FIXMODE_2D_ONLY = 1,
        FIXMODE_3D_ONLY = 2,
        FIXMODE_AUTO = 3,
    };

    enum
    {
        UTC_STANDARD_AUTO = 0,  // receiver selects based on GNSS configuration
        UTC_STANDARD_USA = 3,  // UTC as operated by the U.S. Naval Observatory (USNO);
        UTC_STANDARD_RUS = 6,  // UTC as operated by the former Soviet Union; (GLONASS time)
        UTC_STANDARD_CHN = 7,  // UTC as operated by the National Time Service Center, China;
                               // derived from BeiDou time
    };

    enum
    {
        MASK_DYN = 0b00000000001,             // Apply dynamic model settings
        MASK_MINEL = 0b00000000010,           // Apply minimum elevation settings
        MASK_POSFIXMODE = 0b00000000100,      // Apply fix mode settings
        MASK_DRLIM = 0b00000001000,           // Reserved
        MASK_POSMASK = 0b00000010000,         // Apply position mask settings
        MASK_TIMEMASK = 0b00000100000,        // Apply time mask settings
        MASK_STATICHOLDMASK = 0b00001000000,  // Apply static hold settings
        MASK_DGPSMASK = 0b00010000000,        // Apply DGPS settings.
        MASK_CNOTHRESHOLD = 0b00100000000,    // Apply CNO threshold settings
        MASK_UTC = 0b10000000000,             // Apply UTC settings
    };

    uint16_t mask;
    uint8_t dynModel;
    uint8_t fixMode;
    int32_t fixedAlt;       // (1e-2 m) Fixed altitude (mean sea level) for 2D fix mode.
    uint32_t fixedAltVar;   // (0.0001 m^2)Fixed altitude variance for 2D mode.
    int8_t minElev;         // (deg) Minimum Elevation for a GNSS satellite to be used in NAV
    uint8_t drLimit;        // s Reserved
    uint16_t pDop;          // (0.1) - Position DOP Mask to use
    uint16_t tDop;          // (0.1) - Time DOP Mask to use
    uint16_t pAcc;          // m Position Accuracy Mask
    uint16_t tAcc;          // m Time Accuracy Mask
    uint8_t staticHoldThr;  // esh cm/s Static hold threshold
    uint8_t dgnssTimeout;   // s DGNSS timeout
    uint8_t cnoThreshNumS;  // Vs - Number of satellites required to have C/N0 above cnoThresh for a
                            // fix to be attempted
    uint8_t cnoThresh;      // dBHz C/N0 threshold for deciding whether to attempt a fix
    uint8_t reserved1[2];   //  - Reserved
    uint16_t staticHoldMax;  // Dist m Static hold distance threshold (before quitting static hold)
    uint8_t utcStandard;     // - UTC standard to be used:
    uint8_t reserved2[5];

} __attribute__((packed)) CFG_NAV5_t;

typedef struct
{
    enum
    {
        PORT_I2C = 0,
        PORT_UART1 = 1,
        PORT_USB = 3,
        PORT_SPI = 4
    };
    enum
    {
        CHARLEN_8BIT = 0b11000000,
        PARITY_NONE = 0b100000000000,
        STOP_BITS_1 = 0x0000
    };
    enum
    {
        IN_UBX = 0b00000001,
        IN_NMEA = 0b00000010,
        IN_RTCM = 0b00000100,
        IN_RTCM3 = 0b00100000,
    };
    enum
    {
        OUT_UBX = 0b00000001,
        OUT_NMEA = 0b00000010,
        OUT_RTCM3 = 0b00100000,
    };

    uint8_t portID;
    uint8_t reserved1;
    uint16_t txReady;
    uint32_t mode;  // What the mode the serial data is on (See charlen, parity and stop bits)
    uint32_t baudrate;
    uint16_t inProtoMask;   // Which input protocols are enabled
    uint16_t outProtoMask;  // Which output protocols are enabled
    uint16_t flags;
    uint8_t reserved2[2];
} __attribute__((packed)) CFG_PRT_t;

typedef struct
{
    enum
    {
        RAM = 0,
        BBR = 1,
        FLASH = 2,
        DEFAULT = 7,
    };
    enum
    {
        REQUEST = 0,
        POLL = 1,
    };

    enum
    {
        VALSET_float = 2,
        VALSET_fixed = 3,
    };

    enum
    {
        MSGOUT_RELPOSNED = 0x20910090,  // Output rate of the UBX-NAV-RELPOSNED message on port USB
        MSGOUT_PVT = 0x20910009,        // Output rate of the UBX-NAV-PVT message on port USB
        MSGOUT_POSECEF = 0x20910027,    // Output rate of the UBX-NAV-POSECEF message on port USB
        MSGOUT_VELECEF = 0x20910040,    // Output rate of the UBX-NAV-VELECEF message on port USB
        MSGOUT_RAWX = 0x209102a7,       // Output rate of the UBX-RXM-RAWX message on port USB
        MSGOUT_SFRBX = 0x20910234,      // Output rate of the UBX-RXM-SFRBX message on port USB
    };

    enum
    {  // outgoing message rates for RTCM 3x on usb type U1
       // suggested messages for stationary base
       //clang-format off
        RTCM_1005USB = 0x209102c0,  // CFG-MSGOUT-RTCM_3X_TYPE1005_USB -- Stationary RTK Reference Station ARP
        RTCM_1074USB = 0x20910361,  // CFG-MSGOUT-RTCM_3X_TYPE1074_USB -- GPS MSM 4
        RTCM_1084USB = 0x20910366,  // CFG-MSGOUT-RTCM_3X_TYPE1084_USB -- GLONASS MSM 4
        RTCM_1094USB = 0x2091036b,  // CFG-MSGOUT-RTCM_3X_TYPE1094_USB -- Galileo MSM 4
        RTCM_1124USB = 0x20910370,  // CFG-MSGOUT-RTCM_3X_TYPE1124_USB -- Beidou MSM 4
        RTCM_1230USB = 0x20910306,  // CFG-MSGOUT-RTCM_3X_TYPE1230_USB -- Glonass L1 and L2
                                    // Code-Phase Biases suggested messages for mobile base
        RTCM_4072_0USB = 0x20910301,  // CFG-MSGOUT-RTCM_3X_TYPE4072_0_USB -- UBLOX Proprietary RTCM message
        RTCM_4072_1USB = 0x20910384,  // CFG-MSGOUT-RTCM_3X_TYPE4072_1_USB --
        RTCM_1077USB = 0x209102cf,  // CFG-MSGOUT-RTCM_3X_TYPE1077_USB __ GPS MSM 7 (high precision)
        RTCM_1087USB = 0x209102d4,  // CFG-MSGOUT-RTCM_3X_TYPE1087_USB __ GLONASS MSM 7 (high precision)
        RTCM_1097USB = 0x2091031b,  // CFG-MSGOUT-RTCM_3X_TYPE1097_USB __ Galileo MSM 7 (high precision)
        RTCM_1127USB = 0x209102d9,  // CFG-MSGOUT-RTCM_3X_TYPE1127_USB __ Beidou MSM 7 (high precision)
                         //!!!!also use RTCM_1230USB above!!!///
       //clang-format o
    };

    enum
    {
        USB_INPROT_UBX = 0x10770001,   // Flag to indicate if UBX should be an input protocol on USB
        USB_INPROT_NMEA = 0x10770002,  // Flag to indicate if NMEA should be an input on USB
        USB_INPROT_RTCM3X = 0x10770004,  // Flag to indicate if RTCM3X should be an input on USB
        USB_OUTPROT_UBX = 0x10780001,  // Flag to indicate if UBX should bean output protocol on USB
        USB_OUTPROT_NMEA = 0x10780002,    // Flag to indicate if NMEA should be an output on USB
        USB_OUTPROT_RTCM3X = 0x10780004,  // Flag to indicate if RTCM3X should be an outputoon USB
    };

    enum
    {

        DYNMODEL = 0x20110021,  // Dynamic platform model
        DYNMODE_PORTABLE = 0,
        DYNMODE_STATIONARY = 2,
        DYNMODE_PEDESTRIAN = 3,
        DYNMODE_AUTOMOTIVE = 4,
        DYNMODE_SEA = 5,
        DYNMODE_AIRBORNE_1G = 6,
        DYNMODE_AIRBORNE_2G = 7,
        DYNMODE_AIRBORNE_4G = 8,
        DYNMODE_WRIST_WORN = 9,
        DYNMODE_BIKE = 10,
    };

    enum
    {
        MSGOUT_SVIN = 0x2091008b,
        TMODE_MODE = 0x20030001,
        TMODE_SVIN_MIN_DUR = 0x40030010,    // survey in minimum duration s
        TMODE_SVIN_ACC_LIMIT = 0x40030011,  // Survey-in position accuracy limit mm
    };

    // enum not finished, but not needed.  The rest is not needed.
    enum
    {
        SIGNAL_GPS = 0x1031001f,      // GPS enable
        SIGNAL_GPS_L1 = 0x10310001,   // GPS L1C/A
        SIGNAL_GPS_L2 = 0x10310003,   // GPS L2C (only on u-blox F9 platform products)
        SIGNAL_GAL = 0x10310021,      // Galileo enable
        SIGNAL_GAL_E1 = 0x10310007,   // Galileo E1
        SIGNAL_GAL_E5B = 0x1031000a,  // Galileo E5b (only on u-blox F9 platform products)
        SIGNAL_BDS = 0x10310022,      // BeiDou Enable
        SIGNAL_BDS_B1 = 0x1031000d,   // BeiDou B1I
        SIGNAL_BDS_B2 = 0x1031000e,   // BeiDou B2I

    };

    enum
    {
        // used for nav rate messages
        RATE_MEAS = 0x30210001,  // Nominal time between GNSS measurements (e.g. 100ms results in
                                 // 10Hz measurement rate, 1000ms = 1Hz measurement rate)
        RATE_NAV = 0x30210002,  // Ratio of number of measurements to number of navigation solutions
        RATE_TIMEREF = 0x20210003,  // Time system to which measurements are aligned
    };

    enum
    {  // Constants for cfg-rate-timeref/ RATE_TIMEREF
        TIME_REF_UTC = 0,
        TIME_REF_GPS = 1,
        TIME_REF_GLONASS = 2,
        TIME_REF_BUIDOU = 3,
        TIME_REF_GALILEO = 4
    };

    uint8_t version;  // 0 poll request, 1 poll (receiver to return config data key and value pairs)
    uint8_t layer;
    uint8_t reserved1[2];
    uint32_t cfgDataKey;
    uint64_t cfgData;
} __attribute__((packed)) CFG_VALGET_t;

typedef struct
{
    enum
    {
        RAM = 0b00000001,
        BBR = 0b00000010,
        FLASH = 0b00000100,
        DEFAULT = 0b01000000,
    };

    enum
    {
        VERSION_0 = 0,
        VERSION_1 = 1,
    };

    enum
    {
        VALSET_float = 2,
        VALSET_fixed = 3,
    };

    enum
    {
        MSGOUT_RELPOSNED = 0x20910090,  // Output rate of the UBX-NAV-RELPOSNED message on port USB
        MSGOUT_PVT = 0x20910009,        // Output rate of the UBX-NAV-PVT message on port USB
        MSGOUT_POSECEF = 0x20910027,    // Output rate of the UBX-NAV-POSECEF message on port USB
        MSGOUT_VELECEF = 0x20910040,    // Output rate of the UBX-NAV-VELECEF message on port USB
        MSGOUT_RAWX = 0x209102a7,       // Output rate of the UBX-RXM-RAWX message on port USB
        MSGOUT_SFRBX = 0x20910234,      // Output rate of the UBX-RXM-SFRBX message on port USB
    };

    enum
    {                               // outgoing message rates for RTCM 3x on usb type U1
                                    // suggested messages for stationary base
        RTCM_1005USB = 0x209102c0,  // CFG-MSGOUT-RTCM_3X_TYPE1005_USB -- Stationary RTK Ref Station
        RTCM_1074USB = 0x20910361,  // CFG-MSGOUT-RTCM_3X_TYPE1074_USB -- GPS MSM 4
        RTCM_1084USB = 0x20910366,  // CFG-MSGOUT-RTCM_3X_TYPE1084_USB -- GLONASS MSM 4
        RTCM_1094USB = 0x2091036b,  // CFG-MSGOUT-RTCM_3X_TYPE1094_USB -- Galileo MSM 4
        RTCM_1124USB = 0x20910370,  // CFG-MSGOUT-RTCM_3X_TYPE1124_USB -- Beidou MSM 4
        RTCM_1230USB = 0x20910306,  // CFG-MSGOUT-RTCM_3X_TYPE1230_USB -- Glonass L1 and L2
                                    // Code-Phase Biases suggested messages for mobile base
        RTCM_4072_0USB = 0x20910301,  // CFG-MSGOUT-RTCM_3X_TYPE4072_0_USB -- UBLOX Proprietary
        RTCM_4072_1USB = 0x20910384,  // CFG-MSGOUT-RTCM_3X_TYPE4072_1_USB --
        RTCM_1077USB = 0x209102cf,    // CFG-MSGOUT-RTCM_3X_TYPE1077_USB __ GPS MSM 7
        RTCM_1087USB = 0x209102d4,    // CFG-MSGOUT-RTCM_3X_TYPE1087_USB __ GLONASS MSM 7
        RTCM_1097USB = 0x2091031b,    // CFG-MSGOUT-RTCM_3X_TYPE1097_USB __ Galileo MSM 7
        RTCM_1127USB = 0x209102d9,    // CFG-MSGOUT-RTCM_3X_TYPE1127_USB __ Beidou MSM 7
                                      //!!!!also use RTCM_1230USB above!!!///
    };

    enum
    {
        USB_INPROT_UBX = 0x10770001,  // Flag to indicate if UBX should be an input protocol on USB
        USB_INPROT_NMEA =
            0x10770002,  // Flag to indicate if NMEA should be an input protocol on USB
        USB_INPROT_RTCM3X =
            0x10770004,  // Flag to indicate if RTCM3X should be an input protocol on USB
        USB_OUTPROT_UBX = 0x10780001,  // Flag to indicate if UBX should bean output protocol on USB
        USB_OUTPROT_NMEA =
            0x10780002,  // Flag to indicate if NMEA should bean output protocol on USB
        USB_OUTPROT_RTCM3X =
            0x10780004,  // Flag to indicate if RTCM3X should bean output protocol on USB
    };

    enum
    {

        DYNMODEL = 0x20110021,  // Dynamic platform model
        DYNMODE_PORTABLE = 0,
        DYNMODE_STATIONARY = 2,
        DYNMODE_PEDESTRIAN = 3,
        DYNMODE_AUTOMOTIVE = 4,
        DYNMODE_SEA = 5,
        DYNMODE_AIRBORNE_1G = 6,
        DYNMODE_AIRBORNE_2G = 7,
        DYNMODE_AIRBORNE_4G = 8,
        DYNMODE_WRIST_WORN = 9,
        DYNMODE_BIKE = 10,
    };

    enum
    {
        MSGOUT_SVIN = 0x2091008b,
        TMODE_MODE = 0x20030001,
        TMODE_SVIN_MIN_DUR = 0x40030010,    // survey in minimum duration s
        TMODE_SVIN_ACC_LIMIT = 0x40030011,  // Survey-in position accuracy limit mm
    };

    // enum not finished, but not needed.  The rest is not needed.
    enum
    {
        SIGNAL_GPS = 0x1031001f,      // GPS enable
        SIGNAL_GPS_L1 = 0x10310001,   // GPS L1C/A
        SIGNAL_GPS_L2 = 0x10310003,   // GPS L2C (only on u-blox F9 platform products)
        SIGNAL_GAL = 0x10310021,      // Galileo enable
        SIGNAL_GAL_E1 = 0x10310007,   // Galileo E1
        SIGNAL_GAL_E5B = 0x1031000a,  // Galileo E5b (only on u-blox F9 platform products)
        SIGNAL_BDS = 0x10310022,      // BeiDou Enable
        SIGNAL_BDS_B1 = 0x1031000d,   // BeiDou B1I
        SIGNAL_BDS_B2 = 0x1031000e,   // BeiDou B2I

    };

    enum
    {
        // used for nav rate messages
        RATE_MEAS = 0x30210001,  // Nominal time between GNSS measurements (e.g. 100ms results in
                                 // 10Hz measurement rate, 1000ms = 1Hz measurement rate)
        RATE_NAV = 0x30210002,  // Ratio of number of measurements to number of navigation solutions
        RATE_TIMEREF = 0x20210003,  // Time system to which measurements are aligned
    };

    enum
    {  // Constants for cfg-rate-timeref/ RATE_TIMEREF
        TIME_REF_UTC = 0,
        TIME_REF_GPS = 1,
        TIME_REF_GLONASS = 2,
        TIME_REF_BUIDOU = 3,
        TIME_REF_GALILEO = 4
    };

    uint8_t version;
    uint8_t layer;
    uint8_t reserved1[2];
    uint32_t cfgDataKey;

    union
    {
        uint8_t bytes[4];
        uint16_t half_word[2];
        uint32_t word;
    } cfgData;

} __attribute__((packed)) CFG_VALSET_t;

typedef struct
{
    enum
    {
        TIME_REF_UTC = 0,
        TIME_REF_GPS = 1,
        TIME_REF_GLONASS = 2,
        TIME_REF_BUIDOU = 3,
        TIME_REF_GALILEO = 4
    };
    uint16_t measRate;  // (ms) The elapsed time between GNSS measurements which defines the rate
    uint16_t navRate;   // (cycles) The ratio between the number of measurements and the number of
                        // navigation solutions, e.g. 5 means five measurements for every navigation
                        // solution
    uint16_t timeRef;   // Time system to which measurements are aligned
} __attribute__((packed)) CFG_RATE_t;

typedef struct
{
    enum
    {
        VALIDITY_FLAGS_VALIDDATE = 0b01,  // Valid UTC Date (see Time Validity section for details)
        VALIDITY_FLAGS_VALIDTIME =
            0b10,  // Valid UTC Time of Day (see Time Validity section for details)
        VALIDITY_FLAGS_FULLYRESOLVED =
            0b100,  // UTC Time of Day has been fully resolved (no seconds uncertainty)
    };

    enum
    {
        FIX_STATUS_GNSS_FIX_OK = 0b00000001,  // Valid Fix
        FIX_STATUS_DIFF_SOLN = 0b00000010,    // Differential Corrections were applied
        FIX_STATUS_PSM_STATE_NOT_ACTIVE = 0b00000000,
        FIX_STATUS_PSM_STATE_ENABLED = 0b00000100,
        FIX_STATUS_PSM_STATE_ACQUISITION = 0b00001000,
        FIX_STATUS_PSM_STATE_TRACKING = 0b00001100,
        FIX_STATUS_PSM_STATE_POWER_OPTIMIZED_TRACKING = 0b00010000,
        FIX_STATUS_PSM_STATE_INACTIVE = 0b00010100,
        FIX_STATUS_HEADING_VALID = 0b00100000,
        FIX_STATUS_CARR_SOLN_NONE = 0b00000000,
        FIX_STATUS_CARR_SOLN_FLOAT = 0b01000000,
        FIX_STATUS_CARR_SOLN_FIXED = 0b10000000,
    };

    uint32_t iTOW;    // ms GPS time of week of the  navigation epoch . See the  description of iTOW
                      // for details.
    uint16_t year;    // y Year (UTC)
    uint8_t month;    // month Month, range 1..12 (UTC)
    uint8_t day;      // d Day of month, range 1..31 (UTC)
    uint8_t hour;     // h Hour of day, range 0..23 (UTC)
    uint8_t min;      // min Minute of hour, range 0..59 (UTC)
    uint8_t sec;      // s Seconds of minute, range 0..60 (UTC)
    uint8_t valid;    // - Validity flags (see  graphic below )
    uint32_t tAcc;    // ns Time accuracy estimate (UTC)
    int32_t nano;     // ns Fraction of second, range -1e9 .. 1e9 (UTC)
    uint8_t fixType;  // - GNSSfix Type:
    uint8_t flags;    // - Fix status flags (see  graphic below )
    uint8_t flags2;   // - Additional flags (see  graphic below )
    uint8_t numSV;    // - Number of satellites used in Nav Solution
    int32_t lon;      // 1e-7 deg Longitude
    int32_t lat;      // 1e-7 deg Latitude
    int32_t height;   // mm Height above ellipsoid
    int32_t hMSL;     // mm Height above mean sea level
    uint32_t hAcc;    // mm Horizontal accuracy estimate
    uint32_t vAcc;    // mm Vertical accuracy estimate
    int32_t velN;     // mm/s NED north velocity
    int32_t velE;     // mm/s NED east velocity
    int32_t velD;     // mm/s NED down velocity
    int32_t gSpeed;   // mm/s Ground Speed (2-D)
    int32_t headMot;  // 1e-5 deg Heading of motion (2-D)
    uint32_t sAcc;    // mm/s Speed accuracy estimate
    uint32_t headAcc;      // 1e-5 deg Heading accuracy estimate (both motion and vehicle)
    uint16_t pDOP;         // 0.01  - Position DOP
    uint8_t reserved1[6];  // - Reserved
    int32_t headVeh;       // 1e-5 deg Heading of vehicle (2-D)
    int16_t magDec;        // 1e-2 deg Magnetic declination
    uint16_t magAcc;       // 1e-2 deg Magnetic declination accuracy
} __attribute__((packed)) NAV_PVT_t;

typedef struct
{
    enum
    {
        FLAGS_gnssFixOK = 0b000000001,
        FLAGS_diffSoln = 0b000000010,
        FLAGS_relPosValid = 0b000000100,
        FLAGS_carrSoln_float = 0b000001000,
        FLAGS_carrSoln_fixed = 0b000010000,
        FLAGS_isMoving = 0b000100000,
        FLAGS_refPosMiss = 0b001000000,
        FLAGS_refObsMiss = 0b010000000,
        FLAGS_relPosHeadingValid = 0b100000000,
    };
    uint8_t version;        // Message version (0x01 for this version)
    uint8_t reserved1;      // Reserved
    uint16_t refStationId;  // Reference Station ID. Must be in the range 0..4095
    uint32_t iTow;  // GPS time of week ms of the navigation epoch. See the description of iTOW for
                    // details.
    int32_t relPosN;        // North component cm of relative position vector
    int32_t relPosE;        // East component cm of relative position vector
    int32_t relPosD;        // Down component cm of relative position vector
    int32_t relPosLength;   // Length cm of relative position vector
    int32_t relPosHeading;  // Heading deg of the relative position vector. Scaled 1e-5
    uint8_t reserved2[4];   // reserved
    int8_t relPosHPN;       // See Interface Description pg 157
    int8_t relPosHPE;       // See Interface Description pg 157
    int8_t relPosHPD;       // See Interface Description pg 157
    int8_t relPosHPLength;  // See Interface Description pg 157
    uint32_t accN;          // Accuracy mm of relative position North component
    uint32_t accE;          // Accuracy mm of relative position East component
    uint32_t accD;          // Accuracy mm of relative position Down component
    uint32_t accLength;     // Accuracy mm of Length of the relative position vector
    uint32_t accHeading;    // Accuracy deg of heading of the relative position vector
    uint8_t reserved3[4];   // Reserved
    uint32_t flags;         // See graphic in Interface Description pg 158

} __attribute__((packed)) NAV_RELPOSNED_t;

typedef struct
{
    uint8_t version;       // Message version (0x01 for this version)
    uint8_t reserved1[3];  // Reserved
    uint32_t iTow;   // GPS time of week ms of the navigation epoch. See the description of iTOW for
                     // details.
    uint32_t dur;    // Passed survey-in observation time s
    uint32_t meanX;  // Current survey-in mean position ECEF X coordinate cm
    uint32_t meanY;  // Current survey-in mean position ECEF Y coordinate cm
    uint32_t meanZ;  // Current survey-in mean position ECEF Z coordinate cm
    uint8_t meanXHP;       // See Interface Description pg 165
    uint8_t meanYHP;       // See Interface Description pg 165
    uint8_t meanZHP;       // See Interface Description pg 165
    uint8_t reserved2;     // Reserved
    uint32_t meanAcc;      // Current survey-in mean position accuracy mm
    uint32_t obs;          // number of position observations used during survey-in
    uint8_t valid;         // Survey-in postion validity flag, 1=valid, otherwise 0
    uint8_t active;        // survey-in in progress flag, 1 = in-progress, otherwise 0
    uint8_t reserved3[2];  // Reserved

} __attribute__((packed)) NAV_SVIN_t;

typedef struct
{
    uint32_t iTOW;  // ms GPS time of week of the  navigation epoch . See the  description of iTOW
                    // for details.
    int32_t ecefX;  // cm ECEF X coordinate
    int32_t ecefY;  // cm ECEF Y coordinate
    int32_t ecefZ;  // cm ECEF Z coordinate
    uint32_t pAcc;  // cm Position Accuracy Estimate
} __attribute__((packed)) NAV_POSECEF_t;

typedef struct
{
    uint32_t iTOW;   // ms GPS time of week of the  navigation epoch . See the  description of iTOW
                     // for details.
    int32_t ecefVX;  // cm ECEF X velocity
    int32_t ecefVY;  // cm ECEF Y velocity
    int32_t ecefVZ;  // cm ECEF Z velocity
    uint32_t sAcc;   // cm Speed Accuracy Estimate
} __attribute__((packed)) NAV_VELECEF_t;

typedef struct
{
    uint16_t pending[6];   // bytes - # bytes pending in transmitter buffer for each target
    uint8_t usage[6];      // % - max usage transmitter buffer during the last sysmon period
    uint8_t peakUsage[6];  // % - maximum usage transmitter buffer for each target
    uint8_t tUsage;        // % - Maximum usage of trans. buffer during last sysmon for all targets
    uint8_t tPeakUsage;    // % - Maximum usage of trans. buffer for all targets
    uint8_t errors;        // Error bitmask
    uint8_t reserved;

    enum
    {
        ALLOC = 0x40,
        MEM = 0x20,
        LIMIT = 0x10
    };
} __attribute__((packed)) MON_TXBUF_t;

typedef struct
{
    uint8_t version;   // Message version (0x00 for this version)
    uint8_t nPorts;    // Number of ports included
    uint8_t txErrors;  // tx error bitmask (see graphic below)
    uint8_t reserved;
    uint8_t portIds[4];  // The identifiers of the protocols reported inthe msgs array. 0: UBX, 1:
                         // NMEA, 2:RTCM2, 5: RTCM3, 256: No protocolreported.
    struct repeated_block
    {
        uint16_t portId;       // Unique identifier for the port. See sectionCommunications ports in
                               // Integration manual for details.
        uint16_t txPending;    // Number of bytes pending in transmitterbuffer
        uint32_t txBytes;      // Number of bytes ever sent
        uint8_t txUsage;       // Maximum usage transmitter buffer duringthe last sysmon period
        uint8_t txPeakUsage;   // Maximum usage transmitter buffer
        uint16_t rxPending;    // Number of bytes in receiver buffer
        uint32_t rxBytes;      // Number of bytes ever received
        uint8_t rxUsage;       // Maximum usage receiver buffer during thelast sysmon period
        uint8_t rxPeakUsage;   // Maximum usage receiver buffer
        uint16_t overrunErrs;  // Number of 100ms timeslots with overrunerrors
        uint16_t msgs[4];  // Number of successfully parsed messagesfor each protocol. The reported
                           // protocols are identified through the portIds field
        uint8_t reserved[8];
        uint32_t skipped;  // number of skipped bytes
    };
    repeated_block blocks[5];

    enum
    {
        ALLOC = 0x02,
        MEM = 0x01
    };
} __attribute__((packed)) MON_COMMS_t;

typedef struct
{
    char swVersion[30];      // Zero-terminated Software Version String.
    char hwVersion[10];      // Zero-terminated Hardware Version String
    char extension[30][10];  // Extended software information strings
} __attribute__((packed)) MON_VER_t;

typedef float R4;
typedef double R8;
typedef int8_t I1;
typedef uint8_t U1;
typedef uint16_t U2;
typedef uint32_t U4;
typedef uint8_t X1;

typedef struct
{
    R8 rcvTow;  // Measurement time of week in receiver localtime approximately aligned to the GPS
                // timesystem. The receiver local time of week, week number and leap second
                // information can be used to translate the time to other time systems. More
                // information about the difference in time systems can be found in RINEX 3
                // documentation. For a receiver operating in GLONASS only mode, UTC time can be
                // determined by subtracting the leapS field from GPS time regardless of whether the
                // GPS leap seconds are valid.
    U2 week;    // GPS week number in receiver local time.
    I1 leapS;  // GPS leap seconds (GPS-UTC). This field represents the receiver's best knowledge of
               // the leap seconds offset. A flag is given in the recStat bitfield to indicate if
               // the leap seconds are known.
    I1 numMeas;  // number of measurements to follow
    X1 recStat;  // Receiver tracking status bitfield (see graphic below)
    U1 version;  // Message Version
    U1 reserved1[2];

    struct RawxMeas
    {
        R8 prMeas;  // Pseudorange measurement [m]. GLONASS inter-frequency channel delays are
                    // compensated with an internal calibration table.
        R8 cpMeas;  // Carrier phase measurement [cycles]. The carrierphase initial ambiguity is
                    // initialized using anapproximate value to make the magnitude of the phase
                    // close to the pseudorange measurement. Clock resets are applied to both phase
                    // and code measurements in accordance with the RINEX specification.
        R4 doMeas;  // Doppler measurement (positive sign forapproaching satellites) [Hz]
        U1 gnssId;  // GNSS identifier (see Satellite Numbering for a list of identifiers)
        U1 svId;    // Satellite identifier (see Satellite Numbering)
        U1 sigId;   // New style signal identifier (see Signal Identifiers)
        U1 freqId;  // Only used for GLONASS: This is the frequencyslot + 7 (range from 0 to 13)
        U2 locktime;  // Carrier phase locktime counter (maximum 64500ms)
        U1 cno;       // Carrier-to-noise density ratio (signal strength)[dB-Hz]
        X1 prStdev;   // (0.01*2^n)  Estimated pseudorange measurement standard deviation
        X1 cpStdev;   // (0.004)     Estimated carrier phase measurement standard deviation (note a
                      // raw value of 0x0F indicates thevalue is invalid) (see graphic below)
        X1 doStdev;   // (0.002*2^n) Estimated Doppler measurement standard deviation.
        X1 trkStat;   // Tracking status bitfield
        U1 reserved;
    };
    RawxMeas meas[30];

    enum
    {
        recStat_clkReset = 0b10,
        recStat_leapSec = 0b01,
    };
    enum
    {
        trkStat_subHalfCyc = 0b1000,
        trkStat_HalfCyc = 0b0100,
        trkStat_cpValid = 0b0010,
        trkStat_prValid = 0b0001,
    };
} __attribute__((packed)) RXM_RAWX_t;

enum
{
    GnssID_GPS = 0,
    GnssID_SBAS = 1,
    GnssID_Galileo = 2,
    GnssID_Beidou = 3,
    GnssID_Qzss = 5,
    GnssID_Glonass = 6
};

inline int sysId(int svId)
{
    //clang-format off
    return (svId >= 1 && svId <= 32)    ? GnssID_GPS
         : (svId >= 33 && svId <= 64)   ? GnssID_Beidou
         : (svId >= 65 && svId <= 96)   ? GnssID_Glonass
         : (svId >= 120 && svId <= 158) ? GnssID_SBAS
         : (svId >= 159 && svId <= 163) ? GnssID_Beidou
         : (svId >= 193 && svId <= 197) ? GnssID_Qzss
         : (svId >= 211 && svId <= 246) ? GnssID_Galileo
         : (svId == 255)                ? GnssID_Glonass 
         : -1;
    //clang-format on
}

enum
{
    GPS_L1_CA,
    GPS_L2_CL,
    GPS_L2_CM,
    Galileo_E1_C,
    Galileo_E1_B,
    Galileo_E5_BI,
    Galileo_E5_BQ,
    Beidou_B1I_D1,
    Beidou_B1I_D2,
    Beidou_B2I_D1,
    Beidou_B2I_D2,
    QZSS_L1_CA,
    Glonass_L1,
    Glonass_L2,
};

inline int sigId(int gnssId, int sigId)
{
    //clang-format off
    return (gnssId == 0 && sigId == 0)  ? GPS_L1_CA
         : (gnssId == 0 && sigId == 3)  ? GPS_L2_CL
         : (gnssId == 0 && sigId == 4) ? GPS_L2_CM
         : (gnssId == 2 && sigId == 0) ? Galileo_E1_C
         : (gnssId == 2 && sigId == 1) ? Galileo_E1_B
         : (gnssId == 2 && sigId == 5) ? Galileo_E5_BI
         : (gnssId == 2 && sigId == 6) ? Galileo_E5_BQ
         : (gnssId == 3 && sigId == 0) ? Beidou_B1I_D1
         : (gnssId == 3 && sigId == 1) ? Beidou_B1I_D2
         : (gnssId == 3 && sigId == 2) ? Beidou_B2I_D1
         : (gnssId == 3 && sigId == 3) ? Beidou_B2I_D2
         : (gnssId == 5 && sigId == 0) ? QZSS_L1_CA
         : (gnssId == 6 && sigId == 0) ? Glonass_L1
         : (gnssId == 6 && sigId == 2) ? Glonass_L2
         : -1;
    //clang-format on
}

typedef struct
{
    U1 gnssId;
    U1 svId;
    U1 reserved1;
    U1 freqId;
    U1 numWords;
    U1 chn;
    U1 version;
    U1 reserved2;
    U4 dwrd[10];
} __attribute__((packed)) RXM_SFRBX_t;

typedef union
{
    uint8_t buffer[BUFFER_SIZE];
    ACK_ACK_t ACK_ACK;
    ACK_NACK_t ACK_NACK;
    CFG_MSG_t CFG_MSG;
    CFG_PRT_t CFG_PRT;
    CFG_RATE_t CFG_RATE;
    CFG_NAV5_t CFG_NAV5;
    CFG_VALSET_t CFG_VALSET;
    CFG_VALGET_t CFG_VALGET;
    NAV_PVT_t NAV_PVT;
    NAV_POSECEF_t NAV_POSECEF;
    NAV_VELECEF_t NAV_VELECEF;
    NAV_RELPOSNED_t NAV_RELPOSNED;
    RXM_RAWX_t RXM_RAWX;
    RXM_SFRBX_t RXM_SFRBX;
    NAV_SVIN_t NAV_SVIN;
    MON_TXBUF_t MON_TXBUF;
    MON_VER_t MON_VER;
    MON_COMMS_t MON_COMM;
} UBX_message_t;

}  // namespace ublox
