#ifndef PACKET_H
#define PACKET_H


#include <Arduino.h>

namespace Telemetry {


constexpr uint8_t MAGIC_0 = 0xA5;
constexpr uint8_t MAGIC_1 = 0x5A;

constexpr uint8_t PROTOCOL_VERSION = 1;

enum PacketType : uint8_t {
    PACKET_FAST        = 0x01,
    PACKET_STATUS      = 0x02,
    PACKET_ORIENTATION = 0x03,
    PACKET_COMMAND     = 0x04,
    PACKET_ACK         = 0x05,
    PACKET_ALERT       = 0x06
};


constexpr uint8_t STATUS_RECORDING = 1 << 0;
constexpr uint8_t STATUS_5V_ACTIVE = 1 << 1;
constexpr uint8_t STATUS_GPS_FIX   = 1 << 2;
constexpr uint8_t STATUS_GPS_OK    = 1 << 3;
constexpr uint8_t STATUS_BARO_OK   = 1 << 4;
constexpr uint8_t STATUS_IMU_OK    = 1 << 5;
constexpr uint8_t STATUS_FLASH_OK  = 1 << 6;
constexpr uint8_t STATUS_LORA_OK   = 1 << 7;


constexpr size_t FAST_PACKET_SIZE = 40;


constexpr size_t STATUS_PACKET_SIZE = 32;


constexpr size_t ORIENTATION_PACKET_SIZE = 22;


constexpr size_t COMMAND_PACKET_SIZE = 14;

enum CommandId : uint8_t {
    CMD_CLEAR_FLASH         = 0x01,
    CMD_START_RECORDING     = 0x02,
    CMD_STOP_RECORDING      = 0x03,
    CMD_CALIBRATE_IMU       = 0x04,
    CMD_CALIBRATE_BAROMETER = 0x05,
    CMD_5V_ON               = 0x07,
    CMD_5V_OFF              = 0x08,
    CMD_BUZZER_MUTE         = 0x09,
    CMD_BUZZER_UNMUTE       = 0x0A,
    CMD_DVR_TRIGGER         = 0x0B
};


constexpr size_t ACK_PACKET_SIZE = 12;

enum AckResult : uint8_t {
    ACK_APPLIED = 0,
    ACK_REFUSED = 1,
    ACK_IGNORED = 2,
    ACK_UNKNOWN = 3
};


constexpr size_t ALERT_MESSAGE_LEN = 32;
constexpr size_t ALERT_PACKET_SIZE = 10 + 2 + ALERT_MESSAGE_LEN + 2;

enum AlertSeverity : uint8_t {
    ALERT_INFO     = 0,
    ALERT_WARNING  = 1,
    ALERT_CRITICAL = 2
};

enum AlertCode : uint8_t {
    ALERT_SENSOR_FAULT           = 1,
    ALERT_FLASH_FULL             = 2,
    ALERT_I2C_BUS_RECOVERED      = 3,
    ALERT_WATCHDOG_REBOOT        = 4,
    ALERT_RECORDING_AUTO_STOPPED = 5,
    ALERT_BROWNOUT_REBOOT        = 6,
    ALERT_RADIO_STUCK_RECOVERED  = 7
};


struct FastData {
    int32_t latitude_e6;
    int32_t longitude_e6;

    int16_t altitude_cm;

    int16_t accel_x_mg;
    int16_t accel_y_mg;
    int16_t accel_z_mg;

    int16_t gyro_x_cdeg;
    int16_t gyro_y_cdeg;
    int16_t gyro_z_cdeg;

    uint32_t pressure_pa;

    uint8_t status;
    uint8_t gps_satellites;
};

struct StatusData {
    uint16_t battery_mv;

    uint16_t flash_percent;

    int8_t lora_rssi;
    int8_t lora_snr;

    uint8_t gps_satellites;
    uint8_t status;

    uint32_t flash_bytes_written;
    uint32_t flash_errors;

    uint32_t gps_unix_time;
};

struct OrientationData {
    int16_t quat_w;
    int16_t quat_x;
    int16_t quat_y;
    int16_t quat_z;

    uint8_t calibration;
    uint8_t reserved;
};

struct CommandData {
    uint16_t sequence;
    uint8_t command;
    uint32_t param;
};

struct AckData {
    uint16_t acked_sequence;
    uint8_t acked_command;
    uint8_t result;
};

struct AlertData {
    uint8_t code;
    uint8_t severity;
    char message[ALERT_MESSAGE_LEN];
};


inline void writeU16(uint8_t* buffer, size_t& index, uint16_t value)
{
    buffer[index++] = value & 0xFF;
    buffer[index++] = (value >> 8) & 0xFF;
}

inline void writeI16(uint8_t* buffer, size_t& index, int16_t value)
{
    writeU16(buffer, index, static_cast<uint16_t>(value));
}

inline void writeU32(uint8_t* buffer, size_t& index, uint32_t value)
{
    buffer[index++] = value & 0xFF;
    buffer[index++] = (value >> 8) & 0xFF;
    buffer[index++] = (value >> 16) & 0xFF;
    buffer[index++] = (value >> 24) & 0xFF;
}

inline void writeI32(uint8_t* buffer, size_t& index, int32_t value)
{
    writeU32(buffer, index, static_cast<uint32_t>(value));
}

inline uint16_t readU16(const uint8_t* buffer, size_t offset)
{
    return static_cast<uint16_t>(buffer[offset]) |
           (static_cast<uint16_t>(buffer[offset + 1]) << 8);
}

inline uint32_t readU32(const uint8_t* buffer, size_t offset)
{
    return static_cast<uint32_t>(buffer[offset]) |
           (static_cast<uint32_t>(buffer[offset + 1]) << 8) |
           (static_cast<uint32_t>(buffer[offset + 2]) << 16) |
           (static_cast<uint32_t>(buffer[offset + 3]) << 24);
}


inline uint16_t crc16(const uint8_t* data, size_t length)
{
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < length; i++) {

        crc ^= static_cast<uint16_t>(data[i]) << 8;

        for (uint8_t bit = 0; bit < 8; bit++) {

            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            }
            else {
                crc <<= 1;
            }
        }
    }

    return crc;
}


inline size_t serializeFast(
    const FastData& data,
    uint16_t sequence,
    uint32_t timestamp,
    uint8_t* buffer
) {
    size_t i = 0;

    buffer[i++] = MAGIC_0;
    buffer[i++] = MAGIC_1;

    buffer[i++] = PROTOCOL_VERSION;
    buffer[i++] = PACKET_FAST;

    writeU16(buffer, i, sequence);

    writeU32(buffer, i, timestamp);

    writeI32(buffer, i, data.latitude_e6);
    writeI32(buffer, i, data.longitude_e6);

    writeI16(buffer, i, data.altitude_cm);

    writeI16(buffer, i, data.accel_x_mg);
    writeI16(buffer, i, data.accel_y_mg);
    writeI16(buffer, i, data.accel_z_mg);

    writeI16(buffer, i, data.gyro_x_cdeg);
    writeI16(buffer, i, data.gyro_y_cdeg);
    writeI16(buffer, i, data.gyro_z_cdeg);

    writeU32(buffer, i, data.pressure_pa);

    buffer[i++] = data.status;
    buffer[i++] = data.gps_satellites;

    uint16_t crc = crc16(buffer, i);

    writeU16(buffer, i, crc);

#ifdef TELEMETRY_DEBUG_ASSERTS
    if (i != FAST_PACKET_SIZE) {
        Serial.print("PACKET SIZE MISMATCH (fast): expected ");
        Serial.print(FAST_PACKET_SIZE);
        Serial.print(", got ");
        Serial.println(i);
    }
#endif

    return i;
}


inline size_t serializeStatus(
    const StatusData& data,
    uint16_t sequence,
    uint32_t timestamp,
    uint8_t* buffer
) {
    size_t i = 0;

    buffer[i++] = MAGIC_0;
    buffer[i++] = MAGIC_1;

    buffer[i++] = PROTOCOL_VERSION;
    buffer[i++] = PACKET_STATUS;

    writeU16(buffer, i, sequence);

    writeU32(buffer, i, timestamp);

    writeU16(buffer, i, data.battery_mv);
    writeU16(buffer, i, data.flash_percent);

    buffer[i++] = static_cast<uint8_t>(data.lora_rssi);
    buffer[i++] = static_cast<uint8_t>(data.lora_snr);

    buffer[i++] = data.gps_satellites;
    buffer[i++] = data.status;

    writeU32(buffer, i, data.flash_bytes_written);
    writeU32(buffer, i, data.flash_errors);

    writeU32(buffer, i, data.gps_unix_time);

    uint16_t crc = crc16(buffer, i);

    writeU16(buffer, i, crc);

#ifdef TELEMETRY_DEBUG_ASSERTS
    if (i != STATUS_PACKET_SIZE) {
        Serial.print("PACKET SIZE MISMATCH (status): expected ");
        Serial.print(STATUS_PACKET_SIZE);
        Serial.print(", got ");
        Serial.println(i);
    }
#endif

    return i;
}


inline size_t serializeOrientation(
    const OrientationData& data,
    uint16_t sequence,
    uint32_t timestamp,
    uint8_t* buffer
) {
    size_t i = 0;

    buffer[i++] = MAGIC_0;
    buffer[i++] = MAGIC_1;

    buffer[i++] = PROTOCOL_VERSION;
    buffer[i++] = PACKET_ORIENTATION;

    writeU16(buffer, i, sequence);

    writeU32(buffer, i, timestamp);

    writeI16(buffer, i, data.quat_w);
    writeI16(buffer, i, data.quat_x);
    writeI16(buffer, i, data.quat_y);
    writeI16(buffer, i, data.quat_z);

    buffer[i++] = data.calibration;
    buffer[i++] = data.reserved;

    uint16_t crc = crc16(buffer, i);

    writeU16(buffer, i, crc);

#ifdef TELEMETRY_DEBUG_ASSERTS
    if (i != ORIENTATION_PACKET_SIZE) {
        Serial.print("PACKET SIZE MISMATCH (orientation): expected ");
        Serial.print(ORIENTATION_PACKET_SIZE);
        Serial.print(", got ");
        Serial.println(i);
    }
#endif

    return i;
}


inline size_t serializeCommand(
    const CommandData& data,
    uint16_t sequence,
    uint8_t* buffer
) {
    size_t i = 0;

    buffer[i++] = MAGIC_0;
    buffer[i++] = MAGIC_1;

    buffer[i++] = PROTOCOL_VERSION;
    buffer[i++] = PACKET_COMMAND;

    writeU16(buffer, i, sequence);

    buffer[i++] = data.command;
    buffer[i++] = 0;

    writeU32(buffer, i, data.param);

    uint16_t crc = crc16(buffer, i);

    writeU16(buffer, i, crc);

#ifdef TELEMETRY_DEBUG_ASSERTS
    if (i != COMMAND_PACKET_SIZE) {
        Serial.print("PACKET SIZE MISMATCH (command): expected ");
        Serial.print(COMMAND_PACKET_SIZE);
        Serial.print(", got ");
        Serial.println(i);
    }
#endif

    return i;
}


inline bool parseCommand(const uint8_t* buffer, size_t length, CommandData& out)
{
    if (length != COMMAND_PACKET_SIZE) {
        return false;
    }
    if (buffer[0] != MAGIC_0 || buffer[1] != MAGIC_1) {
        return false;
    }
    if (buffer[2] != PROTOCOL_VERSION) {
        return false;
    }
    if (buffer[3] != PACKET_COMMAND) {
        return false;
    }

    uint16_t crc = crc16(buffer, length - 2);
    if (crc != readU16(buffer, length - 2)) {
        return false;
    }

    out.sequence = readU16(buffer, 4);
    out.command = buffer[6];
    out.param = readU32(buffer, 8);
    return true;
}


inline size_t serializeAck(
    const AckData& data,
    uint16_t sequence,
    uint8_t* buffer
) {
    size_t i = 0;

    buffer[i++] = MAGIC_0;
    buffer[i++] = MAGIC_1;

    buffer[i++] = PROTOCOL_VERSION;
    buffer[i++] = PACKET_ACK;

    writeU16(buffer, i, sequence);

    writeU16(buffer, i, data.acked_sequence);
    buffer[i++] = data.acked_command;
    buffer[i++] = data.result;

    uint16_t crc = crc16(buffer, i);

    writeU16(buffer, i, crc);

#ifdef TELEMETRY_DEBUG_ASSERTS
    if (i != ACK_PACKET_SIZE) {
        Serial.print("PACKET SIZE MISMATCH (ack): expected ");
        Serial.print(ACK_PACKET_SIZE);
        Serial.print(", got ");
        Serial.println(i);
    }
#endif

    return i;
}


inline size_t serializeAlert(
    const AlertData& data,
    uint16_t sequence,
    uint32_t timestamp,
    uint8_t* buffer
) {
    size_t i = 0;

    buffer[i++] = MAGIC_0;
    buffer[i++] = MAGIC_1;

    buffer[i++] = PROTOCOL_VERSION;
    buffer[i++] = PACKET_ALERT;

    writeU16(buffer, i, sequence);

    writeU32(buffer, i, timestamp);

    buffer[i++] = data.code;
    buffer[i++] = data.severity;

    for (size_t m = 0; m < ALERT_MESSAGE_LEN; m++) {
        buffer[i++] = static_cast<uint8_t>(data.message[m]);
    }

    uint16_t crc = crc16(buffer, i);

    writeU16(buffer, i, crc);

#ifdef TELEMETRY_DEBUG_ASSERTS
    if (i != ALERT_PACKET_SIZE) {
        Serial.print("PACKET SIZE MISMATCH (alert): expected ");
        Serial.print(ALERT_PACKET_SIZE);
        Serial.print(", got ");
        Serial.println(i);
    }
#endif

    return i;
}

}

#endif
