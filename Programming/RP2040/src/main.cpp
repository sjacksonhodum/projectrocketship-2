#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM6DS.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_MAX1704X.h>
#include <RH_RF95.h>
#include "hardware/watchdog.h"

#define TELEMETRY_DEBUG_ASSERTS

#include "packet.h"


constexpr uint32_t CHECKPOINT_MAGIC = 0x43503031;

enum Checkpoint : uint32_t {
  CP_LOOP_TOP           = 1,
  CP_RADIO_COMMAND      = 2,
  CP_SENSOR_UPDATE      = 3,
  CP_RECORDING_START    = 4,
  CP_RECORDING_PROMOTE  = 5,
  CP_FLASH_DATA_WRITE   = 6,
  CP_FLASH_HEADER_WRITE = 7,
  CP_RADIO_AVAILABLE    = 8,
  CP_RADIO_RECV         = 9,
  CP_RADIO_SEND         = 10,
  CP_SERIAL_COMMAND     = 11,
  CP_DVR_PULSE          = 12,
  CP_UPDATE_GPS         = 13,
  CP_UPDATE_IMU         = 14,
  CP_UPDATE_BAROMETER   = 15,
  CP_UPDATE_POWER       = 16,
  CP_UPDATE_FLASH       = 17,
  CP_UPDATE_LORA        = 18,
  CP_SENSOR_RETRY       = 19,
  CP_FIVE_V_ENABLE      = 20,
  CP_RETRY_CHECK        = 21,
  CP_I2C_RECOVERY_CHECK = 22,
  CP_RADIO_STUCK_CHECK  = 23,
  CP_LAUNCH_DETECT      = 24,
  CP_RECORDING_ARM      = 25,
  CP_FIVE_V_AUTOOFF     = 26,
  CP_DVR_SERVICE        = 27,
  CP_BUZZER_UPDATE      = 28,
  CP_NEOPIXEL_UPDATE    = 29,
  CP_TX_POWER_RAMP      = 30,
  CP_SEND_SCHEDULE      = 31
};

inline const char* checkpointName(uint32_t cp) {
  switch (cp) {
    case CP_LOOP_TOP:           return "loop top";
    case CP_RADIO_COMMAND:      return "radio/serial command check";
    case CP_SENSOR_UPDATE:      return "sensor update";
    case CP_RECORDING_START:    return "recording start (DVR pulse)";
    case CP_RECORDING_PROMOTE:  return "recording promote (3s arm elapsed)";
    case CP_FLASH_DATA_WRITE:   return "flash data write";
    case CP_FLASH_HEADER_WRITE: return "flash header write";
    case CP_RADIO_AVAILABLE:    return "radio available() check";
    case CP_RADIO_RECV:         return "radio recv()";
    case CP_RADIO_SEND:         return "radio send()";
    case CP_SERIAL_COMMAND:     return "serial command received";
    case CP_DVR_PULSE:          return "DVR button-press pulse";
    case CP_UPDATE_GPS:         return "updateGPS()";
    case CP_UPDATE_IMU:         return "updateIMU()";
    case CP_UPDATE_BAROMETER:   return "updateBarometer()";
    case CP_UPDATE_POWER:       return "updatePower()";
    case CP_UPDATE_FLASH:       return "updateFlash()";
    case CP_UPDATE_LORA:        return "updateLoRa()";
    case CP_SENSOR_RETRY:       return "sensor retry (IMU/baro re-probe)";
    case CP_FIVE_V_ENABLE:      return "5V enable (sensor board re-probe)";
    case CP_RETRY_CHECK:        return "retryFailedSensorBoardDevices() check";
    case CP_I2C_RECOVERY_CHECK: return "checkI2cBusRecovery()";
    case CP_RADIO_STUCK_CHECK:  return "checkRadioStuckInTx()";
    case CP_LAUNCH_DETECT:      return "updateLaunchDetect()";
    case CP_RECORDING_ARM:      return "updateRecordingArm()";
    case CP_FIVE_V_AUTOOFF:     return "updateFiveVAutoOff()";
    case CP_DVR_SERVICE:        return "serviceDvrTrigger()";
    case CP_BUZZER_UPDATE:      return "buzzer update";
    case CP_NEOPIXEL_UPDATE:    return "updateNeoPixels()";
    case CP_TX_POWER_RAMP:      return "TX power ramp step";
    case CP_SEND_SCHEDULE:      return "FAST/STATUS/ORIENTATION send schedule";
    default:                    return "none/unknown";
  }
}

bool verboseCheckpoints = false;

inline void setCheckpoint(Checkpoint cp) {
  watchdog_hw->scratch[0] = static_cast<uint32_t>(cp);
  watchdog_hw->scratch[1] = CHECKPOINT_MAGIC;

  if (verboseCheckpoints) {
    Serial.print("CP ");
    Serial.print(static_cast<uint32_t>(cp));
    Serial.print(" @");
    Serial.print(millis());
    Serial.print("ms: ");
    Serial.println(checkpointName(cp));
  }
}

inline uint32_t lastCheckpointBeforeThisBoot() {
  if (watchdog_hw->scratch[1] != CHECKPOINT_MAGIC) {
    return 0;
  }
  return watchdog_hw->scratch[0];
}

inline void setLastTxPowerScratch(int8_t dbm) {
  watchdog_hw->scratch[2] = static_cast<uint32_t>(dbm);
}

inline int8_t lastTxPowerBeforeThisBoot() {
  if (watchdog_hw->scratch[1] != CHECKPOINT_MAGIC) {
    return -128;
  }
  return static_cast<int8_t>(watchdog_hw->scratch[2]);
}

inline const char* resetReasonName(RP2040::resetReason_t reason) {
  switch (reason) {
    case RP2040::PWRON_RESET:    return "PWRON_RESET (genuine power-on)";
    case RP2040::RUN_PIN_RESET:  return "RUN_PIN_RESET (reset line pulsed - check for a loose/flaky connector or wiring)";
    case RP2040::SOFT_RESET:     return "SOFT_RESET (software-requested reset)";
    case RP2040::WDT_RESET:      return "WDT_RESET (watchdog timeout - see checkpoint above)";
    case RP2040::DEBUG_RESET:    return "DEBUG_RESET (debugger/picotool reset)";
    case RP2040::GLITCH_RESET:   return "GLITCH_RESET (core voltage glitch detector tripped)";
    case RP2040::BROWNOUT_RESET: return "BROWNOUT_RESET (voltage dip)";
    default:                     return "UNKNOWN_RESET";
  }
}


constexpr uint32_t GPS_BAUD = 115200;

constexpr uint32_t FAST_INTERVAL_MS = 125;
constexpr uint32_t STATUS_INTERVAL_MS = 2000;
constexpr uint32_t ORIENTATION_INTERVAL_MS = 500;



Adafruit_LSM6DSOX lsm6dso;


constexpr uint8_t GPS_SERIAL_TX_PIN = 12;
constexpr uint8_t GPS_SERIAL_RX_PIN = 13;


constexpr uint8_t RFM95_SCK_PIN = 2;
constexpr uint8_t RFM95_MOSI_PIN = 3;
constexpr uint8_t RFM95_MISO_PIN = 4;
constexpr uint8_t RFM95_CS = 6;
constexpr uint8_t RFM95_RST = 10;
constexpr uint8_t RFM95_INT = 7;

constexpr uint8_t LORA_EN_PIN = 11;

constexpr float RF95_FREQ_MHZ = 433.0;

constexpr uint8_t RF95_SPREADING_FACTOR = 8;
constexpr long RF95_BANDWIDTH_HZ = 250000;
constexpr uint8_t RF95_CODING_RATE_DENOM = 5;
constexpr int8_t RF95_TX_POWER_DBM = 20;

RH_RF95 rf95(RFM95_CS, RFM95_INT);


constexpr uint8_t GPIO1_ALWAYS_HIGH_PIN = 1;


constexpr uint8_t FIVE_V_SWITCH_PIN = 21;

bool fiveVEnabled = true;


constexpr uint8_t DVR_TRIGGER_PIN = 22;


Adafruit_MAX17048 maxlipo;


constexpr uint8_t BUZZER_PIN = 15;


constexpr uint8_t NEOPIXEL_PIN = 25;
constexpr uint8_t NEOPIXEL_COUNT = 4;
constexpr uint8_t NEOPIXEL_BRIGHTNESS = 110;

Adafruit_NeoPixel pixels(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);


struct TelemetryState {
  double latitude = 0.0;
  double longitude = 0.0;
  double gpsAltitude = 0.0;
  uint8_t gpsSatellites = 0;
  bool gpsFix = false;
  bool gpsOK = false;

  uint32_t gpsUnixTime = 0;

  float accelX = 0.0;
  float accelY = 0.0;
  float accelZ = 0.0;
  float gyroX = 0.0;
  float gyroY = 0.0;
  float gyroZ = 0.0;
  bool imuOK = false;

  float baroAltitude = 0.0;
  uint32_t baroPressure = 101325;
  bool baroOK = false;

  uint16_t batteryMV = 0;
  bool fiveVActive = true;
  bool fuelGaugeOK = false;

  bool recording = false;
  bool flashOK = false;
  bool loraOK = false;
  uint16_t flashPercent = 0;
  uint32_t flashBytesWritten = 0;
  uint32_t flashErrors = 0;
  int8_t loraRSSI = -127;
  int8_t loraSNR = 0;
};

TelemetryState telemetry;

bool anyCriticalSensorFault() {
  return !telemetry.imuOK || !telemetry.baroOK || !telemetry.fuelGaugeOK;
}


uint16_t fastSequence = 0;
uint16_t statusSequence = 0;
uint16_t orientationSequence = 0;
uint16_t ackSequence = 0;
uint16_t alertSequence = 0;

bool verboseRawPackets = false;

uint8_t packetBuffer[64];


void processUBXByte(uint8_t b);
extern uint8_t ubxClass;
extern uint8_t ubxId;
extern uint8_t ubxPayload[];
extern bool ubxLastMessageIsFresh;

void sendUBX(uint8_t msgClass, uint8_t msgId, const uint8_t* payload, uint16_t length) {
  uint8_t ckA = 0;
  uint8_t ckB = 0;

  auto checksum = [&](uint8_t b) {
    ckA = static_cast<uint8_t>(ckA + b);
    ckB = static_cast<uint8_t>(ckB + ckA);
  };

  Serial1.write(0xB5);
  Serial1.write(0x62);

  Serial1.write(msgClass);
  checksum(msgClass);
  Serial1.write(msgId);
  checksum(msgId);

  uint8_t lenLow = length & 0xFF;
  uint8_t lenHigh = (length >> 8) & 0xFF;
  Serial1.write(lenLow);
  checksum(lenLow);
  Serial1.write(lenHigh);
  checksum(lenHigh);

  for (uint16_t i = 0; i < length; i++) {
    Serial1.write(payload[i]);
    checksum(payload[i]);
  }

  Serial1.write(ckA);
  Serial1.write(ckB);
}

bool waitForUBXAck(uint8_t msgClass, uint8_t msgId, uint32_t timeoutMs) {
  uint32_t deadline = millis() + timeoutMs;

  while (millis() < deadline) {
    while (Serial1.available() > 0) {
      processUBXByte(static_cast<uint8_t>(Serial1.read()));

      if (ubxLastMessageIsFresh) {
        ubxLastMessageIsFresh = false;

        if (ubxClass == 0x05 && ubxPayload[0] == msgClass && ubxPayload[1] == msgId) {
          return ubxId == 0x01;
        }
      }
    }

    __asm volatile("wfi");
  }

  return false;
}

void sendUBXConfig(const char* label, uint8_t msgClass, uint8_t msgId,
                    const uint8_t* payload, uint16_t length) {
  constexpr uint8_t maxAttempts = 5;

  for (uint8_t attempt = 1; attempt <= maxAttempts; attempt++) {
    sendUBX(msgClass, msgId, payload, length);

    if (waitForUBXAck(msgClass, msgId, 250)) {
      Serial.print("  ");
      Serial.print(label);
      Serial.println(": ACKed");
      return;
    }
  }

  Serial.print("  ");
  Serial.print(label);
  Serial.println(
      ": NOT acknowledged after 5 attempts - module isn't responding to "
      "our commands at all (check GP0 -> GPS RX wiring) or rejected it "
      "(NAK). GPS behavior is unconfigured/unknown from here.");
}

void configureGPSRate() {
  uint8_t payload[6] = {
      0x64, 0x00,
      0x01, 0x00,
      0x01, 0x00,
  };

  sendUBXConfig("CFG-RATE 10Hz", 0x06, 0x08, payload, sizeof(payload));
}

void setMessageRate(const char* label, uint8_t msgClass, uint8_t msgId, uint8_t rate) {
  uint8_t payload[3] = {msgClass, msgId, rate};
  sendUBXConfig(label, 0x06, 0x01, payload, sizeof(payload));
}

void configureGPSProtocol() {
  constexpr uint8_t NMEA_GGA = 0x00;
  constexpr uint8_t NMEA_GLL = 0x01;
  constexpr uint8_t NMEA_GSA = 0x02;
  constexpr uint8_t NMEA_GSV = 0x03;
  constexpr uint8_t NMEA_RMC = 0x04;
  constexpr uint8_t NMEA_VTG = 0x05;

  setMessageRate("CFG-MSG NMEA-GGA off", 0xF0, NMEA_GGA, 0);
  setMessageRate("CFG-MSG NMEA-GLL off", 0xF0, NMEA_GLL, 0);
  setMessageRate("CFG-MSG NMEA-GSA off", 0xF0, NMEA_GSA, 0);
  setMessageRate("CFG-MSG NMEA-GSV off", 0xF0, NMEA_GSV, 0);
  setMessageRate("CFG-MSG NMEA-RMC off", 0xF0, NMEA_RMC, 0);
  setMessageRate("CFG-MSG NMEA-VTG off", 0xF0, NMEA_VTG, 0);

  setMessageRate("CFG-MSG NAV-PVT on", 0x01, 0x07, 1);
}


constexpr uint16_t UBX_NAV_PVT_LENGTH = 92;
constexpr uint16_t UBX_PAYLOAD_BUFFER_SIZE = 92;

enum class UBXState : uint8_t {
  SYNC1,
  SYNC2,
  CLASS,
  ID,
  LEN1,
  LEN2,
  PAYLOAD,
  CK_A,
  CK_B,
};

UBXState ubxState = UBXState::SYNC1;
uint8_t ubxClass = 0;
uint8_t ubxId = 0;
uint16_t ubxLength = 0;
uint16_t ubxPayloadIndex = 0;
uint8_t ubxPayload[UBX_PAYLOAD_BUFFER_SIZE];
uint8_t ubxCkA = 0;
uint8_t ubxCkB = 0;

bool ubxEverValid = false;
uint32_t lastUBXValidMs = 0;
uint32_t ubxChecksumFailures = 0;
uint32_t ubxBytesReceived = 0;
uint32_t ubxValidMessageCount = 0;
uint32_t navPvtValidCount = 0;

bool ubxLastMessageIsFresh = false;

int32_t readI32LE(const uint8_t* buffer, uint16_t offset) {
  uint32_t value =
      static_cast<uint32_t>(buffer[offset]) |
      (static_cast<uint32_t>(buffer[offset + 1]) << 8) |
      (static_cast<uint32_t>(buffer[offset + 2]) << 16) |
      (static_cast<uint32_t>(buffer[offset + 3]) << 24);
  return static_cast<int32_t>(value);
}

uint32_t civilUTCToUnixTime(int y, unsigned m, unsigned d, unsigned hh, unsigned mm, unsigned ss) {
  y -= m <= 2;
  const int era = (y >= 0 ? y : y - 399) / 400;
  const unsigned yoe = static_cast<unsigned>(y - era * 400);
  const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  const int32_t days = era * 146097 + static_cast<int32_t>(doe) - 719468;

  return static_cast<uint32_t>(days) * 86400UL + hh * 3600UL + mm * 60UL + ss;
}

void handleNavPvt(const uint8_t* payload) {
  navPvtValidCount++;

  uint16_t utcYear = static_cast<uint16_t>(payload[4] | (payload[5] << 8));
  uint8_t utcMonth = payload[6];
  uint8_t utcDay = payload[7];
  uint8_t utcHour = payload[8];
  uint8_t utcMin = payload[9];
  uint8_t utcSec = payload[10];
  uint8_t validFlags = payload[11];
  bool utcDateTimeValid = (validFlags & 0x03) == 0x03;

  if (utcDateTimeValid && utcYear >= 2020 && utcYear <= 2099) {
    telemetry.gpsUnixTime =
        civilUTCToUnixTime(utcYear, utcMonth, utcDay, utcHour, utcMin, utcSec);
  }

  uint8_t fixType = payload[20];
  uint8_t flags = payload[21];
  bool gnssFixOK = (flags & 0x01) != 0;
  uint8_t numSV = payload[23];

  int32_t lonRaw = readI32LE(payload, 24);
  int32_t latRaw = readI32LE(payload, 28);
  int32_t hMSLRaw = readI32LE(payload, 36);

  telemetry.gpsSatellites = static_cast<uint8_t>(numSV <= 50 ? numSV : 0);

  bool fixGood = gnssFixOK && fixType >= 3;
  telemetry.gpsFix = fixGood;

  if (!fixGood) {
    return;
  }

  double lat = latRaw / 1.0e7;
  double lon = lonRaw / 1.0e7;
  double altitude = hMSLRaw / 1000.0;

  if (lat >= -90.0 && lat <= 90.0 && lon >= -180.0 && lon <= 180.0) {
    telemetry.latitude = lat;
    telemetry.longitude = lon;
  }
  if (altitude >= -1000.0 && altitude <= 10000.0) {
    telemetry.gpsAltitude = altitude;
  }
}

void processUBXByte(uint8_t b) {
  switch (ubxState) {
    case UBXState::SYNC1:
      ubxState = (b == 0xB5) ? UBXState::SYNC2 : UBXState::SYNC1;
      break;

    case UBXState::SYNC2:
      ubxState = (b == 0x62) ? UBXState::CLASS : UBXState::SYNC1;
      break;

    case UBXState::CLASS:
      ubxClass = b;
      ubxCkA = b;
      ubxCkB = b;
      ubxState = UBXState::ID;
      break;

    case UBXState::ID:
      ubxId = b;
      ubxCkA = static_cast<uint8_t>(ubxCkA + b);
      ubxCkB = static_cast<uint8_t>(ubxCkB + ubxCkA);
      ubxState = UBXState::LEN1;
      break;

    case UBXState::LEN1:
      ubxLength = b;
      ubxCkA = static_cast<uint8_t>(ubxCkA + b);
      ubxCkB = static_cast<uint8_t>(ubxCkB + ubxCkA);
      ubxState = UBXState::LEN2;
      break;

    case UBXState::LEN2:
      ubxLength |= (static_cast<uint16_t>(b) << 8);
      ubxCkA = static_cast<uint8_t>(ubxCkA + b);
      ubxCkB = static_cast<uint8_t>(ubxCkB + ubxCkA);
      ubxPayloadIndex = 0;
      ubxState = (ubxLength == 0) ? UBXState::CK_A : UBXState::PAYLOAD;
      break;

    case UBXState::PAYLOAD:
      if (ubxPayloadIndex < UBX_PAYLOAD_BUFFER_SIZE) {
        ubxPayload[ubxPayloadIndex] = b;
      }
      ubxCkA = static_cast<uint8_t>(ubxCkA + b);
      ubxCkB = static_cast<uint8_t>(ubxCkB + ubxCkA);
      ubxPayloadIndex++;
      if (ubxPayloadIndex >= ubxLength) {
        ubxState = UBXState::CK_A;
      }
      break;

    case UBXState::CK_A:
      if (b != ubxCkA) {
        ubxChecksumFailures++;
        ubxState = UBXState::SYNC1;
        break;
      }
      ubxState = UBXState::CK_B;
      break;

    case UBXState::CK_B:
      if (b == ubxCkB) {
        ubxEverValid = true;
        lastUBXValidMs = millis();
        ubxLastMessageIsFresh = true;
        ubxValidMessageCount++;

        if (ubxClass == 0x01 && ubxId == 0x07 && ubxLength == UBX_NAV_PVT_LENGTH) {
          handleNavPvt(ubxPayload);
        }
      } else {
        ubxChecksumFailures++;
      }
      ubxState = UBXState::SYNC1;
      break;
  }
}

void updateGPS() {
  while (Serial1.available() > 0) {
    ubxBytesReceived++;
    processUBXByte(static_cast<uint8_t>(Serial1.read()));
  }

  telemetry.gpsOK = ubxEverValid;

  if (!ubxEverValid || (millis() - lastUBXValidMs) >= 5000) {
    telemetry.gpsOK = false;
    telemetry.gpsFix = false;
  }
}


uint8_t BUZZER_VOLUME = 10;
constexpr uint16_t BUZZER_FREQ_HZ = 2000;

struct BeepStep {
  uint16_t onMs;
  uint16_t offMs;
};

const BeepStep BEEP_BOOT[] = {{800, 0}};
const BeepStep BEEP_GPS_LOCK[] = {{120, 120}, {120, 0}};
const BeepStep BEEP_RECORDING_START[] = {{100, 100}, {100, 100}, {100, 0}};
const BeepStep BEEP_RECORDING_STOP[] =
    {{100, 100}, {100, 100}, {100, 100}, {100, 100}, {100, 0}};

const BeepStep BEEP_HEARTBEAT_IDLE[] = {{60, 940}};
const BeepStep BEEP_HEARTBEAT_RECORDING[] = {{50, 150}};
const BeepStep BEEP_FAULT[] = {{80, 150}};
const BeepStep BEEP_RECOVERY[] = {{700, 700}};

enum class BuzzerPhase : uint8_t { IDLE, TONE_ON, TONE_OFF };

bool buzzerMuted = false;

const BeepStep* buzzerPattern = nullptr;
uint8_t buzzerStepCount = 0;
uint8_t buzzerStepIndex = 0;
bool buzzerRepeating = false;
bool buzzerEventActive = false;
BuzzerPhase buzzerPhase = BuzzerPhase::IDLE;
uint32_t buzzerPhaseStartMs = 0;

void buzzerEnterPhase(BuzzerPhase phase, uint32_t now) {
  buzzerPhase = phase;
  buzzerPhaseStartMs = now;
  if (phase == BuzzerPhase::TONE_ON) {
    analogWrite(BUZZER_PIN, buzzerMuted ? 0 : BUZZER_VOLUME);
  } else {
    analogWrite(BUZZER_PIN, 0);
  }
}

void buzzerSetPattern(const BeepStep* steps, uint8_t count, bool repeating, bool isEvent) {
  if (!isEvent && buzzerPattern == steps) {
    return;
  }
  buzzerPattern = steps;
  buzzerStepCount = count;
  buzzerStepIndex = 0;
  buzzerRepeating = repeating;
  buzzerEventActive = isEvent;
  buzzerEnterPhase(BuzzerPhase::TONE_ON, millis());
}

template <size_t N>
void triggerEventBeep(const BeepStep (&steps)[N]) {
  buzzerSetPattern(steps, N, false, true);
}

void setBuzzerMuted(bool muted) {
  buzzerMuted = muted;
  if (muted && buzzerPhase == BuzzerPhase::TONE_ON) {
    analogWrite(BUZZER_PIN, 0);
  }
}

void updateBuzzer() {
  if (buzzerPattern == nullptr || buzzerPhase == BuzzerPhase::IDLE) {
    return;
  }

  uint32_t now = millis();
  const BeepStep& step = buzzerPattern[buzzerStepIndex];
  uint32_t elapsed = now - buzzerPhaseStartMs;

  if (buzzerPhase == BuzzerPhase::TONE_ON) {
    if (elapsed >= step.onMs) {
      buzzerEnterPhase(BuzzerPhase::TONE_OFF, now);
    }
    return;
  }

  if (elapsed >= step.offMs) {
    buzzerStepIndex++;
    if (buzzerStepIndex >= buzzerStepCount) {
      if (buzzerRepeating) {
        buzzerStepIndex = 0;
      } else {
        buzzerPattern = nullptr;
        buzzerPhase = BuzzerPhase::IDLE;
        buzzerEventActive = false;
        analogWrite(BUZZER_PIN, 0);
        return;
      }
    }
    buzzerEnterPhase(BuzzerPhase::TONE_ON, now);
  }
}


constexpr float ACCEL_LAUNCH_THRESHOLD_G = 3.0f;
constexpr uint32_t LAUNCH_SUSTAIN_MS = 150;
constexpr uint32_t MAX_RECORDING_DURATION_MS = 10UL * 60UL * 1000UL;
constexpr float LANDED_ACCEL_LOW_G = 0.7f;
constexpr float LANDED_ACCEL_HIGH_G = 1.3f;
constexpr uint32_t LANDED_SUSTAIN_MS = 5000;
constexpr uint32_t MIN_RECORDING_BEFORE_LANDING_CHECK_MS = 3000;
constexpr uint32_t RECOVERY_BEACON_GRACE_MS = 30000;

constexpr uint32_t MANUAL_OVERRIDE_COOLDOWN_MS = 10000;
uint32_t manualOverrideUntilMs = 0;

void noteManualRecordingOverride(uint32_t now) {
  manualOverrideUntilMs = now + MANUAL_OVERRIDE_COOLDOWN_MS;
}

uint32_t launchCandidateStartMs = 0;
uint32_t recordingStartMs = 0;
uint32_t landedCandidateStartMs = 0;
uint32_t recordingStoppedAtMs = 0;
bool recoveryBeaconActive = false;


constexpr uint32_t DVR_TRIGGER_GROUND_MS = 1000;
constexpr uint32_t DVR_TRIGGER_MIN_GAP_MS = 1200;

bool dvrPulseActive = false;
uint32_t dvrPulseEndAt = 0;
uint32_t dvrNextPulseAt = 0;

void setDvrFloating() {
  pinMode(DVR_TRIGGER_PIN, INPUT);
}

void pullDvrToGround() {
  digitalWrite(DVR_TRIGGER_PIN, LOW);
  pinMode(DVR_TRIGGER_PIN, OUTPUT);
}

void triggerDvrRecordToggle() {
  uint32_t now = millis();
  if (dvrPulseActive) return;
  if ((int32_t)(now - dvrNextPulseAt) < 0) return;
  setCheckpoint(CP_DVR_PULSE);
  pullDvrToGround();
  dvrPulseActive = true;
  dvrPulseEndAt = now + DVR_TRIGGER_GROUND_MS;
  dvrNextPulseAt = now + DVR_TRIGGER_MIN_GAP_MS;
}

void serviceDvrTrigger(uint32_t now) {
  if (!dvrPulseActive) return;
  if ((int32_t)(now - dvrPulseEndAt) < 0) return;
  dvrPulseActive = false;
  setDvrFloating();
}


constexpr uint32_t DATA_RECORDING_START_DELAY_MS = 3000;
constexpr uint32_t FIVE_V_OFF_AFTER_STOP_MS = 4UL * 60UL * 1000UL;

bool dataRecordingArmed = false;
uint32_t dataRecordingArmedAtMs = 0;

bool fiveVOffScheduled = false;
uint32_t fiveVOffAtMs = 0;

void setFiveVEnabled(bool enabled);
void sendAlert(Telemetry::AlertCode code, Telemetry::AlertSeverity severity, const char* message);


bool startRecording(uint32_t now) {
  if (telemetry.recording || dataRecordingArmed) {
    return false;
  }
  if (telemetry.flashOK && telemetry.flashPercent >= 99) {
    Serial.println("Start recording refused: flash full - send CLEAR first.");
    return false;
  }

  triggerDvrRecordToggle();

  if (!fiveVEnabled) {
    Serial.println("Start recording: DVR triggered, but data logging refused (5V rail is off).");
    return false;
  }

  fiveVOffScheduled = false;

  setCheckpoint(CP_RECORDING_START);
  dataRecordingArmed = true;
  dataRecordingArmedAtMs = now + DATA_RECORDING_START_DELAY_MS;
  return true;
}

void stopRecording(uint32_t now) {
  bool wasRecordingOrArmed = telemetry.recording || dataRecordingArmed;
  dataRecordingArmed = false;

  if (wasRecordingOrArmed) {
    triggerDvrRecordToggle();
  }

  if (telemetry.recording) {
    telemetry.recording = false;
    launchCandidateStartMs = 0;
    recordingStoppedAtMs = now;
    triggerEventBeep(BEEP_RECORDING_STOP);
  }

  if (wasRecordingOrArmed) {
    fiveVOffScheduled = true;
    fiveVOffAtMs = now + FIVE_V_OFF_AFTER_STOP_MS;
  }
}

void updateRecordingArm(uint32_t now) {
  if (!dataRecordingArmed) return;
  if (!fiveVEnabled) {
    dataRecordingArmed = false;
    return;
  }
  if ((int32_t)(now - dataRecordingArmedAtMs) < 0) return;

  setCheckpoint(CP_RECORDING_PROMOTE);
  dataRecordingArmed = false;
  telemetry.recording = true;
  recordingStartMs = now;
  landedCandidateStartMs = 0;
  recoveryBeaconActive = false;
  recordingStoppedAtMs = 0;
  triggerEventBeep(BEEP_RECORDING_START);
}

void updateFiveVAutoOff(uint32_t now) {
  if (!fiveVOffScheduled) return;
  if ((int32_t)(now - fiveVOffAtMs) < 0) return;

  fiveVOffScheduled = false;
  if (!telemetry.recording && !dataRecordingArmed) {
    setFiveVEnabled(false);
  }
}

void updateLaunchDetect(uint32_t now) {
  float accelMagG = sqrtf(
      telemetry.accelX * telemetry.accelX +
      telemetry.accelY * telemetry.accelY +
      telemetry.accelZ * telemetry.accelZ) / 9.80665f;

  bool autoDetectSuppressed = now < manualOverrideUntilMs;

  if (!telemetry.recording) {
    if (!autoDetectSuppressed && accelMagG >= ACCEL_LAUNCH_THRESHOLD_G) {
      if (launchCandidateStartMs == 0) {
        launchCandidateStartMs = now;
      } else if (now - launchCandidateStartMs >= LAUNCH_SUSTAIN_MS) {
        startRecording(now);
      }
    } else {
      launchCandidateStartMs = 0;
    }
  } else {
    if (now - recordingStartMs >= MAX_RECORDING_DURATION_MS) {
      stopRecording(now);
      sendAlert(Telemetry::ALERT_RECORDING_AUTO_STOPPED, Telemetry::ALERT_WARNING,
                "auto-stop: max recording duration reached");
    } else if (!autoDetectSuppressed && now - recordingStartMs >= MIN_RECORDING_BEFORE_LANDING_CHECK_MS) {
      if (accelMagG >= LANDED_ACCEL_LOW_G && accelMagG <= LANDED_ACCEL_HIGH_G) {
        if (landedCandidateStartMs == 0) {
          landedCandidateStartMs = now;
        } else if (now - landedCandidateStartMs >= LANDED_SUSTAIN_MS) {
          stopRecording(now);
          sendAlert(Telemetry::ALERT_RECORDING_AUTO_STOPPED, Telemetry::ALERT_INFO,
                    "auto-stop: landing detected");
        }
      } else {
        landedCandidateStartMs = 0;
      }
    }
  }

  if (!telemetry.recording && !recoveryBeaconActive && recordingStoppedAtMs != 0 &&
      (now - recordingStoppedAtMs) >= RECOVERY_BEACON_GRACE_MS) {
    recoveryBeaconActive = true;
  }
}

void updateBuzzerBackgroundPattern() {
  if (buzzerEventActive) {
    return;
  }

  if (recoveryBeaconActive) {
    buzzerSetPattern(BEEP_RECOVERY, 1, true, false);
  } else if (anyCriticalSensorFault()) {
    buzzerSetPattern(BEEP_FAULT, 1, true, false);
  } else if (telemetry.recording) {
    buzzerSetPattern(BEEP_HEARTBEAT_RECORDING, 1, true, false);
  } else {
    buzzerSetPattern(BEEP_HEARTBEAT_IDLE, 1, true, false);
  }
}


constexpr uint32_t LIGHT_FLASH_PERIOD_MS = 400;
constexpr uint32_t LIGHT_FAULT_FLASH_PERIOD_MS = 300;
constexpr uint32_t GPS_BREATHE_PERIOD_MS = 2400;

bool flashPhaseOn(uint32_t periodMs, uint32_t now) {
  return (now % periodMs) < (periodMs / 2);
}

uint32_t breatheColor(uint8_t r, uint8_t g, uint8_t b, uint32_t periodMs, uint32_t now) {
  float phase = static_cast<float>(now % periodMs) / static_cast<float>(periodMs);
  float level = 0.5f - 0.5f * cosf(phase * 2.0f * PI);
  return pixels.Color(
      static_cast<uint8_t>(r * level),
      static_cast<uint8_t>(g * level),
      static_cast<uint8_t>(b * level));
}

void updateLight1Power(uint32_t now) {
  uint32_t color;
  if (!telemetry.fiveVActive) {
    color = pixels.Color(0, 0, 0);
  } else if (telemetry.recording) {
    color = flashPhaseOn(LIGHT_FLASH_PERIOD_MS, now) ? pixels.Color(255, 0, 0) : pixels.Color(0, 0, 0);
  } else {
    color = pixels.Color(0, 200, 0);
  }
  pixels.setPixelColor(0, color);
}

void updateLight2GPS(uint32_t now) {
  uint32_t color;
  if (!telemetry.gpsOK) {
    color = pixels.Color(255, 110, 0);
  } else if (telemetry.gpsFix) {
    color = breatheColor(0, 70, 255, GPS_BREATHE_PERIOD_MS, now);
  } else if (telemetry.gpsSatellites > 0) {
    color = pixels.Color(150, 0, 220);
  } else {
    color = pixels.Color(200, 0, 0);
  }
  pixels.setPixelColor(1, color);
}

void updateLight3Recording(uint32_t now) {
  uint32_t color;
  if (!telemetry.flashOK) {
    color = pixels.Color(255, 110, 0);
  } else if (anyCriticalSensorFault()) {
    color = flashPhaseOn(LIGHT_FAULT_FLASH_PERIOD_MS, now) ? pixels.Color(255, 110, 0) : pixels.Color(0, 0, 0);
  } else if (telemetry.recording) {
    color = flashPhaseOn(LIGHT_FLASH_PERIOD_MS, now) ? pixels.Color(200, 0, 0) : pixels.Color(0, 0, 0);
  } else {
    color = pixels.Color(0, 0, 0);
  }
  pixels.setPixelColor(2, color);
}

void updateLight4Link() {
  uint32_t color = telemetry.loraOK ? pixels.Color(0, 200, 0) : pixels.Color(255, 110, 0);
  pixels.setPixelColor(3, color);
}

bool bootAnimationActive = true;
uint32_t bootAnimationStartMs = 0;
bool bootOK = false;

void startBootAnimation() {
  bootOK = telemetry.imuOK && telemetry.loraOK;
  bootAnimationStartMs = millis();
  bootAnimationActive = true;
}

bool updateBootAnimation(uint32_t now) {
  if (!bootAnimationActive) {
    return false;
  }

  uint32_t duration = bootOK ? 500 : 2000;
  if (now - bootAnimationStartMs >= duration) {
    bootAnimationActive = false;
    return false;
  }

  uint32_t color = bootOK ? pixels.Color(0, 200, 0) : pixels.Color(255, 110, 0);
  for (uint8_t i = 0; i < NEOPIXEL_COUNT; i++) {
    pixels.setPixelColor(i, color);
  }
  return true;
}

void updateNeoPixels() {
  uint32_t now = millis();

  if (!updateBootAnimation(now)) {
    updateLight1Power(now);
    updateLight2GPS(now);
    updateLight3Recording(now);
    updateLight4Link();
  }

  pixels.show();
}


float madgwickQ0 = 1.0f, madgwickQ1 = 0.0f, madgwickQ2 = 0.0f, madgwickQ3 = 0.0f;
constexpr float MADGWICK_BETA = 0.1f;
uint32_t madgwickUpdateCount = 0;
constexpr uint32_t MADGWICK_CONVERGED_AFTER_UPDATES = 200;

void madgwickUpdate(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
  float q0 = madgwickQ0, q1 = madgwickQ1, q2 = madgwickQ2, q3 = madgwickQ3;
  float recipNorm;

  float qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  float qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  float qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  float qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    float _2q0 = 2.0f * q0, _2q1 = 2.0f * q1, _2q2 = 2.0f * q2, _2q3 = 2.0f * q3;
    float _4q0 = 4.0f * q0, _4q1 = 4.0f * q1, _4q2 = 4.0f * q2;
    float _8q1 = 8.0f * q1, _8q2 = 8.0f * q2;
    float q0q0 = q0 * q0, q1q1 = q1 * q1, q2q2 = q2 * q2, q3q3 = q3 * q3;

    float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 +
               _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    float s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 +
               _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    float s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

    recipNorm = 1.0f / sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    qDot1 -= MADGWICK_BETA * s0;
    qDot2 -= MADGWICK_BETA * s1;
    qDot3 -= MADGWICK_BETA * s2;
    qDot4 -= MADGWICK_BETA * s3;
  }

  q0 += qDot1 * dt;
  q1 += qDot2 * dt;
  q2 += qDot3 * dt;
  q3 += qDot4 * dt;

  recipNorm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  madgwickQ0 = q0 * recipNorm;
  madgwickQ1 = q1 * recipNorm;
  madgwickQ2 = q2 * recipNorm;
  madgwickQ3 = q3 * recipNorm;

  if (madgwickUpdateCount < MADGWICK_CONVERGED_AFTER_UPDATES) {
    madgwickUpdateCount++;
  }
}

bool madgwickConverged() {
  return madgwickUpdateCount >= MADGWICK_CONVERGED_AFTER_UPDATES;
}


constexpr float ACCEL_SIGN_X = 1.0f;
constexpr float ACCEL_SIGN_Y = 1.0f;
constexpr float ACCEL_SIGN_Z = 1.0f;
constexpr float GYRO_SIGN_X = 1.0f;
constexpr float GYRO_SIGN_Y = 1.0f;
constexpr float GYRO_SIGN_Z = 1.0f;

uint32_t lastIMUUpdateUs = 0;


float imuCalibQ0 = 1.0f, imuCalibQ1 = 0.0f, imuCalibQ2 = 0.0f, imuCalibQ3 = 0.0f;
bool imuCalibrated = false;

void calibrateIMUUpright() {
  imuCalibQ0 = madgwickQ0;
  imuCalibQ1 = -madgwickQ1;
  imuCalibQ2 = -madgwickQ2;
  imuCalibQ3 = -madgwickQ3;
  imuCalibrated = true;
}

void getCalibratedQuaternion(float& w, float& x, float& y, float& z) {
  if (!imuCalibrated) {
    w = madgwickQ0;
    x = madgwickQ1;
    y = madgwickQ2;
    z = madgwickQ3;
    return;
  }

  w = imuCalibQ0 * madgwickQ0 - imuCalibQ1 * madgwickQ1 - imuCalibQ2 * madgwickQ2 - imuCalibQ3 * madgwickQ3;
  x = imuCalibQ0 * madgwickQ1 + imuCalibQ1 * madgwickQ0 + imuCalibQ2 * madgwickQ3 - imuCalibQ3 * madgwickQ2;
  y = imuCalibQ0 * madgwickQ2 - imuCalibQ1 * madgwickQ3 + imuCalibQ2 * madgwickQ0 + imuCalibQ3 * madgwickQ1;
  z = imuCalibQ0 * madgwickQ3 + imuCalibQ1 * madgwickQ2 - imuCalibQ2 * madgwickQ1 + imuCalibQ3 * madgwickQ0;
}

constexpr uint32_t IMU_READ_INTERVAL_MS = 10;
uint32_t lastIMUReadMs = 0;

void updateIMU() {
  if (!telemetry.imuOK) {
    return;
  }

  uint32_t nowMs = millis();
  if (nowMs - lastIMUReadMs < IMU_READ_INTERVAL_MS) {
    return;
  }
  lastIMUReadMs = nowMs;

  sensors_event_t accelEvent;
  sensors_event_t gyroEvent;
  lsm6dso.getEvent(&accelEvent, &gyroEvent, nullptr);

  telemetry.accelX = accelEvent.acceleration.x * ACCEL_SIGN_X;
  telemetry.accelY = accelEvent.acceleration.y * ACCEL_SIGN_Y;
  telemetry.accelZ = accelEvent.acceleration.z * ACCEL_SIGN_Z;

  telemetry.gyroX = gyroEvent.gyro.x * GYRO_SIGN_X;
  telemetry.gyroY = gyroEvent.gyro.y * GYRO_SIGN_Y;
  telemetry.gyroZ = gyroEvent.gyro.z * GYRO_SIGN_Z;

  uint32_t now = micros();
  float dt = (lastIMUUpdateUs == 0) ? 0.0f : static_cast<float>(now - lastIMUUpdateUs) / 1000000.0f;
  lastIMUUpdateUs = now;

  if (dt > 0.0f && dt < 0.5f) {
    madgwickUpdate(
        telemetry.gyroX, telemetry.gyroY, telemetry.gyroZ,
        telemetry.accelX, telemetry.accelY, telemetry.accelZ,
        dt);
  }
}


uint8_t ms5607Address = 0x76;

constexpr uint8_t MS5607_CMD_RESET = 0x1E;
constexpr uint8_t MS5607_CMD_CONVERT_D1_OSR4096 = 0x48;
constexpr uint8_t MS5607_CMD_CONVERT_D2_OSR4096 = 0x58;
constexpr uint8_t MS5607_CMD_ADC_READ = 0x00;
constexpr uint8_t MS5607_CMD_PROM_READ_BASE = 0xA0;

uint16_t ms5607Prom[8] = {0};

bool ms5607WriteCommand(uint8_t cmd) {
  Wire.beginTransmission(ms5607Address);
  Wire.write(cmd);
  return Wire.endTransmission() == 0;
}

bool ms5607ReadU16(uint16_t& value) {
  if (Wire.requestFrom(ms5607Address, static_cast<uint8_t>(2)) != 2) {
    return false;
  }
  uint16_t high = Wire.read();
  uint16_t low = Wire.read();
  value = static_cast<uint16_t>((high << 8) | low);
  return true;
}

bool ms5607ReadU24(uint32_t& value) {
  if (Wire.requestFrom(ms5607Address, static_cast<uint8_t>(3)) != 3) {
    return false;
  }
  uint32_t b2 = Wire.read();
  uint32_t b1 = Wire.read();
  uint32_t b0 = Wire.read();
  value = (b2 << 16) | (b1 << 8) | b0;
  return true;
}

bool ms5607CheckCRC(const uint16_t* prom) {
  uint16_t promCopy[8];
  for (int i = 0; i < 8; i++) {
    promCopy[i] = prom[i];
  }
  uint16_t original = promCopy[7] & 0x000F;
  promCopy[7] &= 0xFF00;

  uint16_t remainder = 0;
  for (int i = 0; i < 16; i++) {
    if (i % 2 == 0) {
      remainder ^= static_cast<uint16_t>(promCopy[i / 2] >> 8);
    } else {
      remainder ^= static_cast<uint16_t>(promCopy[i / 2] & 0x00FF);
    }
    for (int bit = 8; bit > 0; bit--) {
      if (remainder & 0x8000) {
        remainder = static_cast<uint16_t>((remainder << 1) ^ 0x3000);
      } else {
        remainder = static_cast<uint16_t>(remainder << 1);
      }
    }
  }
  remainder = (remainder >> 12) & 0x000F;

  return remainder == original;
}

bool ms5607TryBegin(uint8_t address) {
  ms5607Address = address;

  if (!ms5607WriteCommand(MS5607_CMD_RESET)) {
    return false;
  }
  delay(5);

  for (uint8_t i = 0; i < 8; i++) {
    if (!ms5607WriteCommand(static_cast<uint8_t>(MS5607_CMD_PROM_READ_BASE + 2 * i))) {
      return false;
    }
    if (!ms5607ReadU16(ms5607Prom[i])) {
      return false;
    }
    rp2040.wdt_reset();
  }

  if (ms5607Prom[0] == 0 || ms5607Prom[0] == 0xFFFF) {
    return false;
  }

  return ms5607CheckCRC(ms5607Prom);
}

bool ms5607Begin() {
  if (ms5607TryBegin(0x76)) {
    return true;
  }
  return ms5607TryBegin(0x77);
}

bool ms5607Read(uint32_t& pressurePa) {
  uint32_t rawPressure;
  uint32_t rawTemperature;

  if (!ms5607WriteCommand(MS5607_CMD_CONVERT_D1_OSR4096)) {
    return false;
  }
  delay(10);
  if (!ms5607WriteCommand(MS5607_CMD_ADC_READ) || !ms5607ReadU24(rawPressure)) {
    return false;
  }

  if (!ms5607WriteCommand(MS5607_CMD_CONVERT_D2_OSR4096)) {
    return false;
  }
  delay(10);
  if (!ms5607WriteCommand(MS5607_CMD_ADC_READ) || !ms5607ReadU24(rawTemperature)) {
    return false;
  }

  int64_t dT = static_cast<int64_t>(rawTemperature) - (static_cast<int64_t>(ms5607Prom[5]) << 8);
  int64_t temp = 2000 + ((dT * ms5607Prom[6]) >> 23);

  int64_t off = (static_cast<int64_t>(ms5607Prom[2]) << 17) +
                ((static_cast<int64_t>(ms5607Prom[4]) * dT) >> 6);
  int64_t sens = (static_cast<int64_t>(ms5607Prom[1]) << 16) +
                 ((static_cast<int64_t>(ms5607Prom[3]) * dT) >> 7);

  if (temp < 2000) {
    int64_t t2 = (dT * dT) >> 31;
    int64_t off2 = (61 * (temp - 2000) * (temp - 2000)) >> 4;
    int64_t sens2 = 2 * (temp - 2000) * (temp - 2000);

    if (temp < -1500) {
      off2 += 15 * (temp + 1500) * (temp + 1500);
      sens2 += 8 * (temp + 1500) * (temp + 1500);
    }

    temp -= t2;
    off -= off2;
    sens -= sens2;
  }

  int64_t p = (((static_cast<int64_t>(rawPressure) * sens) >> 21) - off) >> 15;

  pressurePa = static_cast<uint32_t>(p);
  return true;
}

constexpr float METERS_TO_FEET = 3.28084f;


float baroZeroPressurePa = 0.0f;
bool baroZeroCaptured = false;

void calibrateBarometerZero(uint32_t currentPressurePa) {
  baroZeroPressurePa = static_cast<float>(currentPressurePa);
  baroZeroCaptured = true;
}

constexpr uint32_t BARO_READ_INTERVAL_MS = 50;
uint32_t lastBaroReadMs = 0;

void updateBarometer() {
  if (!fiveVEnabled) {
    return;
  }

  uint32_t now = millis();
  if (now - lastBaroReadMs < BARO_READ_INTERVAL_MS) {
    return;
  }
  lastBaroReadMs = now;

  uint32_t pressurePa;

  if (!ms5607Read(pressurePa)) {
    telemetry.baroOK = false;
    return;
  }

  telemetry.baroOK = true;
  telemetry.baroPressure = pressurePa;

  if (!baroZeroCaptured) {
    calibrateBarometerZero(pressurePa);
  }

  float altitudeAglM =
      44330.0f * (1.0f - powf(static_cast<float>(pressurePa) / baroZeroPressurePa, 1.0f / 5.255f));

  telemetry.baroAltitude = altitudeAglM * METERS_TO_FEET;
}


constexpr uint32_t POWER_READ_INTERVAL_MS = 500;
uint32_t lastPowerReadMs = 0;

void updatePower() {
  telemetry.fiveVActive = fiveVEnabled;

  if (!telemetry.fuelGaugeOK) {
    return;
  }

  uint32_t now = millis();
  if (now - lastPowerReadMs < POWER_READ_INTERVAL_MS) {
    return;
  }
  lastPowerReadMs = now;

  telemetry.batteryMV = static_cast<uint16_t>(maxlipo.cellVoltage() * 1000.0f);
}


constexpr uint8_t EEPROM_I2C_ADDR = 0x50;
constexpr uint32_t EEPROM_SIZE_BYTES = 65536;
constexpr uint8_t EEPROM_PAGE_SIZE = 128;
constexpr uint32_t EEPROM_DATA_START = 16;
constexpr uint16_t EEPROM_HEADER_MAGIC = 0xF109;

uint32_t eepromWriteOffset = EEPROM_DATA_START;

bool eepromWriteBytes(uint32_t address, const uint8_t* data, uint8_t length) {
  Wire.beginTransmission(EEPROM_I2C_ADDR);
  Wire.write(static_cast<uint8_t>((address >> 8) & 0xFF));
  Wire.write(static_cast<uint8_t>(address & 0xFF));
  for (uint8_t i = 0; i < length; i++) {
    Wire.write(data[i]);
  }
  if (Wire.endTransmission() != 0) {
    return false;
  }
  delay(5);
  return true;
}

bool eepromReadBytes(uint32_t address, uint8_t* data, uint8_t length) {
  Wire.beginTransmission(EEPROM_I2C_ADDR);
  Wire.write(static_cast<uint8_t>((address >> 8) & 0xFF));
  Wire.write(static_cast<uint8_t>(address & 0xFF));
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom(EEPROM_I2C_ADDR, length) != length) {
    return false;
  }
  for (uint8_t i = 0; i < length; i++) {
    data[i] = Wire.read();
  }
  return true;
}

constexpr uint32_t EEPROM_HEADER_SLOT_SIZE = 8;
constexpr uint32_t EEPROM_HEADER_SLOT_A = 0;
constexpr uint32_t EEPROM_HEADER_SLOT_B = EEPROM_HEADER_SLOT_SIZE;

bool eepromHeaderNextSlotIsB = false;

bool eepromReadHeaderSlot(uint32_t slotAddr, uint32_t& offsetOut) {
  uint8_t raw[EEPROM_HEADER_SLOT_SIZE];
  if (!eepromReadBytes(slotAddr, raw, sizeof(raw))) {
    return false;
  }

  uint16_t magic = static_cast<uint16_t>((raw[0] << 8) | raw[1]);
  if (magic != EEPROM_HEADER_MAGIC) {
    return false;
  }

  uint16_t storedCrc = static_cast<uint16_t>((raw[6] << 8) | raw[7]);
  if (Telemetry::crc16(raw, 6) != storedCrc) {
    return false;
  }

  uint32_t offset =
      (static_cast<uint32_t>(raw[2]) << 24) |
      (static_cast<uint32_t>(raw[3]) << 16) |
      (static_cast<uint32_t>(raw[4]) << 8) |
      static_cast<uint32_t>(raw[5]);
  if (offset < EEPROM_DATA_START || offset >= EEPROM_SIZE_BYTES) {
    return false;
  }

  offsetOut = offset;
  return true;
}

bool eepromWriteHeaderSlot(uint32_t slotAddr, uint32_t offset) {
  uint8_t raw[EEPROM_HEADER_SLOT_SIZE] = {
      static_cast<uint8_t>((EEPROM_HEADER_MAGIC >> 8) & 0xFF),
      static_cast<uint8_t>(EEPROM_HEADER_MAGIC & 0xFF),
      static_cast<uint8_t>((offset >> 24) & 0xFF),
      static_cast<uint8_t>((offset >> 16) & 0xFF),
      static_cast<uint8_t>((offset >> 8) & 0xFF),
      static_cast<uint8_t>(offset & 0xFF),
      0,
      0,
  };
  uint16_t crc = Telemetry::crc16(raw, 6);
  raw[6] = static_cast<uint8_t>((crc >> 8) & 0xFF);
  raw[7] = static_cast<uint8_t>(crc & 0xFF);
  setCheckpoint(CP_FLASH_HEADER_WRITE);
  return eepromWriteBytes(slotAddr, raw, sizeof(raw));
}

bool eepromWriteHeader(uint32_t offset) {
  uint32_t slotAddr = eepromHeaderNextSlotIsB ? EEPROM_HEADER_SLOT_B : EEPROM_HEADER_SLOT_A;
  bool ok = eepromWriteHeaderSlot(slotAddr, offset);
  if (ok) {
    eepromHeaderNextSlotIsB = !eepromHeaderNextSlotIsB;
  }
  return ok;
}

bool eepromBegin() {
  uint32_t offsetA = 0;
  uint32_t offsetB = 0;
  bool validA = eepromReadHeaderSlot(EEPROM_HEADER_SLOT_A, offsetA);
  bool validB = eepromReadHeaderSlot(EEPROM_HEADER_SLOT_B, offsetB);

  if (validA && validB) {
    if (offsetA >= offsetB) {
      eepromWriteOffset = offsetA;
      eepromHeaderNextSlotIsB = true;
    } else {
      eepromWriteOffset = offsetB;
      eepromHeaderNextSlotIsB = false;
    }
    return true;
  }

  if (validA || validB) {
    if (validA) {
      eepromWriteOffset = offsetA;
      eepromHeaderNextSlotIsB = true;
    } else {
      eepromWriteOffset = offsetB;
      eepromHeaderNextSlotIsB = false;
    }
    return true;
  }

  eepromWriteOffset = EEPROM_DATA_START;
  eepromHeaderNextSlotIsB = false;
  return eepromWriteHeaderSlot(EEPROM_HEADER_SLOT_A, eepromWriteOffset);
}

void eepromClearLog() {
  eepromWriteOffset = EEPROM_DATA_START;
  eepromWriteHeaderSlot(EEPROM_HEADER_SLOT_A, eepromWriteOffset);
  eepromWriteHeaderSlot(EEPROM_HEADER_SLOT_B, eepromWriteOffset);
  eepromHeaderNextSlotIsB = true;
}

void printRawPacket(const uint8_t* buffer, size_t length);

void dumpFlashToSerial(uint32_t startOffset = EEPROM_DATA_START) {
  if (!telemetry.flashOK) {
    Serial.println("# No EEPROM detected - nothing to dump.");
    return;
  }

  if (startOffset < EEPROM_DATA_START || startOffset > eepromWriteOffset) {
    Serial.print("# Invalid DUMP offset ");
    Serial.print(startOffset);
    Serial.print(" - valid range is ");
    Serial.print(EEPROM_DATA_START);
    Serial.print(" to ");
    Serial.println(eepromWriteOffset);
    return;
  }

  Serial.println("# FLASH DUMP START");

  uint32_t pos = startOffset;
  uint32_t limit = eepromWriteOffset;
  uint32_t packetsFound = 0;
  uint8_t header[4];
  uint8_t packet[Telemetry::FAST_PACKET_SIZE];

  while (pos + 4 <= limit) {
    if (!eepromReadBytes(pos, header, 4)) {
      Serial.println("# READ ERROR, aborting dump");
      break;
    }
    if (header[0] != Telemetry::MAGIC_0 || header[1] != Telemetry::MAGIC_1) {
      Serial.print("# Lost sync at offset ");
      Serial.println(pos);
      break;
    }

    size_t packetLength;
    switch (header[3]) {
      case Telemetry::PACKET_FAST:        packetLength = Telemetry::FAST_PACKET_SIZE; break;
      case Telemetry::PACKET_STATUS:      packetLength = Telemetry::STATUS_PACKET_SIZE; break;
      case Telemetry::PACKET_ORIENTATION: packetLength = Telemetry::ORIENTATION_PACKET_SIZE; break;
      default:
        Serial.print("# Unknown packet type at offset ");
        Serial.println(pos);
        packetLength = 0;
        break;
    }

    if (packetLength == 0 || pos + packetLength > limit) {
      break;
    }

    memcpy(packet, header, 4);
    if (packetLength > 4 &&
        !eepromReadBytes(pos + 4, packet + 4, static_cast<uint8_t>(packetLength - 4))) {
      Serial.println("# READ ERROR, aborting dump");
      break;
    }

    printRawPacket(packet, packetLength);
    rp2040.wdt_reset();
    pos += packetLength;
    packetsFound++;
  }

  Serial.print("# FLASH DUMP END - ");
  Serial.print(packetsFound);
  Serial.println(" packets");
}

void logPacketToFlash(const uint8_t* buffer, size_t length) {
  if (!telemetry.flashOK || !telemetry.recording) {
    return;
  }

  setCheckpoint(CP_FLASH_DATA_WRITE);

  size_t written = 0;
  while (written < length) {
    uint32_t pageOffset = eepromWriteOffset % EEPROM_PAGE_SIZE;
    size_t remaining = length - written;
    size_t roomInPage = EEPROM_PAGE_SIZE - pageOffset;
    uint8_t chunk = static_cast<uint8_t>(remaining < roomInPage ? remaining : roomInPage);

    if (eepromWriteOffset + chunk > EEPROM_SIZE_BYTES) {
      eepromWriteOffset = EEPROM_DATA_START;
      continue;
    }

    if (!eepromWriteBytes(eepromWriteOffset, buffer + written, chunk)) {
      telemetry.flashErrors++;
      return;
    }

    eepromWriteOffset += chunk;
    written += chunk;
    telemetry.flashBytesWritten += chunk;
  }

  eepromWriteHeader(eepromWriteOffset);
}

void updateFlash() {
  if (!telemetry.flashOK) {
    telemetry.flashPercent = 0;
    return;
  }

  uint32_t capacity = EEPROM_SIZE_BYTES - EEPROM_DATA_START;
  uint32_t bytesUsed = eepromWriteOffset - EEPROM_DATA_START;
  uint32_t percent = (bytesUsed * 100UL) / capacity;
  telemetry.flashPercent = static_cast<uint16_t>(percent > 100 ? 100 : percent);

  if (telemetry.recording && telemetry.flashPercent >= 99) {
    stopRecording(millis());
    Serial.println(
        "Flash at 99% - recording auto-stopped to avoid overwriting data. "
        "Send CLEAR then START to record again.");
  }
}


void updateLoRa() {
  telemetry.loraRSSI = -127;
  telemetry.loraSNR = 0;
}


uint8_t buildStatusFlags() {
  uint8_t flags = 0;

  if (telemetry.recording) {
    flags |= Telemetry::STATUS_RECORDING;
  }
  if (telemetry.fiveVActive) {
    flags |= Telemetry::STATUS_5V_ACTIVE;
  }
  if (telemetry.gpsFix) {
    flags |= Telemetry::STATUS_GPS_FIX;
  }
  if (telemetry.gpsOK) {
    flags |= Telemetry::STATUS_GPS_OK;
  }
  if (telemetry.baroOK) {
    flags |= Telemetry::STATUS_BARO_OK;
  }
  if (telemetry.imuOK) {
    flags |= Telemetry::STATUS_IMU_OK;
  }
  if (telemetry.flashOK) {
    flags |= Telemetry::STATUS_FLASH_OK;
  }
  if (telemetry.loraOK) {
    flags |= Telemetry::STATUS_LORA_OK;
  }

  return flags;
}


void printRawPacket(const uint8_t* buffer, size_t length) {
  constexpr size_t MAX_LINE = 800;
  char line[MAX_LINE];
  static const char hexDigits[] = "0123456789ABCDEF";

  size_t safeLength = length < (MAX_LINE - 1) / 3 ? length : (MAX_LINE - 1) / 3;
  size_t pos = 0;
  for (size_t i = 0; i < safeLength; i++) {
    line[pos++] = hexDigits[(buffer[i] >> 4) & 0x0F];
    line[pos++] = hexDigits[buffer[i] & 0x0F];
    line[pos++] = ' ';
  }
  line[pos++] = '\n';
  Serial.write(line, pos);
}


bool radioSend(const uint8_t* buffer, size_t length) {
  if (!telemetry.loraOK) {
    return false;
  }

  if (rf95.mode() == RHGenericDriver::RHModeTx) {
    return false;
  }

  setCheckpoint(CP_RADIO_SEND);
  return rf95.send(buffer, length);
}

void sendAck(uint16_t ackedSequence, uint8_t ackedCommand, Telemetry::AckResult result) {
  Telemetry::AckData data;
  data.acked_sequence = ackedSequence;
  data.acked_command = ackedCommand;
  data.result = static_cast<uint8_t>(result);

  uint8_t buffer[Telemetry::ACK_PACKET_SIZE];
  size_t length = Telemetry::serializeAck(data, ackSequence, buffer);

  if (verboseRawPackets) printRawPacket(buffer, length);
  radioSend(buffer, length);
  ackSequence++;
}

void sendAlert(Telemetry::AlertCode code, Telemetry::AlertSeverity severity, const char* message) {
  Telemetry::AlertData data;
  data.code = static_cast<uint8_t>(code);
  data.severity = static_cast<uint8_t>(severity);
  memset(data.message, 0, sizeof(data.message));
  strncpy(data.message, message, sizeof(data.message) - 1);

  uint8_t buffer[Telemetry::ALERT_PACKET_SIZE];
  size_t length = Telemetry::serializeAlert(data, alertSequence, millis(), buffer);

  if (verboseRawPackets) printRawPacket(buffer, length);
  radioSend(buffer, length);
  alertSequence++;

  Serial.print("ALERT: ");
  Serial.println(message);
}


Telemetry::FastData buildFastData() {
  Telemetry::FastData data;

  data.latitude_e6 = static_cast<int32_t>(telemetry.latitude * 1000000.0);
  data.longitude_e6 = static_cast<int32_t>(telemetry.longitude * 1000000.0);

  data.altitude_cm = static_cast<int16_t>(telemetry.baroAltitude * 30.48);

  data.accel_x_mg = static_cast<int16_t>(
      telemetry.accelX / 9.80665 * 1000.0);
  data.accel_y_mg = static_cast<int16_t>(
      telemetry.accelY / 9.80665 * 1000.0);
  data.accel_z_mg = static_cast<int16_t>(
      telemetry.accelZ / 9.80665 * 1000.0);

  data.gyro_x_cdeg = static_cast<int16_t>(
      telemetry.gyroX * 57.2957795 * 100.0);
  data.gyro_y_cdeg = static_cast<int16_t>(
      telemetry.gyroY * 57.2957795 * 100.0);
  data.gyro_z_cdeg = static_cast<int16_t>(
      telemetry.gyroZ * 57.2957795 * 100.0);

  data.pressure_pa = telemetry.baroPressure;
  data.status = buildStatusFlags();
  data.gps_satellites = telemetry.gpsSatellites;

  return data;
}

bool sendFastPacket() {
  Telemetry::FastData data = buildFastData();

  size_t length = Telemetry::serializeFast(
      data,
      fastSequence,
      millis(),
      packetBuffer);

  if (verboseRawPackets) printRawPacket(packetBuffer, length);
  bool sent = radioSend(packetBuffer, length);
  fastSequence++;
  return sent;
}


constexpr uint32_t FLASH_FAST_LOG_INTERVAL_MS = 100;
uint16_t flashFastSequence = 0;

void logFastSampleToFlash() {
  if (!telemetry.flashOK || !telemetry.recording) {
    return;
  }

  Telemetry::FastData data = buildFastData();

  uint8_t buffer[Telemetry::FAST_PACKET_SIZE];
  size_t length = Telemetry::serializeFast(data, flashFastSequence, millis(), buffer);

  logPacketToFlash(buffer, length);
  flashFastSequence++;
}


bool sendStatusPacket() {
  Telemetry::StatusData data;

  data.battery_mv = telemetry.batteryMV;
  data.flash_percent = telemetry.flashPercent;
  data.lora_rssi = telemetry.loraRSSI;
  data.lora_snr = telemetry.loraSNR;
  data.gps_satellites = telemetry.gpsSatellites;
  data.status = buildStatusFlags();
  data.flash_bytes_written = telemetry.flashBytesWritten;
  data.flash_errors = telemetry.flashErrors;
  data.gps_unix_time = telemetry.gpsUnixTime;

  size_t length = Telemetry::serializeStatus(
      data,
      statusSequence,
      millis(),
      packetBuffer);

  if (verboseRawPackets) printRawPacket(packetBuffer, length);
  bool sent = radioSend(packetBuffer, length);
  logPacketToFlash(packetBuffer, length);
  statusSequence++;
  return sent;
}


bool sendOrientationPacket() {
  if (!telemetry.imuOK) {
    return true;
  }

  float qw, qx, qy, qz;
  getCalibratedQuaternion(qw, qx, qy, qz);

  Telemetry::OrientationData data;
  data.quat_w = static_cast<int16_t>(constrain(qw * 10000.0f, -32767.0f, 32767.0f));
  data.quat_x = static_cast<int16_t>(constrain(qx * 10000.0f, -32767.0f, 32767.0f));
  data.quat_y = static_cast<int16_t>(constrain(qy * 10000.0f, -32767.0f, 32767.0f));
  data.quat_z = static_cast<int16_t>(constrain(qz * 10000.0f, -32767.0f, 32767.0f));

  uint8_t converged = madgwickConverged() ? 3 : 0;
  data.calibration = static_cast<uint8_t>((converged & 0x03) << 6);
  data.reserved = 0;

  size_t length = Telemetry::serializeOrientation(
      data,
      orientationSequence,
      millis(),
      packetBuffer);

  if (verboseRawPackets) printRawPacket(packetBuffer, length);
  bool sent = radioSend(packetBuffer, length);
  logPacketToFlash(packetBuffer, length);
  orientationSequence++;
  return sent;
}


void initIMU() {
  if (!lsm6dso.begin_I2C()) {
    Serial.println("ERROR: LSM6DSO not detected!");
    telemetry.imuOK = false;
  } else {
    Serial.println("LSM6DSO OK");
    telemetry.imuOK = true;
    lsm6dso.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
    lsm6dso.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
    lsm6dso.setAccelDataRate(LSM6DS_RATE_104_HZ);
    lsm6dso.setGyroDataRate(LSM6DS_RATE_104_HZ);
  }
}

void initFuelGauge() {
  if (!maxlipo.begin(&Wire)) {
    Serial.println("ERROR: MAX17048 fuel gauge not detected!");
    telemetry.fuelGaugeOK = false;
  } else {
    Serial.println("MAX17048 fuel gauge OK");
    telemetry.fuelGaugeOK = true;
  }
}

void initBarometer() {
  if (!ms5607Begin()) {
    Serial.println("ERROR: MS5607 barometer not detected at 0x76 or 0x77!");
    telemetry.baroOK = false;
  } else {
    Serial.print("MS5607 barometer OK @ 0x");
    Serial.println(ms5607Address, HEX);
    telemetry.baroOK = true;
  }
}

void initSensorBoardDevices() {
  initIMU();
  rp2040.wdt_reset();
  initBarometer();
}

constexpr uint32_t SENSOR_RETRY_INTERVAL_MS = 3000;
constexpr uint8_t SENSOR_RETRY_MAX_ATTEMPTS = 3;
uint32_t lastSensorRetryMs = 0;
uint8_t sensorRetryAttemptsLeft = SENSOR_RETRY_MAX_ATTEMPTS;

void retryFailedSensorBoardDevices(uint32_t now) {
  if (!fiveVEnabled || sensorRetryAttemptsLeft == 0) {
    return;
  }
  if (telemetry.imuOK && telemetry.baroOK) {
    return;
  }
  if (now - lastSensorRetryMs < SENSOR_RETRY_INTERVAL_MS) {
    return;
  }
  lastSensorRetryMs = now;
  sensorRetryAttemptsLeft--;
  setCheckpoint(CP_SENSOR_RETRY);
  if (!telemetry.imuOK) {
    initIMU();
    rp2040.wdt_reset();
  }
  if (!telemetry.baroOK) {
    initBarometer();
    rp2040.wdt_reset();
  }

  if (sensorRetryAttemptsLeft == 0 && (!telemetry.imuOK || !telemetry.baroOK)) {
    char msg[Telemetry::ALERT_MESSAGE_LEN];
    snprintf(msg, sizeof(msg), "sensor fault: imu=%d baro=%d",
             telemetry.imuOK ? 1 : 0, telemetry.baroOK ? 1 : 0);
    sendAlert(Telemetry::ALERT_SENSOR_FAULT, Telemetry::ALERT_CRITICAL, msg);
  }
}

void checkI2cBusRecovery() {
  if (!Wire.getTimeoutFlag()) {
    return;
  }
  Wire.clearTimeoutFlag();
  sendAlert(Telemetry::ALERT_I2C_BUS_RECOVERED, Telemetry::ALERT_WARNING,
            "I2C bus hang auto-recovered");
}

void setFiveVEnabled(bool enabled) {
  fiveVEnabled = enabled;
  digitalWrite(FIVE_V_SWITCH_PIN, enabled ? HIGH : LOW);

  if (enabled) {
    fiveVOffScheduled = false;

    delay(20);
    setCheckpoint(CP_FIVE_V_ENABLE);
    initSensorBoardDevices();
    rp2040.wdt_reset();
    sensorRetryAttemptsLeft = SENSOR_RETRY_MAX_ATTEMPTS;
  } else {
    delay(20);

    telemetry.imuOK = false;
    telemetry.fuelGaugeOK = false;
    telemetry.baroOK = false;
  }
}


Telemetry::AckResult handleCommand(const Telemetry::CommandData& cmd) {
  switch (cmd.command) {
    case Telemetry::CMD_CLEAR_FLASH:
      eepromClearLog();
      Serial.println("CMD: clear flash");
      return Telemetry::ACK_APPLIED;

    case Telemetry::CMD_START_RECORDING:
      noteManualRecordingOverride(millis());
      if (startRecording(millis())) {
        Serial.println("CMD: start recording (DVR triggered, data recording in 3s)");
        return Telemetry::ACK_APPLIED;
      }
      Serial.println("CMD: start recording refused (5V rail off or already recording)");
      return Telemetry::ACK_REFUSED;

    case Telemetry::CMD_STOP_RECORDING:
      stopRecording(millis());
      noteManualRecordingOverride(millis());
      Serial.println("CMD: stop recording (DVR stopped, 5V off in 4min)");
      return Telemetry::ACK_APPLIED;

    case Telemetry::CMD_CALIBRATE_IMU:
      calibrateIMUUpright();
      Serial.println("CMD: calibrate IMU (current pose = upright zero)");
      return Telemetry::ACK_APPLIED;

    case Telemetry::CMD_CALIBRATE_BAROMETER:
      calibrateBarometerZero(telemetry.baroPressure);
      Serial.println("CMD: calibrate barometer (current pressure = zero altitude)");
      return Telemetry::ACK_APPLIED;

    case Telemetry::CMD_DVR_TRIGGER:
      triggerDvrRecordToggle();
      Serial.println("CMD: DVR debug button-press toggle fired");
      return Telemetry::ACK_APPLIED;

    case Telemetry::CMD_5V_ON:
      setFiveVEnabled(true);
      Serial.println("CMD: 5V rail ON");
      return Telemetry::ACK_APPLIED;

    case Telemetry::CMD_5V_OFF:
      setFiveVEnabled(false);
      Serial.println("CMD: 5V rail OFF");
      return Telemetry::ACK_APPLIED;

    case Telemetry::CMD_BUZZER_MUTE:
      setBuzzerMuted(true);
      Serial.println("CMD: buzzer muted");
      return Telemetry::ACK_APPLIED;

    case Telemetry::CMD_BUZZER_UNMUTE:
      setBuzzerMuted(false);
      Serial.println("CMD: buzzer unmuted");
      return Telemetry::ACK_APPLIED;

    default:
      Serial.print("CMD: unknown command id ");
      Serial.println(cmd.command);
      return Telemetry::ACK_UNKNOWN;
  }
}

constexpr uint32_t RX_DWELL_MS = 60;
uint32_t rxModeEnteredMs = 0;
bool radioWasInRxMode = false;

bool radioSafeToSend() {
  bool nowInRx = telemetry.loraOK && rf95.mode() == RHGenericDriver::RHModeRx;
  if (nowInRx && !radioWasInRxMode) {
    rxModeEnteredMs = millis();
  }
  radioWasInRxMode = nowInRx;
  return !nowInRx || (millis() - rxModeEnteredMs >= RX_DWELL_MS);
}

constexpr uint32_t RADIO_STUCK_TX_MS = 1000;
uint32_t radioTxEnteredMs = 0;
bool radioWasInTxMode = false;

void checkRadioStuckInTx(uint32_t now) {
  bool nowInTx = telemetry.loraOK && rf95.mode() == RHGenericDriver::RHModeTx;
  if (nowInTx && !radioWasInTxMode) {
    radioTxEnteredMs = now;
  }
  radioWasInTxMode = nowInTx;

  if (nowInTx && (now - radioTxEnteredMs) >= RADIO_STUCK_TX_MS) {
    rf95.setModeIdle();
    radioWasInTxMode = false;
    Serial.println("Radio stuck in TX mode - forced back to idle.");
    sendAlert(Telemetry::ALERT_RADIO_STUCK_RECOVERED, Telemetry::ALERT_WARNING,
              "radio stuck in TX, forced idle");
  }
}

void checkRadioCommands() {
  if (!telemetry.loraOK) {
    return;
  }

  setCheckpoint(CP_RADIO_AVAILABLE);
  if (!rf95.available()) {
    return;
  }

  uint8_t buffer[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t length = sizeof(buffer);

  setCheckpoint(CP_RADIO_RECV);
  if (!rf95.recv(buffer, &length)) {
    return;
  }

  Telemetry::CommandData cmd;
  if (Telemetry::parseCommand(buffer, length, cmd)) {
    Telemetry::AckResult result = handleCommand(cmd);
    sendAck(cmd.sequence, cmd.command, result);
  } else {
    Serial.print("Received radio packet failed parseCommand, len=");
    Serial.print(length);
    Serial.print(": ");
    printRawPacket(buffer, length);
  }
}

char serialCmdBuf[16];
uint8_t serialCmdLen = 0;

void handleSerialCommand(const char* cmd) {
  setCheckpoint(CP_SERIAL_COMMAND);
  if (strcasecmp(cmd, "DUMP") == 0) {
    dumpFlashToSerial();
  } else if (strncasecmp(cmd, "DUMP ", 5) == 0) {
    const char* arg = cmd + 5;
    while (*arg == ' ') {
      arg++;
    }
    char* endPtr = nullptr;
    unsigned long offset = strtoul(arg, &endPtr, 10);
    if (endPtr == arg) {
      Serial.println("# DUMP <offset> needs a numeric byte offset");
    } else {
      dumpFlashToSerial(static_cast<uint32_t>(offset));
    }
  } else if (strcasecmp(cmd, "CLEAR") == 0) {
    eepromClearLog();
    Serial.println("# Flash log cleared.");
  } else if (strcasecmp(cmd, "RECORD") == 0) {
    noteManualRecordingOverride(millis());
    if (startRecording(millis())) {
      Serial.println("# Start recording: DVR triggered, data recording in 3s.");
    } else {
      Serial.println("# Start recording refused: 5V rail off or already recording.");
    }
  } else if (strcasecmp(cmd, "STOP") == 0) {
    stopRecording(millis());
    noteManualRecordingOverride(millis());
    Serial.println("# Stop recording: DVR stopped, 5V rail off in 4min.");
  } else if (strcasecmp(cmd, "CALIBRATE_IMU") == 0) {
    calibrateIMUUpright();
    Serial.println("# IMU calibrated (current pose = upright zero).");
  } else if (strcasecmp(cmd, "CALIBRATE_BARO") == 0) {
    calibrateBarometerZero(telemetry.baroPressure);
    Serial.println("# Barometer calibrated (current pressure = zero altitude).");
  } else if (strcasecmp(cmd, "5VON") == 0) {
    setFiveVEnabled(true);
    Serial.println("# 5V rail ON.");
  } else if (strcasecmp(cmd, "5VOFF") == 0) {
    setFiveVEnabled(false);
    Serial.println("# 5V rail OFF.");
  } else if (strcasecmp(cmd, "MUTE") == 0) {
    setBuzzerMuted(true);
    Serial.println("# Buzzer muted.");
  } else if (strcasecmp(cmd, "UNMUTE") == 0) {
    setBuzzerMuted(false);
    Serial.println("# Buzzer unmuted.");
  } else if (strcasecmp(cmd, "DVRTEST") == 0) {
    triggerDvrRecordToggle();
    Serial.println("# DVR debug button-press toggle fired.");
  } else if (strcasecmp(cmd, "RAWON") == 0) {
    verboseRawPackets = true;
    Serial.println("# Raw packet hex dump ON (every FAST/STATUS/ORIENTATION/ACK/ALERT send).");
  } else if (strcasecmp(cmd, "RAWOFF") == 0) {
    verboseRawPackets = false;
    Serial.println("# Raw packet hex dump OFF.");
  } else if (strcasecmp(cmd, "FUELGAUGE") == 0) {
    initFuelGauge();
  } else if (strcasecmp(cmd, "VERBOSEON") == 0) {
    verboseCheckpoints = true;
    Serial.println("# Verbose checkpoint logging ON (prints every CP_* transition, including every loop top).");
  } else if (strcasecmp(cmd, "VERBOSEOFF") == 0) {
    verboseCheckpoints = false;
    Serial.println("# Verbose checkpoint logging OFF.");
  } else {
    Serial.print("# Unknown command: ");
    Serial.println(cmd);
    Serial.println(
        "# Known commands: DUMP, CLEAR, RECORD, STOP, CALIBRATE_IMU, "
        "CALIBRATE_BARO, 5VON, 5VOFF, MUTE, UNMUTE, DVRTEST, RAWON, RAWOFF, "
        "FUELGAUGE, VERBOSEON, VERBOSEOFF");
  }
}

void checkSerialCommands() {
  while (Serial.available() > 0) {
    char c = static_cast<char>(Serial.read());

    if (c == '\n' || c == '\r') {
      if (serialCmdLen > 0) {
        serialCmdBuf[serialCmdLen] = '\0';
        handleSerialCommand(serialCmdBuf);
        serialCmdLen = 0;
      }
    } else if (serialCmdLen < sizeof(serialCmdBuf) - 1) {
      serialCmdBuf[serialCmdLen++] = c;
    }
  }
}


void setupRadio() {
  SPI.setSCK(RFM95_SCK_PIN);
  SPI.setTX(RFM95_MOSI_PIN);
  SPI.setRX(RFM95_MISO_PIN);

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  if (!rf95.init()) {
    Serial.println("ERROR: RFM9x not detected!");
    telemetry.loraOK = false;
    return;
  }

  rf95.setFrequency(RF95_FREQ_MHZ);
  rf95.setTxPower(RF95_TX_POWER_DBM, false);
  rf95.setSpreadingFactor(RF95_SPREADING_FACTOR);
  rf95.setSignalBandwidth(RF95_BANDWIDTH_HZ);
  rf95.setCodingRate4(RF95_CODING_RATE_DENOM);

  telemetry.loraOK = true;

  Serial.print("RFM9x OK @ ");
  Serial.print(RF95_FREQ_MHZ);
  Serial.println(" MHz");
}


bool bootWasWatchdogReset = false;
bool bootWasBrownout = false;
uint32_t lastCheckpoint = 0;

void setup() {
  bootWasWatchdogReset = watchdog_caused_reboot();

  bootWasBrownout = rp2040.getResetReason() == RP2040::BROWNOUT_RESET;

  lastCheckpoint = lastCheckpointBeforeThisBoot();

  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("================================");
  Serial.println(" RP2040 TELEMETRY TEST");
  Serial.println("================================");

  Serial.println("[BOOT] buzzer/neopixel init...");
  pinMode(BUZZER_PIN, OUTPUT);
  analogWriteFreq(BUZZER_FREQ_HZ);
  pixels.begin();
  pixels.setBrightness(NEOPIXEL_BRIGHTNESS);
  pixels.show();
  Serial.println("[BOOT] buzzer/neopixel init OK");

  Serial.println("[BOOT] GPIO1 high...");
  pinMode(GPIO1_ALWAYS_HIGH_PIN, OUTPUT);
  digitalWrite(GPIO1_ALWAYS_HIGH_PIN, HIGH);
  Serial.println("[BOOT] GPIO1 high OK");

  constexpr bool DIAG_5V_START_DISABLED = true;
  Serial.println(DIAG_5V_START_DISABLED ? "[BOOT] 5V rail starting OFF (diagnostic)..." : "[BOOT] 5V rail enable...");
  pinMode(FIVE_V_SWITCH_PIN, OUTPUT);
  digitalWrite(FIVE_V_SWITCH_PIN, DIAG_5V_START_DISABLED ? LOW : HIGH);
  fiveVEnabled = !DIAG_5V_START_DISABLED;
  delay(20);
  Serial.println(DIAG_5V_START_DISABLED ? "[BOOT] 5V rail OFF OK - send 5VON to enable" : "[BOOT] 5V rail enable OK");

  Serial.println("[BOOT] LORA_EN high...");
  pinMode(LORA_EN_PIN, OUTPUT);
  digitalWrite(LORA_EN_PIN, HIGH);
  Serial.println("[BOOT] LORA_EN high OK");

  Serial.println("[BOOT] DVR button pad floating (idle)...");
  setDvrFloating();
  Serial.println("[BOOT] DVR button pad floating (idle) OK");

  Serial.println("[BOOT] Wire.begin()...");
  Wire.setSDA(8);
  Wire.setSCL(9);
  Wire.begin();
  Serial.println("[BOOT] Wire.begin() OK");

  constexpr uint32_t I2C_TRANSACTION_TIMEOUT_MS = 100;
  Serial.println("[BOOT] Wire.setTimeout()...");
  Wire.setTimeout(I2C_TRANSACTION_TIMEOUT_MS, true);
  Serial.println("[BOOT] Wire.setTimeout() OK");

  Serial.println("[BOOT] initSensorBoardDevices() (IMU + barometer)...");
  initSensorBoardDevices();
  Serial.println("[BOOT] initSensorBoardDevices() done");

  Serial.println("[BOOT] eepromBegin() (external I2C flight-log chip)...");
  if (!eepromBegin()) {
    Serial.println(
        "NOTE: I2C EEPROM not detected (expected if not soldered yet) - "
        "flight logging disabled.");
    telemetry.flashOK = false;
  } else {
    Serial.print("I2C EEPROM OK, resuming log at offset ");
    Serial.println(eepromWriteOffset);
    telemetry.flashOK = true;
  }

  Serial.println("[BOOT] GPS UART (Serial1) begin()...");
  Serial1.setFIFOSize(512);
  Serial1.setTX(GPS_SERIAL_TX_PIN);
  Serial1.setRX(GPS_SERIAL_RX_PIN);
  Serial1.begin(GPS_BAUD);

  Serial.println("Flyfish M10 GPS UART initialized at 115200 baud");

  Serial.println("[BOOT] GPS settle delay (500ms)...");
  delay(500);
  Serial.println("[BOOT] GPS settle delay OK");

  Serial.println("Configuring GPS (watch for ACKed/NOT acknowledged below):");

  configureGPSRate();
  configureGPSProtocol();
  Serial.println("[BOOT] GPS config handshake done");

  ubxChecksumFailures = 0;
  ubxValidMessageCount = 0;
  navPvtValidCount = 0;

  Serial.println("[BOOT] setupRadio()...");
  setupRadio();
  Serial.println("[BOOT] setupRadio() done");

  Serial.println("[BOOT] boot animation + beep...");
  startBootAnimation();
  triggerEventBeep(BEEP_BOOT);
  Serial.println("[BOOT] boot animation + beep OK");

  Serial.println();
  Serial.println("Starting telemetry...");
  Serial.println();

  Serial.println("[BOOT] wdt_begin(1500)...");
  rp2040.wdt_begin(1500);
  Serial.println("[BOOT] wdt_begin(1500) OK - watchdog armed");


  if (bootWasWatchdogReset) {
    Serial.print("Rebooted via watchdog. Last checkpoint: ");
    Serial.print(lastCheckpoint);
    Serial.print(" (");
    Serial.print(checkpointName(lastCheckpoint));
    Serial.println(")");
  }
  if (bootWasBrownout) {
    Serial.println("Reset cause: BROWNOUT (voltage dip power-cycled the chip).");
  }

  Serial.print("Reset reason: ");
  Serial.println(resetReasonName(rp2040.getResetReason()));

  {
    int8_t lastTxPower = lastTxPowerBeforeThisBoot();
    if (lastTxPower != -128) {
      Serial.print("Last known TX power before this reset: ");
      Serial.print(lastTxPower);
      Serial.println(" dBm");
    } else {
      Serial.println(
          "Last known TX power before this reset: unknown (scratch "
          "registers didn't survive - likely a genuine power interruption, "
          "not a watchdog/software reset).");
    }
  }

  Serial.println("[BOOT] setup() complete, entering loop()...");
}


void loop() {
  static uint32_t lastFast = 0;
  static uint32_t lastStatus = 0;
  static uint32_t lastOrientation = 0;
  static uint32_t lastFlashFastLog = 0;
  static uint32_t lastGPSWarning = 0;
  static bool wasGPSFix = false;
  static uint32_t loopEnteredMs = millis();

  setCheckpoint(CP_LOOP_TOP);

  static bool bootAlertsSent = false;
  if (!bootAlertsSent && (millis() - loopEnteredMs) >= 2000) {
    bootAlertsSent = true;

    if (bootWasWatchdogReset) {
      char msg[Telemetry::ALERT_MESSAGE_LEN];
      snprintf(msg, sizeof(msg), "watchdog reboot, checkpoint=%lu", (unsigned long)lastCheckpoint);
      sendAlert(Telemetry::ALERT_WATCHDOG_REBOOT, Telemetry::ALERT_CRITICAL, msg);
    }

    if (bootWasBrownout) {
      sendAlert(Telemetry::ALERT_BROWNOUT_REBOOT, Telemetry::ALERT_CRITICAL,
                "brownout reset (voltage dip)");
    }
  }

  checkRadioCommands();
  checkSerialCommands();

  setCheckpoint(CP_RADIO_COMMAND);

  setCheckpoint(CP_UPDATE_GPS);
  updateGPS();
  setCheckpoint(CP_UPDATE_IMU);
  updateIMU();
  setCheckpoint(CP_UPDATE_BAROMETER);
  updateBarometer();
  setCheckpoint(CP_UPDATE_POWER);
  updatePower();
  setCheckpoint(CP_UPDATE_FLASH);
  updateFlash();
  setCheckpoint(CP_UPDATE_LORA);
  updateLoRa();

  setCheckpoint(CP_SENSOR_UPDATE);

  uint32_t now = millis();

  setCheckpoint(CP_RETRY_CHECK);
  retryFailedSensorBoardDevices(now);
  setCheckpoint(CP_I2C_RECOVERY_CHECK);
  checkI2cBusRecovery();
  setCheckpoint(CP_RADIO_STUCK_CHECK);
  checkRadioStuckInTx(now);
  setCheckpoint(CP_LAUNCH_DETECT);
  updateLaunchDetect(now);
  setCheckpoint(CP_RECORDING_ARM);
  updateRecordingArm(now);
  setCheckpoint(CP_FIVE_V_AUTOOFF);
  updateFiveVAutoOff(now);
  setCheckpoint(CP_DVR_SERVICE);
  serviceDvrTrigger(now);

  if (telemetry.gpsFix && !wasGPSFix) {
    triggerEventBeep(BEEP_GPS_LOCK);
  }
  wasGPSFix = telemetry.gpsFix;

  setCheckpoint(CP_BUZZER_UPDATE);
  updateBuzzerBackgroundPattern();
  updateBuzzer();
  setCheckpoint(CP_NEOPIXEL_UPDATE);
  updateNeoPixels();

  constexpr uint32_t RADIO_FIRST_TX_DELAY_MS = 1500;
  bool safeToSend = radioSafeToSend() && (now - loopEnteredMs) >= RADIO_FIRST_TX_DELAY_MS;

  setCheckpoint(CP_TX_POWER_RAMP);
  if (telemetry.loraOK) {
    setLastTxPowerScratch(RF95_TX_POWER_DBM);
  }

  setCheckpoint(CP_SEND_SCHEDULE);
  if (now - lastFast >= FAST_INTERVAL_MS && safeToSend) {
    if (sendFastPacket()) {
      lastFast = now;
    }
  }

  if (now - lastStatus >= STATUS_INTERVAL_MS && safeToSend) {
    if (sendStatusPacket()) {
      lastStatus = now;
    }
  }

  if (now - lastOrientation >= ORIENTATION_INTERVAL_MS && safeToSend) {
    if (sendOrientationPacket()) {
      lastOrientation = now;
    }
  }

  if (now - lastFlashFastLog >= FLASH_FAST_LOG_INTERVAL_MS) {
    lastFlashFastLog = now;
    logFastSampleToFlash();
  }

  if (now - lastGPSWarning >= 5000) {
    lastGPSWarning = now;

    if (ubxBytesReceived < 10) {
      Serial.println(
          "WARNING: No GPS data received at all. Check that the GPS "
          "module is plugged in, wired to GPIO12/GPIO13, and configured "
          "for 115200 baud.");
    } else if (ubxValidMessageCount == 0) {
      Serial.print("WARNING: Receiving bytes but zero valid UBX messages since boot config (");
      Serial.print(ubxChecksumFailures);
      Serial.println(
          " checksum failures). This means garbled data, not \"no fix\" - "
          "almost always a baud rate mismatch or noisy/loose wiring, not a "
          "sky-view problem.");
    } else if (navPvtValidCount == 0) {
      Serial.print("WARNING: ");
      Serial.print(ubxValidMessageCount);
      Serial.print(" other UBX message(s) validated fine, but zero NAV-PVT (");
      Serial.print(ubxChecksumFailures);
      Serial.println(
          " checksum failures). The link/protocol works, but position "
          "messages specifically aren't coming through - not a sky-view "
          "issue, something is wrong with the NAV-PVT enable or its "
          "larger 92-byte frames specifically.");
    } else if (!telemetry.gpsFix) {
      Serial.print("GPS: link OK (");
      Serial.print(navPvtValidCount);
      Serial.print(" NAV-PVT ok, ");
      Serial.print(ubxChecksumFailures);
      Serial.print(" checksum failures so far), ");
      Serial.print(telemetry.gpsSatellites);
      Serial.println(
          " satellites, but no 3D fix yet - needs clearer sky view. "
          "Lat/lon hold the last known-good position until this clears.");
    }
  }

  rp2040.wdt_reset();
}
