
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <math.h>
#include <ctype.h>
#include <string.h>
#include <RH_RF95.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_NeoPixel.h>

#define TELEMETRY_DEBUG_ASSERTS
#include "packet.h"


constexpr uint8_t RFM95_SCK_PIN = 18;
constexpr uint8_t RFM95_MISO_PIN = 16;
constexpr uint8_t RFM95_MOSI_PIN = 19;
constexpr uint8_t RFM95_CS = 8;
constexpr uint8_t RFM95_RST = 9;
constexpr uint8_t RFM95_INT = 7;

constexpr float RF95_FREQ_MHZ = 433.0;

constexpr uint8_t RF95_SPREADING_FACTOR = 8;
constexpr long RF95_BANDWIDTH_HZ = 250000;
constexpr uint8_t RF95_CODING_RATE_DENOM = 5;
constexpr int8_t RF95_TX_POWER_DBM = 20;

RH_RF95 rf95(RFM95_CS, RFM95_INT);
bool radioOK = false;

constexpr uint8_t OLED_SDA_PIN = 0;
constexpr uint8_t OLED_SCL_PIN = 1;
constexpr uint8_t OLED_I2C_ADDR = 0x3C;
constexpr uint8_t OLED_WIDTH = 128;
constexpr uint8_t OLED_HEIGHT = 64;

Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);
bool displayOK = false;

constexpr uint8_t NEOPIXEL_PIN = 22;
constexpr uint8_t NEOPIXEL_COUNT = 3;
constexpr uint8_t NEOPIXEL_BRIGHTNESS = 38;

Adafruit_NeoPixel pixels(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

constexpr uint8_t RX_PIXEL = 0;
constexpr uint8_t TX_PIXEL = 1;
constexpr uint8_t IDLE_PIXEL = 2;

constexpr uint32_t RX_BLINK_MS = 80;
constexpr uint32_t TX_BLINK_MS = 80;
constexpr float GLOW_PERIOD_MS = 2400.0f;
constexpr float GLOW_MIN = 0.05f;
constexpr float GLOW_MAX = 0.55f;

uint32_t rxUntilMs = 0;
uint32_t txStartMs = 0;
uint32_t txUntilMs = 0;

void triggerPixelEvent() {
  uint32_t now = millis();
  rxUntilMs = now + RX_BLINK_MS;
  txStartMs = rxUntilMs;
  txUntilMs = txStartMs + TX_BLINK_MS;
}

void updatePixels(uint32_t now) {
  float phase = fmodf(static_cast<float>(now), GLOW_PERIOD_MS) / GLOW_PERIOD_MS;
  float level = GLOW_MIN + (GLOW_MAX - GLOW_MIN) * (0.5f - 0.5f * cosf(phase * 2.0f * PI));
  uint8_t v = static_cast<uint8_t>(roundf(level * 255.0f));

  pixels.setPixelColor(IDLE_PIXEL, pixels.Color(v, v, v));
  pixels.setPixelColor(RX_PIXEL, (now < rxUntilMs) ? pixels.Color(0, 0, 255) : pixels.Color(0, 0, 0));
  bool txOn = (now >= txStartMs) && (now < txUntilMs);
  pixels.setPixelColor(TX_PIXEL, txOn ? pixels.Color(255, 0, 0) : pixels.Color(0, 0, 0));
  pixels.show();
}


constexpr uint32_t LINK_TIMEOUT_MS = 3000;

struct GroundState {
  bool linkUp = false;
  int16_t rssi = -127;
  int8_t snr = 0;
  bool gpsFix = false;
  uint8_t gpsSatellites = 0;
  float altitudeM = 0.0f;
  uint16_t batteryMV = 0;
  uint32_t totalGood = 0;
  uint16_t packetRate = 0;
  uint32_t lastRxMs = 0;
};

GroundState state;


void printRawPacket(const uint8_t* buffer, uint8_t length) {
  for (uint8_t i = 0; i < length; i++) {
    if (buffer[i] < 0x10) {
      Serial.print('0');
    }
    Serial.print(buffer[i], HEX);
    Serial.print(' ');
  }
  Serial.println();
}

void patchStatusRssi(uint8_t* buffer) {
  int16_t rssi = constrain(rf95.lastRssi(), -127, 127);
  int8_t snr = constrain(rf95.lastSNR(), -127, 127);
  buffer[14] = static_cast<uint8_t>(rssi);
  buffer[15] = static_cast<uint8_t>(snr);

  size_t crcCoverage = Telemetry::STATUS_PACKET_SIZE - 2;
  uint16_t crc = Telemetry::crc16(buffer, crcCoverage);
  buffer[crcCoverage] = crc & 0xFF;
  buffer[crcCoverage + 1] = (crc >> 8) & 0xFF;
}

bool handlePacket(uint8_t* buffer, uint8_t length) {
  if (length < 4 || buffer[0] != Telemetry::MAGIC_0 || buffer[1] != Telemetry::MAGIC_1) {
    return false;
  }

  uint8_t packetType = buffer[3];
  size_t expected;
  switch (packetType) {
    case Telemetry::PACKET_FAST:        expected = Telemetry::FAST_PACKET_SIZE; break;
    case Telemetry::PACKET_STATUS:      expected = Telemetry::STATUS_PACKET_SIZE; break;
    case Telemetry::PACKET_ORIENTATION: expected = Telemetry::ORIENTATION_PACKET_SIZE; break;
    case Telemetry::PACKET_ACK:         expected = Telemetry::ACK_PACKET_SIZE; break;
    case Telemetry::PACKET_ALERT:       expected = Telemetry::ALERT_PACKET_SIZE; break;
    default: return false;
  }
  if (length != expected) {
    return false;
  }

  if (packetType == Telemetry::PACKET_FAST) {
    int16_t altCm = static_cast<int16_t>(buffer[18] | (buffer[19] << 8));
    state.altitudeM = altCm / 100.0f;
    uint8_t status = buffer[36];
    state.gpsFix = (status & Telemetry::STATUS_GPS_FIX) != 0;
    state.gpsSatellites = buffer[37];
  } else if (packetType == Telemetry::PACKET_STATUS) {
    patchStatusRssi(buffer);
    state.batteryMV = static_cast<uint16_t>(buffer[10] | (buffer[11] << 8));
  }

  printRawPacket(buffer, length);

  state.rssi = rf95.lastRssi();
  state.snr = rf95.lastSNR();
  state.linkUp = true;
  state.totalGood++;
  state.lastRxMs = millis();
  return true;
}


char serialCmdBuf[80];
uint8_t serialCmdLen = 0;

int hexNibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

void handleSerialLine(const char* line) {
  if (strncmp(line, "TX:", 3) != 0) {
    return;
  }

  const char* hexText = line + 3;
  size_t hexLen = strlen(hexText);
  size_t payloadLen = hexLen / 2;

  if (hexLen == 0 || hexLen % 2 != 0 || payloadLen > RH_RF95_MAX_MESSAGE_LEN) {
    Serial.println("# BAD TX HEX");
    return;
  }

  uint8_t payload[RH_RF95_MAX_MESSAGE_LEN];
  for (size_t i = 0; i < payloadLen; i++) {
    int hi = hexNibble(hexText[i * 2]);
    int lo = hexNibble(hexText[i * 2 + 1]);
    if (hi < 0 || lo < 0) {
      Serial.println("# BAD TX HEX");
      return;
    }
    payload[i] = static_cast<uint8_t>((hi << 4) | lo);
  }

  if (!radioOK) {
    Serial.println("# TX requested but radio not available");
    return;
  }

  if (!rf95.send(payload, payloadLen)) {
    Serial.println("# TX FAILED: rf95.send() returned false (radio busy/stuck?)");
    return;
  }
  if (!rf95.waitPacketSent(500)) {
    Serial.println("# TX FAILED: waitPacketSent() timed out after 500ms - TX likely never completed");
    return;
  }
  triggerPixelEvent();

  Serial.print("# TX ");
  Serial.println(hexText);
}

void checkSerialCommands() {
  while (Serial.available() > 0) {
    char c = static_cast<char>(Serial.read());

    if (c == '\n' || c == '\r') {
      if (serialCmdLen > 0) {
        serialCmdBuf[serialCmdLen] = '\0';
        handleSerialLine(serialCmdBuf);
        serialCmdLen = 0;
      }
    } else if (serialCmdLen < sizeof(serialCmdBuf) - 1) {
      serialCmdBuf[serialCmdLen++] = c;
    }
  }
}


constexpr uint32_t PAGE_INTERVAL_MS = 500;
uint8_t pageIndex = 0;
uint32_t lastPageSwitchMs = 0;

void updateDisplay(uint32_t now) {
  if (!displayOK) {
    return;
  }
  if (now - lastPageSwitchMs < PAGE_INTERVAL_MS) {
    return;
  }
  lastPageSwitchMs = now;
  pageIndex = (pageIndex + 1) % 2;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  display.println("GROUND STATION");
  if (state.linkUp && (now - state.lastRxMs) <= LINK_TIMEOUT_MS) {
    display.print("LINK OK  ");
    display.print(state.packetRate);
    display.println(" pkt/s");
  } else {
    display.println("NO SIGNAL");
  }

  display.setCursor(0, 40);
  if (pageIndex == 0) {
    display.print("RSSI ");
    display.print(state.rssi);
    display.print("dBm SNR ");
    display.println(state.snr);
    display.print(state.packetRate);
    display.print(" pkt/s TOTAL ");
    display.println(state.totalGood);
  } else {
    display.print("GPS ");
    display.print(state.gpsFix ? "FIX" : "NO FIX");
    display.print(" sats ");
    display.println(state.gpsSatellites);
    display.print("ALT ");
    display.print(state.altitudeM * 3.28084f, 0);
    display.print("ft BATT ");
    display.print(state.batteryMV / 1000.0f, 2);
    display.println("V");
  }

  display.display();
}


void setup() {
  Serial.begin(115200);

  pixels.begin();
  pixels.setBrightness(NEOPIXEL_BRIGHTNESS);
  pixels.clear();
  pixels.show();

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
    Serial.println("# RADIO INIT FAILED");
    radioOK = false;
  } else {
    rf95.setFrequency(RF95_FREQ_MHZ);
    rf95.setTxPower(RF95_TX_POWER_DBM, false);
    rf95.setSpreadingFactor(RF95_SPREADING_FACTOR);
    rf95.setSignalBandwidth(RF95_BANDWIDTH_HZ);
    rf95.setCodingRate4(RF95_CODING_RATE_DENOM);
    radioOK = true;
    Serial.println("# Radio OK");
  }

  Wire.setSDA(OLED_SDA_PIN);
  Wire.setSCL(OLED_SCL_PIN);
  Wire.begin();

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR)) {
    Serial.println("# OLED INIT FAILED");
    displayOK = false;
  } else {
    displayOK = true;
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("GROUND STATION");
    display.println("NO SIGNAL");
    display.display();
  }
}


uint32_t rateWindowStartMs = 0;
uint32_t rateWindowCount = 0;

void loop() {
  checkSerialCommands();

  if (radioOK && rf95.available()) {
    uint8_t buffer[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t length = sizeof(buffer);
    if (rf95.recv(buffer, &length) && handlePacket(buffer, length)) {
      triggerPixelEvent();
      rateWindowCount++;
    }
  }

  uint32_t now = millis();

  if (now - rateWindowStartMs >= 1000) {
    state.packetRate = static_cast<uint16_t>(rateWindowCount);
    rateWindowCount = 0;
    rateWindowStartMs = now;
  }

  if (state.linkUp && (now - state.lastRxMs) > LINK_TIMEOUT_MS) {
    state.linkUp = false;
  }

  updateDisplay(now);
  updatePixels(now);
}
