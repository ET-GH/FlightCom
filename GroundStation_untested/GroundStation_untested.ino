#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>
#include "rocket_protocol.h"

#define LORA_SCK 5
#define LORA_MISO 3
#define LORA_MOSI 6
#define LORA_CS 7
#define LORA_RST 8
#define LORA_DIO1 9
#define LORA_BUSY 36
#define LORA_TXEN 10
#define LORA_RXEN 21

static constexpr float RADIO_FREQUENCY_MHZ = 2445.0;
static constexpr float RADIO_BANDWIDTH_KHZ = 203.125;
static constexpr uint8_t RADIO_SPREADING_FACTOR = 8;
static constexpr uint8_t RADIO_CODING_RATE_DENOMINATOR = 5;
static constexpr int8_t RADIO_TX_POWER_DBM = 3;
static constexpr uint16_t RADIO_PREAMBLE_SYMBOLS = 12;

SX1280 radio = new Module(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY);
volatile bool rxFlag = false;
static uint16_t txSequence = 0;

void onRadioDio1() { rxFlag = true; }

static void startReceiveMode() {
  int state = radio.startReceive();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print("RX_START_ERR,"); Serial.println(state);
  }
}

static bool sendBinary(const uint8_t *data, size_t length) {
  radio.standby();
  int state = radio.transmit(data, length);
  startReceiveMode();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print("TX_ERR,"); Serial.println(state);
    return false;
  }
  return true;
}

static void sendCommand(uint8_t command, const uint8_t *payload, uint8_t payloadLength) {
  uint8_t packet[ROCKET_PROTOCOL_HEADER_SIZE + 2 + 8];
  if (payloadLength > 8) return;
  size_t i = RocketProtocol_EncodeHeader(packet, sizeof(packet), ROCKET_PKT_COMMAND,
                                          txSequence++, millis());
  packet[i++] = command;
  packet[i++] = payloadLength;
  if (payloadLength) { memcpy(packet + i, payload, payloadLength); i += payloadLength; }
  if (sendBinary(packet, i)) {
    Serial.print("COMMAND_SENT,seq="); Serial.print((uint16_t)(txSequence - 1));
    Serial.print(",command="); Serial.println(RocketProtocol_CommandText(command));
  }
}

static void printFlags(uint16_t f) {
  Serial.print("flags=0x"); Serial.print(f, HEX);
  if (f & ROCKET_FLAG_INITIALIZED) Serial.print(",INITIALIZED");
  if (f & ROCKET_FLAG_BARO_VALID) Serial.print(",BARO_VALID");
  if (f & ROCKET_FLAG_FUSION_VALID) Serial.print(",FUSION_VALID");
  if (f & ROCKET_FLAG_EKF_VALID) Serial.print(",EKF_VALID");
  if (f & ROCKET_FLAG_CONTROLLER_ENABLED) Serial.print(",CONTROLLER_ENABLED");
  if (f & ROCKET_FLAG_CONTROLLER_ACTIVE) Serial.print(",CONTROLLER_ACTIVE");
  if (f & ROCKET_FLAG_APOGEE_REACHED) Serial.print(",APOGEE_REACHED");
  if (f & ROCKET_FLAG_SENSOR_FAULT) Serial.print(",SENSOR_FAULT");
  Serial.println();
}

static const char *sensorHealthText(uint8_t value) {
  switch (value & 0x03u) {
    case ROCKET_SENSOR_OK: return "OK";
    case ROCKET_SENSOR_STALE: return "STALE";
    case ROCKET_SENSOR_FAULT: return "FAULT";
    default: return "UNKNOWN";
  }
}

static void parseTelemetry(const uint8_t *p, size_t len, const RocketPacketHeader &h) {
  if (len < ROCKET_PROTOCOL_HEADER_SIZE + 29u) { Serial.println("PARSE_ERR,telemetry_length"); return; }
  size_t i = ROCKET_PROTOCOL_HEADER_SIZE;
  uint16_t flags = RocketProtocol_ReadU16(p + i); i += 2;
  uint8_t state = p[i++];
  uint8_t status = p[i++];
  float altitude = RocketProtocol_ReadI16(p + i) / 10.0f; i += 2;
  float velocity = RocketProtocol_ReadI16(p + i) / 100.0f; i += 2;
  float acceleration = RocketProtocol_ReadI16(p + i) / 100.0f; i += 2;
  float predicted = RocketProtocol_ReadU16(p + i) / 10.0f; i += 2;
  float target = RocketProtocol_ReadU16(p + i) / 10.0f; i += 2;
  float roll = RocketProtocol_ReadI16(p + i) / 10.0f; i += 2;
  float pitch = RocketProtocol_ReadI16(p + i) / 10.0f; i += 2;
  float yaw = RocketProtocol_ReadI16(p + i) / 10.0f; i += 2;
  uint8_t deployment = p[i++];
  uint8_t health = p[i++];
  uint16_t failedReads = RocketProtocol_ReadU16(p + i); i += 2;
  uint8_t message = p[i++];

  Serial.print("TELEMETRY,seq="); Serial.print(h.sequence);
  Serial.print(",time_ms="); Serial.print(h.time_ms);
  Serial.print(",mode="); Serial.print((state >> 4) & 0x0F);
  Serial.print(",phase="); Serial.print(state & 0x0F);
  Serial.print(",status="); Serial.print(RocketProtocol_StatusText(status));
  Serial.print(",altitude_m="); Serial.print(altitude, 1);
  Serial.print(",velocity_mps="); Serial.print(velocity, 2);
  Serial.print(",acceleration_mps2="); Serial.print(acceleration, 2);
  Serial.print(",predicted_apogee_m="); Serial.print(predicted, 1);
  Serial.print(",target_apogee_m="); Serial.print(target, 1);
  Serial.print(",roll_deg="); Serial.print(roll, 1);
  Serial.print(",pitch_deg="); Serial.print(pitch, 1);
  Serial.print(",yaw_deg="); Serial.print(yaw, 1);
  Serial.print(",deployment_pct="); Serial.print(deployment);
  Serial.print(",imu="); Serial.print(sensorHealthText(health));
  Serial.print(",mag="); Serial.print(sensorHealthText(health >> 2));
  Serial.print(",baro="); Serial.print(sensorHealthText(health >> 4));
  Serial.print(",failed_reads="); Serial.print(failedReads);
  Serial.print(",message="); Serial.println(RocketProtocol_MessageText(message));
  printFlags(flags);
}

static void parseEvent(const uint8_t *p, size_t len, const RocketPacketHeader &h) {
  if (len < ROCKET_PROTOCOL_HEADER_SIZE + 10u) { Serial.println("PARSE_ERR,event_length"); return; }
  size_t i = ROCKET_PROTOCOL_HEADER_SIZE;
  uint16_t changed = RocketProtocol_ReadU16(p + i); i += 2;
  uint16_t current = RocketProtocol_ReadU16(p + i); i += 2;
  uint8_t oldState = p[i++], newState = p[i++], status = p[i++], message = p[i++];
  uint16_t detail = RocketProtocol_ReadU16(p + i);
  Serial.print("EVENT,seq="); Serial.print(h.sequence);
  Serial.print(",time_ms="); Serial.print(h.time_ms);
  Serial.print(",message="); Serial.print(RocketProtocol_MessageText(message));
  Serial.print(",status="); Serial.print(RocketProtocol_StatusText(status));
  Serial.print(",old_mode="); Serial.print(oldState >> 4);
  Serial.print(",old_phase="); Serial.print(oldState & 0x0F);
  Serial.print(",new_mode="); Serial.print(newState >> 4);
  Serial.print(",new_phase="); Serial.print(newState & 0x0F);
  Serial.print(",changed_flags=0x"); Serial.print(changed, HEX);
  Serial.print(",current_flags=0x"); Serial.print(current, HEX);
  Serial.print(",detail="); Serial.println(detail);
}

static void parseAck(const uint8_t *p, size_t len, const RocketPacketHeader &h) {
  if (len < ROCKET_PROTOCOL_HEADER_SIZE + 6u) { Serial.println("PARSE_ERR,ack_length"); return; }
  size_t i = ROCKET_PROTOCOL_HEADER_SIZE;
  uint16_t commandSequence = RocketProtocol_ReadU16(p + i); i += 2;
  uint8_t command = p[i++], result = p[i++];
  uint16_t detail = RocketProtocol_ReadU16(p + i);
  Serial.print("ACK,seq="); Serial.print(h.sequence);
  Serial.print(",command_seq="); Serial.print(commandSequence);
  Serial.print(",command="); Serial.print(RocketProtocol_CommandText(command));
  Serial.print(",result="); Serial.print(result);
  Serial.print(",detail="); Serial.println(detail);
}

static void parsePacket(const uint8_t *p, size_t len) {
  RocketPacketHeader h;
  if (!RocketProtocol_DecodeHeader(p, len, &h)) { Serial.println("PARSE_ERR,bad_header"); return; }
  switch (h.type) {
    case ROCKET_PKT_TELEMETRY: parseTelemetry(p, len, h); break;
    case ROCKET_PKT_EVENT: parseEvent(p, len, h); break;
    case ROCKET_PKT_ACK: parseAck(p, len, h); break;
    default: Serial.print("PACKET,unknown_type="); Serial.println(h.type); break;
  }
}

static void processSerialCommand(String line) {
  line.trim();
  if (line == "ping") { sendCommand(ROCKET_CMD_PING, nullptr, 0); return; }
  if (line == "snapshot") { sendCommand(ROCKET_CMD_REQUEST_SNAPSHOT, nullptr, 0); return; }
  if (line == "standard") { sendCommand(ROCKET_CMD_RETURN_STANDARD, nullptr, 0); return; }
  if (line.startsWith("target ")) {
    float metres = line.substring(7).toFloat();
    uint16_t dm = (uint16_t)constrain(lroundf(metres * 10.0f), 0L, 65535L);
    uint8_t b[2]; RocketProtocol_WriteU16(b, dm);
    sendCommand(ROCKET_CMD_SET_TARGET_APOGEE, b, 2); return;
  }
  if (line.startsWith("controller ")) {
    uint8_t enabled = (line.substring(11).toInt() != 0) ? 1 : 0;
    sendCommand(ROCKET_CMD_SET_CONTROLLER, &enabled, 1); return;
  }
  if (line.startsWith("airbrake ")) {
    uint8_t pct = (uint8_t)constrain(line.substring(9).toInt(), 0, 100);
    sendCommand(ROCKET_CMD_MANUAL_AIRBRAKE, &pct, 1); return;
  }
  if (line.startsWith("mode ")) {
    int firstSpace = line.indexOf(' ', 5);
    uint8_t mode = (uint8_t)line.substring(5, firstSpace < 0 ? line.length() : firstSpace).toInt();
    uint16_t seconds = firstSpace < 0 ? 0 : (uint16_t)line.substring(firstSpace + 1).toInt();
    uint8_t b[3] = { mode, 0, 0 }; RocketProtocol_WriteU16(b + 1, seconds);
    sendCommand(ROCKET_CMD_SET_MODE, b, 3); return;
  }
  Serial.println("COMMANDS: ping | snapshot | target <m> | controller <0|1> | airbrake <0-100> | mode <id> <seconds> | standard");
}

void setup() {
  Serial.begin(115200); delay(1500);
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  radio.setRfSwitchPins(LORA_RXEN, LORA_TXEN);
  int state = radio.begin(RADIO_FREQUENCY_MHZ, RADIO_BANDWIDTH_KHZ,
                          RADIO_SPREADING_FACTOR, RADIO_CODING_RATE_DENOMINATOR,
                          RADIOLIB_SX128X_SYNC_WORD_PRIVATE, RADIO_TX_POWER_DBM,
                          RADIO_PREAMBLE_SYMBOLS);
  if (state != RADIOLIB_ERR_NONE) { Serial.print("RADIO_INIT_ERR,"); Serial.println(state); while (true) delay(1000); }
  radio.explicitHeader(); radio.setCRC(2); radio.setDio1Action(onRadioDio1);
  Serial.println("RADIO_READY,protocol=2");
  startReceiveMode();
}

void loop() {
  if (Serial.available()) processSerialCommand(Serial.readStringUntil('\n'));
  if (rxFlag) {
    rxFlag = false;
    uint8_t packet[255]; size_t len = radio.getPacketLength();
    if (len > sizeof(packet)) len = sizeof(packet);
    int state = radio.readData(packet, len);
    if (state == RADIOLIB_ERR_NONE) {
      Serial.print("LINK,RSSI="); Serial.print(radio.getRSSI());
      Serial.print(",SNR="); Serial.print(radio.getSNR());
      Serial.print(",LEN="); Serial.println(len);
      parsePacket(packet, len);
    } else { Serial.print("RX_ERR,"); Serial.println(state); }
    startReceiveMode();
  }
}
