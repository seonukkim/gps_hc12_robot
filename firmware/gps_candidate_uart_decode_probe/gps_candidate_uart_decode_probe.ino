// firmware/gps_candidate_uart_decode_probe/gps_candidate_uart_decode_probe.ino
//
// Read-only bit-bang UART decode probe for OpenRB-150 GPS candidate pins.
//
// Purpose: edge activity alone is not enough. This sketch listens to a small set
// of candidate pins and tries to decode 8N1 UART text without rewiring. It is
// intended to distinguish real GPS NMEA from floating/noisy pin activity.
//
// SAFETY / READ-ONLY:
// - Uses USB Serial for logs only.
// - Candidate pins are set to INPUT only and read with digitalRead only.
// - Never writes Serial1, Serial2, or Serial3.
// - Does not initialize motors, RC/PPM, HC-12, I2C, GPS parser, or autonomy.
// - Does not modify motor/path-following behavior and enables no motion.

#include <Arduino.h>

constexpr long USB_BAUD = 115200;

#ifndef UART_DECODE_SECONDS
#define UART_DECODE_SECONDS 12
#endif

#ifndef UART_START_WAIT_US
#define UART_START_WAIT_US 250
#endif

constexpr uint32_t DECODE_MS = static_cast<uint32_t>(UART_DECODE_SECONDS) * 1000UL;
constexpr uint32_t START_WAIT_US = UART_START_WAIT_US;
constexpr size_t PREVIEW_LIMIT = 180;

struct PinCandidate {
  uint8_t pin;
  const char *label;
  const char *known_function;
};

PinCandidate pins[] = {
    {6, "D6", "selected_activity_candidate_unknown_role"},
    {13, "D13", "Serial3_RX_PB23_side_exp_candidate"},
    {27, "D27", "Serial1_RX_PA13_SD_SPI_candidate"},
    {29, "D29", "Serial2_RX_PA15_center_5pin_expected_GPS_TX_target"},
};

constexpr uint8_t PIN_COUNT = sizeof(pins) / sizeof(pins[0]);
uint32_t bauds[] = {9600, 4800, 38400, 57600, 115200};
constexpr uint8_t BAUD_COUNT = sizeof(bauds) / sizeof(bauds[0]);

struct DecodeStats {
  uint32_t byte_count;
  uint32_t printable_count;
  uint32_t framing_error_count;
  bool contains_dollar;
  bool contains_gp;
  bool contains_gn;
  bool contains_rmc;
  bool contains_gga;
  bool contains_nmea_like;
  char preview[PREVIEW_LIMIT + 1];
  size_t preview_len;
  char rolling[4];
};

void resetStats(DecodeStats *stats) {
  memset(stats, 0, sizeof(*stats));
}

void appendPreviewByte(DecodeStats *stats, uint8_t b) {
  if (stats->preview_len >= PREVIEW_LIMIT) {
    return;
  }

  if (b == '\r') {
    if (stats->preview_len + 2 <= PREVIEW_LIMIT) {
      stats->preview[stats->preview_len++] = '\\';
      stats->preview[stats->preview_len++] = 'r';
    }
  } else if (b == '\n') {
    if (stats->preview_len + 2 <= PREVIEW_LIMIT) {
      stats->preview[stats->preview_len++] = '\\';
      stats->preview[stats->preview_len++] = 'n';
    }
  } else if (b == '\t') {
    if (stats->preview_len + 2 <= PREVIEW_LIMIT) {
      stats->preview[stats->preview_len++] = '\\';
      stats->preview[stats->preview_len++] = 't';
    }
  } else if (b >= 32 && b <= 126) {
    stats->preview[stats->preview_len++] = static_cast<char>(b);
  } else {
    if (stats->preview_len + 4 <= PREVIEW_LIMIT) {
      const char hex[] = "0123456789ABCDEF";
      stats->preview[stats->preview_len++] = '\\';
      stats->preview[stats->preview_len++] = 'x';
      stats->preview[stats->preview_len++] = hex[(b >> 4) & 0x0F];
      stats->preview[stats->preview_len++] = hex[b & 0x0F];
    }
  }
  stats->preview[stats->preview_len] = '\0';
}

void updatePatternFlags(DecodeStats *stats, uint8_t b) {
  if (b == '$') {
    stats->contains_dollar = true;
  }

  stats->rolling[0] = stats->rolling[1];
  stats->rolling[1] = stats->rolling[2];
  stats->rolling[2] = static_cast<char>(b);
  stats->rolling[3] = '\0';

  if (strcmp(stats->rolling, "$GP") == 0) {
    stats->contains_gp = true;
  }
  if (strcmp(stats->rolling, "$GN") == 0) {
    stats->contains_gn = true;
  }
  if (strcmp(stats->rolling, "RMC") == 0) {
    stats->contains_rmc = true;
  }
  if (strcmp(stats->rolling, "GGA") == 0) {
    stats->contains_gga = true;
  }

  stats->contains_nmea_like =
      stats->contains_gp || stats->contains_gn || stats->contains_rmc || stats->contains_gga;
}

void recordByte(DecodeStats *stats, uint8_t b, bool stop_high) {
  stats->byte_count++;
  if (b >= 32 && b <= 126) {
    stats->printable_count++;
  }
  if (!stop_high) {
    stats->framing_error_count++;
  }
  appendPreviewByte(stats, b);
  updatePatternFlags(stats, b);
}

bool readOneByte8n1(uint8_t pin, uint32_t bit_us, uint8_t *out_byte, bool *stop_high) {
  // Called just after a HIGH->LOW start edge has been observed.
  delayMicroseconds(bit_us + bit_us / 2);

  uint8_t value = 0;
  for (uint8_t bit = 0; bit < 8; ++bit) {
    if (digitalRead(pin) == HIGH) {
      value |= (1 << bit);
    }
    delayMicroseconds(bit_us);
  }

  *stop_high = (digitalRead(pin) == HIGH);
  *out_byte = value;
  delayMicroseconds(bit_us);
  return true;
}

void decodePinBaud(const PinCandidate &candidate, uint32_t baud) {
  pinMode(candidate.pin, INPUT);  // INPUT only, no pullup.
  delay(5);

  DecodeStats stats;
  resetStats(&stats);
  const uint32_t bit_us = 1000000UL / baud;
  const uint32_t start_ms = millis();
  int last = digitalRead(candidate.pin);

  Serial.print(F("UART_DECODE_START pin_number="));
  Serial.print(candidate.pin);
  Serial.print(F(" label="));
  Serial.print(candidate.label);
  Serial.print(F(" baud="));
  Serial.print(baud);
  Serial.print(F(" seconds="));
  Serial.print(UART_DECODE_SECONDS);
  Serial.print(F(" known_function="));
  Serial.println(candidate.known_function);

  while (millis() - start_ms < DECODE_MS) {
    const int now = digitalRead(candidate.pin);
    if (last == HIGH && now == LOW) {
      uint8_t b = 0;
      bool stop_high = false;
      readOneByte8n1(candidate.pin, bit_us, &b, &stop_high);
      recordByte(&stats, b, stop_high);
      last = digitalRead(candidate.pin);
      continue;
    }
    last = now;
    delayMicroseconds(START_WAIT_US);
  }

  Serial.print(F("UART_DECODE_RESULT pin_number="));
  Serial.print(candidate.pin);
  Serial.print(F(" label="));
  Serial.print(candidate.label);
  Serial.print(F(" baud="));
  Serial.print(baud);
  Serial.print(F(" byte_count="));
  Serial.print(stats.byte_count);
  Serial.print(F(" printable_count="));
  Serial.print(stats.printable_count);
  Serial.print(F(" framing_error_count="));
  Serial.print(stats.framing_error_count);
  Serial.print(F(" contains_dollar="));
  Serial.print(stats.contains_dollar ? F("true") : F("false"));
  Serial.print(F(" contains_gp="));
  Serial.print(stats.contains_gp ? F("true") : F("false"));
  Serial.print(F(" contains_gn="));
  Serial.print(stats.contains_gn ? F("true") : F("false"));
  Serial.print(F(" contains_rmc="));
  Serial.print(stats.contains_rmc ? F("true") : F("false"));
  Serial.print(F(" contains_gga="));
  Serial.print(stats.contains_gga ? F("true") : F("false"));
  Serial.print(F(" nmea_like="));
  Serial.print(stats.contains_nmea_like ? F("true") : F("false"));
  Serial.print(F(" ascii_preview=\""));
  Serial.print(stats.preview);
  Serial.print(F("\" known_function="));
  Serial.println(candidate.known_function);
}

void setup() {
  Serial.begin(USB_BAUD);
  const uint32_t start_ms = millis();
  while (!Serial && millis() - start_ms < 8000) {
    delay(10);
  }

  Serial.println();
  Serial.println(F("Firmware: gps_candidate_uart_decode_probe read-only-bitbang-uart 2026-06-03"));
  Serial.println(F("READ-ONLY: USB logs only. Candidate pins set INPUT, digitalRead only."));
  Serial.println(F("Never writes Serial1/2/3. No motors, RC, HC-12, I2C, GPS parser, autonomy, or motion."));
  Serial.print(F("candidate_pins="));
  for (uint8_t i = 0; i < PIN_COUNT; ++i) {
    Serial.print(F("D"));
    Serial.print(pins[i].pin);
    if (i + 1 < PIN_COUNT) {
      Serial.print(F(","));
    }
  }
  Serial.println();
  Serial.print(F("bauds="));
  for (uint8_t i = 0; i < BAUD_COUNT; ++i) {
    Serial.print(bauds[i]);
    if (i + 1 < BAUD_COUNT) {
      Serial.print(F(","));
    }
  }
  Serial.println();
  Serial.print(F("seconds_each="));
  Serial.println(UART_DECODE_SECONDS);
  Serial.println(F("This decodes electrical UART text only. NMEA-like output must still be validated by UART probe."));
}

void loop() {
  Serial.println();
  Serial.println(F("UART_DECODE_SWEEP_START"));
  for (uint8_t baud_i = 0; baud_i < BAUD_COUNT; ++baud_i) {
    for (uint8_t pin_i = 0; pin_i < PIN_COUNT; ++pin_i) {
      decodePinBaud(pins[pin_i], bauds[baud_i]);
      delay(100);
    }
  }
  Serial.println(F("UART_DECODE_SWEEP_DONE"));
  delay(5000);
}
