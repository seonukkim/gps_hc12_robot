// firmware/gps_pin_activity_probe/gps_pin_activity_probe.ino
//
// Passive PIN-LEVEL activity probe for OpenRB-150. Proves, electrically, whether
// a GPS TX signal actually reaches any OpenRB UART pin — independent of UART/baud.
//
// A powered, transmitting 3.3 V UART TX line idles HIGH and pulses LOW during
// data, so a connected GPS produces MANY transitions on the OpenRB RX pin. If the
// pin shows no transitions (static HIGH / static LOW), or only floating noise,
// the GPS TX signal is NOT arriving — a power / connector / TX-not-connected
// problem, not a baud/firmware problem.
//
// SAFETY / READ-ONLY: USB Serial is used for logs only. Candidate pins are set to
// INPUT (NO pullup) and only read with digitalRead. This sketch NEVER writes to
// Serial1/Serial2/Serial3, and does NOT init or drive motors/ESC/Servo, RC/PPM,
// HC-12, I2C, or any autonomy. It changes no motor/path-following behaviour and
// enables no motion.
//
// Candidate pins (OpenRB-150 variant), the "likely RX" ones are GPS-TX targets:
//   D13 Serial3 RX (PB23)   D14 Serial3 TX (PB22)
//   D26 Serial1 TX (PA12)   D27 Serial1 RX (PA13)
//   D28 Serial2 TX (PA14, center 5-pin)   D29 Serial2 RX (PA15, center 5-pin)
// The GPS is wired to the center 5-pin connector, so D29 (Serial2 RX) is the
// prime GPS-TX target.

#include <Arduino.h>

constexpr long USB_BAUD = 115200;
#ifndef PIN_SAMPLE_MS
#define PIN_SAMPLE_MS 10000
#endif
#ifndef UART_ACTIVITY_MIN_TRANSITIONS
#define UART_ACTIVITY_MIN_TRANSITIONS 100
#endif
constexpr uint32_t SAMPLE_MS = PIN_SAMPLE_MS;
constexpr uint32_t ACTIVITY_MIN_TRANS = UART_ACTIVITY_MIN_TRANSITIONS;

// Per-pin category codes.
constexpr int CAT_ACTIVITY = 0;
constexpr int CAT_STATIC_HIGH = 1;
constexpr int CAT_STATIC_LOW = 2;
constexpr int CAT_FLOATING = 3;

struct PinCand {
  uint8_t pin;
  const char *label;
  const char *func;  // underscores only, so the field stays space-delimited
  bool likelyRx;     // a UART RX pin = where a GPS TX would land
};

PinCand candidates[] = {
    {13, "Serial3_RX", "Serial3_RX_PB23_side_exp_GPS_TX_target", true},
    {14, "Serial3_TX", "Serial3_TX_PB22_side_exp", false},
    {26, "Serial1_TX", "Serial1_TX_PA12_SD_SPI", false},
    {27, "Serial1_RX", "Serial1_RX_PA13_SD_SPI_GPS_TX_target", true},
    {28, "Serial2_TX", "Serial2_TX_PA14_center_5pin", false},
    {29, "Serial2_RX", "Serial2_RX_PA15_center_5pin_GPS_TX_target", true},
};
constexpr uint8_t CAND_COUNT = sizeof(candidates) / sizeof(candidates[0]);

uint32_t passCount = 0;

const char *categoryName(int cat) {
  switch (cat) {
    case CAT_ACTIVITY: return "ACTIVITY";
    case CAT_STATIC_HIGH: return "STATIC_HIGH";
    case CAT_STATIC_LOW: return "STATIC_LOW";
    default: return "FLOATING_OR_NOISE";
  }
}

int sampleAndReport(const PinCand &c) {
  pinMode(c.pin, INPUT);  // INPUT only, NO pullup
  delay(5);

  uint32_t high = 0;
  uint32_t low = 0;
  uint32_t trans = 0;
  uint32_t samples = 0;
  int last = digitalRead(c.pin);
  uint32_t start = millis();
  while (millis() - start < SAMPLE_MS) {
    int v = digitalRead(c.pin);
    if (v == HIGH) {
      high++;
    } else {
      low++;
    }
    if (v != last) {
      trans++;
      last = v;
    }
    samples++;
  }

  double ratio = (samples > 0) ? static_cast<double>(high) / static_cast<double>(samples) : 0.0;
  bool uartAct = trans >= ACTIVITY_MIN_TRANS;
  int cat;
  if (uartAct) {
    cat = CAT_ACTIVITY;
  } else if (ratio >= 0.9) {
    cat = CAT_STATIC_HIGH;
  } else if (ratio <= 0.1) {
    cat = CAT_STATIC_LOW;
  } else {
    cat = CAT_FLOATING;
  }

  Serial.print("pin_label=");
  Serial.print(c.label);
  Serial.print(" arduino_pin=");
  Serial.print(c.pin);
  Serial.print(" is_likely_rx=");
  Serial.print(c.likelyRx ? "true" : "false");
  Serial.print(" high_count=");
  Serial.print(high);
  Serial.print(" low_count=");
  Serial.print(low);
  Serial.print(" transition_count=");
  Serial.print(trans);
  Serial.print(" samples=");
  Serial.print(samples);
  Serial.print(" high_ratio=");
  Serial.print(ratio, 3);
  Serial.print(" possible_uart_activity=");
  Serial.print(uartAct ? "true" : "false");
  Serial.print(" pin_state=");
  Serial.print(categoryName(cat));
  Serial.print(" expected_function_if_known=");
  Serial.println(c.func);
  return cat;
}

void runPass() {
  passCount++;
  Serial.println();
  Serial.print("PIN_SWEEP_START pass=");
  Serial.print(passCount);
  Serial.print(" sample_ms=");
  Serial.print(SAMPLE_MS);
  Serial.print(" activity_min_transitions=");
  Serial.println(ACTIVITY_MIN_TRANS);

  uint8_t rxTotal = 0;
  uint8_t rxActivity = 0;
  uint8_t rxStaticHigh = 0;
  uint8_t rxStaticLow = 0;
  uint8_t rxFloating = 0;
  bool anyActivity = false;

  for (uint8_t i = 0; i < CAND_COUNT; ++i) {
    int cat = sampleAndReport(candidates[i]);
    if (cat == CAT_ACTIVITY) {
      anyActivity = true;
    }
    if (candidates[i].likelyRx) {
      rxTotal++;
      if (cat == CAT_ACTIVITY) rxActivity++;
      else if (cat == CAT_STATIC_HIGH) rxStaticHigh++;
      else if (cat == CAT_STATIC_LOW) rxStaticLow++;
      else rxFloating++;
    }
  }

  Serial.print("PIN_SWEEP_DONE pass=");
  Serial.print(passCount);
  Serial.print(" any_pin_activity=");
  Serial.print(anyActivity ? "true" : "false");
  Serial.print(" rx_pins=");
  Serial.print(rxTotal);
  Serial.print(" rx_with_activity=");
  Serial.print(rxActivity);
  Serial.print(" rx_static_high=");
  Serial.print(rxStaticHigh);
  Serial.print(" rx_static_low=");
  Serial.print(rxStaticLow);
  Serial.print(" rx_floating=");
  Serial.println(rxFloating);
}

void setup() {
  Serial.begin(USB_BAUD);
  uint32_t start = millis();
  while (!Serial && millis() - start < 8000) {
    delay(10);
  }

  Serial.println();
  Serial.println("Firmware: gps_pin_activity_probe read-only-pin-level 2026-06-03");
  Serial.println(
      "READ-ONLY: USB logs only. Pins set INPUT (no pullup), digitalRead only. Never "
      "writes Serial1/2/3. No motors, RC, HC-12, I2C, autonomy. No motion.");
  Serial.print("candidate_pins=");
  for (uint8_t i = 0; i < CAND_COUNT; ++i) {
    Serial.print(candidates[i].label);
    Serial.print("(D");
    Serial.print(candidates[i].pin);
    Serial.print(")");
    if (i + 1 < CAND_COUNT) Serial.print(",");
  }
  Serial.println();
  Serial.print("sample_ms_each=");
  Serial.print(SAMPLE_MS);
  Serial.print(" activity_min_transitions=");
  Serial.println(ACTIVITY_MIN_TRANS);
  Serial.println(
      "A driven, transmitting UART TX makes MANY transitions on the RX pin. Static "
      "HIGH/LOW or only-noise means the GPS TX signal is NOT reaching the pin.");
}

void loop() {
  runPass();
  delay(3000);
}
