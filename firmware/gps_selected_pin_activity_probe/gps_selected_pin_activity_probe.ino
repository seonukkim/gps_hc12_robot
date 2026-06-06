// firmware/gps_selected_pin_activity_probe/gps_selected_pin_activity_probe.ino
//
// Read-only selected-pin activity probe for OpenRB-150.
//
// Purpose: locate where GPS TX electrical edges arrive on a small, safe set of
// OpenRB pins without scanning D0-D29 blindly. This does not prove NMEA parsing;
// it only detects electrical edges that look like UART activity.
//
// SAFETY / READ-ONLY:
// - Uses USB Serial for logs only.
// - Candidate pins are set to INPUT only and read with digitalRead only.
// - Never writes Serial1, Serial2, or Serial3.
// - Does not initialize motors, RC/PPM, HC-12, I2C, GPS parser, or autonomy.
// - Does not modify motor/path-following behavior and enables no motion.

#include <Arduino.h>

constexpr long USB_BAUD = 115200;

#ifndef SELECTED_PIN_SAMPLE_MS
#define SELECTED_PIN_SAMPLE_MS 5000
#endif

#ifndef SELECTED_PIN_UART_ACTIVITY_MIN_TRANSITIONS
#define SELECTED_PIN_UART_ACTIVITY_MIN_TRANSITIONS 100
#endif

constexpr uint32_t SAMPLE_MS = SELECTED_PIN_SAMPLE_MS;
constexpr uint32_t ACTIVITY_MIN_TRANSITIONS = SELECTED_PIN_UART_ACTIVITY_MIN_TRANSITIONS;

constexpr int CAT_ACTIVITY = 0;
constexpr int CAT_STATIC_HIGH = 1;
constexpr int CAT_STATIC_LOW = 2;
constexpr int CAT_FLOATING_OR_NOISY = 3;

struct PinCandidate {
  uint8_t pin;
  const char *label;
  const char *known_function;
};

PinCandidate candidates[] = {
    {6, "D6", "selected_candidate_unknown_role"},
    {13, "D13", "Serial3_RX_PB23_side_exp_candidate"},
    {14, "D14", "Serial3_TX_PB22_side_exp_candidate"},
    {26, "D26", "Serial1_TX_PA12_SD_SPI_candidate"},
    {27, "D27", "Serial1_RX_PA13_SD_SPI_candidate"},
    {28, "D28", "Serial2_TX_PA14_center_5pin_candidate"},
    {29, "D29", "Serial2_RX_PA15_center_5pin_expected_GPS_TX_target"},
};

constexpr uint8_t CANDIDATE_COUNT = sizeof(candidates) / sizeof(candidates[0]);
uint32_t pass_count = 0;

const char *pinStateName(double high_ratio, uint32_t transitions) {
  if (transitions >= ACTIVITY_MIN_TRANSITIONS) {
    return "UART_ACTIVITY";
  }
  if (high_ratio >= 0.9) {
    return "STATIC_HIGH";
  }
  if (high_ratio <= 0.1) {
    return "STATIC_LOW";
  }
  return "FLOATING_OR_NOISY";
}

int sampleAndReport(const PinCandidate &candidate, uint32_t *transition_count_out) {
  pinMode(candidate.pin, INPUT);  // INPUT only, no pullup.
  delay(5);

  uint32_t high_count = 0;
  uint32_t low_count = 0;
  uint32_t transition_count = 0;
  uint32_t samples = 0;

  int last = digitalRead(candidate.pin);
  const uint32_t start_ms = millis();
  while (millis() - start_ms < SAMPLE_MS) {
    const int value = digitalRead(candidate.pin);
    if (value == HIGH) {
      high_count++;
    } else {
      low_count++;
    }
    if (value != last) {
      transition_count++;
      last = value;
    }
    samples++;
  }

  const double high_ratio =
      samples > 0 ? static_cast<double>(high_count) / static_cast<double>(samples) : 0.0;
  const bool possible_uart_activity = transition_count >= ACTIVITY_MIN_TRANSITIONS;
  const char *pin_state = pinStateName(high_ratio, transition_count);
  int category = CAT_FLOATING_OR_NOISY;
  if (possible_uart_activity) {
    category = CAT_ACTIVITY;
  } else if (high_ratio >= 0.9) {
    category = CAT_STATIC_HIGH;
  } else if (high_ratio <= 0.1) {
    category = CAT_STATIC_LOW;
  }
  *transition_count_out = transition_count;

  Serial.print(F("pin_number="));
  Serial.print(candidate.pin);
  Serial.print(F(" label="));
  Serial.print(candidate.label);
  Serial.print(F(" high_count="));
  Serial.print(high_count);
  Serial.print(F(" low_count="));
  Serial.print(low_count);
  Serial.print(F(" transition_count="));
  Serial.print(transition_count);
  Serial.print(F(" samples="));
  Serial.print(samples);
  Serial.print(F(" high_ratio="));
  Serial.print(high_ratio, 3);
  Serial.print(F(" pin_state="));
  Serial.print(pin_state);
  Serial.print(F(" possible_uart_activity="));
  Serial.print(possible_uart_activity ? F("true") : F("false"));
  Serial.print(F(" known_function="));
  Serial.println(candidate.known_function);

  return category;
}

void runPass() {
  pass_count++;
  Serial.println();
  Serial.print(F("SELECTED_PIN_SWEEP_START pass="));
  Serial.print(pass_count);
  Serial.print(F(" sample_ms_each="));
  Serial.print(SAMPLE_MS);
  Serial.print(F(" activity_min_transitions="));
  Serial.println(ACTIVITY_MIN_TRANSITIONS);

  uint8_t pins_with_activity = 0;
  uint8_t static_high_count = 0;
  uint8_t static_low_count = 0;
  uint8_t floating_or_noisy_count = 0;
  uint32_t max_transition_count = 0;
  uint8_t max_transition_pin = candidates[0].pin;

  for (uint8_t i = 0; i < CANDIDATE_COUNT; ++i) {
    uint32_t transition_count = 0;
    const int category = sampleAndReport(candidates[i], &transition_count);
    if (transition_count > max_transition_count) {
      max_transition_count = transition_count;
      max_transition_pin = candidates[i].pin;
    }
    if (category == CAT_ACTIVITY) {
      pins_with_activity++;
    } else if (category == CAT_STATIC_HIGH) {
      static_high_count++;
    } else if (category == CAT_STATIC_LOW) {
      static_low_count++;
    } else {
      floating_or_noisy_count++;
    }
  }

  Serial.print(F("SELECTED_PIN_SWEEP_DONE pass="));
  Serial.print(pass_count);
  Serial.print(F(" any_pin_activity="));
  Serial.print(pins_with_activity > 0 ? F("true") : F("false"));
  Serial.print(F(" pins_with_activity="));
  Serial.print(pins_with_activity);
  Serial.print(F(" static_high_count="));
  Serial.print(static_high_count);
  Serial.print(F(" static_low_count="));
  Serial.print(static_low_count);
  Serial.print(F(" floating_or_noisy_count="));
  Serial.print(floating_or_noisy_count);
  Serial.print(F(" max_transition_pin="));
  Serial.print(max_transition_pin);
  Serial.print(F(" max_transition_count="));
  Serial.println(max_transition_count);
}

void setup() {
  Serial.begin(USB_BAUD);
  const uint32_t start_ms = millis();
  while (!Serial && millis() - start_ms < 8000) {
    delay(10);
  }

  Serial.println();
  Serial.println(F("Firmware: gps_selected_pin_activity_probe read-only-selected-pin 2026-06-03"));
  Serial.println(F("READ-ONLY: USB logs only. Selected pins set INPUT, digitalRead only."));
  Serial.println(F("Never writes Serial1/2/3. No motors, RC, HC-12, I2C, GPS parser, autonomy, or motion."));
  Serial.print(F("selected_pins="));
  for (uint8_t i = 0; i < CANDIDATE_COUNT; ++i) {
    Serial.print(F("D"));
    Serial.print(candidates[i].pin);
    if (i + 1 < CANDIDATE_COUNT) {
      Serial.print(F(","));
    }
  }
  Serial.println();
  Serial.print(F("sample_ms_each="));
  Serial.print(SAMPLE_MS);
  Serial.print(F(" activity_min_transitions="));
  Serial.println(ACTIVITY_MIN_TRANSITIONS);
  Serial.println(F("This locates electrical activity only. It does not prove NMEA parsing."));
}

void loop() {
  runPass();
  delay(3000);
}
