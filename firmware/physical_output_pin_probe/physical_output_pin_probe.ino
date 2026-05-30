#include <Servo.h>

#ifndef PHYSICAL_PIN_A_CMD
#define PHYSICAL_PIN_A_CMD 0.0
#endif

#ifndef PHYSICAL_PIN_B_CMD
#define PHYSICAL_PIN_B_CMD 0.0
#endif

#ifndef PHYSICAL_PIN_PROBE_MS
#define PHYSICAL_PIN_PROBE_MS 500
#endif

#ifndef PHYSICAL_PIN_PROBE_START_DELAY_MS
#define PHYSICAL_PIN_PROBE_START_DELAY_MS 3000
#endif

constexpr uint8_t PHYSICAL_PIN_A = 4;
constexpr uint8_t PHYSICAL_PIN_B = 5;
constexpr long USB_BAUD = 115200;
constexpr uint16_t OUTPUT_NEUTRAL_US = 1500;
constexpr uint16_t OUTPUT_RANGE_US = 300;
constexpr float PHYSICAL_PIN_A_CMD_VALUE = PHYSICAL_PIN_A_CMD;
constexpr float PHYSICAL_PIN_B_CMD_VALUE = PHYSICAL_PIN_B_CMD;
constexpr uint32_t PROBE_MS_VALUE = PHYSICAL_PIN_PROBE_MS;
constexpr uint32_t START_DELAY_MS_VALUE = PHYSICAL_PIN_PROBE_START_DELAY_MS;
constexpr uint32_t PRINT_PERIOD_MS = 100;

Servo outputA;
Servo outputB;
uint32_t startMs = 0;
uint32_t lastPrintMs = 0;

float clampUnit(float value) {
  if (value > 1.0f) return 1.0f;
  if (value < -1.0f) return -1.0f;
  return value;
}

int clampPulse(int pulse) {
  if (pulse < OUTPUT_NEUTRAL_US - OUTPUT_RANGE_US) {
    return OUTPUT_NEUTRAL_US - OUTPUT_RANGE_US;
  }
  if (pulse > OUTPUT_NEUTRAL_US + OUTPUT_RANGE_US) {
    return OUTPUT_NEUTRAL_US + OUTPUT_RANGE_US;
  }
  return pulse;
}

void writePhysicalPins(float pinACmd, float pinBCmd) {
  int pinAPulse = OUTPUT_NEUTRAL_US + static_cast<int>(clampUnit(pinACmd) * OUTPUT_RANGE_US);
  int pinBPulse = OUTPUT_NEUTRAL_US + static_cast<int>(clampUnit(pinBCmd) * OUTPUT_RANGE_US);
  outputA.writeMicroseconds(clampPulse(pinAPulse));
  outputB.writeMicroseconds(clampPulse(pinBPulse));
}

void printStatus(const char *phase, uint32_t elapsedMs, float pinACmd, float pinBCmd) {
  Serial.print(F("physical_output_pin_probe=true"));
  Serial.print(F(" phase="));
  Serial.print(phase);
  Serial.print(F(" elapsed_ms="));
  Serial.print(elapsedMs);
  Serial.print(F(" physical_pin_a_cmd="));
  Serial.print(PHYSICAL_PIN_A_CMD_VALUE, 3);
  Serial.print(F(" physical_pin_b_cmd="));
  Serial.print(PHYSICAL_PIN_B_CMD_VALUE, 3);
  Serial.print(F(" physical_pin_probe_ms="));
  Serial.print(PROBE_MS_VALUE);
  Serial.print(F(" physical_pin_probe_start_delay_ms="));
  Serial.print(START_DELAY_MS_VALUE);
  Serial.print(F(" physical_pin_a="));
  Serial.print(PHYSICAL_PIN_A);
  Serial.print(F(" physical_pin_b="));
  Serial.print(PHYSICAL_PIN_B);
  Serial.print(F(" physical_pin_a_function=\"same Servo PWM output as openrb_robot_controller ESC_LEFT_PIN\""));
  Serial.print(F(" physical_pin_b_function=\"same Servo PWM output as openrb_robot_controller ESC_RIGHT_PIN\""));
  Serial.print(F(" output_neutral_us="));
  Serial.print(OUTPUT_NEUTRAL_US);
  Serial.print(F(" output_range_us="));
  Serial.print(OUTPUT_RANGE_US);
  Serial.print(F(" written_pin_a_cmd="));
  Serial.print(pinACmd, 3);
  Serial.print(F(" written_pin_b_cmd="));
  Serial.print(pinBCmd, 3);
  Serial.println();
}

void setup() {
  Serial.begin(USB_BAUD);
  uint32_t waitStart = millis();
  while (!Serial && millis() - waitStart < 3000) {
  }

  outputA.attach(PHYSICAL_PIN_A);
  outputB.attach(PHYSICAL_PIN_B);
  writePhysicalPins(0.0f, 0.0f);

  startMs = millis();
  lastPrintMs = 0;

  Serial.println(F("physical_output_pin_probe starting"));
  Serial.println(F("This sketch bypasses RC, GPS, HC-12, mixers, calibration, and logical wheel conversion."));
  Serial.print(F("PHYSICAL_PIN_A="));
  Serial.println(PHYSICAL_PIN_A);
  Serial.print(F("PHYSICAL_PIN_B="));
  Serial.println(PHYSICAL_PIN_B);
  Serial.print(F("PHYSICAL_PIN_A_CMD="));
  Serial.println(PHYSICAL_PIN_A_CMD_VALUE, 3);
  Serial.print(F("PHYSICAL_PIN_B_CMD="));
  Serial.println(PHYSICAL_PIN_B_CMD_VALUE, 3);
  Serial.print(F("PHYSICAL_PIN_PROBE_START_DELAY_MS="));
  Serial.println(START_DELAY_MS_VALUE);
  Serial.print(F("PHYSICAL_PIN_PROBE_MS="));
  Serial.println(PROBE_MS_VALUE);
}

void loop() {
  uint32_t now = millis();
  uint32_t elapsed = now - startMs;
  const char *phase = "WAIT";
  float pinACmd = 0.0f;
  float pinBCmd = 0.0f;

  if (elapsed < START_DELAY_MS_VALUE) {
    phase = "WAIT";
  } else if (elapsed < START_DELAY_MS_VALUE + PROBE_MS_VALUE) {
    phase = "PULSE";
    pinACmd = PHYSICAL_PIN_A_CMD_VALUE;
    pinBCmd = PHYSICAL_PIN_B_CMD_VALUE;
  } else {
    phase = "STOP";
  }

  writePhysicalPins(pinACmd, pinBCmd);

  if (lastPrintMs == 0 || now - lastPrintMs >= PRINT_PERIOD_MS) {
    lastPrintMs = now;
    printStatus(phase, elapsed, pinACmd, pinBCmd);
  }
}
