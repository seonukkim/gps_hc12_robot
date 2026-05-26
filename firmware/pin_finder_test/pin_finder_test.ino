#include <Arduino.h>

// Safe physical pin finder for OpenRB-150 UART bring-up.
//
// This sketch does not include Servo, does not attach ESC pins, and does not
// initialize motor outputs. It only toggles candidate digital pins so the
// physical pads can be found with a meter, logic probe, or LED plus resistor.
//
// Default candidates are D13 and D14. Compile with
//   -DPIN_FINDER_INCLUDE_EXTENDED=1
// to also try D0, D1, D26, D27, D28, and D29 when they are mapped by the board
// core. Invalid candidates are skipped when the core exposes PINS_COUNT.

constexpr long USB_BAUD = 115200;

#ifndef PIN_FINDER_INCLUDE_EXTENDED
#define PIN_FINDER_INCLUDE_EXTENDED 0
#endif

#ifndef PIN_FINDER_ACTIVE_MS
#define PIN_FINDER_ACTIVE_MS 5000
#endif

#ifndef PIN_FINDER_TOGGLE_MS
#define PIN_FINDER_TOGGLE_MS 250
#endif

struct CandidatePin {
  const char *label;
  uint8_t pin;
};

#if PIN_FINDER_INCLUDE_EXTENDED
constexpr CandidatePin CANDIDATES[] = {
    {"D13", 13},
    {"D14", 14},
    {"D0", 0},
    {"D1", 1},
    {"D26", 26},
    {"D27", 27},
    {"D28", 28},
    {"D29", 29},
};
#else
constexpr CandidatePin CANDIDATES[] = {
    {"D13", 13},
    {"D14", 14},
};
#endif

constexpr size_t CANDIDATE_COUNT = sizeof(CANDIDATES) / sizeof(CANDIDATES[0]);

size_t activeIndex = 0;
uint32_t activeStartedMs = 0;
uint32_t lastToggleMs = 0;
bool outputHigh = false;

bool isMappedByCore(uint8_t pin) {
#ifdef PINS_COUNT
  return pin < PINS_COUNT;
#else
  (void)pin;
  return true;
#endif
}

void releasePin(const CandidatePin &candidate) {
  if (!isMappedByCore(candidate.pin)) {
    return;
  }
  digitalWrite(candidate.pin, LOW);
  pinMode(candidate.pin, INPUT);
}

void printCandidateHeader(const CandidatePin &candidate) {
  Serial.print("active_pin=");
  Serial.print(candidate.label);
  Serial.print(" numeric_pin=");
  Serial.print(candidate.pin);
  Serial.print(" core_mapped=");
  Serial.print(isMappedByCore(candidate.pin) ? "true" : "false");
  Serial.print(" active_ms=");
  Serial.print(PIN_FINDER_ACTIVE_MS);
  Serial.print(" toggle_ms=");
  Serial.println(PIN_FINDER_TOGGLE_MS);
}

void activateCandidate(size_t index) {
  activeIndex = index;
  activeStartedMs = millis();
  lastToggleMs = 0;
  outputHigh = false;

  const CandidatePin &candidate = CANDIDATES[activeIndex];
  printCandidateHeader(candidate);

  if (!isMappedByCore(candidate.pin)) {
    Serial.println("candidate_skipped=not_mapped_by_core");
    return;
  }

  pinMode(candidate.pin, OUTPUT);
  digitalWrite(candidate.pin, LOW);
}

void setup() {
  Serial.begin(USB_BAUD);

  uint32_t start = millis();
  while (!Serial && millis() - start < 1500) {
    delay(10);
  }

  Serial.println();
  Serial.println("Serial pin finder starting.");
  Serial.println("No Servo library is included. No motor pins are attached or driven.");
  Serial.println("Disconnect GPS. Disconnect HC-12 if candidate pins may overlap or if unsure.");
  Serial.println("Measure the active pin with a meter, logic probe, or LED plus resistor.");
  Serial.print("candidate_count=");
  Serial.print(CANDIDATE_COUNT);
  Serial.print(" include_extended=");
  Serial.println(PIN_FINDER_INCLUDE_EXTENDED ? "true" : "false");

  activateCandidate(0);
}

void loop() {
  const CandidatePin &candidate = CANDIDATES[activeIndex];
  uint32_t now = millis();

  if (isMappedByCore(candidate.pin) && now - lastToggleMs >= PIN_FINDER_TOGGLE_MS) {
    lastToggleMs = now;
    outputHigh = !outputHigh;
    digitalWrite(candidate.pin, outputHigh ? HIGH : LOW);

    Serial.print("pin=");
    Serial.print(candidate.label);
    Serial.print(" state=");
    Serial.println(outputHigh ? "HIGH" : "LOW");
  }

  if (now - activeStartedMs >= PIN_FINDER_ACTIVE_MS) {
    releasePin(candidate);
    size_t nextIndex = (activeIndex + 1) % CANDIDATE_COUNT;
    activateCandidate(nextIndex);
  }
}
