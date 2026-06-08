// Safe RC PPM channel probe for OpenRB-150.
//
// This sketch uses the same PPM input pin and frame decoding style as
// openrb_robot_controller, but it does not attach Servo or motor outputs.

constexpr uint8_t PPM_PIN = 6;
constexpr uint8_t CHANNEL_COUNT = 8;
constexpr uint32_t USB_BAUD = 115200;
constexpr uint32_t SERIAL_WAIT_TIMEOUT_MS = 3000;
constexpr uint32_t PRINT_PERIOD_MS = 500;
constexpr uint16_t PPM_SYNC_US = 4000;
constexpr uint16_t VALID_MIN_US = 800;
constexpr uint16_t VALID_MAX_US = 2200;
constexpr uint16_t CHANGE_THRESHOLD_US = 100;
constexpr const char *PPM_INTERRUPT_EDGE_NAME = "FALLING";

volatile uint16_t ppmChannels[CHANNEL_COUNT] = {0};
volatile uint8_t ppmIndex = 0;
volatile uint32_t lastPpmEdgeMicros = 0;
volatile uint32_t lastPpmFrameMs = 0;

uint16_t channelMin[CHANNEL_COUNT];
uint16_t channelMax[CHANNEL_COUNT];
uint16_t changeReference[CHANNEL_COUNT] = {0};
bool changeReferenceValid[CHANNEL_COUNT] = {false};

void ppmISR() {
  uint32_t now = micros();
  uint32_t width = now - lastPpmEdgeMicros;
  lastPpmEdgeMicros = now;

  if (width > PPM_SYNC_US) {
    ppmIndex = 0;
    lastPpmFrameMs = millis();
    return;
  }

  if (ppmIndex < CHANNEL_COUNT) {
    ppmChannels[ppmIndex] = width;
    ppmIndex++;
  }
}

bool isValidPulse(uint16_t pulseUs) {
  return pulseUs >= VALID_MIN_US && pulseUs <= VALID_MAX_US;
}

void printChannelValue(const char *prefix, uint8_t channelIndex, uint16_t value) {
  Serial.print(prefix);
  Serial.print(channelIndex + 1);
  Serial.print("_us=");
  Serial.print(value);
}

void setup() {
  Serial.begin(USB_BAUD);

  uint32_t startMs = millis();
  while (!Serial && millis() - startMs < SERIAL_WAIT_TIMEOUT_MS) {
    delay(10);
  }

  for (uint8_t i = 0; i < CHANNEL_COUNT; ++i) {
    channelMin[i] = UINT16_MAX;
    channelMax[i] = 0;
  }

  Serial.println(F("RC channel probe - motors disabled"));
  Serial.print(F("ppm_pin=D"));
  Serial.println(PPM_PIN);
  Serial.print(F("channel_count="));
  Serial.println(CHANNEL_COUNT);
  Serial.print(F("usb_baud="));
  Serial.println(USB_BAUD);
  Serial.print(F("print_period_ms="));
  Serial.println(PRINT_PERIOD_MS);
  Serial.print(F("ppm_interrupt_edge="));
  Serial.println(PPM_INTERRUPT_EDGE_NAME);
  Serial.print(F("ppm_sync_us="));
  Serial.println(PPM_SYNC_US);
  Serial.println(F("Move one stick or switch at a time and record changed channels."));

  pinMode(PPM_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmISR, FALLING);
}

void loop() {
  static uint32_t lastPrintMs = 0;
  uint32_t nowMs = millis();

  if (nowMs - lastPrintMs < PRINT_PERIOD_MS) {
    return;
  }
  lastPrintMs = nowMs;

  uint16_t snapshot[CHANNEL_COUNT];
  uint32_t frameMs;

  noInterrupts();
  for (uint8_t i = 0; i < CHANNEL_COUNT; ++i) {
    snapshot[i] = ppmChannels[i];
  }
  frameMs = lastPpmFrameMs;
  interrupts();

  for (uint8_t i = 0; i < CHANNEL_COUNT; ++i) {
    if (isValidPulse(snapshot[i])) {
      if (snapshot[i] < channelMin[i]) {
        channelMin[i] = snapshot[i];
      }
      if (snapshot[i] > channelMax[i]) {
        channelMax[i] = snapshot[i];
      }
    }
  }

  Serial.print(F("RCCH ppm_age_ms="));
  Serial.print(frameMs == 0 ? 999999UL : nowMs - frameMs);

  for (uint8_t i = 0; i < CHANNEL_COUNT; ++i) {
    Serial.print(' ');
    printChannelValue("ch", i, snapshot[i]);
  }

  for (uint8_t i = 0; i < CHANNEL_COUNT; ++i) {
    Serial.print(F(" ch"));
    Serial.print(i + 1);
    Serial.print(F("_min="));
    if (channelMin[i] == UINT16_MAX) {
      Serial.print(F("NA"));
    } else {
      Serial.print(channelMin[i]);
    }

    Serial.print(F(" ch"));
    Serial.print(i + 1);
    Serial.print(F("_max="));
    if (channelMax[i] == 0) {
      Serial.print(F("NA"));
    } else {
      Serial.print(channelMax[i]);
    }
  }

  Serial.print(F(" changed_channels="));
  bool anyChanged = false;
  for (uint8_t i = 0; i < CHANNEL_COUNT; ++i) {
    if (!isValidPulse(snapshot[i])) {
      continue;
    }

    if (!changeReferenceValid[i]) {
      changeReference[i] = snapshot[i];
      changeReferenceValid[i] = true;
      continue;
    }

    int16_t delta = static_cast<int16_t>(snapshot[i]) - static_cast<int16_t>(changeReference[i]);
    if (abs(delta) > CHANGE_THRESHOLD_US) {
      if (anyChanged) {
        Serial.print(',');
      }
      Serial.print(F("ch"));
      Serial.print(i + 1);
      Serial.print(':');
      Serial.print(changeReference[i]);
      Serial.print(F("->"));
      Serial.print(snapshot[i]);
      changeReference[i] = snapshot[i];
      anyChanged = true;
    }
  }

  if (!anyChanged) {
    Serial.print(F("none"));
  }

  Serial.println();
}
