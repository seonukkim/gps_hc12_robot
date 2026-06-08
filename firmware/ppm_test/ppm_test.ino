// Interrupt-based PPM reader.
// Confirm the actual OpenRB-150 interrupt-capable pin mapping before use.

constexpr uint8_t PPM_PIN = 6;
constexpr uint8_t CHANNEL_COUNT = 8;
constexpr uint16_t PPM_SYNC_US = 4000;
constexpr const char *PPM_INTERRUPT_EDGE_NAME = "FALLING";

volatile uint16_t ppmChannels[CHANNEL_COUNT] = {0};
volatile uint8_t ppmIndex = 0;
volatile uint32_t lastEdgeMicros = 0;
volatile bool frameReady = false;

void ppmISR() {
  uint32_t now = micros();
  uint32_t pulseWidth = now - lastEdgeMicros;
  lastEdgeMicros = now;

  if (pulseWidth > PPM_SYNC_US) {
    ppmIndex = 0;
    return;
  }

  if (ppmIndex < CHANNEL_COUNT) {
    ppmChannels[ppmIndex] = pulseWidth;
    ppmIndex++;
    if (ppmIndex == CHANNEL_COUNT) {
      frameReady = true;
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PPM_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmISR, FALLING);
  Serial.println("PPM test ready. Confirm pin 6 supports interrupts on the target board.");
  Serial.print("PPM decoder edge=");
  Serial.print(PPM_INTERRUPT_EDGE_NAME);
  Serial.print(" sync_us=");
  Serial.println(PPM_SYNC_US);
}

void loop() {
  static uint32_t lastPrint = 0;
  if (!frameReady || millis() - lastPrint < 200) {
    return;
  }

  noInterrupts();
  uint16_t snapshot[CHANNEL_COUNT];
  for (uint8_t i = 0; i < CHANNEL_COUNT; ++i) {
    snapshot[i] = ppmChannels[i];
  }
  frameReady = false;
  interrupts();

  lastPrint = millis();
  for (uint8_t i = 0; i < CHANNEL_COUNT; ++i) {
    Serial.print("CH");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(snapshot[i]);
    Serial.print("us  ");
  }
  Serial.println();
}
