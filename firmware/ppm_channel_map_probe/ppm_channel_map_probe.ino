// PPM channel-map probe for OpenRB-150.
//
// Purpose: diagnose which receiver PPM channel is a stable 2-position
// Manual/Auto switch, and detect channel-slip / misaligned frames.
//
// SAFETY: this sketch does NOT initialize GPS, HC-12, motors (Servo/ESC), or
// any autonomous control. It only reads the PPM input pin and prints
// diagnostics at 115200 baud. It cannot move the rover.
//
// It reuses the same PPM_PIN and frame-decoding style as
// openrb_robot_controller so the channel indexes match the main firmware.
//
// Output line types:
//   PPMHDR  - one-time header describing fields.
//   PPMEVT  - emitted immediately when any channel changes LOW/MID/HIGH state.
//   PPMSUM  - emitted every 1000 ms with window stats.
// Log either type with tools/analyze_ppm_log.py.

constexpr uint8_t PPM_PIN = 6;            // Matches openrb_robot_controller.
constexpr uint8_t CHANNEL_COUNT = 8;
constexpr uint32_t USB_BAUD = 115200;
constexpr uint32_t SERIAL_WAIT_TIMEOUT_MS = 3000;
constexpr uint32_t SUMMARY_PERIOD_MS = 1000;

constexpr uint16_t PPM_SYNC_US = 3000;    // Frame gap, matches main firmware.
constexpr uint16_t VALID_MIN_US = 800;    // Below this = out of range.
constexpr uint16_t VALID_MAX_US = 2200;   // Above this = out of range.
constexpr uint16_t LOW_MAX_US = 1300;     // < LOW_MAX_US -> LOW.
constexpr uint16_t HIGH_MIN_US = 1700;    // > HIGH_MIN_US -> HIGH, else MID.
constexpr uint32_t FRAME_STALE_MS = 100;  // Older than this = stale/failsafe.
constexpr uint8_t STABLE_FRAMES = 3;      // Frames a state must hold to count.

enum ChannelState { STATE_UNKNOWN = 0, STATE_LOW, STATE_MID, STATE_HIGH };

volatile uint16_t ppmChannels[CHANNEL_COUNT] = {0};
volatile uint8_t ppmIndex = 0;
volatile uint8_t ppmLastFrameChannelCount = 0;
volatile uint32_t ppmFrameCounter = 0;
volatile uint32_t lastPpmEdgeMicros = 0;
volatile uint32_t lastPpmFrameMs = 0;

// Confirmed (hysteresis-filtered) state per channel.
ChannelState stableState[CHANNEL_COUNT];
// Candidate state pending confirmation.
ChannelState candidateState[CHANNEL_COUNT];
uint8_t candidateCount[CHANNEL_COUNT];

// Run-level observation: has this channel ever been seen LOW / MID / HIGH.
bool seenLow[CHANNEL_COUNT];
bool seenMid[CHANNEL_COUNT];
bool seenHigh[CHANNEL_COUNT];

// Summary-window accumulators (reset every SUMMARY_PERIOD_MS).
uint16_t winMin[CHANNEL_COUNT];
uint16_t winMax[CHANNEL_COUNT];
uint16_t winLowCount[CHANNEL_COUNT];
uint16_t winMidCount[CHANNEL_COUNT];
uint16_t winHighCount[CHANNEL_COUNT];
uint32_t winFrameCount = 0;
uint32_t winInvalidCount = 0;

uint32_t processedFrameCounter = 0;

void printDetailLine(const __FlashStringHelper *tag, uint32_t nowMs, uint32_t frameCounter,
                     uint32_t frameMs, uint8_t frameChannelCount, bool frameValid,
                     const char *invalidReason, const uint16_t *snapshot,
                     const char *changedList);
void printSummaryLine(uint32_t nowMs, uint32_t frameCounter, uint32_t frameMs);
void printModeCandidates();

void ppmISR() {
  uint32_t now = micros();
  uint32_t width = now - lastPpmEdgeMicros;
  lastPpmEdgeMicros = now;

  if (width > PPM_SYNC_US) {
    ppmLastFrameChannelCount = ppmIndex;
    ppmIndex = 0;
    lastPpmFrameMs = millis();
    ppmFrameCounter++;
    return;
  }

  if (ppmIndex < CHANNEL_COUNT) {
    ppmChannels[ppmIndex] = width;
    ppmIndex++;
  }
}

ChannelState classifyState(uint16_t pulseUs) {
  if (pulseUs < VALID_MIN_US || pulseUs > VALID_MAX_US) {
    return STATE_UNKNOWN;
  }
  if (pulseUs < LOW_MAX_US) {
    return STATE_LOW;
  }
  if (pulseUs > HIGH_MIN_US) {
    return STATE_HIGH;
  }
  return STATE_MID;
}

const __FlashStringHelper *stateName(ChannelState state) {
  switch (state) {
    case STATE_LOW:
      return F("LOW");
    case STATE_MID:
      return F("MID");
    case STATE_HIGH:
      return F("HIGH");
    default:
      return F("UNK");
  }
}

void resetSummaryWindow() {
  for (uint8_t i = 0; i < CHANNEL_COUNT; ++i) {
    winMin[i] = UINT16_MAX;
    winMax[i] = 0;
    winLowCount[i] = 0;
    winMidCount[i] = 0;
    winHighCount[i] = 0;
  }
  winFrameCount = 0;
  winInvalidCount = 0;
}

void setup() {
  Serial.begin(USB_BAUD);

  uint32_t startMs = millis();
  while (!Serial && millis() - startMs < SERIAL_WAIT_TIMEOUT_MS) {
    delay(10);
  }

  for (uint8_t i = 0; i < CHANNEL_COUNT; ++i) {
    stableState[i] = STATE_UNKNOWN;
    candidateState[i] = STATE_UNKNOWN;
    candidateCount[i] = 0;
    seenLow[i] = false;
    seenMid[i] = false;
    seenHigh[i] = false;
  }
  resetSummaryWindow();

  Serial.println(F("PPMHDR ppm_channel_map_probe - motors/GPS/HC12 disabled, read-only"));
  Serial.print(F("PPMHDR ppm_pin=D"));
  Serial.print(PPM_PIN);
  Serial.print(F(" channel_count="));
  Serial.print(CHANNEL_COUNT);
  Serial.print(F(" usb_baud="));
  Serial.print(USB_BAUD);
  Serial.print(F(" low_max_us="));
  Serial.print(LOW_MAX_US);
  Serial.print(F(" high_min_us="));
  Serial.print(HIGH_MIN_US);
  Serial.print(F(" valid_min_us="));
  Serial.print(VALID_MIN_US);
  Serial.print(F(" valid_max_us="));
  Serial.print(VALID_MAX_US);
  Serial.print(F(" frame_stale_ms="));
  Serial.print(FRAME_STALE_MS);
  Serial.print(F(" stable_frames="));
  Serial.println(STABLE_FRAMES);
  Serial.println(F("PPMHDR Flip the suspected Manual/Auto switch slowly and watch PPMEVT lines."));
  Serial.println(F("PPMHDR A stable mode channel toggles cleanly between LOW and HIGH and holds."));

  pinMode(PPM_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmISR, RISING);
}

void loop() {
  uint16_t snapshot[CHANNEL_COUNT];
  uint32_t frameCounter;
  uint8_t frameChannelCount;
  uint32_t frameMs;

  noInterrupts();
  for (uint8_t i = 0; i < CHANNEL_COUNT; ++i) {
    snapshot[i] = ppmChannels[i];
  }
  frameCounter = ppmFrameCounter;
  frameChannelCount = ppmLastFrameChannelCount;
  frameMs = lastPpmFrameMs;
  interrupts();

  uint32_t nowMs = millis();

  // Process only newly completed frames so counts reflect real frames.
  bool newFrame = (frameCounter != processedFrameCounter);
  if (newFrame) {
    processedFrameCounter = frameCounter;

    // Frame validity + invalid reason (first failing check wins).
    bool frameValid = true;
    char invalidReason[20] = "OK";
    if (frameMs == 0) {
      frameValid = false;
      strcpy(invalidReason, "NO_FRAME");
    } else if (nowMs - frameMs > FRAME_STALE_MS) {
      frameValid = false;
      strcpy(invalidReason, "STALE");
    } else if (frameChannelCount < CHANNEL_COUNT) {
      frameValid = false;
      strcpy(invalidReason, "INCOMPLETE");
    } else {
      for (uint8_t i = 0; i < CHANNEL_COUNT; ++i) {
        if (snapshot[i] < VALID_MIN_US || snapshot[i] > VALID_MAX_US) {
          frameValid = false;
          snprintf(invalidReason, sizeof(invalidReason), "OUT_OF_RANGE:ch%u", i + 1);
          break;
        }
      }
    }

    winFrameCount++;
    if (!frameValid) {
      winInvalidCount++;
    }

    // Per-channel accumulation + run-level seen-state + hysteresis transitions.
    bool anyStateChanged = false;
    char changedList[64] = "";
    for (uint8_t i = 0; i < CHANNEL_COUNT; ++i) {
      uint16_t value = snapshot[i];
      ChannelState state = classifyState(value);

      if (state != STATE_UNKNOWN) {
        if (value < winMin[i]) winMin[i] = value;
        if (value > winMax[i]) winMax[i] = value;
        if (state == STATE_LOW) {
          winLowCount[i]++;
          seenLow[i] = true;
        } else if (state == STATE_MID) {
          winMidCount[i]++;
          seenMid[i] = true;
        } else {
          winHighCount[i]++;
          seenHigh[i] = true;
        }
      }

      // Hysteresis: a state must persist STABLE_FRAMES frames to be confirmed.
      if (state == candidateState[i]) {
        if (candidateCount[i] < 255) candidateCount[i]++;
      } else {
        candidateState[i] = state;
        candidateCount[i] = 1;
      }

      if (candidateCount[i] >= STABLE_FRAMES && candidateState[i] != stableState[i]) {
        // Confirmed stable transition.
        char piece[12];
        snprintf(piece, sizeof(piece), "%sch%u", anyStateChanged ? "," : "", i + 1);
        if (strlen(changedList) + strlen(piece) < sizeof(changedList)) {
          strcat(changedList, piece);
        }
        stableState[i] = candidateState[i];
        anyStateChanged = true;
      }
    }

    // Emit a full event line only when a confirmed state change occurred.
    if (anyStateChanged) {
      printDetailLine(F("PPMEVT"), nowMs, frameCounter, frameMs, frameChannelCount,
                      frameValid, invalidReason, snapshot, changedList);
    }
  }

  // Periodic 1 s summary, independent of frame arrival.
  static uint32_t lastSummaryMs = 0;
  if (nowMs - lastSummaryMs >= SUMMARY_PERIOD_MS) {
    lastSummaryMs = nowMs;
    printSummaryLine(nowMs, frameCounter, frameMs);
    resetSummaryWindow();
  }
}

void printDetailLine(const __FlashStringHelper *tag, uint32_t nowMs, uint32_t frameCounter,
                     uint32_t frameMs, uint8_t frameChannelCount, bool frameValid,
                     const char *invalidReason, const uint16_t *snapshot,
                     const char *changedList) {
  Serial.print(tag);
  Serial.print(F(" timestamp_ms="));
  Serial.print(nowMs);
  Serial.print(F(" frame_counter="));
  Serial.print(frameCounter);
  Serial.print(F(" frame_age_ms="));
  Serial.print(frameMs == 0 ? 999999UL : nowMs - frameMs);
  Serial.print(F(" frame_valid="));
  Serial.print(frameValid ? F("true") : F("false"));
  Serial.print(F(" invalid_reason="));
  Serial.print(invalidReason);
  Serial.print(F(" channel_count="));
  Serial.print(frameChannelCount);

  for (uint8_t i = 0; i < CHANNEL_COUNT; ++i) {
    Serial.print(F(" CH"));
    Serial.print(i + 1);
    Serial.print(F("_us="));
    Serial.print(snapshot[i]);
  }
  for (uint8_t i = 0; i < CHANNEL_COUNT; ++i) {
    Serial.print(F(" CH"));
    Serial.print(i + 1);
    Serial.print(F("_state="));
    Serial.print(stateName(classifyState(snapshot[i])));
  }

  Serial.print(F(" changed_channels="));
  Serial.print(strlen(changedList) > 0 ? changedList : "none");

  printModeCandidates();
  Serial.println();
}

void printSummaryLine(uint32_t nowMs, uint32_t frameCounter, uint32_t frameMs) {
  Serial.print(F("PPMSUM timestamp_ms="));
  Serial.print(nowMs);
  Serial.print(F(" frame_counter="));
  Serial.print(frameCounter);
  Serial.print(F(" frame_age_ms="));
  Serial.print(frameMs == 0 ? 999999UL : nowMs - frameMs);
  Serial.print(F(" frames="));
  Serial.print(winFrameCount);
  Serial.print(F(" invalid_frames="));
  Serial.print(winInvalidCount);

  for (uint8_t i = 0; i < CHANNEL_COUNT; ++i) {
    Serial.print(F(" CH"));
    Serial.print(i + 1);
    Serial.print(F("_min="));
    Serial.print(winMin[i] == UINT16_MAX ? 0 : winMin[i]);
    Serial.print(F(" CH"));
    Serial.print(i + 1);
    Serial.print(F("_max="));
    Serial.print(winMax[i]);
    Serial.print(F(" CH"));
    Serial.print(i + 1);
    Serial.print(F("_low="));
    Serial.print(winLowCount[i]);
    Serial.print(F(" CH"));
    Serial.print(i + 1);
    Serial.print(F("_mid="));
    Serial.print(winMidCount[i]);
    Serial.print(F(" CH"));
    Serial.print(i + 1);
    Serial.print(F("_high="));
    Serial.print(winHighCount[i]);
  }

  printModeCandidates();
  Serial.println();
}

void printModeCandidates() {
  // A 2-position mode switch should reach both LOW and HIGH cleanly.
  Serial.print(F(" possible_mode_channel_candidates="));
  bool any = false;
  for (uint8_t i = 0; i < CHANNEL_COUNT; ++i) {
    if (seenLow[i] && seenHigh[i]) {
      if (any) Serial.print(',');
      Serial.print(F("CH"));
      Serial.print(i + 1);
      any = true;
    }
  }
  if (!any) {
    Serial.print(F("none"));
  }
}
