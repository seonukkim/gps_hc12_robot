#include <Arduino.h>

// Safe Serial3 physical loopback test for OpenRB-150.
//
// This sketch does not include Servo, does not attach ESC pins, and does not
// initialize motor outputs. It writes known text to Serial3 and reports PASS
// only when bytes are received back on Serial3.

constexpr long USB_BAUD = 115200;

#ifndef SERIAL3_LOOPBACK_BAUD
#define SERIAL3_LOOPBACK_BAUD 9600
#endif

constexpr long TEST_BAUD = SERIAL3_LOOPBACK_BAUD;
constexpr uint32_t REPORT_MS = 1000;
constexpr uint32_t WRITE_MS = 250;
constexpr size_t PREVIEW_MAX = 128;

uint32_t lastMs = 0;
uint32_t lastWriteMs = 0;
uint32_t writesThisSecond = 0;
uint32_t totalWrites = 0;
uint32_t charsThisSecond = 0;
uint32_t totalChars = 0;
uint32_t sequence = 0;
char preview[PREVIEW_MAX] = {0};
size_t previewLen = 0;

void appendPreview(char c) {
  if (previewLen >= PREVIEW_MAX - 1) {
    return;
  }

  const uint8_t byteValue = static_cast<uint8_t>(c);
  const char *escaped = nullptr;
  char hexText[5] = {0};

  switch (c) {
    case '\r':
      escaped = "\\r";
      break;
    case '\n':
      escaped = "\\n";
      break;
    case '\t':
      escaped = "\\t";
      break;
    default:
      if (byteValue >= 32 && byteValue <= 126) {
        preview[previewLen++] = c;
        preview[previewLen] = '\0';
        return;
      }
      snprintf(hexText, sizeof(hexText), "\\x%02X", byteValue);
      escaped = hexText;
      break;
  }

  for (size_t i = 0; escaped[i] != '\0' && previewLen < PREVIEW_MAX - 1; ++i) {
    preview[previewLen++] = escaped[i];
  }
  preview[previewLen] = '\0';
}

void resetPreview() {
  previewLen = 0;
  preview[0] = '\0';
}

void writeLoopbackText() {
  Serial3.print("$S3LOOP,SEQ=");
  Serial3.print(sequence++);
  Serial3.print(",OPENRB*00\r\n");
  writesThisSecond++;
  totalWrites++;
}

void setup() {
  Serial.begin(USB_BAUD);

  uint32_t start = millis();
  while (!Serial && millis() - start < 1500) {
    delay(10);
  }

  Serial3.begin(TEST_BAUD);
  Serial.println();
  Serial.println("Serial3 physical loopback test starting.");
  Serial.println("No Servo library is included. No motor pins are attached or driven.");
  Serial.println("Disconnect GPS.");
  Serial.println("Disconnect HC-12 if candidate pins may overlap or if unsure.");
  Serial.println("Wire candidate Serial3 TX to candidate Serial3 RX.");
  Serial.println("No GPS module is needed for this test.");
  Serial.println("PASS is printed only when received bytes are observed on Serial3.");
  Serial.print("selected_port=Serial3 baud=");
  Serial.println(TEST_BAUD);
}

void loop() {
  uint32_t now = millis();

  if (now - lastWriteMs >= WRITE_MS) {
    lastWriteMs = now;
    writeLoopbackText();
  }

  while (Serial3.available() > 0) {
    char c = static_cast<char>(Serial3.read());
    charsThisSecond++;
    totalChars++;
    appendPreview(c);
  }

  if (now - lastMs >= REPORT_MS) {
    lastMs = now;
    bool bytesSeen = charsThisSecond > 0;
    Serial.print("selected_port=Serial3 baud=");
    Serial.print(TEST_BAUD);
    Serial.print(" writes_1s=");
    Serial.print(writesThisSecond);
    Serial.print(" total_writes=");
    Serial.print(totalWrites);
    Serial.print(" chars_1s=");
    Serial.print(charsThisSecond);
    Serial.print(" total_chars=");
    Serial.print(totalChars);
    Serial.print(" raw_preview=\"");
    Serial.print(preview);
    Serial.print("\" bytes_seen=");
    Serial.print(bytesSeen ? "true" : "false");
    Serial.print(" result=");
    Serial.println(bytesSeen ? "PASS" : "FAIL");

    writesThisSecond = 0;
    charsThisSecond = 0;
    resetPreview();
  }
}
