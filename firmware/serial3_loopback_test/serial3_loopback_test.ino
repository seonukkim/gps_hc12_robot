#include <Arduino.h>

constexpr long USB_BAUD = 115200;
constexpr long TEST_BAUD = 9600;
constexpr uint32_t REPORT_MS = 1000;

uint32_t lastMs = 0;
uint32_t charsThisSecond = 0;
uint32_t totalChars = 0;
char preview[128] = {0};
size_t previewLen = 0;

void appendPreview(char c) {
  if (previewLen >= sizeof(preview) - 5) return;
  if (c == '\r') {
    preview[previewLen++] = '\\';
    preview[previewLen++] = 'r';
  } else if (c == '\n') {
    preview[previewLen++] = '\\';
    preview[previewLen++] = 'n';
  } else if ((uint8_t)c >= 32 && (uint8_t)c <= 126) {
    preview[previewLen++] = c;
  } else {
    snprintf(preview + previewLen, sizeof(preview) - previewLen, "\\x%02X", (uint8_t)c);
    previewLen = strlen(preview);
  }
  preview[previewLen] = '\0';
}

void setup() {
  Serial.begin(USB_BAUD);
  uint32_t start = millis();
  while (!Serial && millis() - start < 1500) delay(10);

  Serial3.begin(TEST_BAUD);

  Serial.println();
  Serial.println("Serial3 loopback test.");
  Serial.println("Wire OpenRB D14/TX -> OpenRB D13/RX.");
  Serial.println("No GPS module is needed for this test.");
  Serial.println("No motor pins are attached.");
}

void loop() {
  static uint32_t lastWrite = 0;
  uint32_t now = millis();

  if (now - lastWrite >= 250) {
    lastWrite = now;
    Serial3.print("$LOOP,OPENRB,SERIAL3*00\r\n");
  }

  while (Serial3.available() > 0) {
    char c = (char)Serial3.read();
    charsThisSecond++;
    totalChars++;
    appendPreview(c);
  }

  if (now - lastMs >= REPORT_MS) {
    lastMs = now;
    Serial.print("baud=");
    Serial.print(TEST_BAUD);
    Serial.print(" chars_1s=");
    Serial.print(charsThisSecond);
    Serial.print(" total_chars=");
    Serial.print(totalChars);
    Serial.print(" raw_preview=\"");
    Serial.print(preview);
    Serial.print("\" result=");
    Serial.println(charsThisSecond > 0 ? "PASS" : "FAIL");

    charsThisSecond = 0;
    previewLen = 0;
    preview[0] = '\0';
  }
}
