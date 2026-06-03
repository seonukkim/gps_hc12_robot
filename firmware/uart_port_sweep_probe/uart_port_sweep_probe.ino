// firmware/uart_port_sweep_probe/uart_port_sweep_probe.ino
//
// OpenRB-150 UART port-sweep diagnostic. Identifies which hardware UART a fixed
// external module (e.g. the HC-12) is actually wired to, WITHOUT removing the
// module and WITHOUT a physical loopback or AT command.
//
// It drives Serial1, Serial2, and Serial3 with uniquely tagged text frames and
// also reads all three, reporting per-port TX/RX counts over USB. Identification:
//   - TX side: whichever UART the HC-12 is on is the one whose "@UART,<n>,TX_TEST"
//     frames reach the station (read them with tools/serial_raw_read.py).
//   - RX side: whichever UART's rx_count rises when the station transmits is the
//     one the HC-12 receives on.
//
// SAFETY: initializes ONLY USB Serial + the three hardware UARTs. It does NOT
// initialize or drive motors/ESC/Servo, does NOT parse GPS, and uses no RC/PPM,
// I2C, or autonomy. Text in/out only. It does not change motor/path-following
// code and does not enable any motion.
//
// OpenRB-150 UART pin map (core 0.2.1 variant files variant.cpp / variant.h):
//   Serial1 = SERCOM2, TX D26/PA12, RX D27/PA13  (SD-SPI header: MOSI / SCK)
//   Serial2 = SERCOM4, TX D28/PA14, RX D29/PA15  (SD-SPI header: SS / MISO; GPS lives here)
//   Serial3 = SERCOM5, TX D14/PB22, RX D13/PB23  (dedicated "Serial3 / exp uart" header)
// Note: Serial2 RX normally carries GPS NMEA, so Serial2_rx_preview showing
// "$GP..."/"$GN..." confirms GPS on Serial2 (expected, not the HC-12).

#include <Arduino.h>
#include <stdio.h>
#include <string.h>

#ifndef UART_SWEEP_BAUD
#define UART_SWEEP_BAUD 9600
#endif
#ifndef UART_SWEEP_TX_PERIOD_MS
#define UART_SWEEP_TX_PERIOD_MS 1000
#endif
// Bitmask of which ports to drive on TX: bit0=Serial1, bit1=Serial2, bit2=Serial3.
// Default 0x07 = all three. Serial2 is the GPS port; driving its TX only feeds the
// (normally unconnected) GPS RX line and is safe for this diagnostic.
#ifndef UART_SWEEP_TX_MASK
#define UART_SWEEP_TX_MASK 0x07
#endif

constexpr long USB_BAUD = 115200;
constexpr uint32_t REPORT_PERIOD_MS = 1000;
constexpr size_t RX_PREVIEW_MAX = 48;
constexpr uint8_t PORT_COUNT = 3;

struct UartPort {
  Stream *stream;
  const char *name;
  const char *pins;
  uint32_t txCount;
  uint32_t rxCount;
  char rxPreview[RX_PREVIEW_MAX + 1];
  size_t rxPreviewLen;
  bool rxAnnounced;
};

UartPort ports[PORT_COUNT];

uint8_t checksumXor(const char *body) {
  uint8_t checksum = 0;
  for (const char *p = body; *p != '\0'; ++p) {
    checksum ^= static_cast<uint8_t>(*p);
  }
  return checksum;
}

void appendPreview(UartPort &port, char ch) {
  char printable = (ch >= 32 && ch < 127) ? ch : '.';
  if (port.rxPreviewLen < RX_PREVIEW_MAX) {
    port.rxPreview[port.rxPreviewLen++] = printable;
    port.rxPreview[port.rxPreviewLen] = '\0';
  } else {
    memmove(port.rxPreview, port.rxPreview + 1, RX_PREVIEW_MAX - 1);
    port.rxPreview[RX_PREVIEW_MAX - 1] = printable;
    port.rxPreview[RX_PREVIEW_MAX] = '\0';
  }
}

void sendTagged(UartPort &port, uint8_t portNum) {
  char body[24];
  snprintf(body, sizeof(body), "UART,%u,TX_TEST", static_cast<unsigned>(portNum));
  uint8_t cs = checksumXor(body);
  port.stream->print('@');
  port.stream->print(body);
  port.stream->print('*');
  if (cs < 0x10) {
    port.stream->print('0');
  }
  port.stream->println(cs, HEX);
  port.txCount++;
}

void readPort(UartPort &port) {
  uint32_t newBytes = 0;
  while (port.stream->available() > 0) {
    int b = port.stream->read();
    if (b < 0) {
      break;
    }
    port.rxCount++;
    newBytes++;
    appendPreview(port, static_cast<char>(b));
  }
  if (newBytes > 0) {
    // Real-time RX event: this is the strong signal that a module transmits into
    // this UART (the HC-12 RX port, or GPS NMEA on Serial2).
    Serial.print("uart_rx_event port=");
    Serial.print(port.name);
    Serial.print(" new_bytes=");
    Serial.print(newBytes);
    Serial.print(" total_rx=");
    Serial.print(port.rxCount);
    Serial.print(" data=\"");
    Serial.print(port.rxPreview);
    Serial.println("\"");
    if (!port.rxAnnounced) {
      port.rxAnnounced = true;
      Serial.print("RX_FIRST_DETECTED port=");
      Serial.print(port.name);
      Serial.print(" pins=");
      Serial.println(port.pins);
    }
  }
}

void printBanner() {
  Serial.println();
  Serial.println("Firmware: uart_port_sweep_probe uart-mapping-diagnostic 2026-06-03");
  Serial.println("Safe UART sweep. No motors, GPS parsing, RC, I2C, or autonomy.");
  Serial.print("usb_baud=");
  Serial.print(USB_BAUD);
  Serial.print(" uart_baud=");
  Serial.print(UART_SWEEP_BAUD);
  Serial.print(" tx_period_ms=");
  Serial.print(UART_SWEEP_TX_PERIOD_MS);
  Serial.print(" tx_mask=0x");
  Serial.println(UART_SWEEP_TX_MASK, HEX);
  for (uint8_t i = 0; i < PORT_COUNT; ++i) {
    Serial.print("port=");
    Serial.print(ports[i].name);
    Serial.print(" pins=");
    Serial.print(ports[i].pins);
    Serial.print(" tx_driven=");
    Serial.println((UART_SWEEP_TX_MASK & (1 << i)) ? "yes" : "no");
  }
  Serial.println("Each driven port sends @UART,<port>,TX_TEST*CS once per tx period.");
  Serial.println("Identify HC-12: the station sees frames from the UART the HC-12 TX is on;");
  Serial.println("the UART whose rx_count rises when the station sends is the HC-12 RX port.");
  Serial.println("Serial2 rx showing $GP/$GN NMEA confirms GPS on Serial2 (expected).");
}

void printReport() {
  Serial.print("uart_sweep_alive=true sample_ms=");
  Serial.print(millis());
  for (uint8_t i = 0; i < PORT_COUNT; ++i) {
    Serial.print(" ");
    Serial.print(ports[i].name);
    Serial.print("_tx=");
    Serial.print(ports[i].txCount);
    Serial.print(" ");
    Serial.print(ports[i].name);
    Serial.print("_rx=");
    Serial.print(ports[i].rxCount);
    Serial.print(" ");
    Serial.print(ports[i].name);
    Serial.print("_rx_preview=\"");
    Serial.print(ports[i].rxPreview);
    Serial.print("\"");
  }
  Serial.println();
}

void initPort(uint8_t index, Stream *stream, const char *name, const char *pins) {
  ports[index].stream = stream;
  ports[index].name = name;
  ports[index].pins = pins;
  ports[index].txCount = 0;
  ports[index].rxCount = 0;
  ports[index].rxPreview[0] = '\0';
  ports[index].rxPreviewLen = 0;
  ports[index].rxAnnounced = false;
}

uint32_t lastTxMs = 0;
uint32_t lastReportMs = 0;

void setup() {
  Serial.begin(USB_BAUD);
  uint32_t start = millis();
  while (!Serial && millis() - start < 2000) {
    delay(10);
  }

  Serial1.begin(UART_SWEEP_BAUD);
  Serial2.begin(UART_SWEEP_BAUD);
  Serial3.begin(UART_SWEEP_BAUD);

  initPort(0, &Serial1, "Serial1", "D26 TX / D27 RX (PA12/PA13, SD-SPI MOSI/SCK)");
  initPort(1, &Serial2, "Serial2", "D28 TX / D29 RX (PA14/PA15, SD-SPI SS/MISO; GPS)");
  initPort(2, &Serial3, "Serial3", "D14 TX / D13 RX (PB22/PB23, exp uart)");

  printBanner();
}

void loop() {
  for (uint8_t i = 0; i < PORT_COUNT; ++i) {
    readPort(ports[i]);
  }

  uint32_t now = millis();
  if (now - lastTxMs >= UART_SWEEP_TX_PERIOD_MS) {
    lastTxMs = now;
    for (uint8_t i = 0; i < PORT_COUNT; ++i) {
      if (UART_SWEEP_TX_MASK & (1 << i)) {
        sendTagged(ports[i], i + 1);
      }
    }
  }

  if (now - lastReportMs >= REPORT_PERIOD_MS) {
    lastReportMs = now;
    printReport();
  }
}
