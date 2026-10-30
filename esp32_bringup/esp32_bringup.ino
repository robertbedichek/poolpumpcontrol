/*
  Bring-up test for the Waveshare ESP32-S3-POE-ETH-8DI-8DO (transistor outputs, not relays) before the pool firmware goes on.
  The 8 outputs are forced off (TCA9554 latch 1 = open) at boot and never touched.  Over USB serial (115200) it reports:
    - I2C devices (TCA9554 relay expander 0x20, PCF85063 RTC 0x51)
    - Ethernet (W5500) link and DHCP address
    - a Wi-Fi scan (SSID, RSSI, channel), without joining anything
    - Modbus RTU replies from the analog module (address 1, 9600 8N1): raw bytes and the 8 channels
    - the 8 digital inputs, printed whenever one changes
  Serial commands: s = Wi-Fi scan, m = Modbus read, i = inputs, e = Ethernet, r = repeat everything.
*/
#include <Wire.h>
#include <SPI.h>
#include <ETH.h>
#include <WiFi.h>

#define I2C_SDA_PIN   (42)
#define I2C_SCL_PIN   (41)
#define TCA9554_ADDR  (0x20)
#define ETH_CS_PIN    (16)
#define ETH_INT_PIN   (12)
#define ETH_SCLK_PIN  (15)
#define ETH_MOSI_PIN  (13)
#define ETH_MISO_PIN  (14)
#define RS485_TX_PIN  (17)
#define RS485_RX_PIN  (18)
#define RS485_DE_PIN  (21)    // driver enable, driven as UART RTS in RS485 half-duplex mode

HardwareSerial RS485(1);
const int di_pins[8] = {4, 5, 6, 7, 8, 9, 10, 11};
int last_inputs = -1;

static void outputs_off(void)
{
  Wire.beginTransmission(TCA9554_ADDR);   // output latch: 1 = output open (off), as Waveshare's demo
  Wire.write(1);
  Wire.write(0xFF);
  int a = Wire.endTransmission();
  Wire.beginTransmission(TCA9554_ADDR);   // direction: all outputs
  Wire.write(3);
  Wire.write(0);
  int b = Wire.endTransmission();
  Serial.printf("outputs off: %s\n", a == 0 && b == 0 ? "ok" : "I2C error");
}

static void i2c_scan(void)
{
  Serial.print("I2C devices:");
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf(" 0x%02X", a);
    }
  }
  Serial.println();
}

static void eth_report(void)
{
  Serial.printf("Ethernet: link %s, %d Mb/s %s duplex, IP %s, MAC %s\n", ETH.linkUp() ? "up" : "down",
                ETH.linkSpeed(), ETH.fullDuplex() ? "full" : "half", ETH.localIP().toString().c_str(),
                ETH.macAddress().c_str());
}

static void wifi_scan(void)
{
  Serial.printf("Wi-Fi MAC %s, scanning...\n", WiFi.macAddress().c_str());
  int n = WiFi.scanNetworks();
  Serial.printf("Wi-Fi: %d networks\n", n);
  for (int i = 0; i < n; i++) {
    Serial.printf("  %4d dBm  ch %2d  %s  %s\n", WiFi.RSSI(i), WiFi.channel(i), WiFi.BSSIDstr(i).c_str(),
                  WiFi.SSID(i).length() ? WiFi.SSID(i).c_str() : "(hidden)");
  }
  WiFi.scanDelete();
}

static void rs485_begin(long baud, uint32_t mode)
{
  RS485.end();
  RS485.begin(baud, mode, RS485_RX_PIN, RS485_TX_PIN);
  RS485.setPins(-1, -1, -1, RS485_DE_PIN);
  RS485.setMode(UART_MODE_RS485_HALF_DUPLEX);
}

static uint16_t crc16(const uint8_t *b, size_t n)
{
  uint16_t c = 0xFFFF;
  for (size_t i = 0; i < n; i++) {
    c ^= b[i];
    for (int k = 0; k < 8; k++) {
      c = (c & 1) ? (c >> 1) ^ 0xA001 : c >> 1;
    }
  }
  return c;
}

static void modbus_read(uint8_t fn, uint8_t addr = 1, uint16_t reg = 0)
{
  uint8_t req[8] = {addr, fn, (uint8_t)(reg >> 8), (uint8_t)reg, 0x00, 0x08};
  uint16_t c = crc16(req, 6);
  req[6] = c & 0xFF;
  req[7] = c >> 8;
  while (RS485.available()) {
    RS485.read();
  }
  RS485.write(req, 8);
  RS485.flush();
  uint8_t r[64];
  size_t got = 0;
  unsigned long start = millis();
  while (millis() - start < 300 && got < sizeof(r)) {
    if (RS485.available()) {
      r[got++] = RS485.read();
    }
  }
  Serial.printf("Modbus fn %02X: %u bytes:", fn, (unsigned)got);
  for (size_t i = 0; i < got; i++) {
    Serial.printf(" %02X", r[i]);
  }
  Serial.println();
  if (got >= 5 && got == (size_t)r[2] + 5 && crc16(r, got - 2) == (r[got - 2] | (r[got - 1] << 8))) {
    Serial.print("  CRC ok, channels:");
    for (int i = 0; i < r[2] / 2; i++) {
      Serial.printf(" %u", (r[3 + 2 * i] << 8) | r[4 + 2 * i]);
    }
    Serial.println();
  } else if (got) {
    Serial.println("  bad length or CRC");
  }
}

static void sweep(void)
{
  const long bauds[] = {9600, 4800, 19200, 38400, 115200};
  const uint32_t modes[] = {SERIAL_8N1, SERIAL_8E1, SERIAL_8O1, SERIAL_8N2};
  const char *names[] = {"8N1", "8E1", "8O1", "8N2"};
  for (int m = 0; m < 4; m++) {
    for (long b : bauds) {
      rs485_begin(b, modes[m]);
      Serial.printf("-- %ld %s\n", b, names[m]);
      modbus_read(0x04, 1);
      modbus_read(0x03, 0);
      delay(100);
    }
  }
  rs485_begin(9600, SERIAL_8N1);
}

static void addr_scan(void)
{
  uint8_t req[8] = {0x00, 0x03, 0x40, 0x00, 0x00, 0x01};   // Waveshare: read device address (broadcast)
  uint16_t c = crc16(req, 6);
  req[6] = c & 0xFF;
  req[7] = c >> 8;
  const long bauds[] = {9600, 4800, 19200, 38400, 115200};
  for (long b : bauds) {
    rs485_begin(b, SERIAL_8N1);
    while (RS485.available()) RS485.read();
    RS485.write(req, 8);
    RS485.flush();
    delay(300);
    Serial.printf("-- %ld read-address:", b);
    while (RS485.available()) Serial.printf(" %02X", RS485.read());
    Serial.println();
  }
  rs485_begin(9600, SERIAL_8N1);
  Serial.println("-- 9600 addresses 1..247:");
  for (int a = 1; a <= 247; a++) {
    uint8_t q[8] = {(uint8_t)a, 0x04, 0, 0, 0, 1};
    uint16_t k = crc16(q, 6);
    q[6] = k & 0xFF;
    q[7] = k >> 8;
    while (RS485.available()) RS485.read();
    RS485.write(q, 8);
    RS485.flush();
    delay(60);
    if (RS485.available()) {
      Serial.printf("  addr %d:", a);
      while (RS485.available()) Serial.printf(" %02X", RS485.read());
      Serial.println();
    }
  }
  Serial.println("-- scan done");
}

static void pin_probe(void)
{
  uint8_t req[8] = {1, 0x04, 0, 0, 0, 8};
  uint16_t c = crc16(req, 6);
  req[6] = c & 0xFF;
  req[7] = c >> 8;
  int levels[3] = {0, 0, 0};
  for (int pin = 17; pin <= 18; pin++) {
    rs485_begin(9600, SERIAL_8N1);
    while (RS485.available()) RS485.read();
    RS485.write(req, 8);
    RS485.flush();
    RS485.end();
    pinMode(RS485_TX_PIN, OUTPUT);
    digitalWrite(RS485_TX_PIN, HIGH);          // idle, so the transceiver goes back to receive
    pinMode(RS485_RX_PIN, INPUT);
    int edges = 0, lows = 0, last = digitalRead(RS485_RX_PIN);
    unsigned long start = micros();
    while (micros() - start < 100000) {
      int v = digitalRead(RS485_RX_PIN);
      if (v != last) edges++;
      if (!v) lows++;
      last = v;
    }
    Serial.printf("GPIO18 after request: %d edges, %d low samples, idle level %d\n", edges, lows, last);
    delay(200);
  }
  int e2 = 0, last = digitalRead(RS485_RX_PIN);
  unsigned long start = micros();
  while (micros() - start < 100000) { int v = digitalRead(RS485_RX_PIN); if (v != last) e2++; last = v; }
  Serial.printf("GPIO18 with no request: %d edges, level %d\n", e2, last);
  rs485_begin(9600, SERIAL_8N1);
}

static void pin_hunt(void)
{
  const int pins[] = {0, 1, 2, 3, 18, 21, 38, 39, 40, 43, 44, 45, 46, 47, 48};
  const int n = sizeof(pins) / sizeof(pins[0]);
  int edges[n], last[n];
  uint8_t req[8] = {1, 0x04, 0, 0, 0, 8};
  uint16_t c = crc16(req, 6);
  req[6] = c & 0xFF;
  req[7] = c >> 8;
  rs485_begin(9600, SERIAL_8N1);
  for (int i = 0; i < n; i++) {
    if (pins[i] != RS485_RX_PIN) pinMode(pins[i], INPUT_PULLUP);
  }
  delay(5);
  bool send = Serial.peek() != 'q';
  if (!send) Serial.read();
  if (send) {
    RS485.write(req, 8);
    RS485.flush();
  }
  for (int i = 0; i < n; i++) {
    if (pins[i] != RS485_RX_PIN) pinMode(pins[i], INPUT_PULLUP);
  }
  delay(5);
  for (int i = 0; i < n; i++) {
    edges[i] = 0;
    last[i] = digitalRead(pins[i]);
  }
  unsigned long start = micros();
  while (micros() - start < 150000) {
    for (int i = 0; i < n; i++) {
      int v = digitalRead(pins[i]);
      if (v != last[i]) edges[i]++;
      last[i] = v;
    }
  }
  Serial.print(send ? "edges after request:" : "edges, no request:  ");
  for (int i = 0; i < n; i++) Serial.printf(" %d:%d", pins[i], edges[i]);
  Serial.printf("  (UART got %d bytes)\n", RS485.available());
}

static void set_voltage_mode(void)
{
  for (int i = 0; i < 8; i++) {
    uint8_t q[8] = {1, 0x06, 0x10, (uint8_t)i, 0x00, 0x00};   // data type 0 = 0-5 V
    uint16_t k = crc16(q, 6);
    q[6] = k & 0xFF;
    q[7] = k >> 8;
    while (RS485.available()) RS485.read();
    RS485.write(q, 8);
    RS485.flush();
    delay(100);
    Serial.printf("set ch%d 0-5V:", i + 1);
    while (RS485.available()) Serial.printf(" %02X", RS485.read());
    Serial.println();
  }
}

static int read_inputs(void)
{
  int v = 0;
  for (int i = 0; i < 8; i++) {
    if (digitalRead(di_pins[i]) == LOW) {
      v |= 1 << i;
    }
  }
  return v;
}

static void report_inputs(int v)
{
  Serial.print("inputs DI1..8:");
  for (int i = 0; i < 8; i++) {
    Serial.print(v & (1 << i) ? " 1" : " 0");
  }
  Serial.println();
}

static void everything(void)
{
  i2c_scan();
  eth_report();
  wifi_scan();
  modbus_read(0x04);
  modbus_read(0x03);
  report_inputs(read_inputs());
}

void setup()
{
  Serial.begin(115200);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 100000);
  outputs_off();
  for (int i = 0; i < 8; i++) {
    pinMode(di_pins[i], INPUT_PULLUP);
  }
  rs485_begin(9600, SERIAL_8N1);
  ETH.setHostname("esp32-bringup");
  SPI.begin(ETH_SCLK_PIN, ETH_MISO_PIN, ETH_MOSI_PIN);
  ETH.begin(ETH_PHY_W5500, 1, ETH_CS_PIN, ETH_INT_PIN, -1, SPI);
  WiFi.mode(WIFI_STA);
  delay(4000);
  Serial.println("# bring-up test ready");
  everything();
}

void loop()
{
  int v = read_inputs();
  if (v != last_inputs) {
    last_inputs = v;
    report_inputs(v);
  }
  if (Serial.available()) {
    switch (Serial.read()) {
      case 's': wifi_scan(); break;
      case 'm': modbus_read(0x04); break;
      case 't': modbus_read(0x03, 1, 0x1000); break;
      case 'V': set_voltage_mode(); break;
      case 'i': report_inputs(read_inputs()); break;
      case 'e': eth_report(); break;
      case 'r': everything(); break;
      case 'b': sweep(); break;
      case 'a': addr_scan(); break;
      case 'p': pin_probe(); break;
      case 'h': pin_hunt(); break;
    }
  }
  delay(20);
}
