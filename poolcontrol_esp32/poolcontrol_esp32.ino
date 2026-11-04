
/*
  This controls a pool pump and a diverter valve.

  ESP32-S3 port of poolpumpcontrol.ino, for the Waveshare ESP32-S3-POE-ETH-8DI-8RO (8 SPDT relays behind a
  TCA9554 I2C expander, 8 isolated digital inputs, W5500 Ethernet with PoE, PCF85063 RTC, isolated RS485)
  plus a Waveshare Modbus RTU Analog Input 8CH on the RS485 port.  The control logic is the Arduino
  version's, unchanged; only the I/O, the clock and the telemetry path are new:

  - Relays 1-4 replace the Qwiic quad relay board, in the same order.  Relays 1-3 switch the pump and
    transformer SSR inputs; relay 4's contacts switch the diverter motor's 24 VAC directly (NO = roof,
    NC = pool), so there is no external direction relay.
  - The last firmware for the 8DO board (latch bit 0 = on) is commit 391f587.  This one must not run on
    an 8DO: there it would turn every output on.  poolctl update checks /status for "board": "8RO".
  - DI1 is the timer switch, DI2 the diverter request from solarthermal.  The LCD, the four keys and
    the red button are gone; the poolctl command (or the USB serial console) replaces them.
  - The analog module's channels 1-4 replace A0-A3, in the same order, read in millivolts.
  - Time comes from NTP and is kept in the RTC, so there is no more time correction from homecontrol.
  - Telemetry lines go to the USB serial port, to a ring buffer served at /log, and over HTTPS to
    bedichek.org, which appends them to home/poolpumpcontrol.txt as the gobble script used to.
  - New firmware is uploaded over the network (poolctl update).  A new image must run for a minute
    with the relay board answering before it is marked good; if it resets before then, the
    bootloader goes back to the previous image.

  Creative Commons Licence
  2025 Robert Bedichek
*/

#include <Arduino.h>
#include <string.h>
#include <ctype.h>
#include <time.h>
#include <Wire.h>
#include <SPI.h>
#include <ETH.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <NetworkClientSecure.h>
#include <Update.h>
#include <Preferences.h>
#include <esp_task_wdt.h>
#include <esp_ota_ops.h>
#include <esp_sntp.h>

#include "secrets.h"  // POOL_LAN_TOKEN and POOL_INGEST_TOKEN, written by poolctl; not in git

#define FIRMWARE_VERSION __DATE__ " " __TIME__

// Reset cause captured at boot.  One of: 'P' (power-on), 'B' (brown-out), 'W' (watchdog),
// 'S' (software, e.g. after a firmware update), 'C' (crash), 'X' (external/other).
char reset_cause = 'P';

//   All the operational code uses this time.  It is local (Pacific) time, set from the RTC at boot and from NTP after.
time_t arduino_time;
bool time_valid = false;

/*****************************************************************************************************/
// Board pins (Waveshare wiki and demo code for the ESP32-S3-POE-ETH-8DI-8RO)

#define I2C_SDA_PIN      (42)
#define I2C_SCL_PIN      (41)
#define TCA9554_ADDR     (0x20)   // Relay expander: EXIO1..8 = relays 1..8, bit set = relay energized
#define PCF85063_ADDR    (0x51)   // RTC

#define ETH_CS_PIN       (16)
#define ETH_INT_PIN      (12)
#define ETH_SCLK_PIN     (15)
#define ETH_MOSI_PIN     (13)
#define ETH_MISO_PIN     (14)

#define RS485_TX_PIN     (17)
#define RS485_RX_PIN     (18)
#define RS485_DE_PIN     (21)     // Transceiver direction, driven as the UART's RTS in RS485 half-duplex mode

#define DI1_PIN          (4)      // DI1..DI8 are GPIO4..GPIO11, low when the input is active
#define DI2_PIN          (5)

/*****************************************************************************************************/
// Relays.  This class has the same three methods as the SparkFun Qwiic_Relay the Arduino version used,
// so the pump and valve code below is unchanged.

#define LV_RELAY_MAIN_PUMP_12V            (1)     // When energized, this relay sends 12VDC to the coil of the 240VAC contactor that powers the pool pump
#define LV_RELAY_BOOST_PUMP_12V           (2)     // For the boost pump relay coil
#define LV_RELAY_DIVERTER_TRANSFORMER_12V (3)     // When energized, this powers the SSR that controls power to the 24VAC transformer
#define LV_RELAY_DIVERTER_DIRECTION       (4)     // COM from the 24VAC transformer, NO to the valve's roof leg, NC to its pool leg.  Energized = roof.

class Onboard_Relays {
public:
  bool begin(void);
  void turnRelayOn(uint8_t relay)  { set(relay, true); }
  void turnRelayOff(uint8_t relay) { set(relay, false); }
  bool getState(uint8_t relay)     { return bits & (1 << (relay - 1)); }
  bool verify(void);
private:
  uint8_t bits = 0;
  void set(uint8_t relay, bool on);
};

Onboard_Relays *quad_lv_relay;

/*****************************************************************************************************/
#define _TASK_SLEEP_ON_IDLE_RUN
#include <TaskScheduler.h>

//    Central (and only) task scheduler data structure.
Scheduler ts;

// Speed of the USB serial console
#define SERIAL_BAUD (115200)

typedef enum {m_normal, m_pump, m_boost, m_24vac, m_diverter, m_last} operating_mode_t;

void read_time_and_sensor_inputs_callback(void);

int force_telemetry;               // Incrementing this forces a line of telemetry to be emitted
void emit_telemetry_callback(void);

void monitor_main_pump_callback(void);
void monitor_boost_pump_callback(void);
//-------------------------------------------------------------------------------------
const unsigned long drain_down_time = 5 * 60 * 1000UL; // 5 minutes
const unsigned long periodic_filter_interval = 24UL * 3600UL * 1000UL;
const unsigned long periodic_filter_duration = 2UL * 3600UL * 1000UL;  // Run for 2 hours during daily cleaning
const unsigned long max_main_pump_on_time = 10UL * 3600UL * 1000UL;
const unsigned long max_daily_filtering_time = 3UL * 3600UL * 1000UL;
const unsigned long max_boost_pump_on_time = 1UL * 3600UL * 1000UL;
const unsigned long periodic_boost_duration = 45UL * 60UL * 1000UL;    // Run for 45 minutes

#define SENSOR_ERROR (-999.0)  // Sentinel value for failed temperature sensors

unsigned long last_diverter_valve_request_change;  // Value of millis() the last time the input from the solarthermal controller changed.

void monitor_diverter_valve_callback(void);
//-------------------------------------------------------------------------------------

void monitor_diag_mode_callback(void);
void monitor_serial_console_callback(void);
void poll_inputs_callback(void);
void check_relays_callback(void);
void sync_rtc_callback(void);
void mark_firmware_good_callback(void);

// This string variable is used by multiple functions below, but not at the same time
char cbuf[100];

/*****************************************************************************************************/

bool manual_main_pump_request;              // Set true by the 'button' command or '+' in m_pump mode
unsigned long main_pump_on_off_time;        // Assigned millis() when main pump is switched on
unsigned long boost_pump_on_off_time;
unsigned long main_pump_daily_run_time;     // Number of milliseconds that the main pump has been on today
bool periodic_filter_request;               // Set true when periodic filtering operation is needed (daily at noon)
bool periodic_boost_request;                // Set true when periodic boost pump operation is needed
int last_periodic_filter_day;               // Day of month when last periodic filtering was started
const unsigned long max_diverter_power_on_time = 300 * 1000UL; // 5 minutes

unsigned long diverter_valve_transformer_on_time;  // Assigned millis() when valve transformer is turned on
unsigned long diverter_valve_in_roof_position_time; // Assigned millis() when valve goes to send-to-roof position

Task monitor_main_pump(TASK_SECOND, TASK_FOREVER, &monitor_main_pump_callback, &ts, true);
Task monitor_boost_pump(TASK_SECOND, TASK_FOREVER, &monitor_boost_pump_callback, &ts, true);

operating_mode_t operating_mode;

unsigned long time_entering_diag_mode;  // Assigned millis() when any diag mode is entered
const unsigned long max_diag_mode_time = 10 * 60 * 1000UL; // Ten minutes
Task monitor_diag_mode(TASK_SECOND * 60, TASK_FOREVER, &monitor_diag_mode_callback, &ts, false);
/*****************************************************************************************************/

Task monitor_serial_console(100, TASK_FOREVER, &monitor_serial_console_callback, &ts, true);

Task monitor_diverter_control(TASK_SECOND * 6, TASK_FOREVER, &monitor_diverter_valve_callback, &ts, true);

Task check_relays(TASK_SECOND * 10, TASK_FOREVER, &check_relays_callback, &ts, true);
Task sync_rtc(TASK_SECOND, TASK_FOREVER, &sync_rtc_callback, &ts, true);
Task mark_firmware_good(TASK_SECOND * 60, TASK_ONCE, &mark_firmware_good_callback, &ts, false);

float pool_temperature1_F = SENSOR_ERROR;
float pool_temperature2_F = SENSOR_ERROR;
float pool_temperature_F = SENSOR_ERROR; // Average of the two pool temperatures, or just one of them if the other is out of range
float outside_temperature_F = SENSOR_ERROR;
float pressure_psi;
uint16_t analog_last_mv[4];    // Latest raw reading of each channel, for /status

#define PRESSURE_SENSOR_MIN_MV (100)    // Below this the pressure sensor is dead or disconnected
const float max_pressure_sending_water_to_pool = 23.0;
const float max_pressure_sending_water_to_roof = 25.0;

Task read_time_and_sensor_inputs(500, TASK_FOREVER, &read_time_and_sensor_inputs_callback, &ts, true);
/*****************************************************************************************************/
// Analog module channels, in the order of the Arduino's A0-A3
#define POOL_TEMPERATURE1_CHANNEL   (0)     // 0-5VDC LM36 that is immersed in pool water, about 1' deep under diving board
#define POOL_TEMPERATURE2_CHANNEL   (1)     // A second LM36 in the same probe, for redundancy
#define PRESSURE_CHANNEL            (2)     // 0-5VDC pressure sensor on top of filter canister
#define OUTSIDE_TEMPERATURE_CHANNEL (3)     // An LM36 sensing ambient temperature near the pool equipment

#define TIMER_SWITCH_INPUT_PIN      (DI1_PIN)   // Mechanical timer switch on outside of house
#define DIVERTER_REQUEST_INPUT_PIN  (DI2_PIN)   // Contact driven by the 'solarthermal' controller

bool timer_switch_on;

void setup_pins(void)
{
  // DI3..DI8 are unused; give them pull-ups too so that they read inactive
  for (int pin = DI1_PIN; pin <= DI1_PIN + 7; pin++) {
    pinMode(pin, INPUT_PULLUP);
  }
}

Task poll_inputs(100, TASK_FOREVER, &poll_inputs_callback, &ts, true);

const int debounce_delay = 100;

Task emit_telemetry(TASK_SECOND, TASK_FOREVER, &emit_telemetry_callback, &ts, true);
/*****************************************************************************************************/
// Output.  Everything the controller says goes through 'out': to the USB serial port, to a ring of
// recent lines (served at /log), and to a queue that the upload task sends to bedichek.org.

class Line_Buffer {
public:
  void init(size_t capacity, bool use_psram);
  void append(const char *line, size_t n);      // n includes the newline
  size_t length(void) { return len; }
  // Copies up to max bytes of whole lines from the front; returns the byte count
  size_t peek(char *dst, size_t max);
  void consume(size_t n);
  void copy_tail(String &dst, size_t max);
private:
  char *buf = nullptr;
  size_t cap = 0;
  size_t len = 0;
};

void Line_Buffer::init(size_t capacity, bool use_psram)
{
  buf = (char *)(use_psram && psramFound() ? ps_malloc(capacity) : malloc(capacity));
  cap = buf ? capacity : 0;
}

void Line_Buffer::append(const char *line, size_t n)
{
  if (n > cap) {
    return;
  }
  if (len + n > cap) {    // Drop the oldest whole lines to make room
    size_t drop = len + n - cap;
    while (drop < len && buf[drop - 1] != '\n') {
      drop++;
    }
    consume(drop);
  }
  memcpy(buf + len, line, n);
  len += n;
}

size_t Line_Buffer::peek(char *dst, size_t max)
{
  size_t n = len < max ? len : max;
  while (n > 0 && buf[n - 1] != '\n') {
    n--;
  }
  memcpy(dst, buf, n);
  return n;
}

void Line_Buffer::consume(size_t n)
{
  if (n >= len) {
    len = 0;
  } else {
    memmove(buf, buf + n, len - n);
    len -= n;
  }
}

void Line_Buffer::copy_tail(String &dst, size_t max)
{
  size_t start = len > max ? len - max : 0;
  while (start > 0 && start < len && buf[start - 1] != '\n') {
    start++;
  }
  dst.concat(buf + start, len - start);
}

Line_Buffer recent_lines;     // The last 64 KB, for /log
Line_Buffer upload_queue;     // Not yet accepted by bedichek.org; 1 MB in PSRAM is weeks of telemetry
bool upload_enabled = false;  // Saved in flash; off on the bench so test lines stay out of the pool log ("upload on|off")
SemaphoreHandle_t lines_mutex;

class Tee_Output : public Print {
public:
  size_t write(uint8_t c) override;
  size_t write(const uint8_t *b, size_t n) override { for (size_t i = 0; i < n; i++) write(b[i]); return n; }
private:
  char line[200];
  size_t n = 0;
};

size_t Tee_Output::write(uint8_t c)
{
  Serial.write(c);
  if (c == '\r') {
    return 1;
  }
  if (n < sizeof(line) - 1) {
    line[n++] = c;
  }
  if (c == '\n') {
    if (line[n - 1] != '\n') {
      line[n++] = '\n';
    }
    xSemaphoreTake(lines_mutex, portMAX_DELAY);
    recent_lines.append(line, n);
    if (upload_enabled) {
      upload_queue.append(line, n);
    }
    xSemaphoreGive(lines_mutex);
    n = 0;
  }
  return 1;
}

Tee_Output out;

/*****************************************************************************************************/
// Time.  These have the names and meanings of the TimeLib functions the Arduino version used.

time_t now(void)          { return time(nullptr); }
static struct tm tm_of(time_t t) { struct tm tm; localtime_r(&t, &tm); return tm; }
int hour(time_t t)        { return tm_of(t).tm_hour; }
int minute(time_t t)      { return tm_of(t).tm_min; }
int second(time_t t)      { return tm_of(t).tm_sec; }
int day(time_t t)         { return tm_of(t).tm_mday; }
int month(time_t t)       { return tm_of(t).tm_mon + 1; }
int year(time_t t)        { return tm_of(t).tm_year + 1900; }

#define TIMEZONE "PST8PDT,M3.2.0,M11.1.0"

/*****************************************************************************************************/

// Flag a warning if there is less than this much free heap.
#define LOW_MEMORY_LIMIT (20000)

// Records the smallest amount of free heap seen
uint32_t lowest_memory = UINT32_MAX;

// This is called by each task when that task starts executing and when the task
// is about to end execution (i.e., entry and exits of tasks).  This emits an
// alert if the amount of free memory is too low.

void check_free_memory(const __FlashStringHelper *caller)
{
  static bool warned = false;
  uint32_t fm = ESP.getFreeHeap();
  if (fm < LOW_MEMORY_LIMIT && !warned) {
    out.print(F("# alert low memory: "));
    out.println(fm);
    warned = true;
  }
  if (fm < lowest_memory) {
    lowest_memory = fm;
  }
}

// Helper function to format temperature for display
// Returns "  -" if sensor has failed, otherwise returns formatted temperature
void format_temperature(float temp_F, char *buf, size_t buf_size) {
  if (temp_F == SENSOR_ERROR) {
    strncpy(buf, "  -", buf_size);
    buf[buf_size - 1] = '\0';
  } else {
    // Format as "XX.X"
    dtostrf(temp_F, 4, 1, buf);
  }
}

const char *operating_mode_to_string(operating_mode_t operating_mode)
{
  // m_pump, m_diverter
  const char *s[] = {"Normal", "Main P", "Boost ", "24 VAC", "Divert"};
  if (operating_mode < m_last) {
    return s[operating_mode];
  }
  return "ERR";
}

// This task is enabled when we enter diag mode.  It checks to see if we've been in diag mode
// too long and reverts to operational mode if so.
void monitor_diag_mode_callback(void)
{
  check_free_memory(F("monitor_diag_mode"));

  if ((millis() - time_entering_diag_mode) > max_diag_mode_time) {
    operating_mode = m_normal;
    out.println(F("# diag mode timed out, mode now Normal"));
    monitor_diag_mode.disable();
  }
}

bool diverter_valve_request()
{
  return digitalRead(DIVERTER_REQUEST_INPUT_PIN) == LOW;
}

// What the plus and minus keys did in the diag modes; now the '+' and '-' commands.

void plus_key(void)
{
  switch (operating_mode) {
    case m_normal:
      out.println(F("# + does nothing in Normal mode (the clock is set by NTP)"));
      break;

    case m_pump:
      manual_main_pump_request = true;
      turn_main_pump_on(F("diag mode: main pump="));
      out.println(main_pump_is_on());
      break;

    case m_boost:
      turn_boost_pump_on(F("diag mode, boost pump="));
      out.println(boost_pump_is_on());
      break;

    case m_24vac:
      turn_diverter_valve_transformer_on();
      break;

    case m_diverter:
      out.print(F("# diverter valve="));
      set_diverter_valve_to_send_water_to_roof();
      if (diverter_valve_is_sending_water_to_roof()) {
        out.println("roof");
      } else {
        out.println("pool");
      }
      break;

    default:
      out.println(F("# unknown operating mode"));
      break;
  }
}

void minus_key(void)
{
  switch (operating_mode) {
    case m_normal:
      out.println(F("# - does nothing in Normal mode (the clock is set by NTP)"));
      break;

    case m_pump:
      manual_main_pump_request = false;
      if (boost_pump_is_on()) {
        turn_boost_pump_off(F("main pump turning off"));
      }

      turn_main_pump_off(F("diag"));
      break;

    case m_boost:
      turn_boost_pump_off(F("diag"));
      break;

    case m_24vac:
      turn_diverter_valve_transformer_off();
      break;

    case m_diverter:
      set_diverter_valve_to_return_water_to_pool();
      out.print(F("# diverter valve="));
      if (diverter_valve_is_sending_water_to_roof()) {
        out.println("roof");
      } else {
        out.println("pool");
      }
      break;

    default:
      out.println(F("# unknown operating mode"));
      break;
  }
}

// What a press of the red button did; now the 'button' command.  Toggles the main pump.

void red_button(void)
{
  if (operating_mode != m_normal) {
    out.println(F("# button works only in Normal mode"));
    return;
  }
  if (main_pump_is_on()) {
    manual_main_pump_request = false;
    if (boost_pump_is_on()) {
      turn_boost_pump_off(F("main pump turning off"));
    }

    turn_main_pump_off(F("button"));
  } else {
    // Check if we're in drain-down period (valve in roof mode, waiting for pool piping to drain)
    if (diverter_valve_is_sending_water_to_roof() &&
        !diverter_valve_request() &&
        (millis() - main_pump_on_off_time) < drain_down_time) {
      out.println(F("# Button: cannot turn on pump - waiting for drain-down"));
    } else {
      manual_main_pump_request = true;
      turn_main_pump_on(F("button"));
    }
  }
}

void set_operating_mode(operating_mode_t m)
{
  operating_mode = m;
  out.print(F("# mode now "));
  out.println(operating_mode_to_string(operating_mode));
  if (operating_mode == m_normal) {
    monitor_diag_mode.disable();
  } else {
    monitor_diag_mode.enable();
    time_entering_diag_mode = millis();
  }
}

// Polls the timer switch and the diverter request.  This is the part of the Arduino version's
// key-processing task that watched those two inputs.

void poll_inputs_callback(void)
{
  check_free_memory(F("poll_inputs.."));

  if (operating_mode == m_normal) {
    static bool last_timer_switch_on;
    static bool last_diverter_valve_request;

    timer_switch_on = digitalRead(TIMER_SWITCH_INPUT_PIN) == LOW;
    if (timer_switch_on != last_timer_switch_on) { // Switch state changed
      static unsigned long timer_switch_on_time;

      if ((millis() - timer_switch_on_time) > debounce_delay) {
        if (timer_switch_on) { // While timer switch is closed, run main pump
          if (main_pump_is_on() == false) {
            // Check if we're in drain-down period
            if (diverter_valve_is_sending_water_to_roof() &&
                !diverter_valve_request() &&
                (millis() - main_pump_on_off_time) < drain_down_time) {
              out.println(F("# Timer switch: cannot turn on pump - waiting for drain-down"));
            } else {
              turn_main_pump_on(F("timer switch"));
            }
          }
          // We will turn on the boost pump when we see that the main pump has developed
          // pressure, to ensure that the main pump is working before we turn on the boost pump

        }
      }
      last_timer_switch_on = timer_switch_on;
      timer_switch_on_time = millis();
    }

    if (diverter_valve_request() != last_diverter_valve_request) {
      last_diverter_valve_request = diverter_valve_request();
      if ((millis() - last_diverter_valve_request_change) > debounce_delay) {
        if (diverter_valve_request()) {
          turn_diverter_valve_transformer_on();
          set_diverter_valve_to_send_water_to_roof();
          turn_main_pump_on(F("diverter request"));

          // We must ensure that if the main pump were already on due to the daily filter program, that
          // we cancel the flag that remembers this.  Otherwise, we will turn off the main pump at the end
          // of the periodic filter time even though we want the main pump to stay on to keep heating
          // the water.
          periodic_filter_request = false;


        } else {
          // Logic in monitor_diverter_valve() will turn the main pump if it should be off and
          // change the diverter valve back to pool-mode after a drain-down time.
          // Logic in monitor_pool_pump will turn off the main pump (there may be several reasons for it
          // needing to stay on)
        }
      }
      last_diverter_valve_request_change = millis();
    }
  }
}

void turn_main_pump_on(const __FlashStringHelper *message)
{
  if (message != nullptr) {
    out.print(F("# "));
    out.println(message);
  }
  if (main_pump_is_on() == false) {
    if (quad_lv_relay != nullptr) {
      quad_lv_relay->turnRelayOn(LV_RELAY_MAIN_PUMP_12V);
    }
    main_pump_on_off_time = millis();
    out.print(F("# turn_main_pump_on(): "));
    out.println(main_pump_on_off_time);
  }
}

void turn_main_pump_off(const __FlashStringHelper *message)
{
  if (boost_pump_is_on()) {
    turn_boost_pump_off(F("# alert turn_main_pump_off() found boost pump on"));
  }
  if (main_pump_is_on()) {
    if (quad_lv_relay != nullptr) {
      quad_lv_relay->turnRelayOff(LV_RELAY_MAIN_PUMP_12V);
    }
    main_pump_daily_run_time += millis() - main_pump_on_off_time;
    main_pump_on_off_time = millis();
    out.print(F("# turn_main_pump_off(): "));
    if (message != nullptr) {
      out.print(message);
    }
    out.print(F(" off-time: "));
    out.println(main_pump_on_off_time);
  }
}

bool main_pump_is_on(void)
{
  bool v = false;
  if (quad_lv_relay != nullptr) {
    v = quad_lv_relay->getState(LV_RELAY_MAIN_PUMP_12V);
  }
  return v;
}

void turn_boost_pump_on(const __FlashStringHelper *message)
{

  // Do not try to turn on the boost pump unless the main pump is on and the boost pump is off
  if (main_pump_is_on() && boost_pump_is_on() == false) {
    if (quad_lv_relay != nullptr) {
      quad_lv_relay->turnRelayOn(LV_RELAY_BOOST_PUMP_12V);
    }
    boost_pump_on_off_time = millis();
    out.print(F("# turn_boost_pump_on(): "));
    if (message != nullptr) {
      out.print(message);
    }
    out.println();
  }
}

void turn_boost_pump_off(const __FlashStringHelper *message)
{
  if (boost_pump_is_on()) {
    if (quad_lv_relay != nullptr) {
      quad_lv_relay->turnRelayOff(LV_RELAY_BOOST_PUMP_12V);
    }
    boost_pump_on_off_time = millis();
    out.print(F("# turn_boost_pump_off(): "));
    if (message != nullptr) {
      out.print(message);
    }
    out.print(F(" time:"));
    out.println(boost_pump_on_off_time);
  }
}

bool boost_pump_is_on(void)
{
  bool v = false;
  if (quad_lv_relay != nullptr) {
    v = quad_lv_relay->getState(LV_RELAY_BOOST_PUMP_12V);
  }
  return v;
}

void turn_diverter_valve_transformer_on(void)
{
  if (quad_lv_relay != nullptr) {
    quad_lv_relay->turnRelayOn(LV_RELAY_DIVERTER_TRANSFORMER_12V);
  }
  diverter_valve_transformer_on_time = millis();
  out.print(F("# turning on diverter valve transformer millis()="));
  out.println(diverter_valve_transformer_on_time);
}

void turn_diverter_valve_transformer_off(void)
{
  if (quad_lv_relay != nullptr) {
    quad_lv_relay->turnRelayOff(LV_RELAY_DIVERTER_TRANSFORMER_12V);
  }
  diverter_valve_transformer_on_time = 0;
  out.print(F("# turning off diverter valve transformer millis()="));
  out.println(millis());
}


bool diverter_valve_transformer_is_on(void)
{
  bool v = false;
  if (quad_lv_relay != nullptr) {
    v = quad_lv_relay->getState(LV_RELAY_DIVERTER_TRANSFORMER_12V);
  }
  return v;
}

// Returns true if the diverter valve is sending water to the roof

bool diverter_valve_is_sending_water_to_roof(void)
{
  bool v = false;
  if (quad_lv_relay != nullptr) {
    v = quad_lv_relay->getState(LV_RELAY_DIVERTER_DIRECTION);
  }
  return v;
}

bool diverter_valve_is_sending_water_to_pool(void)
{
  return !diverter_valve_is_sending_water_to_roof();
}

// Set the relay that takes the 24VAC power and
// sends it to the "open" leg of the diverter valve by setting the single-pole
// dual-throw relay.  To actually make the diverter valve turn, one must call
// turn_diverter_valve_transformer_on() before or after this call.

void set_diverter_valve_to_send_water_to_roof(void)
{
  diverter_valve_in_roof_position_time = millis();
  if (diverter_valve_is_sending_water_to_pool()) {
    if (quad_lv_relay != nullptr) {
      quad_lv_relay->turnRelayOn(LV_RELAY_DIVERTER_DIRECTION);
    }
  }
}

// Only reset the diverter valve to return water to pool after the main pump has been off for a few minutes.
// To actually make the diverter valve turn, one must call
// turn_diverter_valve_transformer_on() before or after this call.

void set_diverter_valve_to_return_water_to_pool(void)
{
  if (diverter_valve_is_sending_water_to_roof()) {
    if (quad_lv_relay != nullptr) {
      quad_lv_relay->turnRelayOff(LV_RELAY_DIVERTER_DIRECTION);
    }
  }
}


void monitor_main_pump_callback(void)
{
  if (time_valid && hour(arduino_time) == 0 && minute(arduino_time) == 0) {
    if (main_pump_is_on() && !(operating_mode == m_normal && timer_switch_on)) {
      turn_main_pump_off(F("midnight"));
    }
    main_pump_daily_run_time = 0;
  }
  check_free_memory(F("monitor_pump"));
  if (operating_mode == m_normal && timer_switch_on) {
    // The timer switch runs the main pump regardless of anything else: no drain-down wait, time
    // limit or overpressure shutoff.  monitor_boost_pump_callback() starts the boost pump once there is pressure.
    if (!main_pump_is_on()) {
      turn_main_pump_on(F("timer switch"));
    }
  } else if (operating_mode == m_normal) {
    if (main_pump_is_on()) { // Pump is on, see if any of the reasons for it to turn off have occured

      // Determine appropriate time limit based on whether this is periodic filtering or other operation
      unsigned long pump_time_limit = periodic_filter_request ? periodic_filter_duration : max_main_pump_on_time;

      if ((millis() - main_pump_on_off_time) > pump_time_limit) {
        if (boost_pump_is_on()) {
          turn_boost_pump_off(F("main pump turning off"));
        }
        // Turn off the pump
        if (periodic_filter_request) {
          turn_main_pump_off(F("end of periodic filtering"));
          periodic_filter_request = false;
        } else {
          turn_main_pump_off(F("pump run-time limit"));
        }
        manual_main_pump_request = false; // Cancel request
      } else {
        float max_psi = diverter_valve_is_sending_water_to_roof() ? max_pressure_sending_water_to_roof : max_pressure_sending_water_to_pool;

        if (pressure_psi > max_psi) {
          if (boost_pump_is_on()) {
            turn_boost_pump_off(F("main pump turning off"));
          }

          turn_main_pump_off(F("overpressure"));
          manual_main_pump_request = false;
        } else if (!diverter_valve_request()) {
          // If we are not being asked to send water to the roof, the pool-cleaner timer switch
          // is not requesting pool cleaning, and this is not the time to filter the pool water
          // by running the pump, then turn it off.
          if (!timer_switch_on && !manual_main_pump_request && !periodic_boost_request) {
            if (boost_pump_is_on()) {
              turn_boost_pump_off(F("main pump turning off"));
            }

            turn_main_pump_off(F("lack of timer switch and manual (red button) requests"));
          } else {
            if (diverter_valve_is_sending_water_to_roof() &&
            (millis() - last_diverter_valve_request_change) < 60 * 1000UL) {
              // Although we have other reasons to run the pump, we need to turn it off to let the panels
              // drain of pool water.  We will later turn it back on.  We distinguish this case by seeing
              // if the the diverter request just went away within the last minute.
              if (boost_pump_is_on()) {
                turn_boost_pump_off(F("main pump turning off"));
              }

              turn_main_pump_off(F("let panels drain"));
              manual_main_pump_request = false;
            }
          }
        }
      }
    } else { // Pump is not on, see if it should be, but only after a several minute delay for any draining needed
      if (!diverter_valve_request() && (millis() - main_pump_on_off_time) > drain_down_time) {
        if (timer_switch_on) {
          turn_main_pump_on(F("timer switch"));
        } else if (time_valid) {
          // Check if it's noon and we haven't run periodic filtering today
          int current_hour = hour(arduino_time);
          int current_day = day(arduino_time);
          if (current_hour == 12 && current_day != last_periodic_filter_day) {
            periodic_filter_request = true;
            last_periodic_filter_day = current_day;
            turn_main_pump_on(F("filtering at noon"));
            periodic_boost_request = true;
            manual_main_pump_request = true;
          }
        }
      }
    }
  }
  check_free_memory(F("monitor_pump exit"));
}

#define BOOST_MIN_PSI           (5.0)          // Pressure that shows the main pump is running
#define BOOST_LOW_PRESSURE_MS   (10 * 1000UL)  // The boost pump never runs longer than this without it

// Runs every second.  The boost pump starts as soon as there is pressure, if the timer switch or the
// periodic run wants it, and stops when neither does, when the main pump stops, or after
// BOOST_LOW_PRESSURE_MS without pressure (in every mode).
void monitor_boost_pump_callback(void)
{
  static unsigned long last_pressure_ok;   // millis() when pressure was last >= BOOST_MIN_PSI
  if (pressure_psi >= BOOST_MIN_PSI) {
    last_pressure_ok = millis();
  }

  if (boost_pump_is_on() && (millis() - last_pressure_ok) > BOOST_LOW_PRESSURE_MS) {
    turn_boost_pump_off(F("low pressure"));
    periodic_boost_request = false;
    return;
  }

  if (operating_mode == m_normal) {
    if (!boost_pump_is_on()) {
      if (main_pump_is_on() && pressure_psi >= BOOST_MIN_PSI) {
        if (timer_switch_on) {
          turn_boost_pump_on(F("timer switch request"));
          periodic_boost_request = false; // Timer switch overrides periodic
        } else if (periodic_boost_request) {
          turn_boost_pump_on(F("periodic operation"));
        }
      }
    } else if (!main_pump_is_on()) {
      turn_boost_pump_off(F("main pump off"));
      periodic_boost_request = false;
    } else if (!timer_switch_on && !periodic_boost_request) {
      turn_boost_pump_off(F("no request"));
    }
    if (timer_switch_on) {
      return;   // No time limit while the timer switch is on
    }
  }

  // Check time limits
  unsigned long boost_time_limit = periodic_boost_request ? periodic_boost_duration : max_boost_pump_on_time;
  if (boost_pump_is_on() && (millis() - boost_pump_on_off_time) > boost_time_limit) {
    // Turn off the boost pump
    if (periodic_boost_request) {
      turn_boost_pump_off(F("end of periodic 45-minute run"));
      periodic_boost_request = false;
    } else {
      turn_boost_pump_off(F("time limit"));
    }
  }
}

void monitor_diverter_valve_callback(void)
{
  check_free_memory(F("monitor_diverter_valve"));
  if (operating_mode == m_normal) {
    // If the main pump has been off for two minutes and the diverter valve is set to send
    // water to the roof, we assume that the panels have had time to drain and we
    // turn the diverter valve back to pool mode
    if (!main_pump_is_on() && diverter_valve_is_sending_water_to_roof()) {
      if ((millis() - main_pump_on_off_time) > drain_down_time) {
        turn_diverter_valve_transformer_on();
        set_diverter_valve_to_return_water_to_pool();
      }
    }
    if (diverter_valve_transformer_is_on()) {
      if ((millis() - diverter_valve_transformer_on_time) > max_diverter_power_on_time) {
        turn_diverter_valve_transformer_off();
        out.println(F("# turning off diverter valve power due to time limit"));
      }
    }
  }
  check_free_memory(F("monitor_diverter_valve exit"));
}

/*****************************************************************************************************/
// Output expander (TCA9554).  Register 1 is the output latch, register 3 the direction (1 = input).
// A latch bit of 1 energizes its relay (the opposite of the 8DO board), so `bits` is the latch.
// The expander keeps its state through an ESP32 reset (e.g. a firmware update), so the pumps keep
// running; after a power-up all pins are inputs and every relay is released.

static bool tca_write(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(TCA9554_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

static int tca_read(uint8_t reg)
{
  Wire.beginTransmission(TCA9554_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(TCA9554_ADDR, 1) != 1) {
    return -1;
  }
  return Wire.read();
}

bool Onboard_Relays::begin(void)
{
  int direction = tca_read(3);
  int latch = tca_read(1);
  if (direction < 0 || latch < 0) {
    return false;
  }
  if (direction == 0x00) {          // Already set up: keep the relays as they are
    bits = latch;
    out.print(F("# relays kept through reset: 0x"));
    out.println(bits, HEX);
    return true;
  }
  bits = 0;
  return tca_write(1, 0x00) && tca_write(3, 0x00);
}

void Onboard_Relays::set(uint8_t relay, bool on)
{
  uint8_t b = on ? (bits | (1 << (relay - 1))) : (bits & ~(1 << (relay - 1)));
  if (tca_write(1, b)) {
    bits = b;
  } else {
    out.print(F("# alert relay board did not answer setting relay "));
    out.println(relay);
  }
}

// Reads the latch back and rewrites it if it differs (e.g. the expander lost power).
bool Onboard_Relays::verify(void)
{
  int latch = tca_read(1);
  int direction = tca_read(3);
  if (latch < 0 || direction < 0) {
    return false;
  }
  if (latch != bits || direction != 0x00) {
    out.print(F("# alert relay latch was 0x"));
    out.print(latch, HEX);
    out.print(F(", rewriting 0x"));
    out.println(bits, HEX);
    tca_write(1, bits);
    tca_write(3, 0x00);
  }
  return true;
}

void check_relays_callback(void)
{
  static int failures;
  if (quad_lv_relay == nullptr) {
    quad_lv_relay = new Onboard_Relays();
    if (!quad_lv_relay->begin()) {
      delete quad_lv_relay;
      quad_lv_relay = nullptr;
      if (failures++ % 60 == 0) {
        out.println(F("# alert relay board not found"));
      }
    }
    return;
  }
  if (!quad_lv_relay->verify()) {
    if (failures++ % 60 == 0) {
      out.println(F("# alert relay board did not answer"));
    }
  } else {
    failures = 0;
  }
}

/*****************************************************************************************************/
// Analog module (Waveshare Modbus RTU Analog Input 8CH) on RS485.  Channel values are input
// registers 0-7, read with function 4; in voltage mode (data type 0) they are in millivolts.
// Data types are holding registers 0x1000-0x1007.  Each channel's jumper must be removed for
// voltage mode.

#define ANALOG_MODBUS_ADDR  (1)
#define ANALOG_BAUD         (9600)
#define ANALOG_CHANNELS_USED (4)

HardwareSerial RS485(1);

static uint16_t modbus_crc(const uint8_t *b, size_t n)
{
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < n; i++) {
    crc ^= b[i];
    for (int j = 0; j < 8; j++) {
      crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
    }
  }
  return crc;
}

// Sends a request (the CRC is appended here) and reads a reply of reply_len bytes.  Returns true
// if the reply has the right length, address, function and CRC.
static bool modbus_transaction(uint8_t *req, size_t req_len, uint8_t *reply, size_t reply_len)
{
  uint16_t crc = modbus_crc(req, req_len);
  req[req_len] = crc & 0xFF;
  req[req_len + 1] = crc >> 8;
  while (RS485.available()) {
    RS485.read();
  }
  RS485.write(req, req_len + 2);
  RS485.flush();
  size_t got = 0;
  unsigned long start = millis();
  while (got < reply_len && millis() - start < 100) {
    if (RS485.available()) {
      reply[got++] = RS485.read();
    }
  }
  if (got != reply_len || reply[0] != req[0] || reply[1] != req[1]) {
    return false;
  }
  return modbus_crc(reply, reply_len - 2) == (reply[reply_len - 2] | (reply[reply_len - 1] << 8));
}

bool analog_read_millivolts(uint16_t mv[ANALOG_CHANNELS_USED])
{
  uint8_t req[8] = {ANALOG_MODBUS_ADDR, 0x04, 0x00, 0x00, 0x00, ANALOG_CHANNELS_USED};
  uint8_t reply[5 + 2 * ANALOG_CHANNELS_USED];
  if (!modbus_transaction(req, 6, reply, sizeof(reply))) {
    return false;
  }
  for (int i = 0; i < ANALOG_CHANNELS_USED; i++) {
    mv[i] = (reply[3 + 2 * i] << 8) | reply[4 + 2 * i];
  }
  return true;
}

// Puts the channels we use in 0-5V mode (data type 0) if they are not already.  The module keeps
// the setting, so this normally writes nothing.
bool analog_set_voltage_mode(void)
{
  uint8_t req[8] = {ANALOG_MODBUS_ADDR, 0x03, 0x10, 0x00, 0x00, ANALOG_CHANNELS_USED};
  uint8_t reply[5 + 2 * ANALOG_CHANNELS_USED];
  if (!modbus_transaction(req, 6, reply, sizeof(reply))) {
    return false;
  }
  for (int i = 0; i < ANALOG_CHANNELS_USED; i++) {
    uint16_t type = (reply[3 + 2 * i] << 8) | reply[4 + 2 * i];
    if (type != 0) {
      uint8_t wreq[8] = {ANALOG_MODBUS_ADDR, 0x06, 0x10, (uint8_t)i, 0x00, 0x00};
      uint8_t wreply[8];
      if (!modbus_transaction(wreq, 6, wreply, sizeof(wreply))) {
        return false;
      }
      out.print(F("# analog channel "));
      out.print(i + 1);
      out.println(F(" set to 0-5V mode"));
    }
  }
  return true;
}

bool analog_mode_set = false;
unsigned analog_failures;

// --- Insertion sort (in-place, ascending) --------------------------------
static void insertionSort(int arr[], int n) {
    for (int i = 1; i < n; i++) {
        int key = arr[i];
        int j   = i - 1;
        while (j >= 0 && arr[j] > key) {
            arr[j + 1] = arr[j];
            j--;
        }
        arr[j + 1] = key;
    }
}

// This is called twice per second.  It samples the time, temperature, and pressure.  It records these in globals read by other
// tasks.  The Arduino took 16 quick samples per call; the analog module is read once per call, and the
// last 16 readings (8 seconds) are trimmed and averaged the same way.

void read_time_and_sensor_inputs_callback(void)
{
  check_free_memory(F("read_time_and.."));
  arduino_time = now();

#define NUM_SAMPLES (16)
#define TRIM_COUNT  (4)       // discard this many from each end

  static int pool1_ring[NUM_SAMPLES];
  static int pool2_ring[NUM_SAMPLES];
  static int outside_ring[NUM_SAMPLES];
  static int pressure_ring[NUM_SAMPLES];
  static int samples;         // Number of readings so far, up to NUM_SAMPLES

  if (!analog_mode_set) {
    analog_mode_set = analog_set_voltage_mode();
  }

  uint16_t mv[ANALOG_CHANNELS_USED];
  if (!analog_mode_set || !analog_read_millivolts(mv)) {
    // After 10 seconds without readings, report the sensors as failed.  Pressure reads zero, as an
    // unplugged sensor did on the Arduino: the boost pump stops and overpressure is not detected.
    if (++analog_failures == 20) {
      out.println(F("# alert analog module not answering"));
      pool_temperature1_F = pool_temperature2_F = pool_temperature_F = outside_temperature_F = SENSOR_ERROR;
      pressure_psi = 0;
      samples = 0;
    }
    return;
  }
  if (analog_failures >= 20) {
    out.println(F("# alert analog module answering again"));
  }
  analog_failures = 0;
  memcpy(analog_last_mv, mv, sizeof(analog_last_mv));

  int slot = samples % NUM_SAMPLES;
  pool1_ring[slot] = mv[POOL_TEMPERATURE1_CHANNEL];
  pool2_ring[slot] = mv[POOL_TEMPERATURE2_CHANNEL];
  outside_ring[slot] = mv[OUTSIDE_TEMPERATURE_CHANNEL];
  pressure_ring[slot] = mv[PRESSURE_CHANNEL];
  samples++;
  if (samples < NUM_SAMPLES) {
    return;
  }
  samples = NUM_SAMPLES + slot + 1;   // Keep counting slots without overflowing

  int pool1[NUM_SAMPLES], pool2[NUM_SAMPLES], outside[NUM_SAMPLES], pressure[NUM_SAMPLES];
  memcpy(pool1, pool1_ring, sizeof(pool1));
  memcpy(pool2, pool2_ring, sizeof(pool2));
  memcpy(outside, outside_ring, sizeof(outside));
  memcpy(pressure, pressure_ring, sizeof(pressure));

  insertionSort(pool1, NUM_SAMPLES);
  insertionSort(pool2, NUM_SAMPLES);
  insertionSort(outside, NUM_SAMPLES);
  insertionSort(pressure, NUM_SAMPLES);

  // Sum the middle samples, skipping TRIM_COUNT on each end

  float pool_temperature1_millivolts = 0;
  float pool_temperature2_millivolts = 0;
  float outside_temperature_millivolts = 0;
  float pressure_millivolts = 0;

  const int keepCount = NUM_SAMPLES - 2 * TRIM_COUNT;
  for (int i = TRIM_COUNT; i < NUM_SAMPLES - TRIM_COUNT; i++) {
    pool_temperature1_millivolts += pool1[i];
    pool_temperature2_millivolts += pool2[i];
    outside_temperature_millivolts += outside[i];
    pressure_millivolts += pressure[i];
  }

  pool_temperature1_millivolts /= keepCount;
  pool_temperature2_millivolts /= keepCount;
  outside_temperature_millivolts /= keepCount;
  pressure_millivolts /= keepCount;

  // See the LM36 data shee to understand the conversion of millivolts to degrees C
  float pool_temperature1_C = (pool_temperature1_millivolts - 500.0) / 10.0;

  // Convert Centigrade to Fahrenheit
  float pool_temperature1_this_sample_F = pool_temperature1_C * 9.0 / 5.0 + 32.0;
  pool_temperature1_this_sample_F -= 0.8; // Emperically derived

  // EMA filter
  const float alpha = 0.5;
  if (pool_temperature1_F == SENSOR_ERROR) {
    pool_temperature1_F = pool_temperature1_this_sample_F;
  }
  pool_temperature1_F = alpha * pool_temperature1_this_sample_F + (1.0 - alpha) * pool_temperature1_F;

  if (pool_temperature1_F < 32.0 || pool_temperature1_F > 105.0) {
    // Something is wrong with the sensor
    pool_temperature1_F = SENSOR_ERROR;
  }

  // See the LM36 data shee to understand the conversion of millivolts to degrees C
  float pool_temperature2_C = (pool_temperature2_millivolts - 500.0) / 10.0;

  // Convert Centigrade to Fahrenheit
  float pool_temperature2_this_sample_F = pool_temperature2_C * 9.0 / 5.0 + 32.0;
  pool_temperature2_this_sample_F += 0.8; // Emperically derived

  // EMA filter
  if (pool_temperature2_F == SENSOR_ERROR) {
    pool_temperature2_F = pool_temperature2_this_sample_F;
  }
  pool_temperature2_F = alpha * pool_temperature2_this_sample_F + (1.0 - alpha) * pool_temperature2_F;

  if (pool_temperature2_F < 32.0 || pool_temperature2_F > 105.0) {
    // Something is wrong with the sensor
    pool_temperature2_F = SENSOR_ERROR;
  }

  // Average of the two pool temperatures, or just one of them if the other is out of range
  if (pool_temperature1_F == SENSOR_ERROR && pool_temperature2_F == SENSOR_ERROR) {
    pool_temperature_F = SENSOR_ERROR; // Both sensors failed
  } else if (pool_temperature1_F == SENSOR_ERROR) {
    pool_temperature_F = pool_temperature2_F; // Use only sensor 2
  } else if (pool_temperature2_F == SENSOR_ERROR) {
    pool_temperature_F = pool_temperature1_F; // Use only sensor 1
  } else {
    // Both sensors are good, use average
    pool_temperature_F = (pool_temperature1_F + pool_temperature2_F) / 2.0;
  }

  float outside_temperature_C = (outside_temperature_millivolts - 500.0) / 10.0;

  // Convert to degrees F
  float outside_temperature_this_sample_F = outside_temperature_C * 9.0 / 5.0 + 32.0;
  if (outside_temperature_F == SENSOR_ERROR) {
    outside_temperature_F = outside_temperature_this_sample_F;
  }
  outside_temperature_F = alpha * outside_temperature_this_sample_F + (1.0 - alpha) * outside_temperature_F;


  if (outside_temperature_F < 0.0 || outside_temperature_F > 150.0) {
    outside_temperature_F = SENSOR_ERROR;
  }
  // The sensor reads about 0.29 V with no pressure, so near zero volts means it is dead or disconnected.
  // Overpressure can't be seen then, and the boost pump won't start.  (It failed silently from
  // 2026-11-02 until someone noticed the missing boost runs.)
  {
    static unsigned long last_sensor_ok = millis();
    static bool sensor_alerted = false;
    if (pressure_millivolts >= PRESSURE_SENSOR_MIN_MV) {
      last_sensor_ok = millis();
      if (sensor_alerted) {
        out.println(F("# alert pressure sensor reading again"));
        sensor_alerted = false;
      }
    } else if (!sensor_alerted && (millis() - last_sensor_ok) > 60 * 1000UL) {
      out.print(F("# alert pressure sensor reads "));
      out.print((int)pressure_millivolts);
      out.println(F(" mV, 290 expected at rest: boost pump can't run, overpressure can't be seen"));
      sensor_alerted = true;
    }
  }
  float pressure_volts = pressure_millivolts / 1000.0;
  pressure_volts -= 0.29;   // Pressure sensor reads 0.29 volts when pressure is zero
  if (pressure_volts < 0.0) {
    pressure_volts = 0.0;
  }
  pressure_psi = pressure_volts * 3.2459; // Empircally derived calibration value
  check_free_memory(F("read_time_and.. exit"));
}

// Called periodically.  Emits a telemetry line when something notable changes, and every 10 minutes
// otherwise; bedichek.org appends these to poolpumpcontrol.txt for the graphs.

void emit_telemetry_callback(void)
{
  check_free_memory(F("print_status_to.."));
  static int last_force_telemetry;
  static float last_pool_temperature1_F;
  static float last_pool_temperature2_F;
  static float last_pressure_psi;
  static bool last_pump;
  static bool last_diverter_valve_request;
  static bool last_to_roof;
  static unsigned line_counter = 0;
  static unsigned skipped_record_counter = 0;

  if (!time_valid) {
    return;   // Lines without a real date would land in the wrong place on the graphs
  }

  bool pump = main_pump_is_on();
  bool boost = boost_pump_is_on();
  bool to_roof = diverter_valve_is_sending_water_to_roof();
  unsigned records_to_skip = (operating_mode == m_normal) ? 600 : 60;

  if (force_telemetry != last_force_telemetry ||
      abs(pool_temperature1_F - last_pool_temperature1_F) > 1.5 ||
      abs(pool_temperature2_F - last_pool_temperature2_F) > 1.5 ||
      abs(pressure_psi - last_pressure_psi) > 0.5 ||
      pump != last_pump ||
      diverter_valve_request() != last_diverter_valve_request ||
      to_roof != last_to_roof ||
      skipped_record_counter++ > records_to_skip) {
   last_force_telemetry = force_telemetry;
   last_pool_temperature1_F = pool_temperature1_F;
   last_pool_temperature2_F = pool_temperature2_F;
   last_pressure_psi = pressure_psi;
   last_pump = pump;
   last_diverter_valve_request = diverter_valve_request();
   last_to_roof = to_roof;

   skipped_record_counter = 0;

    if (line_counter == 0) {
        out.println(F("# Date     Time      Pl1  Pl2  Out   PSI   Pmp Bst Req Roof Mode Runtime Boot"));
        line_counter = 20;
      } else {
        line_counter--;
      }
    snprintf(cbuf, sizeof(cbuf), "%4u-%02u-%02u %02u:%02u:%02u ",
               year(arduino_time),
               month(arduino_time),
               day(arduino_time),
               hour(arduino_time),
               minute(arduino_time),
               second(arduino_time));
    out.print(cbuf);

    // Format temperatures, showing "  -" for failed sensors
    char pressure_psi_str[8];
    dtostrf(pressure_psi, 4, 1, pressure_psi_str);

    char pt1_str[8];
    format_temperature(pool_temperature1_F, pt1_str, sizeof(pt1_str));

    char pt2_str[8];
    format_temperature(pool_temperature2_F, pt2_str, sizeof(pt2_str));

    char ot_str[8];
    format_temperature(outside_temperature_F, ot_str, sizeof(ot_str));

    snprintf(cbuf, sizeof(cbuf), " %s %s %s %s",
             pt1_str, pt2_str, ot_str, pressure_psi_str);
    out.print(cbuf);

    snprintf(cbuf, sizeof(cbuf), " %3u %3u %3u %3u  %3u   %5lu %c",
             pump,
             boost,
             diverter_valve_request(),
             to_roof,
             (unsigned)operating_mode,
             main_pump_daily_run_time / 1000UL,
             reset_cause);
    out.println(cbuf);
  }
  check_free_memory(F("print_status_to.. exit"));
}

/*****************************************************************************************************/
// Commands, from the USB serial console or from poolctl (POST /cmd).  One per line:
//   mode normal|pump|boost|24vac|diverter    enter a mode; diag modes revert to Normal after 10 minutes
//   + / -                                    what the plus and minus keys did in the diag modes
//   button                                   what the red button did: toggle the main pump
//   telemetry                                emit a telemetry line now
//   reboot                                   restart the ESP32 (the relays stay as they are)
//   upload on|off                            send telemetry to bedichek.org or not (saved in flash)

void run_command(const char *command)
{
  out.print(F("# command: "));
  out.println(command);
  force_telemetry++;

  if (strncmp(command, "mode ", 5) == 0) {
    const char *names[] = {"normal", "pump", "boost", "24vac", "diverter"};
    for (int m = 0; m < m_last; m++) {
      if (strcmp(command + 5, names[m]) == 0) {
        set_operating_mode((operating_mode_t)m);
        return;
      }
    }
    out.println(F("# modes are normal, pump, boost, 24vac, diverter"));
  } else if (strcmp(command, "+") == 0 || strcmp(command, "on") == 0) {
    plus_key();
    if (operating_mode != m_normal) {
      time_entering_diag_mode = millis();
    }
  } else if (strcmp(command, "-") == 0 || strcmp(command, "off") == 0) {
    minus_key();
    if (operating_mode != m_normal) {
      time_entering_diag_mode = millis();
    }
  } else if (strcmp(command, "button") == 0) {
    red_button();
  } else if (strcmp(command, "telemetry") == 0) {
    // force_telemetry was incremented above
  } else if (strcmp(command, "upload on") == 0 || strcmp(command, "upload off") == 0) {
    upload_enabled = strcmp(command, "upload on") == 0;
    Preferences prefs;
    prefs.begin("pool", false);
    prefs.putBool("upload", upload_enabled);
    prefs.end();
    out.println(upload_enabled ? F("# uploads to bedichek.org on") : F("# uploads to bedichek.org off"));
  } else if (strcmp(command, "reboot") == 0) {
    out.println(F("# rebooting"));
    delay(500);
    ESP.restart();
  } else {
    out.print(F("# Unknown command: "));
    out.println(command);
    out.println(F("# choices are: 'mode normal|pump|boost|24vac|diverter', +, -, button, telemetry, 'upload on|off', reboot"));
  }
}

// Called frequently to poll for and act on received console input.

void monitor_serial_console_callback(void)
{
  static char command_buf[40];

  while (Serial.available() > 0) {  // Check if data is available to read
    char received_char = Serial.read();  // Read one character
    size_t l = strlen(command_buf);
    if (received_char == '\r') {
      continue;
    }
    if (l >= sizeof(command_buf) - 1) {
      out.println(F("# command buffer overflow"));
      command_buf[0] = '\0';
      continue;
    }
    if (received_char == '\n') {
      run_command(command_buf);
      command_buf[0] = '\0';
    } else {
      command_buf[l] = received_char;
      command_buf[l+1] = '\0';
    }
  }
}

/*****************************************************************************************************/
// Clock.  The RTC holds UTC.  At boot the system clock is set from it; NTP then keeps the system clock
// and once an hour after each NTP sync we write the RTC.

static uint8_t bcd2bin(uint8_t v) { return (v & 0x0F) + 10 * (v >> 4); }
static uint8_t bin2bcd(uint8_t v) { return ((v / 10) << 4) | (v % 10); }

bool rtc_read(time_t *t)
{
  Wire.beginTransmission(PCF85063_ADDR);
  Wire.write(0x04);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(PCF85063_ADDR, 7) != 7) {
    return false;
  }
  uint8_t r[7];
  for (int i = 0; i < 7; i++) {
    r[i] = Wire.read();
  }
  if (r[0] & 0x80) {          // Oscillator stopped: the time is not valid
    return false;
  }
  struct tm tm = {};
  tm.tm_sec = bcd2bin(r[0] & 0x7F);
  tm.tm_min = bcd2bin(r[1] & 0x7F);
  tm.tm_hour = bcd2bin(r[2] & 0x3F);
  tm.tm_mday = bcd2bin(r[3] & 0x3F);
  tm.tm_mon = bcd2bin(r[5] & 0x1F) - 1;
  tm.tm_year = bcd2bin(r[6]) + 100;
  if (tm.tm_year < 125 || tm.tm_year > 140) {   // Before 2025: never set; after 2040: garbage (the demo firmware left 2056)
    return false;
  }
  // mktime() works in local time; the RTC holds UTC, so convert with TZ set to UTC
  setenv("TZ", "UTC0", 1);
  tzset();
  *t = mktime(&tm);
  setenv("TZ", TIMEZONE, 1);
  tzset();
  return true;
}

bool rtc_write(time_t t)
{
  struct tm tm;
  gmtime_r(&t, &tm);
  Wire.beginTransmission(PCF85063_ADDR);
  Wire.write(0x04);
  Wire.write(bin2bcd(tm.tm_sec));       // Also clears the oscillator-stopped flag
  Wire.write(bin2bcd(tm.tm_min));
  Wire.write(bin2bcd(tm.tm_hour));
  Wire.write(bin2bcd(tm.tm_mday));
  Wire.write(tm.tm_wday);
  Wire.write(bin2bcd(tm.tm_mon + 1));
  Wire.write(bin2bcd(tm.tm_year - 100));
  return Wire.endTransmission() == 0;
}

volatile bool ntp_synced = false;

void time_sync_notification(struct timeval *tv)
{
  ntp_synced = true;
}

void sync_rtc_callback(void)
{
  static unsigned long last_rtc_write;
  if (ntp_synced) {
    ntp_synced = false;
    if (!time_valid) {
      time_valid = true;
      out.println(F("# clock set from NTP"));
    }
    if (last_rtc_write == 0 || millis() - last_rtc_write > 3600 * 1000UL) {
      if (rtc_write(time(nullptr))) {
        last_rtc_write = millis();
      } else {
        out.println(F("# alert could not write the RTC"));
      }
    }
  }
}

void setup_time(void)
{
  setenv("TZ", TIMEZONE, 1);
  tzset();
  time_t t;
  if (rtc_read(&t)) {
    struct timeval tv = {t, 0};
    settimeofday(&tv, nullptr);
    time_valid = true;
  } else {
    out.println(F("# alert RTC has no valid time; waiting for NTP"));
  }
  sntp_set_time_sync_notification_cb(time_sync_notification);
  configTzTime(TIMEZONE, "pool.ntp.org", "time.apple.com");
}

/*****************************************************************************************************/
// Network: Ethernet (W5500, powered over PoE), static address 192.168.1.5 (poolpumpcontrol in
// /etc/hosts; solartracker and solarthermal are .8 and .9), mDNS name poolpumpcontrol.local, a small
// HTTP server for poolctl, and an upload task that sends telemetry lines to bedichek.org.

#define HOSTNAME "poolpumpcontrol"
#define STATIC_IP IPAddress(192, 168, 1, 5)
#define GATEWAY_IP IPAddress(192, 168, 1, 1)      // The AT&T gateway, also the DNS server
#define SUBNET_MASK IPAddress(255, 255, 255, 0)
#define DNS2_IP IPAddress(1, 1, 1, 1)
#define INGEST_URL "https://bedichek.org/home/api/ingest/poolpumpcontrol"

volatile bool eth_connected = false;
bool status_served = false;              // Someone fetched /status since boot: the network works

void network_event(arduino_event_id_t event, arduino_event_info_t info)
{
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      ETH.setHostname(HOSTNAME);
      break;
    case ARDUINO_EVENT_ETH_CONNECTED:   // The address is static, so link up means ready
    case ARDUINO_EVENT_ETH_GOT_IP:
      eth_connected = true;
      break;
    case ARDUINO_EVENT_ETH_LOST_IP:
    case ARDUINO_EVENT_ETH_DISCONNECTED:
    case ARDUINO_EVENT_ETH_STOP:
      eth_connected = false;
      break;
    default:
      break;
  }
}

WebServer server(80);

static bool authorized(void)
{
  if (server.header("X-Token") == POOL_LAN_TOKEN) {
    return true;
  }
  server.send(403, "text/plain", "forbidden\n");
  return false;
}

void handle_status(void)
{
  status_served = true;
  char pt1[8], pt2[8], ot[8];
  format_temperature(pool_temperature1_F, pt1, sizeof(pt1));
  format_temperature(pool_temperature2_F, pt2, sizeof(pt2));
  format_temperature(outside_temperature_F, ot, sizeof(ot));
  time_t t = now();
  struct tm tm = tm_of(t);
  char when[32];
  strftime(when, sizeof(when), "%Y-%m-%d %H:%M:%S %Z", &tm);
  unsigned long pump_s = main_pump_is_on() ? (millis() - main_pump_on_off_time) / 1000 : 0;
  unsigned long roof_s = diverter_valve_is_sending_water_to_roof() ? (millis() - diverter_valve_in_roof_position_time) / 1000 : 0;
  const esp_partition_t *running = esp_ota_get_running_partition();
  esp_ota_img_states_t state = ESP_OTA_IMG_UNDEFINED;
  esp_ota_get_state_partition(running, &state);

  String s;
  s.reserve(1200);
  s += "{\n";
  s += "  \"board\": \"8RO\",\n";
  s += "  \"time\": \"" + String(when) + "\",\n";
  s += "  \"time_valid\": " + String(time_valid ? "true" : "false") + ",\n";
  s += "  \"mode\": \"" + String(operating_mode_to_string(operating_mode)) + "\",\n";
  s += "  \"pool1_F\": \"" + String(pt1) + "\", \"pool2_F\": \"" + String(pt2) + "\", \"outside_F\": \"" + String(ot) + "\",\n";
  s += "  \"pressure_psi\": " + String(pressure_psi, 1) + ",\n";
  s += "  \"main_pump\": " + String(main_pump_is_on()) + ", \"main_pump_on_s\": " + String(pump_s) +
       ", \"pump_today_s\": " + String(main_pump_daily_run_time / 1000) + ",\n";
  s += "  \"boost_pump\": " + String(boost_pump_is_on()) + ",\n";
  s += "  \"valve\": \"" + String(diverter_valve_is_sending_water_to_roof() ? "roof" : "pool") + "\", \"valve_roof_s\": " + String(roof_s) +
       ", \"valve_power\": " + String(diverter_valve_transformer_is_on()) + ",\n";
  s += "  \"timer_switch\": " + String(timer_switch_on) + ", \"diverter_request\": " + String(diverter_valve_request()) + ",\n";
  s += "  \"manual_request\": " + String(manual_main_pump_request) + ", \"periodic_filter\": " + String(periodic_filter_request) +
       ", \"periodic_boost\": " + String(periodic_boost_request) + ",\n";
  s += "  \"relays\": " + String(quad_lv_relay != nullptr ? tca_read(1) : -1) + ", \"relay_direction_reg\": " + String(quad_lv_relay != nullptr ? tca_read(3) : -1) + ",\n";
  s += "  \"relay_board\": " + String(quad_lv_relay != nullptr ? "true" : "false") +
       ", \"analog_module\": " + String(analog_failures < 20 ? "true" : "false") + ",\n";
  s += "  \"analog_mv\": [" + String(analog_last_mv[0]) + ", " + String(analog_last_mv[1]) + ", " +
       String(analog_last_mv[2]) + ", " + String(analog_last_mv[3]) + "],\n";
  s += "  \"upload\": " + String(upload_enabled ? "true" : "false") + ", \"upload_queue_bytes\": " + String(upload_queue.length()) + ",\n";
  s += "  \"uptime_s\": " + String(millis() / 1000) + ", \"reset_cause\": \"" + String(reset_cause) + "\",\n";
  s += "  \"firmware\": \"" FIRMWARE_VERSION "\", \"partition\": \"" + String(running->label) +
       "\", \"firmware_verified\": " + String(state == ESP_OTA_IMG_PENDING_VERIFY ? "false" : "true") + ",\n";
  s += "  \"free_heap\": " + String(ESP.getFreeHeap()) + ", \"lowest_heap\": " + String(lowest_memory) + "\n";
  s += "}\n";
  server.send(200, "application/json", s);
}

void handle_log(void)
{
  size_t max = server.hasArg("bytes") ? server.arg("bytes").toInt() : 8000;
  String s;
  xSemaphoreTake(lines_mutex, portMAX_DELAY);
  recent_lines.copy_tail(s, max);
  xSemaphoreGive(lines_mutex);
  server.send(200, "text/plain", s);
}

void handle_cmd(void)
{
  if (!authorized()) {
    return;
  }
  String c = server.arg("plain");
  c.trim();
  size_t before = 0;
  xSemaphoreTake(lines_mutex, portMAX_DELAY);
  before = recent_lines.length();
  xSemaphoreGive(lines_mutex);
  run_command(c.c_str());
  // Reply with what the command printed
  String s;
  xSemaphoreTake(lines_mutex, portMAX_DELAY);
  recent_lines.copy_tail(s, recent_lines.length() > before ? recent_lines.length() - before : 0);
  xSemaphoreGive(lines_mutex);
  server.send(200, "text/plain", s);
}

// Firmware update: poolctl posts the .bin as multipart form data.  The control tasks don't run while
// it is written (about 10 seconds); the relays hold their state, and the ESP32 restarts into the new
// image, which has a minute to show it is healthy before the bootloader would go back to this one.

bool update_authorized;

void handle_update_done(void)
{
  if (!update_authorized) {
    server.send(403, "text/plain", "forbidden\n");
    return;
  }
  if (Update.hasError()) {
    server.send(500, "text/plain", String("update failed: ") + Update.errorString() + "\n");
    out.print(F("# alert firmware update failed: "));
    out.println(Update.errorString());
    return;
  }
  server.send(200, "text/plain", "update written, restarting\n");
  out.println(F("# firmware update written, restarting"));
  delay(500);
  ESP.restart();
}

void handle_update_upload(void)
{
  HTTPUpload &upload = server.upload();
  esp_task_wdt_reset();
  if (upload.status == UPLOAD_FILE_START) {
    update_authorized = server.header("X-Token") == POOL_LAN_TOKEN;
    if (update_authorized) {
      out.print(F("# firmware update starting: "));
      out.println(upload.filename);
      Update.begin(UPDATE_SIZE_UNKNOWN);
    }
  } else if (!update_authorized) {
    return;
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    Update.write(upload.buf, upload.currentSize);
  } else if (upload.status == UPLOAD_FILE_END) {
    Update.end(true);
  } else if (upload.status == UPLOAD_FILE_ABORTED) {
    Update.abort();
  }
}

void setup_network(void)
{
  Network.onEvent(network_event);
  SPI.begin(ETH_SCLK_PIN, ETH_MISO_PIN, ETH_MOSI_PIN);
  ETH.begin(ETH_PHY_W5500, 1, ETH_CS_PIN, ETH_INT_PIN, -1, SPI);
  ETH.config(STATIC_IP, GATEWAY_IP, SUBNET_MASK, GATEWAY_IP, DNS2_IP);

  const char *headers[] = {"X-Token"};
  server.collectHeaders(headers, 1);
  server.on("/status", HTTP_GET, handle_status);
  server.on("/", HTTP_GET, handle_status);
  server.on("/log", HTTP_GET, handle_log);
  server.on("/cmd", HTTP_POST, handle_cmd);
  server.on("/update", HTTP_POST, handle_update_done, handle_update_upload);
  server.begin();
  MDNS.begin(HOSTNAME);
  MDNS.addService("http", "tcp", 80);
}

// Runs on its own FreeRTOS task so that a slow upload never delays the pump logic.  Sends up to
// 32 KB of queued lines at a time; lines are dropped from the queue only when bedichek.org accepts them.

void upload_task(void *arg)
{
  static char chunk[32768];
  unsigned long backoff_ms = 10000;
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(backoff_ms));
    if (!eth_connected) {
      continue;
    }
    xSemaphoreTake(lines_mutex, portMAX_DELAY);
    size_t n = upload_queue.peek(chunk, sizeof(chunk));
    xSemaphoreGive(lines_mutex);
    if (n == 0) {
      continue;
    }
    NetworkClientSecure client;
    client.useBuiltinCACertBundle();
    HTTPClient http;
    http.setTimeout(15000);
    int code = -1;
    if (http.begin(client, INGEST_URL)) {
      http.addHeader("Authorization", "Bearer " POOL_INGEST_TOKEN);
      http.addHeader("Content-Type", "text/plain");
      code = http.POST((uint8_t *)chunk, n);
      http.end();
    }
    if (code == 200) {
      xSemaphoreTake(lines_mutex, portMAX_DELAY);
      upload_queue.consume(n);
      xSemaphoreGive(lines_mutex);
      backoff_ms = 10000;
    } else {
      Serial.printf("# upload to bedichek.org failed: %d\n", code);
      backoff_ms = backoff_ms < 300000 ? backoff_ms * 2 : 300000;
    }
  }
}

/*****************************************************************************************************/
// Firmware rollback.  Returning true here tells the core not to mark a new image good at boot; we do
// it after a minute of running with the relay board answering and /status fetched over the network
// (poolctl update polls it), so an image that can't be reached goes back to the old one by itself.

extern "C" bool verifyRollbackLater()   // C linkage, or the core keeps its own version, which marks every image good
{
  return true;
}

void mark_firmware_good_callback(void)
{
  const esp_partition_t *running = esp_ota_get_running_partition();
  esp_ota_img_states_t state;
  if (esp_ota_get_state_partition(running, &state) == ESP_OK && state == ESP_OTA_IMG_PENDING_VERIFY) {
    if (quad_lv_relay != nullptr && status_served) {
      esp_ota_mark_app_valid_cancel_rollback();
      out.println(F("# new firmware marked good"));
    } else {
      out.println(quad_lv_relay == nullptr ? F("# alert new firmware cannot reach the relay board; going back to the previous firmware")
                                           : F("# alert new firmware was not reached over the network; going back to the previous firmware"));
      delay(500);
      esp_ota_mark_app_invalid_rollback_and_reboot();
    }
  }
}

/*****************************************************************************************************/

void setup_reset_cause(void)
{
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  reset_cause = 'P'; break;
    case ESP_RST_BROWNOUT: reset_cause = 'B'; break;
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:      reset_cause = 'W'; break;
    case ESP_RST_SW:       reset_cause = 'S'; break;
    case ESP_RST_PANIC:    reset_cause = 'C'; break;
    default:               reset_cause = 'X'; break;
  }
  if (reset_cause == 'W' || reset_cause == 'C' || reset_cause == 'B') {
    Preferences prefs;
    prefs.begin("pool", false);
    uint32_t count = prefs.getUInt("bad_resets", 0) + 1;
    prefs.putUInt("bad_resets", count);
    prefs.end();
    out.print(F("# alert Reset cause: "));
    out.print(reset_cause);
    out.print(F(", total count="));
    out.println(count);
  }
}

/*
   This is the function that the Arudino run time system calls once, just after start up.
*/
void setup(void)
{
  Serial.begin(SERIAL_BAUD);
  lines_mutex = xSemaphoreCreateMutex();
  recent_lines.init(64 * 1024, true);
  upload_queue.init(1024 * 1024, true);

  Preferences prefs;
  prefs.begin("pool", true);
  upload_enabled = prefs.getBool("upload", false);
  prefs.end();

  setup_reset_cause();
  setup_pins();

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 100000);
  Wire.setTimeOut(25);

  RS485.begin(ANALOG_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  RS485.setPins(-1, -1, -1, RS485_DE_PIN);
  RS485.setMode(UART_MODE_RS485_HALF_DUPLEX);

  setup_time();

  out.print(F("# alert Pool pump and valve controller (ESP32). " FIRMWARE_VERSION " reset cause "));
  out.println(reset_cause);

  check_relays_callback();     // This sets "quad_lv_relay"

  // If we start with the diverter valve direction relay in the position for sending water to roof, but there is no request for this
  // then ensure that the diverter valve is sending water back to the pool.

  if (diverter_valve_is_sending_water_to_roof() &&  !diverter_valve_request() && !main_pump_is_on()) {
    out.println(F("# Turning diverter valve to pool on startup"));
    turn_diverter_valve_transformer_on();
    set_diverter_valve_to_return_water_to_pool();
  }

  setup_network();
  xTaskCreatePinnedToCore(upload_task, "upload", 8192, nullptr, 1, nullptr, 0);

  mark_firmware_good.enableDelayed(TASK_SECOND * 60);

  // 8 second watchdog on the loop task, as on the Arduino
  esp_task_wdt_config_t wdt_config = {.timeout_ms = 8000, .idle_core_mask = 0, .trigger_panic = true};
  esp_task_wdt_reconfigure(&wdt_config);
  esp_task_wdt_add(nullptr);
}

/*
   This is the function that the Arduino run time system calls repeatedly after it has called setup().
*/
void loop(void)
{
  esp_task_wdt_reset();  // Pet the watchdog
  // All the control code executes inside of tasks; the HTTP server is polled here.
  ts.execute();
  server.handleClient();
}
