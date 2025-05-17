
/*
   This controls two 3-way valves on my roof.
      
   Creative Commons Licence
   2025 Robert Bedichek
*/

#include <string.h> //Use the string Library
#include <ctype.h>
#include <TimeLib.h>  // for update/display of time

//   All the operational code uses this time structure.  This is initialized at start time from the battery-backed up DS1307 RTC.
time_t arduino_time;


#include <EEPROM.h>
const bool verbose_rtc = true; // Adds 70 bytes to RAM demand if true
const bool verbose_I2C = true;
/*
 * We have two Sparkfun relay boards and possibly other 1-wire devices
 */
#include <Wire.h>

#include <PinChangeInterrupt.h>

const int button_pins[6] = {7, 8, 9, 10, 11, 12};

#define BUTTON_RED    (0)
#define BUTTON_YELLOW (1)
#define BUTTON_MINUS  (2)
#define BUTTON_PLUS   (3)
#define BUTTON_ENTER  (4)
#define BUTTON_SELECT (5)


/*
 * Low voltage relay bank.  Relays 1 and 2 control the power to inlet valve motors.
 * Relay 3 and 4 control the power to the outlet valve.
 * The NC pins of all four relays are connected to ground.
 * The NO pins of all four relays are connected to +12V.
 * To open the inlet valve, Relay 1 is energized.  To close it, Relay 2 is energized.
 * To open the outlet valve, Relay 3 is energized and to close it Relay 4 is energized.
 */
#include "SparkFun_Qwiic_Relay.h"
#define LV_RELAY_I2C_ADDR  (0x6D)                 // Default I2C address of the mechanical, low voltage, 4-relay board

#define LV_RELAY_MAIN_PUMP_12V (1)
#define LV_RELAY_BOOST_PUMP_12V (2)
#define LV_RELAY_DIVERTER_TRANSFORMER_12V (3)
#define LV_RELAY_DIVERTER_DIRECTION (4)

#define SSR_RELAY_I2C_ADDR (0x08)                 // Default I2C for quad SSR relay

Qwiic_Relay *quad_lv_relay;

#include <SerLCD.h>
SerLCD *lcd;
#define SERLCD_I2C_ADDR (0x72)
const bool fast_lcd_comm = false;
/*
   Counts down when the backlight is on.  When it is zero, we turn the backlight off.  Units are seconds.
*/
unsigned long backlight_timer_in_seconds;            

#define BACKLIGHT_ON_TIME_IN_SECONDS (600 /* 10 minutes */) 

#define POOL_TEMPERATURE_INPUT (A0)     // 0-5VDC LM335 or LM36 that is immersed in pool water, about 1' deep under diving board
#define OUTSIDE_TEMPERATURE_INPUT (A1)  // An LM36 sensing ambient temperature near the pool equipment
#define PRESSURE_INPUT (A2)             // 0-5VDC pressure sensor on top of filter canister


// The four maintenance keys (select, enter, plus, minus -- from top to bottom) are connected to pins
// 12 through 9.  When pressed, they ground their Arduino inputs, when released, the Arduino inputs
// are pulled up an an internal resistor.

#define KEY_1_PIN (12)  // Top most input key
#define KEY_2_PIN (11)
#define KEY_3_PIN (10)
#define KEY_4_PIN (9)  // Bottom most input key

#define YELLOW_BUTTON_INPUT_PIN (8)     // Simple momemtary contact push button
#define RED_BUTTON_INPUT_PIN (7)        // Simple momemtary contact push button
#define DIVERTER_REQUEST_INPUT_PIN (5)  // Fed by an optoisolator

#define _TASK_SLEEP_ON_IDLE_RUN
#include <TaskScheduler.h>

//    Central (and only) task scheduler data structure.
Scheduler ts;

// Speed at which we run the serial connection (both via USB and RS-485)
#define SERIAL_BAUD (115200)

typedef enum {m_oper, m_safe, m_main_pump, m_sweep_pump, m_diverter, m_last} operating_mode_t;

bool toggle_key_pressed = false;

void read_time_and_sensor_inputs_callback(void);
void monitor_lcd_backlight_callback(void);
void print_status_to_serial_callback(void);
void poll_keys_callback(void);
void update_lcd_callback(void);

void main_pump_control_callback(void);
void sweep_pump_control_callback(void);
void diverter_valve_control_callback(void);
void monitor_diag_mode_callback(void);
void monitor_serial_console_callback(void);
void process_pressed_keys_callback(void);

// This string variable is used by multiple functions below, but not at the same time
char cbuf[60];

/*****************************************************************************************************/
// These global variables describe the state of the heat exchanger valve.  It can be open,
// closed, in the process of opening or closing, or in an indeterminate state after we attempted to
// open or close it.

unsigned long valve_motion_start_time = 0;

// These two booleans represent the latest values sensed from the valve itself.  
unsigned long main_pump_on_time_in_seconds;
unsigned long sweep_pump_on_time_in_seconds;
const unsigned long max_pump_on_time_in_seconds = 5UL * 3600UL;
const unsigned long max_sweep_pump_on_time_in_seconds = 3600UL;

unsigned long diverter_valve_transformer_on_time_in_seconds;
unsigned long diverter_valve_in_roof_position_time_in_seconds;

Task main_pump_control(TASK_SECOND, TASK_FOREVER, &main_pump_control_callback, &ts, true);
Task sweep_pump_control(TASK_SECOND * 4, TASK_FOREVER, &sweep_pump_control_callback, &ts, true);
operating_mode_t operating_mode;

unsigned long time_entering_diag_mode_in_seconds;
const unsigned long max_diag_mode_time_in_seconds = 600; // 10 minutes
Task monitor_diag_mode(TASK_SECOND * 60, TASK_FOREVER, &monitor_diag_mode_callback, &ts, false);
/*****************************************************************************************************/

Task monitor_serial_console(TASK_SECOND, TASK_FOREVER, &monitor_serial_console_callback, &ts, true);

Task diverter_valve_control(TASK_SECOND * 6, TASK_FOREVER, &diverter_valve_control_callback, &ts, true);

float pool_temperature_F;
float outside_temperature_F;
float pool_pressure_volts;

Task read_time_and_sensor_inputs(500, TASK_FOREVER, &read_time_and_sensor_inputs_callback, &ts, true);
/*****************************************************************************************************/
// We have two options for how to recognize pressing of one of the four keys.  One is to rapidly poll
// the state of the digital input pins to which the keys are connected.  The other is to enable any-input-change
// interrupts.  We have to use the polling method if we want to use certain libraries, e.g., SoftSerial.  Otherwise,
// using interrupts is preferable.  Since we do not need SoftSerial, we use interrupts.  But it is handy to have
// the option to switch to polling for some debugging purposes.

volatile bool select_key_pressed = false;
volatile bool enter_key_pressed = false;
volatile bool plus_key_pressed = false;
volatile bool minus_key_pressed = false;

// We have two ways of recognizing pressed keys, the interrupt method and the polling method.
// Our prefered and default method is via interrupts.  If we get a future peripheral with conflicting
// resource needs (e.g., SoftSerial) or we want to try polling to isolate certain bugs, we may want
// to enabling polling, so the option remains in the code.

// #define POLL_KEYS
#ifndef POLL_KEYS
//ISR(PCINT2_vect) 
//{
//  poll_keys_callback();
//}
const bool poll_keys_bool = false;
Task process_pressed_keys(100, TASK_FOREVER, &process_pressed_keys_callback, &ts, true);
#else
const bool poll_keys_bool = true;
Task poll_keys(25, TASK_FOREVER, &poll_keys_callback, &ts, true);
#endif

volatile unsigned long lastInterruptTime = 0;
const int debounce_delay = 150;  // 150ms debounce time

Task print_status_to_serial(TASK_SECOND, TASK_FOREVER, &print_status_to_serial_callback, &ts, true);
/*****************************************************************************************************/
// Update slightly faster than once per second so that the LCD time advances regularly in a way that 
// looks normal to humans (i.e., make sure that the seconds display counts up once per second).

unsigned fail_message_time;

// We depend on this being called about once per second and being enabled at boot time.
Task monitor_lcd_backlight(TASK_SECOND, TASK_FOREVER, &monitor_lcd_backlight_callback, &ts, true);
Task update_lcd(900, TASK_FOREVER, &update_lcd_callback, &ts, true);
/*****************************************************************************************************/


// This is a diagnostic function that returns the amount of free RAM, which is the RAM between the top
// of the heap and the bottom of the stack.

extern unsigned int __heap_start;
extern void *__brkval;

int free_memory() {
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}

// Flag a warning if there are fewer than this number of bytes of free memory.
#define LOW_MEMORY_LIMIT (200)

// This will be overwritten on the first call to "check_free_memory(F("..."));".  It
// records the smallest number of bytes of free memory
int lowest_memory = 2000; 

// This is called by each task when that task starts executing and when the task
// is about to end execution (i.e., entry and exits of tasks).  This emits an 
// alert if the amount of free memory is too low.  It also optionally
// emits a trace message letting us see the entry ane exits of tasks.

void check_free_memory(const __FlashStringHelper *caller)
{
  static int trace_initial_calls = 0;
  int fm = free_memory();
  if (fm < LOW_MEMORY_LIMIT) {
    Serial.print(F("# alert low memory: "));
    Serial.println(fm);
  }
  if (fm < lowest_memory) {
    lowest_memory = fm;
  }
  if (trace_initial_calls > 0) {
    Serial.print(F("# alert "));
    Serial.print(caller);
    Serial.print(F(" free memory="));
    Serial.println(fm);
    trace_initial_calls--;
  }
}

// Returns a string that desribes the current diag mode.  For some reason, it stopped working to pass
// in the diag mode, but since we always passed in the global variable "diag_mode", just commenting out
// the parameter here, and the passed arguments where this is called allowed this to compile again.

const char *operating_mode_to_string(operating_mode_t operating_mode) 
{
  // m_main_pump, m_sweep_pump, m_diverter
  char *s[] = {"Oper", "Safe", "D-MN", "D-SP", "D-DV"};
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
  unsigned long now_in_seconds = millis() / 1000UL;
  
  if ((now_in_seconds - time_entering_diag_mode_in_seconds) > max_diag_mode_time_in_seconds) {
    operating_mode = m_oper;
    monitor_diag_mode.disable();
  }
}

/*
   Called every second the backlight is on and turns off the backlight, and disables itself, when the backlight timer
   has counted down to zero.
*/
void monitor_lcd_backlight_callback(void) 
{
  if (backlight_timer_in_seconds > 0) {
    backlight_timer_in_seconds--;
    if (backlight_timer_in_seconds == 0) {
      lcd->clear(); // Changing the backlight scrambles the display, so clear it and let it be refreshed in update_lcd
      lcd->setBacklight(0, 0, 0);
      monitor_lcd_backlight.disable();
      
    }
  }
}

// This is only called if we have compiled with the option to poll keys (and not use interrupts
// to detect key status changes).
void poll_keys_callback(void)
{
  unsigned long currentTime = millis();
  if ((currentTime - lastInterruptTime) > debounce_delay) {  // Debounce check
  
    bool key_select = digitalRead(button_pins[BUTTON_SELECT]) == LOW;
    bool key_enter = digitalRead(button_pins[BUTTON_ENTER]) == LOW;
    bool key_plus = digitalRead(button_pins[BUTTON_PLUS]) == LOW;
    bool key_minus = digitalRead(button_pins[BUTTON_MINUS]) == LOW;
    bool key_yellow = digitalRead(button_pins[BUTTON_YELLOW]) == LOW;
    bool key_red = digitalRead(button_pins[BUTTON_RED]) == LOW;

    if (key_select) {
      select_key_pressed = true;
    }
    if (key_enter) {
      enter_key_pressed = true;
    }
    if (key_plus) {
      plus_key_pressed = true;
    }
    if (key_minus) {
      minus_key_pressed = true;
    }
    
    lastInterruptTime = currentTime;  // Update debounce timer
    if (poll_keys_bool) {
      process_pressed_keys_callback();
    }        
  } 
}

// The interrupt routine above will set global variables indicating which keys have been pressed.
// In this function, we look at those global variables and take action required by the key presses
// and then reset those global variables.

void process_pressed_keys_callback(void)
{
  check_free_memory(F("process_keys.."));
  bool some_key_pressed = select_key_pressed | enter_key_pressed | plus_key_pressed | minus_key_pressed;

  if (select_key_pressed) {
    operating_mode = (operating_mode + 1) % m_last;
    Serial.print(F("# mode now "));
    Serial.println(operating_mode_to_string(operating_mode));
    select_key_pressed = false;
  }
 
  if (plus_key_pressed) {
    switch (operating_mode) {
      case m_oper:
      case m_safe:
        adjustTime(600);
        break;

      case m_main_pump:
        turn_main_pump_on();
        Serial.print(F("# main pump="));
        Serial.println(main_pump_is_on());
        break;

      case m_sweep_pump:
        turn_sweep_pump_on();
        Serial.print(F("# sweep pump="));
        Serial.println(sweep_pump_is_on());
        break;

      case m_diverter:
        set_diverter_valve_to_send_water_to_roof();
        Serial.print(F("# diverter valve="));
        if (diverter_valve_is_sending_water_to_roof()) {
          Serial.println("roof");
        } else {
          Serial.println("pool");
        }
        break; 
    }
    plus_key_pressed = false;
  }

  if (minus_key_pressed) {
    switch (operating_mode) {
      case m_oper:
        adjustTime(-600);
        break;

      case m_main_pump:
        turn_main_pump_off();
        Serial.print(F("# main pump="));
        Serial.println(main_pump_is_on());
        break;

      case m_sweep_pump:
        turn_sweep_pump_off();
        Serial.print(F("# sweep pump="));
        Serial.println(sweep_pump_is_on());
        break;

      case m_diverter:
        set_diverter_valve_to_return_water_to_pool();
        Serial.print(F("# diverter valve="));
        if (diverter_valve_is_sending_water_to_roof()) {
          Serial.println("roof");
        } else {
          Serial.println("pool");
        }
        break; 
    }
    minus_key_pressed = false;
  }
  
  static bool last_red_button_pressed;
  static bool last_yellow_button_pressed;
  static bool last_diverter_valve_request;
  
  bool red_button_pressed = digitalRead(RED_BUTTON_INPUT_PIN) == LOW;
  
  if (red_button_pressed != last_red_button_pressed) { // Button state changed
    static unsigned long last_red_button_press_time;
    unsigned long now = millis();

    if ((now - last_red_button_press_time) > debounce_delay) {
      some_key_pressed = true;
      if (red_button_pressed) { // If button is depressed, toggle the pool pump relay
        if (main_pump_is_on()) {
          Serial.println(F("# Pool pump is on, turning it off"));
          turn_main_pump_off();
        } else {
          Serial.println(F("# Pool pump is off, turning it on"));
          turn_main_pump_on();
        }
      }
    }
    last_red_button_pressed = red_button_pressed;
    last_red_button_press_time = now;
  }
  bool yellow_button_pressed = digitalRead(YELLOW_BUTTON_INPUT_PIN) == LOW;
  if (yellow_button_pressed != last_yellow_button_pressed) { // Button state changed
    static unsigned long last_yellow_button_press_time;
    unsigned long now = millis();

    if ((now - last_yellow_button_press_time) > debounce_delay) {
      if (yellow_button_pressed) { // If button is depressed, toggle the pool pump relay
        some_key_pressed = true;
        if (sweep_pump_is_on()) {
          Serial.println(F("# Pool sweep pump is on, turning it off"));
          turn_sweep_pump_off();
        } else {
          Serial.println(F("# Pool sweep pump is off, turning it on"));
          turn_sweep_pump_on();
        }
      }
    }
    last_yellow_button_pressed = yellow_button_pressed;
    last_yellow_button_press_time = now;
  }
  bool diverter_valve_request = digitalRead(DIVERTER_REQUEST_INPUT_PIN) == LOW;
  if (diverter_valve_request != last_diverter_valve_request) {
    last_diverter_valve_request = diverter_valve_request;
    static unsigned long last_diverter_valve_request_time;
    unsigned now = millis();

    if ((now - last_diverter_valve_request_time) > debounce_delay) {
      some_key_pressed = true;
      if (diverter_valve_request) {
        set_diverter_valve_to_send_water_to_roof();
        Serial.println(F("# diverter request present"));
      } else {
        set_diverter_valve_to_return_water_to_pool();
        Serial.println(F("# diverter request absent"));
      }
      turn_diverter_valve_transformer_on();
    }
  }
  if (some_key_pressed) {
    backlight_timer_in_seconds = BACKLIGHT_ON_TIME_IN_SECONDS;

    if (operating_mode == m_oper || operating_mode == m_safe) {
      monitor_diag_mode.disable();
    } else {
      monitor_diag_mode.enable();
      time_entering_diag_mode_in_seconds = millis() / 1000UL;
    }
  } 
}


void turn_main_pump_on(void)
{
  quad_lv_relay->turnRelayOn(LV_RELAY_MAIN_PUMP_12V);  
  main_pump_on_time_in_seconds = millis() / 1000UL; 
  Serial.print(F("# Starting pump on time timer: "));
  Serial.println(main_pump_on_time_in_seconds);
}

void turn_main_pump_off(void)
{
  quad_lv_relay->turnRelayOff(LV_RELAY_MAIN_PUMP_12V);
}

bool main_pump_is_on(void)
{
  return quad_lv_relay->getState(LV_RELAY_MAIN_PUMP_12V);
}

void turn_sweep_pump_on(void)
{
  quad_lv_relay->turnRelayOn(LV_RELAY_BOOST_PUMP_12V);
  sweep_pump_on_time_in_seconds = millis() / 1000UL; 
  Serial.print(F("# Starting sweep pump on time timer: "));
  Serial.println(sweep_pump_on_time_in_seconds);
}

void turn_sweep_pump_off(void)
{
  quad_lv_relay->turnRelayOff(LV_RELAY_BOOST_PUMP_12V);
}

bool sweep_pump_is_on(void)
{
  return quad_lv_relay->getState(LV_RELAY_BOOST_PUMP_12V);
}

void turn_diverter_valve_transformer_on(void)
{
  quad_lv_relay->turnRelayOn(LV_RELAY_DIVERTER_TRANSFORMER_12V);
  diverter_valve_transformer_on_time_in_seconds = millis() / 1000UL;
  Serial.println(F("# turning on diverter valve transformer"));  
}

void turn_diverter_valve_transformer_off(void)
{
  quad_lv_relay->turnRelayOff(LV_RELAY_DIVERTER_TRANSFORMER_12V);
  diverter_valve_transformer_on_time_in_seconds = 0;
  Serial.println(F("# turning off diverter valve transformer"));
}


bool diverter_valve_transformer_is_on(void)
{
  return quad_lv_relay->getState(LV_RELAY_DIVERTER_TRANSFORMER_12V);
}

// Returns true if the diverter valve is sending water to the roof

bool diverter_valve_is_sending_water_to_roof(void)
{
  return quad_lv_relay->getState(LV_RELAY_DIVERTER_DIRECTION);
}

// Turn on the 24VAC tranformer and set the relay that takes the 24VAC power and
// sends it to the "open" leg of the diverter vavle by setting the single-pole
// dual-throw relay.
 
void set_diverter_valve_to_send_water_to_roof(void)
{
  unsigned long seconds_now = millis() / 1000;
  diverter_valve_in_roof_position_time_in_seconds = seconds_now;
  if (diverter_valve_is_sending_water_to_roof() == false) {
    turn_diverter_valve_transformer_on();
    quad_lv_relay->turnRelayOn(LV_RELAY_DIVERTER_DIRECTION);
  }
  turn_main_pump_on();
}

void set_diverter_valve_to_return_water_to_pool(void)
{
  if (diverter_valve_is_sending_water_to_roof()) {
   turn_diverter_valve_transformer_on();
   quad_lv_relay->turnRelayOff(LV_RELAY_DIVERTER_DIRECTION);
  }
}

void main_pump_control_callback(void)
{
  check_free_memory(F("main_pump_control"));
  if (main_pump_is_on()) { // Pump is on
    unsigned long now_in_seconds = millis() / 1000UL;
    if ((now_in_seconds - main_pump_on_time_in_seconds) > max_pump_on_time_in_seconds) {
      // Turn off the pump
      turn_main_pump_off();
      Serial.print(F("# turning off pool pump at time:"));
      Serial.println(now_in_seconds);
    }
  } else {
    // Run the main pump ever day from five minutes past midnight
    unsigned h = hour(arduino_time);
    unsigned m = minute(arduino_time);
    if (h == 0 && m == 5) {
      turn_main_pump_on();
    }
  }
  check_free_memory(F("main_pump_control exit"));
}

void sweep_pump_control_callback(void)
{
  check_free_memory(F("sweep_pump_control"));
  if (sweep_pump_is_on()) { // Pump is on
    unsigned long now_in_seconds = millis() / 1000UL;
    if ((now_in_seconds - sweep_pump_on_time_in_seconds) > max_sweep_pump_on_time_in_seconds) {
      // Turn off the pump
      turn_sweep_pump_off();
      Serial.print(F("# turning off pool sweep pump at time:"));
      Serial.println(now_in_seconds);
    }
  }
  check_free_memory(F("sweep_pump_control exit"));
}

const unsigned long diverter_motion_time = 30;

void diverter_valve_control_callback(void)
{
  check_free_memory(F("diverter_valve_control"));
  unsigned long now_in_seconds = millis() / 1000UL;
  if (diverter_valve_transformer_is_on()) {
    if ((now_in_seconds - diverter_valve_transformer_on_time_in_seconds) > diverter_motion_time) {
      turn_diverter_valve_transformer_off();
      Serial.println(F("# turning off diverter valve power"));
    }
  }
  check_free_memory(F("diverter_valve_control exit"));
}

// This is called several times a second and updates the LCD display with the latest values and status.

void update_lcd_callback(void)
{
  check_free_memory(F("update_lcd.."));
 
  if (lcd != 0) {
    if (backlight_timer_in_seconds > 0 && !monitor_lcd_backlight.isEnabled()) {
      lcd->setBacklight(255, 255, 255);
      lcd->clear(); // setBacklight scrambles the display, so clear it now to avoid junk that hangs around
                    // on the unwritten parts of the display
      monitor_lcd_backlight.enable();
    }
    
    char valve_cbuf[5];
    unsigned seconds_now = millis() / 1000;
    
    if (fail_message_time != 0 && (seconds_now - fail_message_time) > 64000) {
      fail_message_time = 0;
    } else {
      lcd->setCursor(0 /* column */, 0 /* row */);
      if (valve_motion_start_time > 0) {
        snprintf(valve_cbuf, sizeof(valve_cbuf), "%4ul", (millis() - valve_motion_start_time) / 1000UL);
      } else {
        if (diverter_valve_is_sending_water_to_roof()) {
          strncpy(valve_cbuf, "Roof", sizeof(valve_cbuf));
        } else {
          strncpy(valve_cbuf, "Pool", sizeof(valve_cbuf));
        }
      }
      snprintf(cbuf, sizeof(cbuf), "%02d:%02d:%02d %4s  %4s ",
        hour(arduino_time), minute(arduino_time), second(arduino_time), 
        valve_cbuf,
        operating_mode_to_string(operating_mode));
      lcd->print(cbuf);
    }

    lcd->setCursor(0, 1);
    snprintf(cbuf, sizeof(cbuf), "%3dF %3dF %3d PSI", 
             (int)pool_temperature_F, (int)outside_temperature_F, (unsigned)(pool_pressure_volts * 100.0));
    lcd->print(cbuf);
    
    lcd->setCursor(0, 2);
    unsigned long main_pump_runtime_in_seconds = 0;
    unsigned long sweep_pump_runtime_in_seconds = 0;
    unsigned long diverter_valve_in_roof_position_in_seconds = 0;
    if (main_pump_is_on()) {
      main_pump_runtime_in_seconds = seconds_now - main_pump_on_time_in_seconds;
    } 
    if (sweep_pump_is_on()) {
      sweep_pump_runtime_in_seconds = seconds_now - sweep_pump_on_time_in_seconds;
    }
    if (diverter_valve_is_sending_water_to_roof()) {
      diverter_valve_in_roof_position_in_seconds = seconds_now - diverter_valve_in_roof_position_time_in_seconds;
    }
    snprintf(cbuf, sizeof(cbuf), "%3u:%02u %3u:%02u %3u:%02u   ", 
             (unsigned)(main_pump_runtime_in_seconds / 60), 
             (unsigned)(main_pump_runtime_in_seconds % 60),
             (unsigned)(sweep_pump_runtime_in_seconds / 60), 
             (unsigned)(sweep_pump_runtime_in_seconds % 60),
             (unsigned)(diverter_valve_in_roof_position_in_seconds / 60),
             (unsigned)(diverter_valve_in_roof_position_in_seconds % 60));
             
    lcd->print(cbuf);
    // If the backlight should come on, we find out here because the backlight timer is non zero. Functions
    // that wish to turn on the backlight set that timer to the number of seconds they wish to have the backlight on.
    // In the case we find that the backlight monitor task is not enabled, but the counter is non zero we enable 
    // the backlight monitor task and turn on the backlight.
    
    
  }
  check_free_memory(F("update_lcd.. exit"));
}

// This is called once per second.  It samples the time, temperature, and pressure.  It records these in globals read by other
// tasks.

void read_time_and_sensor_inputs_callback()
{
  check_free_memory(F("read_time_and.."));
  arduino_time = now();
  
  unsigned long raw_pool_temperature_volts = 0;
  unsigned long raw_outside_temperature_volts = 0;
  unsigned long raw_pressure_volts = 0;
  unsigned samples = 20;

  // Reduce sample noise by taking a number of samples and using the arithmetic mean
  for (int i = 0 ; i < samples ; i++) {
    raw_pool_temperature_volts += analogRead(POOL_TEMPERATURE_INPUT);
    raw_outside_temperature_volts += analogRead(OUTSIDE_TEMPERATURE_INPUT);
    raw_pressure_volts += analogRead(PRESSURE_INPUT);
  }
  
  raw_pool_temperature_volts /= samples;  
  raw_outside_temperature_volts /= samples;
  raw_pressure_volts /= samples;

  // See Arduino documents to understand the conversion of raw ADC values to a voltage
  float pool_temperature_millivolts = (float)raw_pool_temperature_volts * 5000.0 / 1023.0;

  // See the LM36 data shee to understand the conversion of millivolts to degrees C
  float pool_temperature_C = (pool_temperature_millivolts - 500.0) / 10.0;

  // Convert Centigrade to Fahrenheit
  pool_temperature_F = pool_temperature_C * 9.0 / 5.0 + 32.0;

  float outside_temperature_millivolts = (float)raw_outside_temperature_volts * 5000.0 / 1023.0;
  float outside_temperature_C = (outside_temperature_millivolts - 500.0) / 10.0;

  // Convert to degrees F
  outside_temperature_F = outside_temperature_C * 9.0 / 5.0 + 32.0;
  pool_pressure_volts = raw_pressure_volts * 5.0 / 1023.0;
  check_free_memory(F("read_time_and.. exit"));
}

// Called periodically.  Sends relevant telemetry back over the USB serial connection to a host
// computer (e.g., M3 Mac Mini) that can aborb these messages and generate plots

void print_status_to_serial_callback(void) 
{
  check_free_memory(F("print_status_to.."));
  static float last_pool_temperature_F;
  static float last_pool_pressure_volts; 
  static bool last_main_pump;
  static bool last_sweep_pump;
  static bool last_to_roof; 
  static char line_counter = 0;
  static int skipped_record_counter = 0;

  bool main_pump = main_pump_is_on();
  bool sweep_pump = sweep_pump_is_on();
  bool to_roof = diverter_valve_is_sending_water_to_roof();
  
  if (abs(pool_temperature_F - last_pool_temperature_F) > 1.0 ||
      abs(pool_pressure_volts - last_pool_pressure_volts) > 1.0 ||
      main_pump != last_main_pump ||
      sweep_pump != last_sweep_pump ||
      to_roof != last_to_roof ||
      skipped_record_counter++ > 10) {
   last_pool_temperature_F = pool_temperature_F;
   last_pool_pressure_volts = pool_pressure_volts;
   last_main_pump = main_pump;
   last_sweep_pump = sweep_pump;
   last_to_roof = to_roof;
   
   skipped_record_counter = 0;
      
    if (line_counter == 0) {
        Serial.println(F("# Date     Time     Pool Out PSI  Main Swp Divert"));
        line_counter = 20;
      } else {
        line_counter--;
      }
    // We generate the output line in chunks, to conversve memory.  But it also makes the code easier to
    // read because we don't have one humongous snprintf().  The size of buf is carefully chosen to be just large enough.
    snprintf(cbuf, sizeof(cbuf), "%4u-%02u-%02u %02u:%02u:%02u ",
               year(arduino_time),
               month(arduino_time),
               day(arduino_time),
               hour(arduino_time),
               minute(arduino_time),
               second(arduino_time));
    Serial.print(cbuf);
    snprintf(cbuf, sizeof(cbuf), " %3u %3u %3u %3u %3u %3u\n", 
             (unsigned)pool_temperature_F, 
             (unsigned)outside_temperature_F,
             (unsigned)(pool_pressure_volts * 100.0),
             main_pump,
             sweep_pump,
             to_roof);
    Serial.print(cbuf);
  }
  check_free_memory(F("print_status_to.. exit"));
}

/*
   Called by failure paths that should never happen.  When we get the RS-485 input working, we'll allow the user
   to do things in this case and perhaps resume operation.
*/
void fail(const __FlashStringHelper *fail_message)
{
  Serial.print(F("FAIL: "));
  Serial.println(fail_message);
  delay(500); // Give the serial link time to propogate the error message before execution ends
  abort();
}

// Called frequently to poll for and act on received console input.  This recognizes characters
// that correspond to keys on the controller, 's' for select, '+' for the plus key, and '-' for the minus key.

void monitor_serial_console_callback(void)
{
  char command_buf[3];
  command_buf[0] = '\0';

  while (Serial.available() > 0) {  // Check if data is available to read
    char received_char = Serial.read();  // Read one character
    int l = strlen(command_buf);
    if (l >= sizeof(command_buf) - 1) {
      Serial.println(F("# command buffer overflow"));
      break;
    }
    if (received_char == '\n') {
      Serial.print(F("# Recieved: "));
      Serial.println(command_buf);

      switch (command_buf[0]) {        
        case 's':
          select_key_pressed = true;
          break;

        case '+':
          plus_key_pressed = true;
          break;

        case '-':
          minus_key_pressed = true;
          break;

        default:
          Serial.print(F("# Unknown command: "));
          Serial.println(command_buf);
          Serial.println(F("# choices are: 's', +, -"));
          break;
      }
      break;
    } else {
      command_buf[l] = received_char;
      command_buf[l+1] = '\0';
    }
  }
}

// Prepare the LCD after booting and before the LCD update task starts.

void setup_lcd(void)
{
  if (lcd != 0) {
    if (fast_lcd_comm) {
      Wire.beginTransmission(SERLCD_I2C_ADDR);
      Wire.write(0x7C);      // Special command indicator
      Wire.write(0x2B);      // Change baud rate command
      Wire.write(4);         // 4 = 115200 baud (see table below)
      Wire.endTransmission();
    }

    lcd->begin(Wire, SERLCD_I2C_ADDR);         // Default I2C address of Sparkfun 4x20 SerLCD
    lcd->setBacklight(255, 255, 255); 
    lcd->setContrast(5);
    lcd->clear();
    
    lcd->setCursor(0 /* column */, 0 /* row */);
    lcd->print(F("Pump Control "));  // Display this on the first row
    
    lcd->setCursor(0,1);
    lcd->print(F(__DATE__));         // Display this on the second row, left-adjusted
    
    lcd->setCursor(0, 2);
    lcd->print(F(__TIME__));         // Display this on the third row, left-adjusted
  }
  backlight_timer_in_seconds = BACKLIGHT_ON_TIME_IN_SECONDS;
}

void setup_arduino_pins(void)
{
  for (int i = 0; i < 6; i++) {
    pinMode(button_pins[i], INPUT_PULLUP);  // enable internal pull-up
    attachPinChangeInterrupt(digitalPinToPCINT(button_pins[i]), poll_keys_callback, FALLING);
  }
  
  pinMode(LED_BUILTIN, OUTPUT); 
  pinMode(DIVERTER_REQUEST_INPUT_PIN, INPUT);
}

void setup_i2c_bus(void)
{
  {
    byte error, address;
    int nDevices;
  
    nDevices = 0;
    for (address = 1; address < 127; address++ )
    {
      // The i2c_scanner uses the return value of
      // the Write.endTransmisstion to see if
      // a device did acknowledge to the address.
      Wire.beginTransmission(address);
      error = Wire.endTransmission();
  
      if (error == 0)
      {
        if (verbose_I2C) {
          Serial.print(F("# I2C device found at address 0x"));
          if (address < 16) {
            Serial.print("0");
          } 
          Serial.print(address, HEX);
        }
        switch (address) {
          case SERLCD_I2C_ADDR:
           lcd = new SerLCD();
           if (verbose_I2C) {
             Serial.print(F(" (SerLCD 4x20)"));
           }
           break;

         case SSR_RELAY_I2C_ADDR:
           if (verbose_I2C) {
               Serial.print(F(" (Quad SSR Qwiic Relay)"));
             }
        
             break;
             
         case LV_RELAY_I2C_ADDR:
           if (verbose_I2C) {
             Serial.print(F(" (Quad Qwiic Relay)"));
           }
           
           quad_lv_relay = new Qwiic_Relay(address);
           if (quad_lv_relay->begin() == 0) {
             Serial.print(F("# Failure to start quad qwiic relay object"));
           }
           break;
          
         default:
          if (quad_lv_relay == (void *)0) {
            Serial.print(F(" (unexpected, will guess that it is the Quad Qwiic Relay at the wrong address)\n"));
            quad_lv_relay = new Qwiic_Relay(address);
            if (quad_lv_relay->begin()) {
              Serial.print(F("# Wayward Qwiic relay found, remapping it to where it is suppose to be\n"));
              quad_lv_relay->changeAddress(LV_RELAY_I2C_ADDR);
            } else {
              Serial.print(F("# unexpected device, unable to treat it as a quad qwiic relay\n"));
            }
          } else {
            Serial.print(F("# unexpected device after finding Qwiic quad relay\n"));
          }
        }
        if (verbose_I2C) {
          Serial.print("\n");
        }
  
        nDevices++;
      } else if (error == 4) {
        Serial.print(F("# Unknown error at address 0x"));
        if (address < 16) {
          Serial.print("0");
        }
        Serial.println(address, HEX);
      }
    }
    if (nDevices == 0) {
      Serial.println(F("# No I2C devices found\n"));
    }
  }
  if (quad_lv_relay == (void *)0) {
    fail(F("REL LV"));
  }
}

// The pool pump controls have no accurate real time clock.  For now, set Arduino time
// to the build time of the firmware.  That's better than nothing.  Eventually, I'd like
// to use the CMOS to record the time every day or so and use this on booting so that
// dates don't move backwards.  Also, perhaps have a light sensor and set an approximate
// time of day based on that.  And maybe use the time at which the diverter valve is
// asked to send water to the roof panels to set the time of day.

void setup_arduino_time(void)
{
  int hh, mmin, ss, dd, mm, yy;
  char monthStr[4];
  sscanf(__TIME__, "%d:%d:%d", &hh, &mmin, &ss);
  sscanf(__DATE__, "%s %d %d", monthStr, &dd, &yy);
  int month = (strcmp(monthStr, "Jan") == 0) ? 1 :
                (strcmp(monthStr, "Feb") == 0) ? 2 :
                (strcmp(monthStr, "Mar") == 0) ? 3 :
                (strcmp(monthStr, "Apr") == 0) ? 4 :
                (strcmp(monthStr, "May") == 0) ? 5 :
                (strcmp(monthStr, "Jun") == 0) ? 6 :
                (strcmp(monthStr, "Jul") == 0) ? 7 :
                (strcmp(monthStr, "Aug") == 0) ? 8 :
                (strcmp(monthStr, "Sep") == 0) ? 9 :
                (strcmp(monthStr, "Oct") == 0) ? 10 :
                (strcmp(monthStr, "Nov") == 0) ? 11 : 12;

  setTime(hh, mmin, ss, dd, month, yy);
}
/*
   This is the function that the Arudino run time system calls once, just after start up.  We have to set the
   pin modes of the ATMEGA correctly as inputs or outputs.  We also fetch values from EEPROM for use during
   our operation and emit a startup message.
*/
void setup(void)
{
  setup_arduino_pins();
  analogReference(DEFAULT);
  setup_arduino_time();
  
  Wire.begin();
  Wire.setClock(400000); 

  Serial.begin(SERIAL_BAUD);
  UCSR0A = UCSR0A | (1 << TXC0); //Clear Transmit Complete Flag
  Serial.println(F("Pool valve controller"));              // Put the first line we print on a fresh line (i.e., left column of output)
  

  setup_i2c_bus(); // This sets "quad_lv_relay" and "lcd"
  setup_lcd();

  // If we start with the diverter valve direction relay in the positive for sending water to roof, but there is no request for this
  // then ensure that the diverter valve is sending water back to the pool.
  
  bool diverter_valve_request = digitalRead(DIVERTER_REQUEST_INPUT_PIN) == LOW;
  if (diverter_valve_is_sending_water_to_roof() &&  !diverter_valve_request) {
    set_diverter_valve_to_return_water_to_pool();
  }
  if (main_pump_is_on()) {
    // Do we need to do anything?
  }
  if (sweep_pump_is_on()) {
    
  }
}

/*
   This is the function that the Arduino run time system calls repeatedly after it has called setup().
*/
void loop(void)
{
  // All code after setup() executes inside of tasks, so the only thing to do here is to call the task scheduler's execute() method.
  ts.execute();
}
