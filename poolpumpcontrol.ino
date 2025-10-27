
/*
  This controls a pool pump and a diverter valve.
       
   Creative Commons Licence
   2025 Robert Bedichek

* Connection to this Arduino via USB serial line is normally on this port: /dev/tty.usbserial-211440, though
* nothing in this code depends on the USB serial line working.  
*/

#include <string.h> //Use the string Library
#include <ctype.h>
#include <TimeLib.h>  // for update/display of time
#include <avr/wdt.h>

//   All the operational code uses this time structure.  This is initialized at start time from the battery-backed up DS1307 RTC.
time_t arduino_time;
const bool verbose_I2C = false;

#include <Wire.h> // For communication with Sparkfun quad relay and LCD display

/*
 * Sparkfun quad low voltage relays.  Relays 1 and 2 control the power to inlet valve motors.
 * Relay 3 and 4 control the power to the outlet valve.
 * The NC pins of all four relays are connected to ground.
 * The NO pins of all four relays are connected to +12V.
 * To open the inlet valve, Relay 1 is energized.  To close it, Relay 2 is energized.
 * To open the outlet valve, Relay 3 is energized and to close it Relay 4 is energized.
 */
#include "SparkFun_Qwiic_Relay.h"
#define LV_RELAY_I2C_ADDR  (0x6D)                 // Default I2C address of the mechanical, low voltage, 4-relay board

#define LV_RELAY_MAIN_PUMP_12V            (1)     // When energized, this relay sends 12VDC to the coil of the 240VAC contactor that powers the pool pump
#define LV_RELAY_BOOST_PUMP_12V           (2)     // For the boost pump relay coil
#define LV_RELAY_DIVERTER_TRANSFORMER_12V (3)     // When energized, this powers the SSR that controls power to the 24VAC transformer
#define LV_RELAY_DIVERTER_DIRECTION       (4)     // When this and the 24VAC transformer are enabled, this drives the diverter valve to roof mode.

Qwiic_Relay *quad_lv_relay;

#include <SerLCD.h>
SerLCD *lcd;
#define SERLCD_I2C_ADDR (0x72)
const bool fast_lcd_comm = false;
/*
   Counts down when the backlight is on.  When it is zero, we turn the backlight off.  Units are seconds.
*/
// unsigned long backlight_timer;            

// #define BACKLIGHT_ON_TIME (600) 


#define _TASK_SLEEP_ON_IDLE_RUN
#include <TaskScheduler.h>

//    Central (and only) task scheduler data structure.
Scheduler ts;

// Speed at which we run the serial connection (both via USB and RS-485)
#define SERIAL_BAUD (115200)

typedef enum {m_normal, m_safe, m_pump, m_boost, m_24vac, m_diverter, m_last} operating_mode_t;

bool toggle_key_pressed = false;

bool date_set;
bool time_set;
void read_time_and_sensor_inputs_callback(void);
// void monitor_lcd_backlight_callback(void);
void print_status_to_serial_callback(void);
void poll_keys_callback(void);
void update_lcd_callback(void);

void monitor_main_pump_callback(void);
void monitor_boost_pump_callback(void);
//-------------------------------------------------------------------------------------
const unsigned long drain_down_time = 5 * 60 * 1000UL; // 5 minutes
const unsigned long periodic_filter_interval = 24UL * 3600UL * 1000UL;
const unsigned long max_main_pump_on_time = 3UL * 3600UL * 1000UL;
const unsigned long max_boost_pump_on_time = 1UL * 3600UL * 1000UL;
const unsigned long periodic_boost_interval = 24UL * 3600UL * 1000UL;  // Run boost pump once per day
const unsigned long periodic_boost_duration = 30UL * 60UL * 1000UL;    // Run for 30 minutes

#define SENSOR_ERROR (-999.0)  // Sentinel value for failed temperature sensors

// Last value read from input pin that is driven from solarthermal controller (asking for pool controller to send water to panels)
bool diverter_valve_request; 
unsigned long last_diverter_valve_request_change;  // Value of millis() the last time the input from the solarthermal controller changed.

void monitor_diverter_valve_callback(void);
//-------------------------------------------------------------------------------------

void monitor_diag_mode_callback(void);
void monitor_serial_console_callback(void);
void process_pressed_keys_callback(void);

// This string variable is used by multiple functions below, but not at the same time
char cbuf[60];

/*****************************************************************************************************/

bool manual_main_pump_request;              // Set true by press of red buttor or '+' key in m_pump mode
unsigned long main_pump_on_off_time;        // Assigned millis() when main pump is switched on
unsigned long boost_pump_on_off_time;
bool periodic_boost_request;                 // Set true when periodic boost pump operation is needed
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

Task monitor_serial_console(TASK_SECOND, TASK_FOREVER, &monitor_serial_console_callback, &ts, true);

Task monitor_diverter_control(TASK_SECOND * 6, TASK_FOREVER, &monitor_diverter_valve_callback, &ts, true);

float pool_temperature1_F = 0.0;
float pool_temperature2_F = 0.0;
float pool_temperature_F = 0.0; // Average of the two pool temperatures, or just one of them if the other is out of range
float outside_temperature_F = 0.0;
float pressure_psi;

const float max_pressure_sending_water_to_pool = 23.0;
const float max_pressure_sending_water_to_roof = 25.0;

Task read_time_and_sensor_inputs(500, TASK_FOREVER, &read_time_and_sensor_inputs_callback, &ts, true);
/*****************************************************************************************************/
#define POOL_TEMPERATURE1_INPUT   (A0)     // 0-5VDC LM36 that is immersed in pool water, about 1' deep under diving board
#define POOL_TEMPERATURE2_INPUT   (A1)     // A second LM36 in the same probe, for redundancy
#define OUTSIDE_TEMPERATURE_INPUT (A2)     // An LM36 sensing ambient temperature near the pool equipment
#define PRESSURE_INPUT            (A3)     // 0-5VDC pressure sensor on top of filter canister

bool timer_switch_on;

void setup_arduino_pins(void)
{
  // Give unconnected pins a pull up resistor so that they don't cause spurious interrupts or waste power
  pinMode(2, INPUT_PULLUP);                          // Unconnected and unused input
  pinMode(3, INPUT_PULLUP);                          // Unconnected and unused input
  pinMode(4, INPUT_PULLUP);                          // Unconnected and unused input
 
#define DIVERTER_REQUEST_INPUT_PIN (5)              // Fed by an optoisolator that is driven by the 'solarthermal' controller
  pinMode(DIVERTER_REQUEST_INPUT_PIN, INPUT_PULLUP); // D5/PD5.  A task polls for changes to this pin.
  
  pinMode(6, INPUT_PULLUP);                          // Unconnected and unused input

#define RED_BUTTON_INPUT_PIN       (7)               // Simple momemtary contact push button
  pinMode(RED_BUTTON_INPUT_PIN, INPUT_PULLUP);       // D7/PD7.  A task polls for changes to this pin.

#define TIMER_SWITCH_INPUT_PIN     (8)               // Mechanical timer switch on outside of house
  pinMode(TIMER_SWITCH_INPUT_PIN, INPUT_PULLUP);     // D8/PB0.  A task polls for changes to this pin.


  // The four maintenance keys (select, enter, plus, minus -- from top to bottom) are connected to pins
  // 12 through 9.  When pressed, they ground their Arduino inputs, when released, the Arduino inputs
  // are pulled up an an internal resistor.


#define KEY_4_PIN (9)                               // Bottom most input key, "-"
  pinMode(KEY_4_PIN, INPUT_PULLUP);                 // D9/PB1.  Normally we take interrupts to recognize presses of this key.

#define KEY_3_PIN (10)                              // Second from the bottom input key, "+"
  pinMode(KEY_3_PIN, INPUT_PULLUP);                 // D10/PB2.  Normally we take interrupts to recognize presses of this key.

#define KEY_2_PIN (11)                              // Second from top input key, "Enter" (unused)
  pinMode(KEY_2_PIN, INPUT_PULLUP);                 // D11/PB3.  Normally we take interrupts to recognize presses of this key.

#define KEY_1_PIN (12)                              // Top most input key, "Select"
  pinMode(KEY_1_PIN, INPUT_PULLUP);                 // D12/PB4.  Normally we take interrupts to recognize presses of this key.
  
  pinMode(LED_BUILTIN, OUTPUT);                     // D13/PB5
}

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

#undef POLL_KEYS
#ifdef POLL_KEYS

/* When in polling mode, this task will call 'process_pressed_keys_callback() if it finds any pressed keys */
Task poll_keys(50 /* Sample 20 times per second*/, TASK_FOREVER, &poll_keys_callback, &ts, true);
#else
ISR(PCINT0_vect) 
{
  poll_keys_callback();
}

/* When in interrupt (not polling) mode, we have to run 'process_pressed_keys_callback() as a task */
Task process_pressed_keys(100, TASK_FOREVER, &process_pressed_keys_callback, &ts, true);
#endif

volatile unsigned long lastInterruptTime = 0;
const int debounce_delay = 100;  // 150ms debounce time

Task print_status_to_serial(TASK_SECOND, TASK_FOREVER, &print_status_to_serial_callback, &ts, true);
/*****************************************************************************************************/
// Update slightly faster than once per second so that the LCD time advances regularly in a way that 
// looks normal to humans (i.e., make sure that the seconds display counts up once per second).
// We depend on this being called about once per second and being enabled at boot time.
// Task monitor_lcd_backlight(TASK_SECOND, TASK_FOREVER, &monitor_lcd_backlight_callback, &ts, true);
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
#define LOW_MEMORY_LIMIT (500)

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
    if (caller != (void *)0) {
      Serial.print(caller);
    }
    Serial.print(F(" free memory="));
    Serial.println(fm);
    trace_initial_calls--;
  }
}

// Helper function to format temperature for display
// Returns "  -" if sensor has failed, otherwise returns formatted temperature
void format_temperature(float temp_F, char *buf, size_t buf_size) {
  if (temp_F == SENSOR_ERROR) {
    strncpy(buf, "  -", buf_size);
    buf[buf_size - 1] = '\0';
  } else {
    // Format as "XXX" (3 digits, right-aligned)
    snprintf(buf, buf_size, "%3d", (int)temp_F);
  }
}

// Returns a string that desribes the current diag mode.  For some reason, it stopped working to pass
// in the diag mode, but since we always passed in the global variable "diag_mode", just commenting out
// the parameter here, and the passed arguments where this is called allowed this to compile again.

const char *operating_mode_to_string(operating_mode_t operating_mode) 
{
  // m_pump, m_diverter
  char *s[] = {"Normal", "Safe  ", "Main P", "Boost ", "24 VAC", "Divert"};
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
    monitor_diag_mode.disable();
  }
}

/*
   Called every second the backlight is on and turns off the backlight, and disables itself, when the backlight timer
   has counted down to zero.
*/
// void monitor_lcd_backlight_callback(void) 
// {
//   if (backlight_timer > 0 && (operating_mode == m_normal || operating_mode == m_safe)) {
//     backlight_timer--;
//     if (backlight_timer == 0) {
//       delay(50);
//       lcd->clear(); // Changing the backlight scrambles the display, so clear it and let it be refreshed in update_lcd
//       delay(50);
//       lcd->setBacklight(0, 0, 0);
//       delay(50);
//       monitor_lcd_backlight.disable();
//     }
//   }
// }

// This is called by a task if we have compiled with the option to poll keys.  If we have 
// compiled to use interrupts, this is the interrupt routine.

void poll_keys_callback(void)
{
  if ((millis() - lastInterruptTime) > debounce_delay) {  // Debounce check
  
    bool key_select = !(PINB & (1 << PB4));
    bool key_enter = !(PINB & (1 << PB3));
    bool key_plus = !(PINB & (1 << PB2));
    bool key_minus = !(PINB & (1 << PB1));

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
    
    lastInterruptTime = millis();  // Update debounce timer
#ifdef POLL_KEYS
    process_pressed_keys_callback();
#endif
  } 
}



// The interrupt routine above will set global variables indicating which keys have been pressed.
// In this function, we look at those global variables and take action required by the key presses
// and then reset those global variables.

void process_pressed_keys_callback(void)
{
  check_free_memory(F("process_keys.."));
  bool some_key_pressed = select_key_pressed | enter_key_pressed | plus_key_pressed | minus_key_pressed;

  bool select_pressed;
  noInterrupts();
  select_pressed = select_key_pressed;
  select_key_pressed = false;
  interrupts();

  if (select_pressed) {
    operating_mode = (operating_mode + 1) % m_last;
    Serial.print(F("# mode now "));
    Serial.println(operating_mode_to_string(operating_mode));
  }
 
  if (plus_key_pressed) {
    switch (operating_mode) {
      case m_normal:   // Move time forward 10 minutes for each press of the plus key
        adjustTime(600);
        break;

      case m_safe:
        {
        }
        break;

      case m_pump:
        manual_main_pump_request = true;
        turn_main_pump_on(F("# main pump="));
        Serial.println(main_pump_is_on());
        break;

      case m_boost:
        turn_boost_pump_on(F("# boost pump="));
        Serial.println(boost_pump_is_on());
        break;

      
      case m_24vac:
        turn_diverter_valve_transformer_on();
        break;

      case m_diverter:
        Serial.print(F("# diverter valve="));
        set_diverter_valve_to_send_water_to_roof();
        if (diverter_valve_is_sending_water_to_roof()) {
          Serial.println("roof");
        } else {
          Serial.println("pool");
        }
        break; 

      default:
        Serial.println(F("# unknown operating mode"));
        break;

    }
    plus_key_pressed = false;
  }

  if (minus_key_pressed) {
    switch (operating_mode) {
      case m_normal:
        adjustTime(-600);
        break;

      case m_safe:
        {
          unsigned relay_history[4] = {0, 0, 0, 0};
          for (int i = 0 ; i < 1000; i++){
  
            for (int relay = 0 ; relay < 4 ; relay++) {
              if (quad_lv_relay != nullptr) {
                relay_history[relay] += quad_lv_relay->getState(relay + 1);
              }
            }
          }
          for (int relay = 0 ; relay < 4 ; relay++) {
            Serial.print(F("# relay count="));
            Serial.println(relay_history[relay]);
          }
        }
        break;

      case m_pump:
        manual_main_pump_request = false;
        turn_main_pump_off(F("# main pump="));
        Serial.println(main_pump_is_on());
        break;

      case m_boost:
        turn_boost_pump_off(F("# boost pump="));
        Serial.println(boost_pump_is_on());
        break;
        
      case m_24vac:
        turn_diverter_valve_transformer_off();
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
        
      default:
        Serial.println(F("# unknown operating mode"));
        break;
    }
    minus_key_pressed = false;
  }
  
  if (operating_mode == m_normal) {
    static bool last_red_button_pressed;
    static bool last_timer_switch_on;
    static bool last_diverter_valve_request;
    
    bool red_button_pressed = digitalRead(RED_BUTTON_INPUT_PIN) == LOW;
    
    if (red_button_pressed != last_red_button_pressed) { // Button state changed
      static unsigned long last_red_button_press_time;

      if ((millis() - last_red_button_press_time) > debounce_delay) {
        some_key_pressed = true;
        if (red_button_pressed) { // If button is depressed, toggle the pool pump relay
          if (main_pump_is_on()) {
            manual_main_pump_request = false;
            turn_main_pump_off(F("# Pool pump is on, turning it off due to press of red button\n"));
          } else {
            // Check if we're in drain-down period (valve in roof mode, waiting for pool piping to drain)
            if (diverter_valve_is_sending_water_to_roof() &&
                !diverter_valve_request &&
                (millis() - main_pump_on_off_time) < drain_down_time) {
              Serial.println(F("# Red button: cannot turn on pump - waiting for drain-down"));
            } else {
              manual_main_pump_request = true;
              turn_main_pump_on(F("# Pool pump is off, turning it on due to press of red button\n"));
            }
          }
        }
      }
      last_red_button_pressed = red_button_pressed;
      last_red_button_press_time = millis();
    }
    timer_switch_on = digitalRead(TIMER_SWITCH_INPUT_PIN) == LOW;
    if (timer_switch_on != last_timer_switch_on) { // Button state changed
      static unsigned long timer_switch_on_time;

      if ((millis() - timer_switch_on_time) > debounce_delay) {
        if (timer_switch_on) { // While timer switch is closed, run main pump
          some_key_pressed = true;
          if (main_pump_is_on() == false) {
            // Check if we're in drain-down period
            if (diverter_valve_is_sending_water_to_roof() &&
                !diverter_valve_request &&
                (millis() - main_pump_on_off_time) < drain_down_time) {
              Serial.println(F("# Timer switch: cannot turn on pump - waiting for drain-down"));
            } else {
              turn_main_pump_on(F("# Main pump is off, turn it on due to timer switch request\n"));
            }
          }
          // We will turn on the boost pump when we see that the main pump has developed
          // pressure, to ensure that the main pump is working before we turn on the boost pump

        } 
      }
      last_timer_switch_on = timer_switch_on;
      timer_switch_on_time = millis();
    }
    diverter_valve_request = digitalRead(DIVERTER_REQUEST_INPUT_PIN) == LOW;
    if (diverter_valve_request != last_diverter_valve_request) {
      last_diverter_valve_request = diverter_valve_request;
      if ((millis() - last_diverter_valve_request_change) > debounce_delay) {
        some_key_pressed = true;
        if (diverter_valve_request) {
          turn_diverter_valve_transformer_on();
          set_diverter_valve_to_send_water_to_roof();
          turn_main_pump_on(F("# diverter request present\n"));

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
  if (some_key_pressed) {
    if (operating_mode == m_normal || operating_mode == m_safe) {
      monitor_diag_mode.disable();
    } else {
      monitor_diag_mode.enable();
      time_entering_diag_mode = millis();
    }
  } 
}
// bool panels_draining_pool_water = false;

void turn_main_pump_on(const __FlashStringHelper *message)
{
  if (message != nullptr) {
    Serial.print(message);
  }
  if (main_pump_is_on() == false) {
    if (quad_lv_relay != nullptr) {
      quad_lv_relay->turnRelayOn(LV_RELAY_MAIN_PUMP_12V);  
    }
    main_pump_on_off_time = millis();
    Serial.print(F("# turn_main_pump_on(): "));
    Serial.println(main_pump_on_off_time);
  }
}

void turn_main_pump_off(const __FlashStringHelper *message)
{
  if (message != nullptr) {
    Serial.print(message);
  }
  if (boost_pump_is_on()) {
    turn_boost_pump_off(F("# alert turn_main_pump_off() found boost pump on"));
  }
  if (main_pump_is_on()) {
    if (quad_lv_relay != nullptr) {
      quad_lv_relay->turnRelayOff(LV_RELAY_MAIN_PUMP_12V);
    }
    main_pump_on_off_time = millis();
    Serial.print(F("# turn_main_pump_off(): "));
    Serial.println(main_pump_on_off_time);
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
  if (message != (void *)0) {
    Serial.print(message);
  }
  // Do not try to turn on the boost pump unless the main pump is on and the boost pump is off
  if (main_pump_is_on() && boost_pump_is_on() == false) {
    if (quad_lv_relay != (void *)0) {
      quad_lv_relay->turnRelayOn(LV_RELAY_BOOST_PUMP_12V);  
    }
    boost_pump_on_off_time = millis();
    Serial.print(F("# turn_boost_pump_on(): "));
    Serial.println(boost_pump_on_off_time);
  }
}

void turn_boost_pump_off(const __FlashStringHelper *message)
{
  if (message != (void *)0) {
    Serial.print(message);
  }
  if (boost_pump_is_on()) {
    if (quad_lv_relay != (void *)0) {
      quad_lv_relay->turnRelayOff(LV_RELAY_BOOST_PUMP_12V);
    }
    boost_pump_on_off_time = millis();
    Serial.print(F("# turn_boost_pump_off(): "));
    Serial.println(boost_pump_on_off_time);
  }
}

bool boost_pump_is_on(void)
{
  bool v = false;
  if (quad_lv_relay != (void *)0) {
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
  Serial.print(F("# turning on diverter valve transformer millis()="));  
  Serial.println(diverter_valve_transformer_on_time);
}

void turn_diverter_valve_transformer_off(void)
{
  if (quad_lv_relay != nullptr) {
    quad_lv_relay->turnRelayOff(LV_RELAY_DIVERTER_TRANSFORMER_12V);
  }
  diverter_valve_transformer_on_time = 0;
  Serial.print(F("# turning off diverter valve transformer millis()="));
  Serial.println(millis());
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
  check_free_memory(F("monitor_pump"));
  if (operating_mode == m_normal) {
    if (main_pump_is_on()) { // Pump is on, see if any of the reasons for it to turn off have occured
      
      if ((millis() - main_pump_on_off_time) > max_main_pump_on_time) {
        // Turn off the pump
        turn_main_pump_off(F("# turning off pool pump due to time limit\n"));
        manual_main_pump_request = false; // Cancel request
      } else {
        float max_psi = diverter_valve_is_sending_water_to_roof() ? max_pressure_sending_water_to_roof : max_pressure_sending_water_to_pool;

        if (pressure_psi > max_psi) {
          turn_main_pump_off(F("# alert turning off pump due to overpressure: PSI="));
          Serial.println(pressure_psi);
          manual_main_pump_request = false;
        } else if (!diverter_valve_request) {
          // If we are not being asked to send water to the roof, the pool-cleaner timer switch
          // is not requesting pool cleaning, and this is not the time to filter the pool water
          // by running the pump, then turn it off.
          if (!timer_switch_on && !manual_main_pump_request) {
            turn_main_pump_off(F("# turning off pump due to lack of timer switch and manual (red button) requests\n"));
          } else {
            if (diverter_valve_is_sending_water_to_roof() &&
            (millis() - last_diverter_valve_request_change) < 60 * 1000UL) {
              // Although we have other reasons to run the pump, we need to turn it off to let the panels
              // drain of pool water.  We will later turn it back on.  We distinguish this case by seeing
              // if the the diverter request just went away within the last minute.
              turn_main_pump_off(F("# turning off pump to let panels drain\n"));
              manual_main_pump_request = false;
            }
          }
        }
      }
    } else { // Pump is not on, see if it should be, but only after a twenty minute delay for any draining needed
      if ((millis() - main_pump_on_off_time) > drain_down_time) {
        if (timer_switch_on) {
          turn_main_pump_on(F("# turning pump back on for timer switch\n"));
        } else if ((millis() - main_pump_on_off_time) > periodic_filter_interval) {
          turn_main_pump_on(F("# periodic filtering\n"));
          manual_main_pump_request = true;
        }
      }
    }
  }
  
  check_free_memory(F("monitor_pump exit"));
}

void monitor_boost_pump_callback(void)
{
  if (operating_mode == m_normal) {
    // Check if it's time for periodic boost pump operation (once per day)
    if (!boost_pump_is_on() &&
        !timer_switch_on &&
        (millis() - boost_pump_on_off_time) > periodic_boost_interval) {
      periodic_boost_request = true;
      Serial.println(F("# Periodic boost pump requested"));
      // Ensure main pump is on before we can run boost pump
      if (!main_pump_is_on()) {
        turn_main_pump_on(F("# turning on main pump for periodic boost operation\n"));
      }
    }

    // Turn on boost pump when conditions are met (timer switch OR periodic request)
    if (main_pump_is_on() && pressure_psi >= 5 && boost_pump_is_on() == false) {
      if (timer_switch_on) {
        turn_boost_pump_on(F("# timer switch requesting boost pump"));
        periodic_boost_request = false; // Timer switch overrides periodic
      } else if (periodic_boost_request) {
        turn_boost_pump_on(F("# periodic boost pump operation"));
      }
    }

    if (boost_pump_is_on() && (main_pump_is_on() == false || pressure_psi < 5)) {
      turn_boost_pump_off(F("# low pressure, turning boost pump off"));
      periodic_boost_request = false;
    }
  }

  // Check time limits
  unsigned long boost_time_limit = periodic_boost_request ? periodic_boost_duration : max_boost_pump_on_time;
  if (boost_pump_is_on() && (millis() - boost_pump_on_off_time) > boost_time_limit) {
    // Turn off the boost pump
    if (periodic_boost_request) {
      turn_boost_pump_off(F("# turning off boost pump after periodic 30-minute run\n"));
      periodic_boost_request = false;
    } else {
      turn_boost_pump_off(F("# turning off boost pump due to time limit\n"));
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
        Serial.println(F("# turning off diverter valve power due to time limit"));
      }
    }
  }
  check_free_memory(F("monitor_diverter_valve exit"));
}

// This is called several times a second and updates the LCD display with the latest values and status.

void update_lcd_callback(void)
{
  check_free_memory(F("update_lcd.."));
 
  if (lcd != 0) {    
    char *valve_cbuf;
    
    lcd->setCursor(0 /* column */, 0 /* row */);
    if (diverter_valve_is_sending_water_to_roof()) {
      valve_cbuf = "Roof";
    } else {
      valve_cbuf = "Pool";
    }
    // First LCD line
    snprintf(cbuf, sizeof(cbuf), "%02d:%02d:%02d %4s %6s",
      hour(arduino_time), minute(arduino_time), second(arduino_time), 
      valve_cbuf,
      operating_mode_to_string(operating_mode));
    lcd->print(cbuf);

    // Second LCD line
    char pt1_lcd[5], pt2_lcd[5], ot_lcd[5];
    format_temperature(pool_temperature1_F, pt1_lcd, sizeof(pt1_lcd));
    format_temperature(pool_temperature2_F, pt2_lcd, sizeof(pt2_lcd));
    format_temperature(outside_temperature_F, ot_lcd, sizeof(ot_lcd));

    lcd->setCursor(0, 1);
    snprintf(cbuf, sizeof(cbuf), "%sF %sF %sF",
             pt1_lcd, pt2_lcd, ot_lcd);
    lcd->print(cbuf);
    
    // Third LCD Line
    lcd->setCursor(0, 2);
    char pressure_psi_str[8];

    dtostrf(pressure_psi, 4, 1, pressure_psi_str);
    snprintf(cbuf, sizeof(cbuf), "%s PSI  ", pressure_psi_str);
    lcd->print(cbuf);
    float main_pump_runtime_in_seconds = 0.0;
    float diverter_valve_in_roof_position_in_seconds = 0.0;

    // Fourth LCD Line
    lcd->setCursor(0, 3);
    
    if (main_pump_is_on()) {
      main_pump_runtime_in_seconds = (millis() - main_pump_on_off_time) / 1000.0;
    } 
    if (diverter_valve_is_sending_water_to_roof()) {
      diverter_valve_in_roof_position_in_seconds = (millis() - diverter_valve_in_roof_position_time) / 1000.0;
    }
    snprintf(cbuf, sizeof(cbuf), "%3u:%02u %3u:%02u", 
             (unsigned)(main_pump_runtime_in_seconds / 60.0), 
             (unsigned)main_pump_runtime_in_seconds % 60,
             (unsigned)(diverter_valve_in_roof_position_in_seconds / 60.0),
             (unsigned)diverter_valve_in_roof_position_in_seconds % 60);
             
    lcd->print(cbuf);
    
  }
  check_free_memory(F("update_lcd.. exit"));
}

// This is called once per second.  It samples the time, temperature, and pressure.  It records these in globals read by other
// tasks.

void read_time_and_sensor_inputs_callback(void)
{
  check_free_memory(F("read_time_and.."));
  arduino_time = now();
  
  unsigned long raw_pool_temperature1_volts = 0;
  unsigned long raw_pool_temperature2_volts = 0;
  unsigned long raw_outside_temperature_volts = 0;
  unsigned long raw_pressure_volts = 0;
  unsigned adc_samples = 20;

  // Reduce sample noise by taking a number of samples and using the arithmetic mean
  for (int i = 0 ; i < adc_samples ; i++) {
    raw_pool_temperature1_volts += analogRead(POOL_TEMPERATURE1_INPUT);
    raw_pool_temperature2_volts += analogRead(POOL_TEMPERATURE2_INPUT);
    raw_outside_temperature_volts += analogRead(OUTSIDE_TEMPERATURE_INPUT);
    raw_pressure_volts += analogRead(PRESSURE_INPUT);
  }
  
  raw_pool_temperature1_volts /= adc_samples;  
  raw_pool_temperature2_volts /= adc_samples;
  raw_outside_temperature_volts /= adc_samples;
  raw_pressure_volts /= adc_samples; 

  // See Arduino documents to understand the conversion of raw ADC values to a voltage
  float pool_temperature1_millivolts = (float)raw_pool_temperature1_volts * 5000.0 / 1023.0;

  // See the LM36 data shee to understand the conversion of millivolts to degrees C
  float pool_temperature1_C = (pool_temperature1_millivolts - 500.0) / 10.0;

  // Convert Centigrade to Fahrenheit
  float pool_temperature1_this_sample_F = pool_temperature1_C * 9.0 / 5.0 + 32.0;

  // EMA filter
  const float alpha = 0.08;
  if (pool_temperature1_F <= SENSOR_ERROR) {
    pool_temperature1_F = pool_temperature1_this_sample_F;
  }
  pool_temperature1_F = alpha * pool_temperature1_this_sample_F + (1.0 - alpha) * pool_temperature1_F;

  if (pool_temperature1_F < 32.0 || pool_temperature1_F > 105.0) {
    // Something is wrong with the sensor
    pool_temperature1_F = SENSOR_ERROR;
  }

  // See Arduino documents to understand the conversion of raw ADC values to a voltage
  float pool_temperature2_millivolts = (float)raw_pool_temperature2_volts * 5000.0 / 1023.0;

  // See the LM36 data shee to understand the conversion of millivolts to degrees C
  float pool_temperature2_C = (pool_temperature2_millivolts - 500.0) / 10.0;

  // Convert Centigrade to Fahrenheit
  float pool_temperature2_this_sample_F = pool_temperature2_C * 9.0 / 5.0 + 32.0;

  // EMA filter
  if (pool_temperature2_F <= SENSOR_ERROR) {
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

  float outside_temperature_millivolts = (float)raw_outside_temperature_volts * 5000.0 / 1023.0;
  float outside_temperature_C = (outside_temperature_millivolts - 500.0) / 10.0;

  // Convert to degrees F
  float outside_temperature_this_sample_F = outside_temperature_C * 9.0 / 5.0 + 32.0;
  if (outside_temperature_F <= SENSOR_ERROR) {
    outside_temperature_F = outside_temperature_this_sample_F;
  }
  outside_temperature_F = alpha * outside_temperature_this_sample_F + (1.0 - alpha) * outside_temperature_F;


  if (outside_temperature_F < 0.0 || outside_temperature_F > 150.0) {
    outside_temperature_F = SENSOR_ERROR;
  }
  float pressure_volts = raw_pressure_volts * 5.0 / 1023.0;
  pressure_volts -= 0.29;   // Pressure sensor reads 0.29 volts when pressure is zero
  if (pressure_volts < 0.0) {
    pressure_volts = 0.0;
  }
  pressure_psi = pressure_volts * 4.905; // Empircally derived calibration value
  check_free_memory(F("read_time_and.. exit"));
}

// Called periodically.  Sends relevant telemetry back over the USB serial connection to a host
// computer (e.g., M3 Mac Mini) that can aborb these messages and generate plots

void print_status_to_serial_callback(void) 
{
  check_free_memory(F("print_status_to.."));
  static float last_pool_temperature1_F;
  static float last_pool_temperature2_F;
  static float last_pressure_psi; 
  static bool last_pump;
  static bool last_diverter_valve_request;
  static bool last_to_roof; 
  static unsigned line_counter = 0;
  static unsigned skipped_record_counter = 0;

  bool pump = main_pump_is_on();
  bool boost = boost_pump_is_on();
  bool to_roof = diverter_valve_is_sending_water_to_roof();
  unsigned records_to_skip = (operating_mode == m_normal) ? 600 : 60;

  if (abs(pool_temperature1_F - last_pool_temperature1_F) > 1.5 ||
      abs(pool_temperature2_F - last_pool_temperature2_F) > 1.5 ||
      abs(pressure_psi - last_pressure_psi) > 0.5 ||
      pump != last_pump ||
      diverter_valve_request != last_diverter_valve_request ||
      to_roof != last_to_roof ||
      skipped_record_counter++ > records_to_skip) {
   last_pool_temperature1_F = pool_temperature1_F;
   last_pool_temperature2_F = pool_temperature2_F;
   last_pressure_psi = pressure_psi;
   last_pump = pump;
   last_diverter_valve_request = diverter_valve_request;
   last_to_roof = to_roof;
   
   skipped_record_counter = 0;
      
    if (line_counter == 0) {
        Serial.println(F("# Date     Time       Pl1  Pl2  Out  PSI  Pmp Bst Req Roof Mode"));
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

    // Format temperatures, showing "  -" for failed sensors
    char pressure_psi_str[8];
    dtostrf(pressure_psi, 4, 1, pressure_psi_str);

    char pt1_str[8];
    format_temperature(pool_temperature1_F, pt1_str, sizeof(pt1_str));

    char pt2_str[8];
    format_temperature(pool_temperature2_F, pt2_str, sizeof(pt2_str));

    char ot_str[8];
    format_temperature(outside_temperature_F, ot_str, sizeof(ot_str));
    
    snprintf(cbuf, sizeof(cbuf), " %s %s %s %s %3u %3u %3u %3u  %3u",
             pt1_str, pt2_str, ot_str, pressure_psi_str,
             pump,
             boost,
             diverter_valve_request,
             to_roof,
             (unsigned)operating_mode);
    Serial.println(cbuf);
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
  if (fail_message != nullptr) {
    Serial.println(fail_message);
  }
  delay(500); // Give the serial link time to propogate the error message before execution ends
  abort();
}

// Called frequently to poll for and act on received console input.  This recognizes characters
// that correspond to keys on the controller, 's' for select, '+' for the plus key, and '-' for the minus key.

void monitor_serial_console_callback(void)
{
  static char command_buf[20];

  while (Serial.available() > 0) {  // Check if data is available to read
    char received_char = Serial.read();  // Read one character
    int l = strlen(command_buf);
    if (l >= sizeof(command_buf) - 1) {
      Serial.println(F("# command buffer overflow"));
      command_buf[0] = '\0';
      continue;
    }
    if (received_char == '\n') {
      Serial.print(F("# Received: "));
      Serial.println(command_buf);

      switch (command_buf[0]) {   
        case 'd': // Set date command, "d year-month-day", e.g., "t 2025-05-23" to set the date
        {
          int year, month, day;
          sscanf(command_buf + 2, "%d-%d-%d", &year, &month, &day);
          setTime(hour(arduino_time), minute(arduino_time), second(arduino_time), day, month, year);
          date_set = true;
        }
        break; 

        case 't': // Set time command, "t hh:mm:ss", e.g., "t 9:23:33" to set the time.
        {
          int hh, mmin, ss;
          sscanf(command_buf + 2, "%d:%d:%d", &hh, &mmin, &ss);
          setTime(hh, mmin, ss, day(arduino_time), month(arduino_time), year(arduino_time));
          time_set = true;
        }
        break;

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
          Serial.println(F("# choices are: 'd year-month-day', 't hour:minute:second','s', +, -"));
          break;
      }
      command_buf[0] = '\0';
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
          if (quad_lv_relay == nullptr) {
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
  if (quad_lv_relay == nullptr) {
    Serial.println(F("# Warning, quad relay board not found"));
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
  wdt_disable();
  setup_arduino_pins();
  analogReference(DEFAULT);

  Wire.begin();
//  Wire.setClock(400000);  Not sure if this matters.

  uint8_t mcusr = MCUSR;
  MCUSR = 0;
 
  Serial.begin(SERIAL_BAUD);
  UCSR0A = UCSR0A | (1 << TXC0); //Clear Transmit Complete Flag
  
  if (mcusr & (1 << BORF)) {
    Serial.println("Reset cause: Brown-out");
  }
  if (mcusr & (1 << WDRF)) {
    Serial.println("Reset cause: Watchdog");
  }
  
  if (!date_set && !time_set) {
    setup_arduino_time();
  }

  Serial.println(F("\n# alert Pool pump and valve controller. " __DATE__ " " __TIME__));

  setup_i2c_bus(); // This sets "quad_lv_relay" and "lcd"
  setup_lcd();

#ifndef POLL_KEYS
  PCICR |= (1 << PCIE0);                                        // Enable Pin Change Interrupt for PORTB (PCIE0) 
  PCICR &= ~((1 << PCIE2) | (1 << PCIE1));                      // Disable PCINT2 (PORTD) and PCINT1 (PORTC)
  PCMSK0 |= (1 << PCINT1) | (1 << PCINT2) | (1 << PCINT3) | (1 << PCINT4);  // Enable interrupts for D9–D12
#endif

  // If we start with the diverter valve direction relay in the position for sending water to roof, but there is no request for this
  // then ensure that the diverter valve is sending water back to the pool.
  
  diverter_valve_request = digitalRead(DIVERTER_REQUEST_INPUT_PIN) == LOW;
  if (diverter_valve_is_sending_water_to_roof() &&  !diverter_valve_request && !main_pump_is_on()) {
    Serial.println(F("# Turning diverter valve to pool on startup"));
    turn_diverter_valve_transformer_on();
    set_diverter_valve_to_return_water_to_pool();
  }

  // On reboot since we don't know how long it has been since the pool pump ran to filter pool water
  // run it now.  This also makes testing quicker.
  turn_main_pump_on(F("# initial filtering\n"));

  // Ditto for the boost pump
  periodic_boost_request = true;
  
  manual_main_pump_request = true;
  wdt_enable(WDTO_8S);  // 8 second watchdog
}

/*
   This is the function that the Arduino run time system calls repeatedly after it has called setup().
*/
void loop(void)
{
  wdt_reset();  // Pet the watchdog
  // All code after setup() executes inside of tasks, so the only thing to do here is to call the task scheduler's execute() method.
  ts.execute();
}
