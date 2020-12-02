// JDL v2.53 (26.11.2020)
// CC3D & Revo Compatible
/*

   Copyright (c) 2011.  All rights reserved.
   An Open Source Arduino based OSD and Camera Control project.

   Program  : ArduCAM-OSD (Supports the variant: minimOSD)
   Version  : V1.9, 14 February 2012
   Author(s): Sandro Benigno
   Coauthor(s):
   Jani Hirvinen   (All the EEPROM routines)
   Michael Oborne  (OSD Configutator)
   Mike Smith      (BetterStream and Fast Serial libraries)
   Special Contribuitor:
   Andrew Tridgell by all the support on MAVLink
   Doug Weibel by his great orientation since the start of this project
   Contributors: James Goppert, Max Levine
   and all other members of DIY Drones Dev team
   Thanks to: Chris Anderson, Jordi Munoz
   Mods and new functions added by: Julian Lilov


   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <http://www.gnu.org/licenses/>

 */

/* ************************************************************ */
/* **************** MAIN PROGRAM - MODULES ******************** */
/* ************************************************************ */

#undef PROGMEM
#define PROGMEM __attribute__((section(".progmem.data")))

#undef PSTR
#define PSTR(s) \
    (__extension__({ static prog_char __c[] PROGMEM = (s); &__c[0]; } \
                   ))


/* **********************************************/
/* ***************** INCLUDES *******************/

// #define membug
// #define FORCEINIT  // You should never use this unless you know what you are doing


// AVR Includes
#include <FastSerial.h>
#include <AP_Common.h>
#include <AP_Math.h>
#include <math.h>
#include <inttypes.h>
#include <avr/pgmspace.h>
// Get the common arduino functions
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "wiring.h"
#endif
#include <EEPROM.h>

#ifdef membug
#include <MemoryFree.h>
#endif

// Configurations
#include "OSD_Config.h"
#include "ArduCam_Max7456.h"
#include "OSD_Vars.h"
#include "OSD_Func.h"

// JRChange: OpenPilot UAVTalk:
#ifdef PROTOCOL_UAVTALK
#include "UAVTalk.h"
#endif
// JRChange: Flight Batt on MinimOSD:
#ifdef FLIGHT_BATT_ON_MINIMOSD
#include "FlightBatt.h"
#endif

/* *************************************************/
/* ***************** DEFINITIONS *******************/

// OSD Hardware
// #define ArduCAM328
#define MinimOSD

#define TELEMETRY_SPEED 57600 // How fast our MAVLink telemetry is coming to Serial port

#define BOOTTIME        2000   // Time in milliseconds that we show boot loading bar and wait user input

// Objects and Serial definitions
FastSerialPort0(Serial);
OSD osd; // OSD object


/* **********************************************/
/* ***************** SETUP() *******************/

void setup()
{
#ifdef ArduCAM328
    pinMode(10, OUTPUT); // USB ArduCam Only
#endif
    pinMode(MAX7456_SELECT, OUTPUT); // OSD CS

    Serial.begin(TELEMETRY_SPEED);

#ifdef membug
    Serial.println(freeMem());
#endif

    // Prepare OSD for displaying
    unplugSlaves();
    osd.init();

    // Start
    startPanels();
    delay(1000);

    // OSD debug for development (Shown at start)
#ifdef membug
    osd.setAndOpenPanel(1, 1);
//    osd.openPanel();
    osd.printf("%i", freeMem());
    osd.closePanel();
#endif

    // Just to easy up development things
#ifdef FORCEINIT
    InitializeOSD();
#endif

// JRChange:
#if 0
    // Check EEPROM to see if we have initialized it already or not
    // also checks if we have new version that needs EEPROM reset
    if (readEEPROM(CHK1) + readEEPROM(CHK2) != VER) {
        osd.setAndOpenPanel(6, 9);
//        osd.openPanel();
        osd.printf_P(PSTR("missing/old config"));
        osd.closePanel();
        InitializeOSD();
    }
#endif
/*
#if defined FLIGHT_BATT_ON_MINIMOSD //  || defined SAFETY_RADIUS
// JRChange: Flight Batt on MinimOSD:
    // Check EEPROM to see if we have initialized the battery values already
    if (readEEPROM(BATT_CHK) != BATT_VER) {
        writeBattSettings();
    }
#endif    
*/

//#ifdef CHECK_FIRMWARE_TYPE    // JDL
//    if (readEEPROM(firmware_type_ADDR) != FIRMWARE_TYPE) {
//        writeEEPROM(firmware_type_ADDR, FIRMWARE_TYPE);
//    }
//#endif

// Get correct panel settings from EEPROM
    readSettings();
    for (panel = 0; panel < npanels; panel++) {
        readPanelSettings();
    }
    panel = 0; // set panel to 0 to start in the first navigation screen
    // Show bootloader bar... No, don't, save code 

#ifdef FLIGHT_BATT_ON_REVO
//    delay(1000); // To give small extra waittime to users
    Serial.flush();
  #ifdef LOGO_AT_BOOT
    delay(BOOTTIME+1000); // To give small extra waittime to users
  #endif  
#else
    loadBar();
#endif    
    
// JRChange: Flight Batt on MinimOSD:
#ifdef FLIGHT_BATT_ON_MINIMOSD
    flight_batt_init();
#endif

    // House cleaning, clear display and enable timers
    osd.clear();

    loop10hz_prevmillis = millis();

} // END of setup();


/* ***********************************************/
/* ***************** MAIN LOOP *******************/

// Mother of all happenings, The loop()
// As simple as possible.
void loop()
{
//  if (uavtalk())
//    update_all();
//      
//  if ((loop10hz_prevmillis + 100) < millis() ) {
//    loop10hz_prevmillis = millis();
//    update_all();
//  }

  if (uavtalk_read() || ((loop10hz_prevmillis + 100) < millis() )) {
    loop10hz_prevmillis = millis();
    update_all();
  }

}

/* *********************************************** */
/* ******** functions used in main loop() ******** */
void update_all() // duration is up to approx. 10ms depending on choosen display features
{
  #ifdef FLIGHT_BATT_ON_MINIMOSD
    flight_batt_read();
  #endif

  #ifdef JR_SPECIALS
  #if not defined COMBINED_HEADING_AND_ROSE
    calculateCompassPoint(); // calculate the compass point which is shown in panHeading
  #endif  // COMBINED_HEADING_AND_ROSE

  #if defined TEMP_SENSOR_LM335Z_ESC || defined TEMP_SENSOR_LM335Z_MOTOR || defined TEMP_SENSOR_LM335Z_AMBIENT
    analogReference(DEFAULT); // Power supply: 5V
  #endif    

//    temp_in_celsius = analogRead(0) * (5.0 / 1024) * 100 - 2.5 - 273.15;
//    revo_baro_temp = revo_baro_temp * .8 + (analogRead(TEMP_SENSOR_PIN) * 0.4849609 - 275.65) * .2;    // 4.966V
//    revo_baro_temp = revo_baro_temp * .8 + (analogRead(TEMP_SENSOR_PIN) * 0.4882812 - 275.65) * .2;    // 5.000V
//    revo_baro_temp = revo_baro_temp * .8 + (analogRead(TEMP_SENSOR_PIN) * 0.4546875 - 275.65) * .2;    // 4.656V
//    revo_baro_temp = revo_baro_temp * .8 + (analogRead(TEMP_SENSOR_PIN) * 0.4750977 - 275.65) * .2;    // 4.865V


  #if defined TEMP_SENSOR_LM335Z_ESC

  #ifdef TEMP_SENSOR_CALIBRATION_VOLTAGE_4_966V
    esc_temp = esc_temp * .8 + (analogRead(TEMP_SENSOR_PIN_ESC) * 0.0969922 - 55.13) + esc_temp_correction;          // 4.966V
  #else    
    #ifdef TEMP_SENSOR_CALIBRATION_VOLTAGE_4_656V
      esc_temp = esc_temp * .8 + (analogRead(TEMP_SENSOR_PIN_ESC) * 0.0909375 - 55.13) + esc_temp_correction;          // 4.656V
    #else  
      esc_temp = esc_temp * .8 + (analogRead(TEMP_SENSOR_PIN_ESC) * 0.09765624 - 55.13) + esc_temp_correction;          // 5.00V
    #endif    
  #endif  

  #endif    
  
  
  #if defined TEMP_SENSOR_LM335Z_MOTOR

  #ifdef TEMP_SENSOR_CALIBRATION_VOLTAGE_4_966V
    motor_temp = motor_temp * .8 + (analogRead(TEMP_SENSOR_PIN_MOTOR) * 0.0969922 - 55.13) + motor_temp_correction;          // 4.966V
  #else    
    #ifdef TEMP_SENSOR_CALIBRATION_VOLTAGE_4_656V
      motor_temp = motor_temp * .8 + (analogRead(TEMP_SENSOR_PIN_MOTOR) * 0.0909375 - 55.13) + motor_temp_correction;          // 4.656V
    #else  
      motor_temp = motor_temp * .8 + (analogRead(TEMP_SENSOR_PIN_MOTOR) * 0.09765624 - 55.13) + motor_temp_correction;          // 5.00V
    #endif  
  #endif    

  #endif    
  

  #if defined TEMP_SENSOR_LM335Z_AMBIENT

  #ifdef TEMP_SENSOR_CALIBRATION_VOLTAGE_4_966V
    ambient_temp = ambient_temp * .8 + (analogRead(TEMP_SENSOR_PIN_AMBIENT) * 0.0969922 - 55.13) + ambient_temp_correction;          // 4.966V
  #else    
    #ifdef TEMP_SENSOR_CALIBRATION_VOLTAGE_4_656V
      ambient_temp = ambient_temp * .8 + (analogRead(TEMP_SENSOR_PIN_AMBIENT) * 0.0909375 - 55.13) + ambient_temp_correction;          // 4.656V
    #else  
      #ifdef TEMP_SENSOR_CALIBRATION_VOLTAGE_4_865V
        ambient_temp = ambient_temp * .8 + (analogRead(TEMP_SENSOR_PIN_AMBIENT) * 0.09501953125 - 55.13) + ambient_temp_correction;          // 4.865V
      #else  
        ambient_temp = ambient_temp * .8 + (analogRead(TEMP_SENSOR_PIN_AMBIENT) * 0.09765624 - 55.13) + ambient_temp_correction;          // 5.00V
      #endif  
    #endif    
  #endif  

  #endif    


  #endif  // JR_SPECIALS

    updateTravelDistance(); // calculate travel distance
    setHeadingPattern(); // generate the heading pattern
    setHomeVars(osd); // calculate and set Distance from home and Direction to home
    writePanels(); // writing enabled panels (check OSD_Panels Tab)
}


void unplugSlaves()
{
    // Unplug list of SPI
#ifdef ArduCAM328
    digitalWrite(10, HIGH); // unplug USB HOST: ArduCam Only
#endif
    digitalWrite(MAX7456_SELECT, HIGH); // unplug OSD
}

/*
uint8_t _pinMask = 0;         // Pin bitmask.
volatile uint8_t *_pinOutput; // Output port register

void NewTone(uint8_t pin, unsigned long frequency, unsigned long length) {
  uint8_t prescaler = _BV(CS10);                 // Try using prescaler 1 first.
  unsigned long top = F_CPU / frequency / 4 - 1; // Calculate the top.
  if (top > 65535) {                             // If not in the range for prescaler 1, use prescaler 256 (61 Hz and lower @ 16 MHz).
    prescaler = _BV(CS12);                       // Set the 256 prescaler bit.
    top = top / 256 - 1;                         // Calculate the top using prescaler 256.
  }

  if (_pinMask == 0) {
    _pinMask = digitalPinToBitMask(pin);                    // Get the port register bitmask for the pin.
    _pinOutput = portOutputRegister(digitalPinToPort(pin)); // Get the output port register for the pin.
    uint8_t *_pinMode = (uint8_t *) portModeRegister(digitalPinToPort(pin)); // Get the port mode register for the pin.
    *_pinMode |= _pinMask; // Set the pin to Output mode.
  }

  ICR1    = top;                     // Set the top.
  if (TCNT1 > top) TCNT1 = top;      // Counter over the top, put within range.
  TCCR1B  = _BV(WGM13)  | prescaler; // Set PWM, phase and frequency corrected (ICR1) and prescaler.
  TCCR1A  = _BV(COM1B0);
  TIMSK1 |= _BV(OCIE1A);             // Activate the timer interrupt.
}

void noNewTone(uint8_t pin) {
  TIMSK1 &= ~_BV(OCIE1A);   // Remove the timer interrupt.
  TCCR1B  = _BV(CS11);      // Default clock prescaler of 8.
  TCCR1A  = _BV(WGM10);     // Set to defaults so PWM can work like normal (PWM, phase corrected, 8bit).
  *_pinOutput &= ~_pinMask; // Set pin to LOW.
  _pinMask = 0; // Flag so we know note is no longer playing.
}

ISR(TIMER1_COMPA_vect) { // Timer interrupt vector.
  *_pinOutput ^= _pinMask; // Toggle the pin state.
}
*/

