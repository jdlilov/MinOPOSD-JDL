// JDL
//
//
//

/*

   refactoring started

   TODO:

        refactor:
                switchPanels
                casts (double) -> (float) etc.

        maybe implement usage of panCallsign

 */


#include "OSD_Config.h"

#ifdef FLIGHT_BATT_ON_MINIMOSD
#include "FlightBatt.h"
#endif

#ifdef PACKETRXOK_ON_MINIMOSD
#include "PacketRxOk.h"
#endif

#ifdef SBUS_RX
  #define PWM_LO              400     //1200 // 400   // [us]	PWM low value
  #define PWM_HI              1500    //1800 //1500   // [us]	PWM high value
#else  
  #define PWM_LO              1200                    // [us]	PWM low value
  #define PWM_HI              1800                    // [us]	PWM high value
#endif

#define PWM_HI_VALID        3000    // [us]	PWM high value valid. Higher values occurs when Rx is not powered.
#define PWM_OFFSET          100     // [us]	PWM offset for detecting stick movement

#define SETUP_TIME          30000   // [ms]	time after boot while we can enter the setup menu
#define SETUP_DEBOUNCE_TIME 500 // [ms]	time for RC-TX stick debouncing

#if defined BATTTYPE_SELECTION
  #define SETUP_LOWEST_MENU   1       // lowest shown setup menue item
#else
  #ifdef SAFETY_RADIUS
    #define SETUP_LOWEST_MENU   2       // lowest shown setup menue item
  #else
    #define SETUP_LOWEST_MENU   4       // lowest shown setup menue item
  #endif
#endif

#ifndef FLIGHT_BATT_ON_MINIMOSD
  #define SETUP_HIGHEST_MENU  4       // highest shown setup menue item
#else
  #define SETUP_HIGHEST_MENU  13      // highest shown setup menue item
#endif

#define WARN_FLASH_TIME     700    // [ms]	time with which the warnings are flashing
#define WARN_RECOVER_TIME   3000    // [ms]	time we stay in the first panel after last warning, if not warning_timeout passed
#define WARN_TIMEOUT_TIME   6000    // [ms]	time we stay in the first panel after initial warning (warning_timeout)

#ifdef JR_SPECIALS
  #if defined TEMP_SENSOR_LM335Z_ESC || defined TEMP_SENSOR_LM335Z_MOTOR
    #define WARN_MAX            9       // number of implemented warnings
  #else
    #ifdef SAFETY_RADIUS
      #define WARN_MAX          8       // number of implemented warnings
    #else
      #ifdef REVO_ADD_ONS
        #define WARN_MAX        7       // number of implemented warnings
      #else
        #define WARN_MAX        6       // number of implemented warnings
      #endif   
    #endif    
  #endif
#else
  #define WARN_MAX              4       // number of implemented warnings
#endif

#ifdef FIXED_WING
  #define TIME_RESET_THROTTLE	60	// [%]	throttle position above which the on time is set to 00:00 (For Fixed Wing)
  #define FIXEDWING_MIN_THROTTLE 6
#else
  #define TIME_RESET_THROTTLE	6	// [%]	throttle position above which the on time is set to 00:00 (For Multirotor)
#endif  


/******* GLOBAL VARS *******/

static boolean setup_menu_active = false;
static boolean warning_active    = false;
static unsigned long warning_timeout   = 0;

static unsigned long warn_text_timer   = 0;
static uint8_t warning_type = 0;

static float convert_speed          = 3.6;
static float convert_length         = 1.0;
static int16_t convert_length_large = 1000;
static uint8_t unit_speed           = 0x81;
static uint8_t unit_length          = 0x8D;
static uint8_t unit_length_large    = 0xFD;

static int16_t chan1_raw_middle     = 0;
static int16_t chan2_raw_middle     = 0;

/******* MAIN FUNCTIONS *******/


/******************************************************************/
// Panel  : startPanels
// Output : Logo panel and initialization
/******************************************************************/
void startPanels()
{
    osd.clear();
#ifdef LOGO_AT_BOOT    
    panLogo(); // display logo
#endif
//    set_converts(); // initialize the units
}


/******************************************************************/
// Panel  : writePanels
// Output : Write the panels
/******************************************************************/
void writePanels()
{
#ifdef JR_SPECIALS

    // reset osd_home to current position when PITCH/ROLL are at max position
    if (!setup_menu_active && osd_armed < 2 && osd_got_home == 1 && chan1_raw > PWM_HI && chan2_raw < PWM_LO) {
        osd_home_lat = osd_lat;
        osd_home_lon = osd_lon;
        osd_home_alt = osd_alt;
#ifdef REVO_ADD_ONS
        osd_baro_home_alt = revo_baro_alt;
#endif        
        reset_home_flag = 1;
    }
#endif


#ifdef JR_SPECIALS	// Time and travel distance and baro home altitide reset when measured current > TIME_RESET_AMPERE for the first time
    
    if (engine_start_time == 0 && osd_throttle > TIME_RESET_THROTTLE && osd_armed > 1)
    {
        engine_start_time = millis();                                  // engine_start_time - exact millis when the engine is initially started
        flight_A = 0;                                                  // start_A - exact milliamps used till the moment of engine initial start
        start_A = osd_total_A;                                         // flight_time - duration of flight in millis
        start_batp = osd_battery_remaining_percent_V;                  // flight_A - used milliamps in flight
	osd_travel_distance = 0;
#ifdef REVO_ADD_ONS
        osd_baro_home_alt = revo_baro_alt;
#endif        
    }
    if (engine_start_time > 0)
    {
    #ifdef FIXED_WING      
      if (osd_armed > 1)
    #else
      if (((osd_throttle > TIME_RESET_THROTTLE) || (osd_aswa > 0)) && osd_armed > 1)
    #endif  
      {
        flight_time = (int) ((millis() - engine_start_time) / 1000);
        flight_A = osd_total_A - start_A;
      }
      else
      {
        engine_start_time = millis() - ( (long) flight_time * 1000);
        start_A = osd_total_A - flight_A;
      }
    }
    else {
        flight_time = (int) (millis() / 1000);
        flight_A = 0;
        start_batp = osd_battery_remaining_percent_V;
    }

#else
    flight_time = (int) (millis() / 1000);
    flight_A = 0;
#endif

    switchPanels(); // switch panels

    if (setup_menu_active) {
        panSetup();
    } else {
        if (ISd(0, Warn_BIT)) {
          panWarn(panWarn_XY[0][0], panWarn_XY[1][0]); // ever check/display warnings
        }
        if ((panel < npanels) || (warning_active && (millis() <= warning_timeout))) { // first or second panel or warning is active before warning_timeout 
            old_panel = panel;
            if (panel == npanels) {
              panel = 0;
            }
            // these GPS related panels are active under all circumstances
            if (ISa(panel, GPSats_BIT)) {
                panGPSats(panGPSats_XY[0][panel], panGPSats_XY[1][panel]); // number of visible sats
            }
            if (ISa(panel, GPL_BIT)) {
                panGPL(panGPL_XY[0][panel], panGPL_XY[1][panel]); // sat fix type
            }

#ifdef REVO_ADD_ONS
            if (ISd(panel, Climb_BIT)) {
                panClimb(panClimb_XY[0][panel], panClimb_XY[1][panel]);
            }
#endif            

            // these GPS related panels are active if GPS was valid before
            if (osd_got_home) {
                if (ISa(panel, GPS_BIT)) {
                    panGPS(panGPS_XY[0][panel], panGPS_XY[1][panel]); // lat & lon
                }
                if (ISb(panel, HDis_BIT)) {
                    panHomeDis(panHomeDis_XY[0][panel], panHomeDis_XY[1][panel]);
                }
                if (ISb(panel, HDir_BIT)) {
                    panHomeDir(panHomeDir_XY[0][panel], panHomeDir_XY[1][panel]);
                }

#ifdef PATHPLAN_WAYPOINTS_SUPPORT
                if (ISb(panel, WDir_BIT)) {
                    panWPDir(panWPDir_XY[0][panel], panWPDir_XY[1][panel]);
                }
#endif


#if defined FLIGHT_BATT_ON_MINIMOSD || defined FLIGHT_BATT_ON_REVO
  #ifdef SAFETY_RADIUS
                if (ISb(panel, SR_EGE_BIT)) {
                    panSR_EGE(panSR_EGE_XY[0][panel], panSR_EGE_XY[1][panel]);
                }
  #endif
#endif
            }

#ifdef REVO_ADD_ONS
            if (ISc(panel, Halt_BIT)) {
                panHomeAlt(panHomeAlt_XY[0][panel], panHomeAlt_XY[1][panel]);
            }
#endif

#ifdef FIXED_WING
            if (ISc(panel, As_BIT)) {
                panAirSpeed(panAirSpeed_XY[0][panel], panAirSpeed_XY[1][panel]);
            }
#endif  

            // these GPS related panels are active if GPS was valid before and we have a sat fix
            if (osd_got_home && osd_fix_type > 1) {
#ifndef REVO_ADD_ONS
                if (ISc(panel, Halt_BIT)) {
                    panHomeAlt(panHomeAlt_XY[0][panel], panHomeAlt_XY[1][panel]);
                }
                if (ISd(panel, Climb_BIT)) {
                    panClimb(panClimb_XY[0][panel], panClimb_XY[1][panel]);
                }
#endif                
                if (ISe(panel,DIST_BIT)) {
                    panDistance(panDistance_XY[0][panel], panDistance_XY[1][panel]);
                }
                if (ISc(panel, Alt_BIT)) {
                    panAlt(panAlt_XY[0][panel], panAlt_XY[1][panel]);
                }
                if (ISc(panel, Vel_BIT)) {
                    panVel(panVel_XY[0][panel], panVel_XY[1][panel]);
                }
                
#ifndef FIXED_WING
                if (ISc(panel, As_BIT)) {
                    panAirSpeed(panAirSpeed_XY[0][panel], panAirSpeed_XY[1][panel]);
                }
#endif  
            }


            if ((osd_got_home && osd_fix_type > 1)
#ifdef REVO_ADD_ONS
//                                                   || (osd_mag_status == 1) || (osd_mag_status == 2) || (osd_mag_status == 3)
                                                     || ((uint8_t)(osd_mag_status-1) <= 2)
#endif                                                   
                                                  ) {
#ifndef COMBINED_HEADING_AND_ROSE                                                    
                if (ISb(panel, Head_BIT)) {
                    panHeading(panHeading_XY[0][panel], panHeading_XY[1][panel]);
                }
#endif                
                if (ISb(panel, Rose_BIT)) {
                    panRose(panRose_XY[0][panel], panRose_XY[1][panel]);
                }
            }

            if (ISd(panel, RSSI_BIT)
#ifdef REVO_ADD_ONS
                                                   && (oplm_rssi != 0)
#endif                                                   
                                                  ) {
                panRSSI(panRSSI_XY[0][panel], panRSSI_XY[1][panel]);
            }

            if (ISa(panel, Rol_BIT)) {
                panRoll(panRoll_XY[0][panel], panRoll_XY[1][panel]);
            }
            if (ISa(panel, Pit_BIT)) {
                panPitch(panPitch_XY[0][panel], panPitch_XY[1][panel]);
            }
            if (ISc(panel, Thr_BIT)) {
                panThr(panThr_XY[0][panel], panThr_XY[1][panel]);
            }
            if (ISc(panel, FMod_BIT)) {
                panFlightMode(panFMod_XY[0][panel], panFMod_XY[1][panel]);
            }
            if (ISa(panel, BatA_BIT)) {
                panBatt_A(panBatt_A_XY[0][panel], panBatt_A_XY[1][panel]);
            }
            if (ISc(panel, CurA_BIT)) {
                panCur_A(panCur_A_XY[0][panel], panCur_A_XY[1][panel]);
            }
            if (ISa(panel, Bp_BIT)) {
                panBatteryPercent(panBatteryPercent_XY[0][panel], panBatteryPercent_XY[1][panel]);
            }
            if (ISe(panel, TEMP_BIT)) {
//                if (osd_temp_mode == 1) {
                panBaroTemp(panTemp_XY[0][panel], panTemp_XY[1][panel]);
//                } else {
//                panTxPID(panTemp_XY[0][panel], panTemp_XY[1][panel]);
//                }
            }
            if (ISb(panel, Time_BIT)) {
                panTime(panTime_XY[0][panel], panTime_XY[1][panel]);
            }

            if (ISc(panel, Hor_BIT)) {
                panHorizon(panHorizon_XY[0][panel], panHorizon_XY[1][panel]);
            }
            panel = old_panel;
        } else { // off panel
            panOff();
        }
    }

#ifdef membug
    // OSD debug for development
    osd.setAndOpenPanel(13, 4);
//    osd.openPanel();
    osd.printf("%i", freeMem());
    osd.closePanel();
#endif
}


/******************************************************************/
// Panel  : switchPanels
// Output : Switch between panels
// TODO   : REFACTOR
/******************************************************************/
void switchPanels()
{
    if (osd_chan7_raw > PWM_HI && osd_chan7_raw < PWM_HI_VALID) {
        if (osd_armed < 1) {
            setup_menu_active = true;
        } 
        else 
            if (!setup_menu_active && (!warning_active || (warning_active && (millis() > warning_timeout) && (millis() > warn_text_timer) && !warning_type))) {
                osd.clear();
            }
        panel = npanels; // off panel
    }
    else 
        if (osd_chan7_raw < PWM_LO && panel != 0) {
            setup_menu_active = false;
            osd.clear();
            panel = 0; // first panel
        } 
        else 
            if (osd_chan7_raw >= PWM_LO && osd_chan7_raw <= PWM_HI && panel != 1 && (!warning_active || (warning_active && (millis() > warning_timeout)))) {
                setup_menu_active = false;
                osd.clear();
                panel = 1; // second panel
            }
}


/******* SPECIAL PANELS *******/


/******************************************************************/
// Panel  : panOff
// Needs  : -
// Output : -
/******************************************************************/
void panOff(void)
{
//#ifdef JR_SPECIALS
// SEARCH GLITCH
//    panGPS(panGPS_XY[0][0], panGPS_XY[1][0]);
//    panRSSI(panRSSI_XY[0][0], panRSSI_XY[1][0]);
//    panGPL(panGPL_XY[0][0], panGPL_XY[1][0]);
//    panFlightMode(panFMod_XY[0][0], panFMod_XY[1][0]);
//    panThr(panThr_XY[0][0], panThr_XY[1][0]);
//#endif
}


/******************************************************************/
// Panel  : panWarn
// Needs  : X, Y locations
// Output : Warnings if there are any
/******************************************************************/
void panWarn(int first_col, int first_line)
{
    static char *warning_string;
    static uint8_t last_warning_type        = 1;
    static unsigned long warn_recover_timer = 0;
    int cycle;

    if (millis() > warn_text_timer) { // if the text or blank text has been shown for a while
        if (warning_type) { // there was a warning, so we now blank it out for a while
            last_warning_type = warning_type; // save the warning type for cycling
            warning_type = 0;
            warning_string    = "            ";                    // blank the warning
            warn_text_timer   = millis() + WARN_FLASH_TIME / 2;   // set clear warning time
        } 
        else {
            cycle = last_warning_type; // start the warning checks cycle where we left it last time
            do { // cycle through the warning checks
                if (++cycle > WARN_MAX) {
                    cycle = 1;
                }
                
                switch (cycle) {

                case 1: // DISARMED
                    if (osd_armed < 2) {
                        warning_type   = cycle;
                        if (osd_armed == 1) {
                            warning_string = " arming ... ";
                        }
                        else {
                            if (osd_atti_status == 1 && osd_stab_status == 1 
                            #ifndef REVO_ADD_ONS 
                                && osd_cpu_status == 1 && osd_stack_status == 1 && osd_memory_status == 1
                            #else
                                && (osd_mag_status < 3)
                            #endif
                              ) {
                                warning_string = " " "\xEC" " disarmed " ;
                            }
                            else {
                                warning_string = " " "\xEB" " disarmed ";
                            }
                        }
                    }
                    else {
                        if (osd_stab_status != 1) {
                            warning_type = cycle;
                            warning_string = " stab alarm ";
                        }
                        else {
                            if ((osd_atti_status != 1) && (osd_armed < 2)) {    // This avoids annoying announcement of erroneous ATTI alarm, caused by "THE BUG" in INS13 during flight
                                warning_type = cycle;
                                warning_string = " atti alarm ";
                            }
                        }
                    }
                    break;

                case 2: // No telemetry communication
                    #ifndef STALL_WARNING
                        if (uavtalk_state() != TELEMETRYSTATS_STATE_CONNECTED) {
                            warning_type   = cycle;
                            warning_string = " no tel com ";
                        }
                    #else
                        if ((osd_armed > 1) && ((long)osd_travel_distance > 50) && (osd_airspeed < stall_threshold)) {
                            warning_type   = cycle;
                            warning_string = "stall alarm!";
                        }
                    #endif                    
                    break;

                case 3: // NO GPS FIX
                    if (osd_fix_type < 2 && osd_got_home) { // to allow flying in the woods (what I really like) without this annoying warning,
                        warning_type   = cycle;                   // this warning is only shown if GPS was valid before (osd_got_home)
                        warning_string = " no gps fix ";
                    }
                    break;
                case 4: // BATT LOW
                    if (osd_vbat_A < ((300 + (battv * batt_type)) * num_cells / 100.0)) {
                        warning_type   = cycle;
                        warning_string = "  batt low  ";
                    }
                    break;

#ifdef JR_SPECIALS
                case 5: // Reset Home
                    if (reset_home_flag == 1) {
                        warning_type   = cycle;
                        warning_string = " reset home ";
                        reset_home_flag = 0;
                    }
                    break;

  #ifndef REVO_ADD_ONS
                case 6: // CPU status or STACK status or MEMORY status or RSSI status
                    if (osd_cpu_status > 1) {
                        if ((osd_cpu_status == 2) && (osd_armed < 2)){
                            warning_type   = cycle;
                            warning_string = "cpu  warning";
                        }
                        else {
                            if (osd_cpu_status == 3) {
                                warning_type   = cycle;
                                warning_string = "cpu critical";
                            }
                        }
                    }

                    if (osd_memory_status > 1) {
                        warning_type   = cycle;
                        warning_string = " memory low ";
                    }

                    if (osd_stack_status > 1) {
                        warning_type   = cycle;
                        warning_string = "stack ovrflw";
                    }
                    
                  #ifdef RSSI_ON_INPUT_CHANNEL
                    if (rssi < rssi_warn_level) {
                        warning_type   = cycle;
                        warning_string = "  rssi low  ";
                    }
                  #endif  
                    break;
  #else
                case 6: // RSSI LOW
//                    if (ISd(0, RSSI_BIT) || ISd(1, RSSI_BIT)) 
                      {
                      if ((oplm_rssi < oplm_rssi_warn_level || oplm_linkquality < oplm_linkq_warn_level) && oplm_rssi != 0) {
                        warning_type   = cycle;
                        warning_string = "link quality";
                      }
                    }
                    break;

                case 7: // Mag status, only while disarmed and used (preflight check)
                    if (osd_armed < 2 && osd_mag_status > 1) {
                        warning_type   = cycle;
                        if (osd_mag_status == 2) {
                            warning_string = "mag  warning";
                        }
                        else {
                            if (osd_mag_status == 3) {
                                warning_string = "mag critical";
                            }
                            else {
                                if (osd_mag_status == 4) {
                                    warning_string = " mag  error ";
                                }
                            }
                        }
                    }
                    break;
  #endif

  #ifdef SAFETY_RADIUS
                case 8: // Go Home! warning
                    if ((osd_armed > 1) && ( 

    #if defined LT_RADIUS_ESTIMATION && defined ST_RADIUS_ESTIMATION && defined BATP_RADIUS_ESTIMATION // 3
                                            (lt_safety_delta < 0) || (batp_safety_delta < 0) || (st_safety_delta < 0)
    #else                                        
      #if defined LT_RADIUS_ESTIMATION && defined BATP_RADIUS_ESTIMATION // 2
                                            (lt_safety_delta < 0) || (batp_safety_delta < 0)
      #else
        #if defined LT_RADIUS_ESTIMATION && defined ST_RADIUS_ESTIMATION // 1
                                            (lt_safety_delta < 0) || (st_safety_delta < 0)
        #else                                            
          #if defined BATP_RADIUS_ESTIMATION // 4
                                            (batp_safety_delta < 0)
          #else
            #if defined LT_RADIUS_ESTIMATION // 5
                                            (lt_safety_delta < 0)
            #endif // 5                                
          #endif // 4
        #endif // 1
      #endif // 2
    #endif // 3

                                             )) {
                        warning_type   = cycle;
                        warning_string = "go home now!";
                    }
                    break;
  #endif                 

  #if defined TEMP_SENSOR_LM335Z_ESC || defined TEMP_SENSOR_LM335Z_MOTOR
                case 9: // Overheating!
                    if ((esc_temp > esc_overheat_threshold) || (motor_temp > motor_overheat_threshold)) {
                        warning_type   = cycle;
                        warning_string = "overheating!";
                    }
                    break;
  #endif                    
#endif

                } // switch 
            } while (!warning_type && cycle != last_warning_type);

            if (warning_type) { // if there a warning
                warning_active     = true;                          // then set warning active
                
                warn_text_timer    = millis() + WARN_FLASH_TIME;   // set show warning time
                warn_recover_timer = millis() + WARN_RECOVER_TIME;
                if (warning_timeout == 0) {
                  warning_timeout = millis() + WARN_TIMEOUT_TIME;
                }
                if ((panel > 0) || (millis() <= warning_timeout)) {
                    osd.clear();
                }
                if (millis() <= warning_timeout) {
                  panel = 0; // switch to first panel if there is a warning
                }
            } else { // if not, we do not want the delay, so a new error shows up immediately
                if (millis() > warn_recover_timer) { // if recover time over since last warning
                    warning_active = false; // no warning active anymore
                    warning_timeout = 0;
                }
            }
        }

        osd.setAndOpenPanel(first_col, first_line);
//        osd.openPanel();
        osd.printf("%s", warning_string);
        osd.closePanel();
    }
}


/******************************************************************/
// Panel  : panSetup
// Needs  : Nothing, uses whole screen
// Output : The settings menu
/******************************************************************/
void panSetup()
{
    static unsigned long setup_debounce_timer = 0;
    static int8_t setup_menu = 0;
    int delta = 100;

    if (millis() > setup_debounce_timer) { // RC-TX stick debouncing
        setup_debounce_timer = millis() + SETUP_DEBOUNCE_TIME;

        osd.clear();
        osd.setAndOpenPanel(5, 3);
//        osd.openPanel();

        osd.printf_P(PSTR("setup screen|||"));

        if (chan1_raw_middle == 0 || chan2_raw_middle == 0) {
            chan1_raw_middle = chan1_raw;
            chan2_raw_middle = chan2_raw;
        }

        if ((chan2_raw - PWM_OFFSET) > chan2_raw_middle) {
            setup_menu++;
        } else if ((chan2_raw + PWM_OFFSET) < chan2_raw_middle) {
            setup_menu--;
        }

        if (setup_menu < SETUP_LOWEST_MENU) {
            setup_menu = SETUP_LOWEST_MENU;
        } else if (setup_menu > SETUP_HIGHEST_MENU) {
            setup_menu = SETUP_HIGHEST_MENU;
        }

        switch (setup_menu) {
#ifdef BATTTYPE_SELECTION
        case 1:
            osd.printf_P(PSTR("batt type: "));
            if (batt_type) {
                osd.printf_P(PSTR("lipo    "));
                if (chan1_raw < chan1_raw_middle - PWM_OFFSET) {
                    batt_type = 0;
                }
            } else {
                osd.printf_P(PSTR("4s liion"));
                if (chan1_raw > chan1_raw_middle + PWM_OFFSET) {
                    batt_type = 1;
                }
            }
            EEPROM.write(batt_type_ADDR, batt_type);
            break;
#endif            

#ifdef SAFETY_RADIUS
//        case 4:
//            delta /= 10;
        case 3:
            delta /= 10;
        case 2:
            osd.printf_P(PSTR("batt capacity:"));
            osd.printf("%8.0f%c", float(batt_capacity), 0x82, 0x20);
            batt_capacity = change_int_val(batt_capacity, batt_capacity_ADDR, delta);
            break;
#endif
/*
            osd.printf_P(PSTR("uavtalk "));
            if (op_uavtalk_mode & UAVTALK_MODE_PASSIVE) {
                osd.printf_P(PSTR("passive"));
                if (chan1_raw < chan1_raw_middle - PWM_OFFSET) {
                    op_uavtalk_mode &= ~UAVTALK_MODE_PASSIVE;
                }
            } else {
                osd.printf_P(PSTR("active "));
                if (chan1_raw > chan1_raw_middle + PWM_OFFSET) {
                    op_uavtalk_mode |= UAVTALK_MODE_PASSIVE;
                }
            }
            break; */
/*        case 2:
            osd.printf_P(PSTR("tempdisp "));
            if (osd_temp_mode) {
                osd.printf_P(PSTR("baro temp"));
                if (chan1_raw < chan1_raw_middle - PWM_OFFSET) {
                    osd_temp_mode = 0;
                }
            } else {
                osd.printf_P(PSTR("txpid values"));
                if (chan1_raw > chan1_raw_middle + PWM_OFFSET) {
                    osd_temp_mode = 1;
                }
            }
            EEPROM.write(temp_mode_ADDR, osd_temp_mode);
            break; */
        case 4:
            osd.printf_P(PSTR("lipo batt cell||warning level: "));
            osd.printf("%4.2f%c", (300 + battv) / 100.0, 0x76, 0x20);
            battv = change_val(battv, battv_ADDR);
            break;
#ifdef FLIGHT_BATT_ON_MINIMOSD
        case 7:
            delta /= 10;
        case 6:
            delta /= 10;
        case 5:
            // volt_div_ratio
            osd.printf_P(PSTR("calibrate||measured volt: "));
            osd.printf("%c%5.2f%c", 0xE2, (float)osd_vbat_A, 0x8E);
            osd.printf("||volt div ratio:  %5i", volt_div_ratio);
            volt_div_ratio = change_int_val(volt_div_ratio, volt_div_ratio_ADDR, delta);
            break;
        case 10:
            delta /= 10;
        case 9:
            delta /= 10;
        case 8:
            // curr_amp_offset
            osd.printf_P(PSTR("calibrate||measured amp:  "));
            osd.printf("%c%5.2f%c", 0xE4, osd_curr_A * .01, 0x8F);
            osd.printf("||amp offset:      %5i", curr_amp_offset);
            curr_amp_offset = change_int_val(curr_amp_offset, curr_amp_offset_ADDR, delta);
            break;
        case 13:
            delta /= 10;
        case 12:
            delta /= 10;
        case 11:
            // curr_amp_per_volt
            osd.printf_P(PSTR("calibrate||measured amp:  "));
            osd.printf("%c%5.2f%c", 0xE4, osd_curr_A * .01, 0x8F);
            osd.printf("||amp per volt:    %5i", curr_amp_per_volt);
            curr_amp_per_volt = change_int_val(curr_amp_per_volt, curr_amp_per_volt_ADDR, delta);
            break;
#endif // ifdef FLIGHT_BATT_ON_MINIMOSD
        }
        osd.closePanel();
    }
}


/******* PANELS *******/


#ifndef FLIGHT_BATT_ON_REVO
/******************************************************************/
// Panel  : panBoot
// Needs  : X, Y locations
// Output : Booting up text and empty bar after that
/******************************************************************/
void panBoot(int first_col, int first_line)
{
    osd.setAndOpenPanel(first_col, first_line);
//    osd.openPanel();
    osd.printf_P(PSTR("booting up:\xed\xf2\xf2\xf2\xf2\xf2\xf2\xf2\xf3"));
    osd.closePanel();
}
#endif

#ifdef LOGO_AT_BOOT
/******************************************************************/
// Panel  : panLogo
// Needs  : X, Y locations
// Output : Startup OSD LOGO
/******************************************************************/
void panLogo()
{
//  osd.setAndOpenPanel(1, 5);
  osd.setAndOpenPanel(1, 7);
//  osd.openPanel();
//  osd.printf_P(PSTR("\x20\x20\x20\x20\x20\xba\xbb\xbc\xbd\xbe|\x20\x20\x20\x20\x20\xca\xcb\xcc\xcd\xce|"));
  VERSION_STRING

  osd.closePanel();
}
#endif

/******************************************************************/
// Panel  : panGPSats
// Needs  : X, Y locations
// Output : 1 symbol and number of locked satellites
/******************************************************************/
void panGPSats(int first_col, int first_line)
{
    osd.setAndOpenPanel(first_col, first_line);
//    osd.openPanel();
//#ifdef JR_SPECIALS // I like it more this way
//    osd.printf("%c%3i", 0x0f, osd_satellites_visible);
//#else
    osd.printf("%c%2i", 0x0f, osd_satellites_visible);
//#endif
    osd.closePanel();
}


/******************************************************************/
// Panel  : panGPL
// Needs  : X, Y locations
// Output : 1 static symbol open lock or 2D or 3D sign
/******************************************************************/
void panGPL(int first_col, int first_line)
{
    osd.setAndOpenPanel(first_col, first_line);
//    osd.openPanel();
    osd.printf("%c", osd_fix_type <= 1 ? osd_fix_type * 0x10 : osd_fix_type - 1);
#ifdef OP_DEBUG // I use this place for debug info
    osd.printf(" %02x", op_alarm);
#endif
    osd.closePanel();
}


/******************************************************************/
// Panel  : panGPS
// Needs  : X, Y locations
// Output : two row numeric value of current GPS location with LAT/LON symbols
/******************************************************************/
void panGPS(int first_col, int first_line)
{
    osd.setAndOpenPanel(first_col, first_line);
//    osd.openPanel();
//#ifdef JR_SPECIALS // I like it more one row style
//    osd.printf("%c%10.6f     %c%10.6f", 0x83, (double)(osd_lat), 0x84, (double)(osd_lon));
//#else
//    osd.printf("%c%11.6f|%c%11.6f", 0x83, (double)osd_lat, 0x84, (double)osd_lon);    // Original Version
    osd.printf("%8.5f|%8.5f", (double)osd_lat, (double)osd_lon);   
//#endif
    osd.closePanel();
}

#ifdef PATHPLAN_WAYPOINTS_SUPPORT
/******************************************************************/
// Panel  : panWPDir
// Needs  : X, Y locations
// Output : All Waypoint data in PathPlan Flight Modes
/******************************************************************/
void panWPDir(int first_col, int first_line)
{
    osd.setAndOpenPanel(first_col, first_line);
//    osd.openPanel();

    if ((osd_mode == 14) || (osd_mode == 12)) {    // 14 == PathPlanner; 12 == RTB; WPInfo is visible

        float osd_waypoint_distance = sqrt(sq(osd_pathdesired_end_east - osd_positionstate_east) + sq(osd_pathdesired_end_north - osd_positionstate_north));
        uint16_t waypoint_eta_sec = 0;
       
       if (osd_groundspeed > 0) {
           waypoint_eta_sec = osd_waypoint_distance / osd_groundspeed;
       }
        if (osd_waypoint_distance > 99999) {
            osd_waypoint_distance = 99999;
        }
        if (waypoint_eta_sec > 5999) {
            waypoint_eta_sec = 5999;
        }
        if (osd_waypointactive_index > 99) {
            osd_waypointactive_index = 99;
        }
        char wpdist_unit_length = unit_length;
        float wpdist_convert_length = convert_length;

        if (osd_waypoint_distance > 9999) {
            wpdist_unit_length = unit_length_large;
            wpdist_convert_length/= convert_length_large;
        }
        if (osd_mode == 12) {
            osd_waypointactive_index = 0;
        }
        osd.printf("%c%c%c%4.0f%c|%02i%c%02i%c%02i", 0x13, 0x14, 0x19, (osd_waypoint_distance * wpdist_convert_length), wpdist_unit_length, osd_waypointactive_index, 0x1A, ((int)(waypoint_eta_sec / 60)) % 60, 0x3A, waypoint_eta_sec % 60);
    }
    else {                                          // WPInfo is hidden
        char *s = "        ";
        osd.printf("%s|%s",s,s);
    }
    osd.closePanel();
}
#endif

/******************************************************************/
// Panel  : panSR_EGE
// Needs  : X, Y locations
// Output : Safety Radius / Efficiency/Glide Estimation
/******************************************************************/

#ifdef SAFETY_RADIUS
void panSR_EGE(int first_col, int first_line)
{
    osd.setAndOpenPanel(first_col, first_line);
//    osd.openPanel();
    
#ifdef LT_RADIUS_ESTIMATION    
    char ltsr_char = 0xF7;
    if (lt_safety_delta < 0) {
        ltsr_char = 0x8A;
    }
#endif

#ifdef ST_RADIUS_ESTIMATION
    char stsr_char = 0xF8;
    if (st_safety_delta < 0) {
        stsr_char = 0x8A;
    }
#endif        

#ifdef BATP_RADIUS_ESTIMATION
    char batpsr_char = 0xFA;
    if (batp_safety_delta < 0) {
        batpsr_char = 0x8A;
    }
#endif        

#if defined LT_RADIUS_ESTIMATION && defined ST_RADIUS_ESTIMATION && defined EFFICIENCY_ESTIMATION // 6
  #ifdef FIXED_WING
        osd.printf("%c%5.0f%c|%c%5.0f%c|%c%5.0f%c", ltsr_char, (float)lt_safety_delta * convert_length, unit_length, stsr_char, (float)st_safety_delta * convert_length, unit_length, 0xFB, flight_efficiency, 0xFC);
  #else        
        osd.printf("%c%5.0f%c|%c%5.0f%c|%c%5.2f%c", ltsr_char, (float)lt_safety_delta * convert_length, unit_length, stsr_char, (float)st_safety_delta * convert_length, unit_length, 0xFB, (float)((flight_efficiency * convert_length) / convert_length_large), 0xF4);
  #endif        
#else        
  #if defined LT_RADIUS_ESTIMATION && defined ST_RADIUS_ESTIMATION && defined BATP_RADIUS_ESTIMATION // 3
          osd.printf("%c%5.0f%c|%c%5.0f%c|%c%5.0f%c", ltsr_char, (float)lt_safety_delta * convert_length, unit_length, stsr_char, (float)st_safety_delta * convert_length, unit_length, batpsr_char, (float)(batp_safety_delta) * convert_length, unit_length);
  #else
    #if defined LT_RADIUS_ESTIMATION && defined BATP_RADIUS_ESTIMATION // 2
          osd.printf("%c%5.0f%c|%c%5.0f%c", ltsr_char, (float)lt_safety_delta * convert_length, unit_length, batpsr_char, (float)batp_safety_delta * convert_length, unit_length);
    #else  
      #if defined LT_RADIUS_ESTIMATION && defined ST_RADIUS_ESTIMATION  // 1
          osd.printf("%c%5.0f%c|%c%5.0f%c", ltsr_char, (float)lt_safety_delta * convert_length, unit_length, stsr_char, (float)st_safety_delta * convert_length, unit_length);
      #else
        #if defined BATP_RADIUS_ESTIMATION  // 4
          osd.printf("%c%5.0f%c", batpsr_char, (float)batp_safety_delta * convert_length, unit_length);
        #else
          #if defined LT_RADIUS_ESTIMATION && defined EFFICIENCY_ESTIMATION // 5

            #if defined FIXED_WING && defined GLIDE_ESTIMATION
              uint8_t eff_symbol = 0xFB;
              uint8_t eff_unit = 0xFC;
              float eff_value = flight_efficiency;
          
              if (osd_throttle < FIXEDWING_MIN_THROTTLE /*&& osd_armed > 1*/ && ((long)flight_efficiency > 0) && (osd_climb < 0.0) && (osd_mode < 7))
              {
                  eff_symbol = 0x7E;
                  eff_unit = 0x8D;
                  eff_value = (osd_baro_home_alt - revo_baro_alt) * osd_groundspeed / osd_climb;
                  if (eff_value > 99999) {
                      eff_value = 99999;
                  }
              }

              if ((long)(lt_safety_delta * convert_length) >= 10000) {
                  osd.printf("%c%5.1f%c|%c%5.0f%c", ltsr_char, (float)((lt_safety_delta * convert_length) / convert_length_large), unit_length_large, eff_symbol, eff_value, eff_unit);
                  
              } 
              else {
                  osd.printf("%c%5.0f%c|%c%5.0f%c", ltsr_char, (float)lt_safety_delta * convert_length, unit_length, eff_symbol, eff_value, eff_unit);
              }
            #else
              #ifdef FIXED_WING
                osd.printf("%c%5.0f%c|%c%5.0f%c", ltsr_char, (float)lt_safety_delta * convert_length, unit_length, 0xFB, flight_efficiency, 0xFC);
              #else
                osd.printf("%c%5.0f%c|%c%5.2f%c", ltsr_char, (float)lt_safety_delta * convert_length, unit_length, 0xFB, (float)((flight_efficiency * convert_length) / convert_length_large), 0xF4);
              #endif
            #endif  // FIXED_WING && GLIDE_ESTIMATION

          #endif // 5  
        #endif // 4  
      #endif // 1
    #endif // 2
  #endif // 3
#endif // 6 

    osd.closePanel();
}
#endif


/******************************************************************/
// Panel  : panHomeDis
// Needs  : X, Y locations
// Output : Distance to home
/******************************************************************/
void panHomeDis(int first_col, int first_line)
{
    osd.setAndOpenPanel(first_col, first_line);
//    osd.openPanel();

    if (osd_direct_distance * convert_length < 10000.0) {
        osd.printf("%c%4.0f%c", 0x12, (float)osd_direct_distance * convert_length, unit_length);
    }
    else {
        osd.printf("%c%4.1f%c", 0x12, (float)osd_direct_distance * convert_length / convert_length_large, unit_length_large);
    }

    osd.closePanel();
}


/******************************************************************/
// Panel  : panHomeDir
// Needs  : X, Y locations
// Output : 2 symbols that are combined as one arrow, shows direction to home
/******************************************************************/
void panHomeDir(int first_col, int first_line)
{
    osd.setAndOpenPanel(first_col, first_line);
//    osd.openPanel();
    showArrow((uint8_t)osd_home_direction);
    osd.closePanel();
}


/******************************************************************/
// Panel  : panHomeAlt
// Needs  : X, Y locations
// Output : Home altitude
/******************************************************************/
void panHomeAlt(int first_col, int first_line)
{
#ifdef REVO_ADD_ONS
    osd.setAndOpenPanel(first_col, first_line);
//    osd.openPanel();
/*
    if (panel == 0) {
        if (osd_got_home && osd_fix_type > 1) {
            osd.printf("%c%4.0f%c  %c%4.0f%c", 0xE6, (double)((revo_baro_alt - osd_baro_home_alt)* convert_length), unit_length, 0xE7, (double)((osd_alt - osd_home_alt) * convert_length), unit_length);
        }
        else {
             osd.printf("%c%4.0f%c  %c %c%c%c%c", 0xE6, (double)((revo_baro_alt - osd_baro_home_alt) * convert_length), unit_length, 0xE7, 0xEA, 0xEA, 0xEA, unit_length);
        }
    }
    else {
        osd.printf("%c%4.0f%c", 0xE6, (double)((revo_baro_alt - osd_baro_home_alt)* convert_length), unit_length);
    }
*/    // JDL Original

    osd.printf("%c%4.0f%c", 0xE6, (float)((revo_baro_alt - osd_baro_home_alt)* convert_length), unit_length);
    osd.closePanel();
#else
    if (osd_got_home && osd_fix_type > 1) {
        osd.setAndOpenPanel(first_col, first_line);
//        osd.openPanel();
        osd.printf("%c%4.0f%c", 0xE7, (float)((osd_alt - osd_home_alt) * convert_length), unit_length);
        osd.closePanel();
    }
#endif
}


/******************************************************************/
// Panel  : panAlt
// Needs  : X, Y locations
// Output : Altitude
/******************************************************************/
void panAlt(int first_col, int first_line)
{
    osd.setAndOpenPanel(first_col, first_line);
//    osd.openPanel();
//    osd.printf("%c%5.0f%c", 0xE6, (double)(osd_alt * convert_length), unit_length);
//    osd.printf("%c%5.0f%c", 0x85, (double)(osd_alt * convert_length), unit_length);    //  JDL Original

#ifdef REVO_ADD_ONS
    osd.printf("%c%5.0f%c %c%4.0f%c", 0x85, (float)osd_alt * convert_length, unit_length, 0xE7, (float)osd_alt - osd_home_alt * convert_length, unit_length);
#else    
    osd.printf("%c%5.0f%c", 0x85, (float)osd_alt * convert_length, unit_length);
#endif    
    osd.closePanel();
}


/******************************************************************/
// Panel  : panVel
// Needs  : X, Y locations
// Output : Velocity
/******************************************************************/
void panVel(int first_col, int first_line)
{
    osd.setAndOpenPanel(first_col, first_line);
//    osd.openPanel();
    osd.printf("%c%3.0f%c", 0xE9, (float)osd_groundspeed * convert_speed, unit_speed);
    osd.closePanel();
}


/******************************************************************/
// Panel  : panAirSpeed
// Needs  : X, Y locations
// Output : AirSpeed
/******************************************************************/
void panAirSpeed(int first_col, int first_line)
{
    osd.setAndOpenPanel(first_col, first_line);
//    osd.openPanel();
//    osd_3D_speed = sqrt(sq(osd_groundspeed) + sq(osd_climb));            // moved to OSD_Func.h / updateTravelDistance

#ifdef FIXED_WING

  #ifndef STALL_WARNING
    osd.printf("%c%3.0f%c|%c%3.0f%c", 0xE8, (float)osd_3D_speed * convert_speed, unit_speed, 0x89, (float)osd_airspeed * convert_speed, unit_speed);
  #else
    static char airspeed_string[4];
    itoa((float)osd_airspeed * convert_speed, airspeed_string, 10);
    if ((osd_armed > 1) && /*((long)osd_travel_distance > 50) && */(osd_airspeed <= (stall_threshold + stall_threshold_warn_margin)) && ((millis() / 200) % 2)) {
        airspeed_string[0] = ' ';
        airspeed_string[1] = '\0';
    }
    osd.printf("%c%3.0f%c|%c%3s%c", 0xE8, (float)osd_3D_speed * convert_speed, unit_speed, 0x89, airspeed_string, unit_speed);
  #endif

#else    
    osd.printf("%c%3.0f%c", 0xE8, (float)osd_3D_speed * convert_speed, unit_speed);
#endif    
    
    osd.closePanel();
}


/******************************************************************/
// Panel  : panDistance
// Needs  : X, Y locations
// Output : travel distance
/******************************************************************/
void panDistance(int first_col, int first_line)
{
    osd.setAndOpenPanel(first_col, first_line);
//    osd.openPanel();
    if ((long)(osd_travel_distance * convert_length) > 1000) {
        osd.printf("%c%5.2f%c", 0xFE, ((osd_travel_distance * convert_length) / convert_length_large), unit_length_large);
    } else {
        osd.printf("%c%5.0f%c", 0xFE, (osd_travel_distance * convert_length), unit_length);
    }
    osd.closePanel();
}


/******************************************************************/
// Panel  : panClimb
// Needs  : X, Y locations
// Output : Climb Rate
/******************************************************************/
void panClimb(int first_col, int first_line)
{
    osd.setAndOpenPanel(first_col, first_line);
//    osd.openPanel();

#ifdef FIXED_WING
    float osd_climb_m_min = abs(osd_climb * 60.0);
    uint8_t symbol_climb = 0x7D;
    if (osd_climb > 0.0)
    {
        symbol_climb = 0x7B;
    }
    osd.printf("%c%4.0f%c", symbol_climb, osd_climb_m_min, 0x7F);
#else    
    osd.printf("%c%3.0f%c", 0x16, osd_climb, 0x88);
#endif

    osd.closePanel();
}


#ifndef COMBINED_HEADING_AND_ROSE
/******************************************************************/
// Panel  : panHeading
// Needs  : X, Y locations
// Output : Symbols with numeric compass heading value
/******************************************************************/
void panHeading(int first_col, int first_line)
{
    osd.setAndOpenPanel(first_col, first_line);
//    osd.openPanel();
#ifdef JR_SPECIALS // show heading with compass point
    osd.printf("%4.0f%c%s", (double)osd_heading, 0xB0, CompassPoint);
#else
    osd.printf("%4.0f%c", (double)osd_heading, 0xb0);
#endif
    osd.closePanel();
}
#endif

/******************************************************************/
// Panel  : panRose
// Needs  : X, Y locations
// Output : a dynamic compass rose that changes along the heading information
/******************************************************************/
void panRose(int first_col, int first_line)
{
    osd.setAndOpenPanel(first_col, first_line);
//    osd.openPanel();
#ifndef COMBINED_HEADING_AND_ROSE    
    osd.printf("%s|%c%s%c", "\x20\xc6\xc6\xc6\xc6\xc6\xc7\xc6\xc6\xc6\xc6\xc6\x20", 0xd0, buf_show, 0xd1);
#else
    osd.printf("%s%3.0f%c|%c%s%c", "\x20\xc6\xc6\xc6\xc6\xc6\xc7\xc6\xc6", (double)osd_heading, 0xB0, 0xd0, buf_show, 0xd1);
#endif
    osd.closePanel();
}


/******************************************************************/
// Panel  : panRSSI
// Needs  : X, Y locations
// Output : RSSI %
/******************************************************************/
void panRSSI(int first_col, int first_line)
{
#ifdef REVO_ADD_ONS
    osd.setAndOpenPanel(first_col, first_line);
//    osd.openPanel();
    osd.printf("%c%4i%c %4i%c", 0xE1, oplm_rssi, 0x8C, oplm_linkquality, 0x8B);
    osd.closePanel();
#endif

#ifdef RSSI_ON_INPUT_CHANNEL
    osd.setAndOpenPanel(first_col, first_line);
//    osd.openPanel();
    osd.printf("%c%3i%c", 0xE1, rssi, 0x8C);
    osd.closePanel();
#endif

}


/******************************************************************/
// Panel  : panRoll
// Needs  : X, Y locations
// Output : -+ value of current Roll from vehicle with degree symbols and roll symbol
/******************************************************************/
void panRoll(int first_col, int first_line)
{
    osd.setAndOpenPanel(first_col, first_line);
//    osd.openPanel();
    osd.printf("%4i%c", osd_roll, 0xb2);
    osd.closePanel();
}


/******************************************************************/
// Panel  : panPitch
// Needs  : X, Y locations
// Output : -+ value of current Pitch from vehicle with degree symbols and pitch symbol
/******************************************************************/
void panPitch(int first_col, int first_line)
{
    osd.setAndOpenPanel(first_col, first_line);
//    osd.openPanel();
    osd.printf("%4i%c", osd_pitch, 0xb1);
    osd.closePanel();
}


/******************************************************************/
// Panel  : panThr
// Needs  : X, Y locations
// Output : Throttle
/******************************************************************/
void panThr(int first_col, int first_line)
{
    osd.setAndOpenPanel(first_col, first_line);
//    osd.openPanel();
    osd.printf("%c%3.0i%c", 0x87, osd_throttle, 0x25);
    osd.closePanel();
}


/******************************************************************/
// Panel  : panFlightMode
// Needs  : X, Y locations
// Output : current flight modes
/******************************************************************/
void panFlightMode(int first_col, int first_line)
{
    char *mode_str = "";

    osd.setAndOpenPanel(first_col, first_line);
//    osd.openPanel();

#if defined VERSION_RELEASE_LP15_09
    if (osd_mode == 0) {
        mode_str = "man"; // MANUAL
    } else if (osd_mode == 1) {
        mode_str = "st1"; // STABILIZED1
    } else if (osd_mode == 2) {
        mode_str = "st2"; // STABILIZED2
    } else if (osd_mode == 3) {
        mode_str = "st3"; // STABILIZED3
    } else if (osd_mode == 4) {
        mode_str = "st4"; // STABILIZED4
    } else if (osd_mode == 5) {
        mode_str = "st5"; // STABILIZED5
    } else if (osd_mode == 6) {
        mode_str = "st6"; // STABILIZED6
    } else if (osd_mode == 7) {
        mode_str = "ph "; // POSITIONHOLD
    } else if (osd_mode == 8) {
        mode_str = "cl "; // COURSELOCK
    } else if (osd_mode == 9) {
        mode_str = "vr "; // VELOCITYROAM
    } else if (osd_mode == 10) {
        mode_str = "hl "; // HOMELEASH
    } else if (osd_mode == 11) {
        mode_str = "ap "; // ABSOLUTEPOSITION
    } else if (osd_mode == 12) {
        mode_str = "rtb"; // RETURNTOBASE
    } else if (osd_mode == 13) {
        mode_str = "lan"; // LAND
    } else if (osd_mode == 14) {
        mode_str = "pp "; // PATHPLANNER
    } else if (osd_mode == 15) {
        mode_str = "poi"; // POI
    } else if (osd_mode == 16) {
        mode_str = "ac "; // AUTOCRUISE
    } else if (osd_mode == 17) {
        mode_str = "at "; // AUTOTAKEOFF
    }
#endif // if defined VERSION_RELEASE_15_05 || defined VERSION_RELEASE_LP15_09

    char c;
    if ((osd_mode > 0) &&(osd_mode < 7) && (osd_aswa > 0)) {
      #ifndef FIXED_WING
      if (((millis() / 300) % 2) == 0) {
        c = 0x20;
      }
      else {
        c = 0x04;
      }
      #else
        c = 0x04;
      #endif    
    }
    else {
      c = 0xE0;
    }
    osd.printf("%c%s", c, mode_str);
//#else    
//    osd.printf("%c%s", 0xE0, mode_str);
//#endif    

    osd.closePanel();
}


/******************************************************************/
// Panel  : panBattery A (Voltage 1)
// Needs  : X, Y locations
// Output : Voltage value as in XX.X and symbol of over all battery status
/******************************************************************/
void panBatt_A(int first_col, int first_line)
{
    osd.setAndOpenPanel(first_col, first_line);
//    osd.openPanel();
//    osd.printf("%c%5.2f%c", 0xE2, (double)osd_vbat_A, 0x8E);
    char batsym;
    if (osd_vbat_A <= ((304/*3.333*/ + (battv * batt_type)) * num_cells / 100.0)) {
        batsym = 0xB4;
    }
    else {
        batsym = 0xB8;
    }
    
#ifdef FLIGHT_BATT_ON_MINIMOSD
    osd.printf("%i%c%c%5.2f%c", num_cells, 0x80, batsym, (float)osd_vbat_A, 0x8E);
#else    
    osd.printf("%c%5.2f%c", batsym, /*(float)*/osd_vbat_A, 0x8E);
#endif    
    osd.closePanel();
}


/******************************************************************/
// Panel  : panCur_A
// Needs  : X, Y locations
// Output : Current
/******************************************************************/
void panCur_A(int first_col, int first_line)
{
    osd.setAndOpenPanel(first_col, first_line);
//    osd.openPanel();
    osd.printf("%c%5.2f%c", 0xE4, osd_curr_A * .01, 0x8F);
    osd.closePanel();
}


/******************************************************************/
// Panel  : panBatteryPercent
// Needs  : X, Y locations
// Output : Battery
// (if defined FLIGHT_BATT_ON_MINIMOSD || defined FLIGHT_BATT_ON_REVO then not percent but consumed mAh)
/******************************************************************/
void panBatteryPercent(int first_col, int first_line)
{
    osd.setAndOpenPanel(first_col, first_line);
//    osd.openPanel();
//#if defined FLIGHT_BATT_ON_MINIMOSD || defined FLIGHT_BATT_ON_REVO
#ifdef FLIGHT_BATT_ON_REVO
//    osd.printf("%c%5i%c", 0xB9, osd_total_A, 0x82);
    osd.printf("%c%5i%c|%c%2.0f %c%2.0f", 0xB9, osd_total_A, 0x82, 0xB7, osd_battery_remaining_percent_V, 0xB6, osd_battery_remaining_percent_A);
#else
    osd.printf("%c%2.0f%c", 0xB7, osd_battery_remaining_percent_V, 0x25);
#endif
    osd.closePanel();
}


/******************************************************************/
// Panel  : panTxPID
// Needs  : X, Y locations
// Output : Current TxPID settings
/******************************************************************/
/*
void panTxPID(int first_col, int first_line)
{
    osd.setAndOpenPanel(first_col, first_line);
    osd.openPanel();
    osd.printf("%c%1.5f%c", 0xED, (double)osd_txpid_cur[0], 0xF3);
    osd.closePanel();
    osd.setAndOpenPanel(first_col, first_line + 1);
    osd.openPanel();
    osd.printf("%c%1.5f%c", 0xED, (double)osd_txpid_cur[1], 0xF3);
    osd.closePanel();
    osd.setAndOpenPanel(first_col, first_line + 2);
    osd.openPanel();
    osd.printf("%c%1.5f%c", 0xED, (double)osd_txpid_cur[2], 0xF3);
    osd.closePanel();
}
*/

/******************************************************************/
// Panel  : panBaroTemp
// Needs  : X, Y locations
// Output : baro temperature
/******************************************************************/
void panBaroTemp(int first_col, int first_line)
{
    osd.setAndOpenPanel(first_col, first_line);
//    osd.openPanel();

#if defined REVO_ADD_ONS && not defined TEMP_SENSOR_LM335Z_ESC && not defined TEMP_SENSOR_LM335Z_MOTOR && not defined TEMP_SENSOR_LM335Z_AMBIENT
    osd.printf("%c%3.0f%c", 0xE3, revo_baro_temp, 0xB0);
#endif    

#if defined TEMP_SENSOR_LM335Z_ESC && not defined TEMP_SENSOR_LM335Z_MOTOR && not defined TEMP_SENSOR_LM335Z_AMBIENT
    osd.printf("%c%3.0f", 0x17, esc_temp);
#endif

#if not defined TEMP_SENSOR_LM335Z_ESC && defined TEMP_SENSOR_LM335Z_MOTOR && not defined TEMP_SENSOR_LM335Z_AMBIENT
    osd.printf("%c%3.0f", 0x18, motor_temp);
#endif

#if defined TEMP_SENSOR_LM335Z_ESC && defined TEMP_SENSOR_LM335Z_MOTOR && not defined TEMP_SENSOR_LM335Z_AMBIENT
    osd.printf("%c%3.0f|%c%3.0f", 0x17, esc_temp, 0x18, motor_temp);
#endif

#if defined TEMP_SENSOR_LM335Z_ESC && defined TEMP_SENSOR_LM335Z_MOTOR && defined TEMP_SENSOR_LM335Z_AMBIENT
    osd.printf("%c%3.0f|%c%3.0f|%c%3.0f%c", 0x17, esc_temp, 0x18, motor_temp, 0xE3, ambient_temp, 0xB0);
#endif

#if not defined TEMP_SENSOR_LM335Z_ESC && not defined TEMP_SENSOR_LM335Z_MOTOR && defined TEMP_SENSOR_LM335Z_AMBIENT
    osd.printf("%c%3.0f%c", 0xE3, ambient_temp, 0xB0);
#endif

#if not defined REVO_ADD_ONS && not defined TEMP_SENSOR_LM335Z_ESC && not defined TEMP_SENSOR_LM335Z_MOTOR && not defined TEMP_SENSOR_LM335Z_AMBIENT
    osd.printf("no temp");
#endif

    osd.closePanel();
}


/******************************************************************/
// Panel  : panTime
// Needs  : X, Y locations
// Output : Time from bootup or start
/******************************************************************/
void panTime(int first_col, int first_line)
{
    osd.setAndOpenPanel(first_col, first_line);
//    osd.openPanel();
#if defined FLIGHT_BATT_ON_REVO
    osd.printf("%c%2i%c%02i|%c%2i%c%02i", 0xB3, ((int)(flight_time / 60)) % 60, 0x3A, flight_time % 60, 0xF9, ((int)(osd_est_flight_time / 60)) % 60, 0x3A, osd_est_flight_time % 60);
#else
    osd.printf("%c%2i%c%02i", 0xB3, ((int)(flight_time / 60)) % 60, 0x3A, flight_time % 60);
#endif
    osd.closePanel();
}


/******************************************************************/
// Panel  : panHorizon
// Needs  : X, Y locations
// Output : artificial horizon
/******************************************************************/
void panHorizon(int first_col, int first_line)
{
    osd.setAndOpenPanel(first_col, first_line);
//    osd.openPanel();
    osd.printf_P(PSTR("\xc8\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\xc9|"));
    osd.printf_P(PSTR("\xc8\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\xc9|"));
    osd.printf_P(PSTR("\xd8\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\xd9|"));
    osd.printf_P(PSTR("\xc8\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\xc9|"));
    osd.printf_P(PSTR("\xc8\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\xc9"));
    osd.closePanel();
    showHorizon((first_col + 1), first_line);
}


/******* HELPER FUNCTIONS *******/


// Setup change function
int change_int_val(int value, int address, int delta)
{
    int value_old = value;

    osd.printf_P(PSTR("|                   "));
    switch (delta) {
    case 100:
        osd.printf_P(PSTR("\xBF"));
        break;
    case 10:
        osd.printf_P(PSTR(" \xBF"));
        break;
    case 1:
        osd.printf_P(PSTR("  \xBF"));
        break;
    }

    if (chan1_raw > chan1_raw_middle + PWM_OFFSET) {
        value += delta;
    } else if (chan1_raw < chan1_raw_middle - PWM_OFFSET) {
        value -= delta;
    }

    if (value != value_old) {
        EEPROM.write(address, value & 0xff);
        EEPROM.write(address + 1, (value >> 8) & 0xff);
    }
    return value;
}


// Setup change function
int change_val(int value, int address)
{
    uint8_t value_old = value;

    if (chan1_raw > chan1_raw_middle + PWM_OFFSET) {
        value++;
    } else if (chan1_raw < chan1_raw_middle - PWM_OFFSET) {
        value--;
    }

    if (value != value_old) {
        EEPROM.write(address, value);
    }
    return value;
}


// Show those fancy 2 char arrows
#define ARROW_CODE (0x90 - 2) // code of the first MAX7456 special arrow char -2
void showArrow(uint8_t rotate_arrow)
{
    rotate_arrow = (rotate_arrow < 2) ? 2 : rotate_arrow * 2;

    if (osd_home_distance * convert_length < 10000.0) {
        osd.printf("%c%c%4.0f%c", ARROW_CODE + rotate_arrow, ARROW_CODE + rotate_arrow + 1, (float)osd_home_distance * convert_length, unit_length);
    }
    else {
        osd.printf("%c%c%4.1f%c", ARROW_CODE + rotate_arrow, ARROW_CODE + rotate_arrow + 1, (float)osd_home_distance * convert_length / convert_length_large, unit_length_large);
    }
}


#ifdef AH_ORIGINAL_VERSION

// Calculate and shows Artificial Horizon
void showHorizon(int start_col, int start_row)
{
    int x, nose, row, minval, hit, subval = 0;
    const int cols = 12;
    const int rows = 5;
    int col_hit[cols];
    float pitch, roll;

    (abs(osd_pitch) == 90) ? pitch = 89.99 * (90 / osd_pitch) * -0.017453293 : pitch = osd_pitch * -0.017453293;
    (abs(osd_roll) == 90) ? roll   = 89.99 * (90 / osd_roll) * 0.017453293 : roll = osd_roll * 0.017453293;

    nose = round(tan(pitch) * (rows * 9));
    for (int col = 1; col <= cols; col++) {
        x = (col * 12) - (cols * 6) - 6; // center X point at middle of each col
        col_hit[col - 1] = (tan(roll) * x) + nose + (rows * 9) - 1; // calculating hit point on Y plus offset to eliminate negative values
        // col_hit[(col-1)] = nose + (rows * 9);
    }

    for (int col = 0; col < cols; col++) {
        hit = col_hit[col];
        if (hit > 0 && hit < (rows * 18)) {
            row    = rows - ((hit - 1) / 18);
            minval = rows * 18 - row * 18 + 1;
            subval = hit - minval;
            subval = round((subval * 9) / 18);
            if (subval == 0) {
                subval = 1;
            }
            printHit(start_col + col, start_row + row - 1, subval);
        }
    }
}

void printHit(byte col, byte row, byte subval)
{
    osd.openSingle(col, row);
    char subval_char;
    switch (subval) {
    case 1:
        // osd.printf_P(PSTR("\x06"));
        subval_char = 0x06;
        break;
    case 2:
        // osd.printf_P(PSTR("\x07"));
        subval_char = 0x07;
        break;
    case 3:
        // osd.printf_P(PSTR("\x08"));
        subval_char = 0x08;
        break;
    case 4:
        // osd.printf_P(PSTR("\x09"));
        subval_char = 0x09;
        break;
    case 5:
        // osd.printf_P(PSTR("\x0a"));
        subval_char = 0x0a;
        break;
    case 6:
        // osd.printf_P(PSTR("\x0b"));
        subval_char = 0x0b;
        break;
    case 7:
        // osd.printf_P(PSTR("\x0c"));
        subval_char = 0x0c;
        break;
    case 8:
        // osd.printf_P(PSTR("\x0d"));
        subval_char = 0x0d;
        break;
    case 9:
        // osd.printf_P(PSTR("\x0e"));
        subval_char = 0x0e;
        break;
    }
    osd.printf("%c", subval_char);
}

#endif // AH_ORIGINAL_VERSION


#ifdef AH_REFACTORED_VERSION
// with different factors we can adapt do different cam optics
#define AH_PITCH_FACTOR  0.017453293             // conversion factor for pitch
#define AH_ROLL_FACTOR   0.017453293             // conversion factor for roll
#define AH_COLS          12                      // number of artificial horizon columns
#define AH_ROWS          5                       // number of artificial horizon rows
#define CHAR_COLS        12                      // number of MAX7456 char columns
#define CHAR_ROWS        18                      // number of MAX7456 char rows
#define CHAR_SPECIAL     9                       // number of MAX7456 special chars for the artificial horizon
#define LINE_CODE        (0x06 - 1)              // code of the first MAX7456 special char -1
#define AH_TOTAL_LINES   AH_ROWS * CHAR_ROWS
#define AH_SPECIAL_LINES AH_ROWS * CHAR_SPECIAL

// Calculate and show artificial horizon
void showHorizon(int start_col, int start_row)
{
    int col, row, pitch_line, middle, hit, subval;

    pitch_line = round(tan(-AH_PITCH_FACTOR * osd_pitch) * AH_SPECIAL_LINES);
    for (col = 1; col <= AH_COLS; col++) {
        middle = col * CHAR_COLS - (AH_COLS * CHAR_COLS / 2) - CHAR_COLS / 2; // center X point at middle of each column
        hit    = tan(AH_ROLL_FACTOR * osd_roll) * middle + pitch_line + AH_SPECIAL_LINES - 1;      // calculating hit point on Y plus offset to eliminate negative values
        if (hit > 0 && hit < AH_TOTAL_LINES) {
            row    = AH_ROWS - ((hit - 1) / CHAR_ROWS);
            subval = hit - (AH_TOTAL_LINES - row * CHAR_ROWS + 1);
            subval = round(subval * CHAR_SPECIAL / CHAR_ROWS);
            if (subval == 0) {
                subval = 1;
            }
            osd.openSingle(start_col + col - 1, start_row + row - 1);
            osd.printf("%c", LINE_CODE + subval);
        }
    }
}

#endif // AH_REFACTORED_VERSION


#ifdef AH_ZERO_CENTERED

// For using this, you must load a special mcm file where the artificial horizon zero left/right arrows are centered!
// e.g. AH_ZeroCentered002.mcm
// with different factors we can adapt do different cam optics
#define AH_PITCH_FACTOR 0.010471976 // conversion factor for pitch
#define AH_ROLL_FACTOR  0.017453293             // conversion factor for roll
#define AH_COLS         12                      // number of artificial horizon columns
#define AH_ROWS         5                       // number of artificial horizon rows
#define CHAR_COLS       12                      // number of MAX7456 char columns
#define CHAR_ROWS       18                      // number of MAX7456 char rows
#define CHAR_SPECIAL    9                       // number of MAX7456 special chars for the artificial horizon
#define LINE_CODE       (0x06 - 1)              // code of the first MAX7456 special char -1
#define AH_TOTAL_LINES  AH_ROWS * CHAR_ROWS

// Calculate and show artificial horizon
// used formula: y = m * x + n <=> y = tan(a) * x + n
void showHorizon(int start_col, int start_row)
{
    int col, row, pitch_line, middle, hit, subval;

    pitch_line = round(tan(-AH_PITCH_FACTOR * osd_pitch) * AH_TOTAL_LINES) + AH_TOTAL_LINES / 2; // 90 total lines
    for (col = 1; col <= AH_COLS; col++) {
        middle = col * CHAR_COLS - (AH_COLS / 2 * CHAR_COLS) - CHAR_COLS / 2; // -66 to +66	center X point at middle of each column
        hit    = tan(AH_ROLL_FACTOR * osd_roll) * middle + pitch_line;               // 1 to 90	calculating hit point on Y plus offset
        if (hit >= 1 && hit <= AH_TOTAL_LINES) {
            row    = (hit - 1) / CHAR_ROWS;                                            // 1 to 5 bottom-up
            subval = (hit - (row * CHAR_ROWS) + 1) / (CHAR_ROWS / CHAR_SPECIAL); // 1 to 9
            osd.openSingle(start_col + col - 1, start_row + AH_ROWS - row - 1);
            osd.printf("%c", LINE_CODE + subval);
        }
    }
}

#endif // AH_ZERO_CENTERED


#ifdef AH_BETTER_RESOLUTION

// For using this, you must load a special mcm file with the new staggered artificial horizon chars!
// e.g. AH_BetterResolutionCharset002.mcm
// with different factors we can adapt do different cam optics
#define AH_PITCH_FACTOR      0.010471976             // conversion factor for pitch
#define AH_ROLL_FACTOR       0.017453293             // conversion factor for roll
#define AH_COLS              12                      // number of artificial horizon columns
#define AH_ROWS              5                       // number of artificial horizon rows
#define CHAR_COLS            12                      // number of MAX7456 char columns
#define CHAR_ROWS            18                      // number of MAX7456 char rows
#define CHAR_SPECIAL         9                       // number of MAX7456 special chars for the artificial horizon
#define AH_TOTAL_LINES       AH_ROWS * CHAR_ROWS     // helper define

#define LINE_SET_STRAIGHT__  (0x06 - 1)              // code of the first MAX7456 straight char -1
#define LINE_SET_STRAIGHT_O  (0x3B - 3)              // code of the first MAX7456 straight overflow char -3
#define LINE_SET_P___STAG_1  (0x3C - 1)              // code of the first MAX7456 positive staggered set 1 char -1
#define LINE_SET_P___STAG_2  (0x45 - 1)              // code of the first MAX7456 positive staggered set 2 char -1
#define LINE_SET_N___STAG_1  (0x4E - 1)              // code of the first MAX7456 negative staggered set 1 char -1
#define LINE_SET_N___STAG_2  (0x57 - 1)              // code of the first MAX7456 negative staggered set 2 char -1
#define LINE_SET_P_O_STAG_1  (0xD4 - 2)              // code of the first MAX7456 positive overflow staggered set 1 char -2
#define LINE_SET_P_O_STAG_2  (0xDA - 1)              // code of the first MAX7456 positive overflow staggered set 2 char -1
#define LINE_SET_N_O_STAG_1  (0xD6 - 2)              // code of the first MAX7456 negative overflow staggered set 1 char -2
#define LINE_SET_N_O_STAG_2  (0xDD - 1)              // code of the first MAX7456 negative overflow staggered set 2 char -1

#define OVERFLOW_CHAR_OFFSET 6 // offset for the overflow subvals

#define ANGLE_1              9                       // angle above we switch to line set 1
#define ANGLE_2              25                      // angle above we switch to line set 2

// Calculate and show artificial horizon
// used formula: y = m * x + n <=> y = tan(a) * x + n
void showHorizon(int start_col, int start_row)
{
    int col, row, pitch_line, middle, hit, subval;
    int roll;
    int line_set = LINE_SET_STRAIGHT__;
    int line_set_overflow = LINE_SET_STRAIGHT_O;
    int subval_overflow   = 9;

    // preset the line char attributes
    roll = osd_roll;
    if ((roll >= 0 && roll < 90) || (roll >= -179 && roll < -90)) { // positive angle line chars
        roll = roll < 0 ? roll + 179 : roll;
        if (abs(roll) > ANGLE_2) {
            line_set = LINE_SET_P___STAG_2;
            line_set_overflow = LINE_SET_P_O_STAG_2;
            subval_overflow = 7;
        } else if (abs(roll) > ANGLE_1) {
            line_set = LINE_SET_P___STAG_1;
            line_set_overflow = LINE_SET_P_O_STAG_1;
            subval_overflow = 8;
        }
    } else { // negative angle line chars
        roll = roll > 90 ? roll - 179 : roll;
        if (abs(roll) > ANGLE_2) {
            line_set = LINE_SET_N___STAG_2;
            line_set_overflow = LINE_SET_N_O_STAG_2;
            subval_overflow = 7;
        } else if (abs(roll) > ANGLE_1) {
            line_set = LINE_SET_N___STAG_1;
            line_set_overflow = LINE_SET_N_O_STAG_1;
            subval_overflow = 8;
        }
    }

    pitch_line = round(tan(-AH_PITCH_FACTOR * osd_pitch) * AH_TOTAL_LINES) + AH_TOTAL_LINES / 2; // 90 total lines
    for (col = 1; col <= AH_COLS; col++) {
        middle = col * CHAR_COLS - (AH_COLS / 2 * CHAR_COLS) - CHAR_COLS / 2; // -66 to +66	center X point at middle of each column
        hit    = tan(AH_ROLL_FACTOR * osd_roll) * middle + pitch_line;               // 1 to 90	calculating hit point on Y plus offset
        if (hit >= 1 && hit <= AH_TOTAL_LINES) {
            row    = (hit - 1) / CHAR_ROWS;                                            // 0 to 4 bottom-up
            subval = (hit - (row * CHAR_ROWS) + 1) / (CHAR_ROWS / CHAR_SPECIAL); // 1 to 9

            // print the line char
            osd.openSingle(start_col + col - 1, start_row + AH_ROWS - row - 1);
            osd.printf("%c", line_set + subval);

            // check if we have to print an overflow line char
            if (subval >= subval_overflow && row < 4) { // only if it is a char which needs overflow and if it is not the upper most row
                osd.openSingle(start_col + col - 1, start_row + AH_ROWS - row - 2);
                osd.printf("%c", line_set_overflow + subval - OVERFLOW_CHAR_OFFSET);
            }
        }
    }
}

#endif // AH_BETTER_RESOLUTION

/*
void set_converts()
{
//    if (EEPROM.read(measure_ADDR) == 0) {
        convert_speed        = 3.6;
        convert_length       = 1.0;
        convert_length_large = 1000;
        unit_speed           = 0x81;
        unit_length          = 0x8D;
        unit_length_large    = 0xFD;
 /*   } else {
        convert_speed        = 2.23;
        convert_length       = 3.28;
        convert_length_large = 5280;
        unit_speed           = 0xfb;
        unit_length          = 0x66;
        unit_length_large    = 0xFA;
    } */ /*
} */
