// JDL
//
// hex files under C:\Users\...\AppData\Local\Temp\build....tmp
//

#define on  1
#define off 0

// Version number, incrementing this will erase/upload factory settings.
// Only devs should increment this
//#define VER 75 - this is not in use in LP MinOpOsd

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// JDL: LP16.09 or "next" r.735+ UAVOs
#define NEXT_UAVO

// JDL: Display Startup Logo
#define LOGO_AT_BOOT

// JDL: Update Charset Furnctionality 
#define CHARSET_UPLOADER

// JDL: Display PathPlan Waypoints Progress
#define PATHPLAN_WAYPOINTS_SUPPORT

// JRChange: Flight Batt on MinimOSD:
//#define FLIGHT_BATT_ON_MINIMOSD

// JRChange: Flight Batt on Revo:
#define FLIGHT_BATT_ON_REVO

// OpenPilot Revo add ons
#define REVO_ADD_ONS

// JDL: RSSI from RC Channel - for CC3D ZMR250
//#define RSSI_ON_INPUT_CHANNEL

// JDL: Battery type (LiPo, LiIo) selection - for Wing Z-84 currently only
//#define BATTTYPE_SELECTION

// JDL: Correction of Remaining Battery % Estimation (voltage based)
#define BAT_VOLTAGE_CURVE                                            // DOESN'T WORK for LiIon (FixedWing) right now - it is disabled in the code for LiIons; currently optimised for Turnigy Graphene & Nano-Tech LiPos

// JDL: LiHV battery type detection - for Quads currently only
#define LIHV_DETECTION

// JDL: Try to soft-reset MAX7456 after hard-reset / if stalled state detected
#define MAX_SOFTRESET                

// JDL: address issue with Runcam2 4K black level too high
//#define RUNCAM2_4K_FIX

// JDL: safety radius
#define SAFETY_RADIUS
#define LT_RADIUS_ESTIMATION
#define ST_RADIUS_ESTIMATION
#define EFFICIENCY_ESTIMATION
//#define BATP_RADIUS_ESTIMATION


//#define TEMP_SENSOR_CALIBRATION_VOLTAGE_4_966V        // For ZMR250v2 Revo4 & EC250 Revo2 QUADS
//#define TEMP_SENSOR_CALIBRATION_VOLTAGE_4_656V        // For Wing Z-84
//#define TEMP_SENSOR_CALIBRATION_VOLTAGE_4_865V        // For GepRC Mark4 Revo6 QUAD

// JDL: Using external LM335Z Sensor for ESC temp monitoring - Wing Z-84 & ZMR250 Revo QUADS & MiniTalon
//#define TEMP_SENSOR_LM335Z_ESC

// JDL: Using second external LM335Z Sensor for Motor temp monitoring - MiniTalon
//#define TEMP_SENSOR_LM335Z_MOTOR

// JDL: Using third LM335Z Sensor for Ambient temp monitoring - MiniTalon & GepRC Mark4 
//#define TEMP_SENSOR_LM335Z_AMBIENT


#define TEMP_SENSOR_PIN_ESC      A1
#define TEMP_SENSOR_PIN_MOTOR    A3
//#define TEMP_SENSOR_PIN_AMBIENT  A2  // A2 for GepRC Mark4  // Batt1
#define TEMP_SENSOR_PIN_AMBIENT  A0  // A0 for MiniTalon  // Batt2

// Batt1 (to the right) - A2
// Batt2 (to the left)  - A0
// RSSI                 - A3
// CURR                 - A1

// JDL: Fixed Wing mode
//#define FIXED_WING

// JDL: Low speed stall warning for fixed wing
//#define STALL_WARNING

// JDL: Glide distance estimation for Fixed Wing
//#define GLIDE_ESTIMATION

// Version string
#define VERSION_STRING          osd.printf_P(PSTR("minoposd jdl 2.56"));

// ---------------------------------------------------------------------

#ifdef RSSI_ON_INPUT_CHANNEL
#define rssi_warn_level              -71
#endif  

#define oplm_rssi_warn_level         -84
#define oplm_linkq_warn_level        105

#define esc_overheat_threshold       60
#define motor_overheat_threshold     75

#define stall_threshold_warn_margin  4 / 3.6         // 4km/h in m/s

//ZMR250v2 Quad Temp Sensor Calibration
//#define esc_temp_correction          0.0 * 0.2          // correction in degC, multiplied by 0.2

//EC250 ZMR250v1 Quad Temp Sensor Calibration
//#define esc_temp_correction          +1.0 * 0.2          // correction in degC, multiplied by 0.2

// Mini Talon Temp Sensors Calibrations 
//#define esc_temp_correction        -3.0 * 0.2          // correction in degC, multiplied by 0.2
//#define motor_temp_correction      -2.0 * 0.2
//#define ambient_temp_correction    -3.0 * 0.2

// Wing-Z84 Temp Sensors Calibrations 
//#define esc_temp_correction          +3.0 * 0.2          // correction in degC, multiplied by 0.2

// GepRC Mark4 Temp Sensors Calibration 
//#define ambient_temp_correction      +2.0 * 0.2          // correction in degC, multiplied by 0.2

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Add MicroOSD KV team mod            // JDL: Enable it!
#define MICRO_OSD_KVTEAM

// JDL: Integrate heading into rose
#define COMBINED_HEADING_AND_ROSE      // JDL: Keep it always enabled! 

// JDL: Enable, compile, upload, setup batt capacity (via RC TX sticks), then disable, compile, upload - only useful for initial setup of a brand new MinOpOSD board
//#define DO_ROUND_STORED_BATT_CAPACITY    // Obsolete
//
//#define stall_threshold              40 / 3.6        // Speed in m/s !! - for Mini Talon - read from EEPROM (2.49+)!
//#define stall_threshold              30 / 3.6        // Speed in m/s !! - for Wing Z84 - read from EEPROM (2.49+)!
//
//#define FIRMWARE_TYPE 1    // 1 - Quad(Revo); 2 - Plane(Revo); 3 - Quad(CC3D)
//#define CHECK_FIRMWARE_TYPE
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//JDL: Type of RX output
//#define SBUS_RX

//#define NEXT_R616

// JRChange: JR specials
#define JR_SPECIALS

// JRChange: OpenPilot UAVTalk:
#define PROTOCOL_UAVTALK

// JRChange: artificial horizon original version
// #define AH_ORIGINAL_VERSION

// JRChange: artificial horizon refactored version
// #define AH_REFACTORED_VERSION

// JRChange: artificial horizon zero centered
// #define AH_ZERO_CENTERED

// JRChange: artificial horizon with better resolution
#define AH_BETTER_RESOLUTION

// JRChange: OP debug
// #define OP_DEBUG


// JRChange: GPS simulation
// #define GPS_SIMULATION

// Version
// #define VERSION_RELEASE_12_10_1		// OpenPilot-RELEASE 12.10.1	Release from 2012.10.26
// #define VERSION_RELEASE_12_10_2		// OpenPilot-RELEASE 12.10.2	'Mayan Apocalypse'
// #define VERSION_RELEASE_13_06_1		// OpenPilot-RELEASE 13.06.1	'Italian Stallion' .1
// #define VERSION_RELEASE_13_06_2		// OpenPilot-RELEASE 13.06.2	'Italian Stallion' .2
// #define VERSION_RELEASE_14_01_1		// OpenPilot-RELEASE 14.01.1	'Cruising Ratt' .1
// #define VERSION_RELEASE_14_06_1		// OpenPilot-RELEASE 14.06.1	'Peanuts Schnapps' .1
// #define VERSION_RELEASE_14_10_1		// OpenPilot-RELEASE 14.10.1	'Mini Me' .1
// #define VERSION_RELEASE_15_01_1		// OpenPilot-RELEASE 15.01.1	'Look, Ma... No Hands!' .1
// #define VERSION_RELEASE_15_02_1		// OpenPilot-RELEASE 15.02.1	'Rajin Cajun' .1
// #define VERSION_RELEASE_15_05
#define VERSION_RELEASE_LP15_09 // First LibrePilot  release     '15.09 Supermoon eclipse'

#define MEASURE_PERIOD 95 // ms


// EEPROM Stepping, be careful not to overstep.
// We reserved floats for just to be sure if some values needs to be
// changed in future.
// byte  = 1
// int   = 4
// float = 8

// Panel 8bit REGISTER with BIT positions
// panA_REG Byte has:
//#define Cen_BIT                   0
#define Pit_BIT                   1
#define Rol_BIT                   2
#define BatA_BIT                  3
#define Bp_BIT                    4
#define GPSats_BIT                5
#define GPL_BIT                   6
#define GPS_BIT                   7

// panB_REG Byte has:
#define Rose_BIT                  0
#define Head_BIT                  1
//#define MavB_BIT                  2
#define HDir_BIT                  3
#define HDis_BIT                  4
#define WDir_BIT                  5
#define SR_EGE_BIT                6 
#define Time_BIT                  7

// panC_REG Byte has:
#define CurA_BIT                  0
#define As_BIT                    1
#define Alt_BIT                   2
#define Vel_BIT                   3
#define Thr_BIT                   4
#define FMod_BIT                  5
#define Hor_BIT                   6
#define Halt_BIT                  7

// panD_REG Byte has:
#define Warn_BIT                  0
//#define Off_BIT                   1
//#define WindS_BIT                 2
#define Climb_BIT                 3
//#define Tune_BIT                  4      // JDL Changed
//#define CALLSIGN_BIT              5
#define RSSI_BIT                  6
//#define Eff_BIT                   7

// panE_REG Byte has:

//#define Ch_BIT                    0
#define TEMP_BIT                  1
#define DIST_BIT                  2

/* *********************************************** */
// EEPROM Storage addresses

#define OffsetBITpanel            250

// First of 8 panels
//#define panCenter_en_ADDR         0
//#define panCenter_x_ADDR          2
//#define panCenter_y_ADDR          4
#define panPitch_en_ADDR          6
#define panPitch_x_ADDR           8
#define panPitch_y_ADDR           10
#define panRoll_en_ADDR           12
#define panRoll_x_ADDR            14
#define panRoll_y_ADDR            16
#define panBatt_A_en_ADDR         18
#define panBatt_A_x_ADDR          20
#define panBatt_A_y_ADDR          22
#define panBatt_B_en_ADDR         24
#define panBatt_B_x_ADDR          26
#define panBatt_B_y_ADDR          28
#define panGPSats_en_ADDR         30
#define panGPSats_x_ADDR          32
#define panGPSats_y_ADDR          34
#define panGPL_en_ADDR            36
#define panGPL_x_ADDR             38
#define panGPL_y_ADDR             40
#define panGPS_en_ADDR            42
#define panGPS_x_ADDR             44
#define panGPS_y_ADDR             46

// Second set of 8 panels
#define panRose_en_ADDR           48
#define panRose_x_ADDR            50
#define panRose_y_ADDR            52
#define panHeading_en_ADDR        54
#define panHeading_x_ADDR         56
#define panHeading_y_ADDR         58
//#define panMavBeat_en_ADDR        60
//#define panMavBeat_x_ADDR         62
//#define panMavBeat_y_ADDR         64
#define panHomeDir_en_ADDR        66
#define panHomeDir_x_ADDR         68
#define panHomeDir_y_ADDR         70
#define panHomeDis_en_ADDR        72
#define panHomeDis_x_ADDR         74
#define panHomeDis_y_ADDR         76
#define panWPDir_en_ADDR          80
#define panWPDir_x_ADDR           82
#define panWPDir_y_ADDR           84
#define panSR_EGE_en_ADDR         86
#define panSR_EGE_x_ADDR          88
#define panSR_EGE_y_ADDR          90
#define panRSSI_en_ADDR           92
#define panRSSI_x_ADDR            94
#define panRSSI_y_ADDR            96

// Third set of 8 panels
#define panCur_A_en_ADDR          98
#define panCur_A_x_ADDR           100
#define panCur_A_y_ADDR           102
//#define panCurB_en_ADDR           104 // (!Not implemented)
//#define panCurB_x_ADDR            106 //
//#define panCurB_y_ADDR            108 //
#define panAlt_en_ADDR            110
#define panAlt_x_ADDR             112
#define panAlt_y_ADDR             114
#define panVel_en_ADDR            116
#define panVel_x_ADDR             118
#define panVel_y_ADDR             120
#define panThr_en_ADDR            122
#define panThr_x_ADDR             124
#define panThr_y_ADDR             126
#define panFMod_en_ADDR           128
#define panFMod_x_ADDR            130
#define panFMod_y_ADDR            132
#define panHorizon_en_ADDR        134
#define panHorizon_x_ADDR         136
#define panHorizon_y_ADDR         138
#define panHomeAlt_en_ADDR        140
#define panHomeAlt_x_ADDR         142
#define panHomeAlt_y_ADDR         144
#define panAirSpeed_en_ADDR       146
#define panAirSpeed_x_ADDR        148
#define panAirSpeed_y_ADDR        150
#define panBatteryPercent_en_ADDR 152
#define panBatteryPercent_x_ADDR  154
#define panBatteryPercent_y_ADDR  156
#define panTime_en_ADDR           158
#define panTime_x_ADDR            160
#define panTime_y_ADDR            162
#define panWarn_en_ADDR           164
#define panWarn_x_ADDR            166
#define panWarn_y_ADDR            168
//#define panOff_en_ADDR            170
//#define panOff_x_ADDR             172
//#define panOff_y_ADDR             174
//#define panWindSpeed_en_ADDR      176
//#define panWindSpeed_x_ADDR       178
//#define panWindSpeed_y_ADDR       180
#define panClimb_en_ADDR          182
#define panClimb_x_ADDR           184
#define panClimb_y_ADDR           186
//#define panTune_en_ADDR           188
//#define panTune_x_ADDR            190
//#define panTune_y_ADDR            192
//#define panEff_en_ADDR            194
//#define panEff_x_ADDR             196
//#define panEff_y_ADDR             198
//#define panCALLSIGN_en_ADDR       200
//#define panCALLSIGN_x_ADDR        202
//#define panCALLSIGN_y_ADDR        204
//#define panCh_en_ADDR             206
//#define panCh_x_ADDR              208
//#define panCh_y_ADDR              210

#define panTemp_en_ADDR           212
#define panTemp_x_ADDR            214
#define panTemp_y_ADDR            216
//#define panFdata_en_ADDR          218
//#define panFdata_x_ADDR           220
//#define panFdata_y_ADDR           222
#define panDistance_en_ADDR       224
#define panDistance_x_ADDR        226
#define panDistance_y_ADDR        228

//#define measure_ADDR              890      // keep this in OSD Setup, even not in use!
//#define overspeed_ADDR            892
#define stall_ADDR                894
#define battv_ADDR                896
//#define battp_ADDR                898
//#define OSD_RSSI_HIGH_ADDR        900
//#define OSD_RSSI_LOW_ADDR         902
//#define RADIO_ON_ADDR             904
//#define ch_toggle_ADDR            906
//#define OSD_RSSI_RAW_ADDR         908
//#define switch_mode_ADDR          910
#define PAL_NTSC_ADDR             912

//#define OSD_BATT_WARN_ADDR        914
//#define OSD_RSSI_WARN_ADDR        916

//#define OSD_BRIGHTNESS_ADDR       918

//#define OSD_CALL_SIGN_ADDR        920      // keep this in OSD Setup, even not in use!
//#define OSD_CALL_SIGN_TOTAL       8        // keep this in OSD Setup, even not in use!

//#define CHK1                      1000
//#define CHK2                      1006

// JRChange: Flight Batt on MinimOSD:
//#define BATT_VER                  1
//#define BATT_CHK                  1011

#define volt_div_ratio_ADDR       1012
#define curr_amp_per_volt_ADDR    1014
#define curr_amp_offset_ADDR      1016

//#define temp_mode_ADDR            1019

#define batt_type_ADDR            1020

#define batt_capacity_ADDR        1021

#define firmware_type_ADDR        1023       // 1 - Quad(Revo); 2 - Plane(Revo); 3 - Quad(CC3D)

#define EEPROM_MAX_ADDR           1024       // this is 328 chip
