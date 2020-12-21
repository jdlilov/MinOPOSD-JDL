// JDL
//
//
//

#define npanels 2 // # of possible panels


#ifdef BAT_VOLTAGE_CURVE
// Battery voltage to remaining capacity curve
static const PROGMEM uint8_t batt_curve[100] = {
    0,0,0,0,0,0,0,0,0,0,0,0,2,7,10,12,13,14,16,18,20,21,22,23,25,26,27,30,33,38,40,42,43,45,47,49,51,
    52,55,57,60,61,62,63,64,65,66,68,70,73,74,75,76,77,78,79,79,80,81,81,82,82,83,84,84,85,85,86,87,
    88,89,90,91,92,92,93,93,94,95,95,95,96,96,96,96,96,97,97,97,97,97,97,98,98,98,98,98,98,98,99
};

#ifdef LIHV_DETECTION
static const PROGMEM uint8_t batt_curve_lihv[100] = {
    0,0,0,0,0,0,0,0,0,0,0,0,0,1,10,15,19,21,25,29,32,33,35,37,39,41,43,45,47,48,49,51,53,55,57,58,60,
    62,64,66,67,68,68,69,70,71,72,73,74,75,77,78,79,80,80,81,82,82,83,84,85,86,88,89,90,91,91,92,93,
    93,94,94,94,95,95,95,96,96,96,97,97,97,98,98,98,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99
};
#endif
#endif

static const float batt_levels[5] = {5.0f, 8.8f, 13.2f, 17.45f, 21.8f};                                

static unsigned long loop10hz_prevmillis; // 10hz loop
#ifdef MAX_SOFTRESET
static unsigned long loop_1_hz_prevmillis; // 1hz loop
#endif

static uint8_t panel             = 0;          // active panel: 0 = first panel. 1 = second panel, 2 = panel off
static uint8_t old_panel         = 0;          

static int16_t chan1_raw         = 0;
static int16_t chan2_raw         = 0;

//static uint8_t ch_toggle         = 0;        // JDL Changed
//static boolean switch_mode       = false;    // JDL Changed

//static uint8_t overspeed         = 0;
#if defined STALL_WARNING    
static float stall_threshold     = 11.1111f; // 40km/h / 3.6;
#endif

//static int8_t temp_in_celsius;

static uint8_t battv             = 0;          // Battery warning voltage - (300 + xx) / 100.0
static float osd_vbat_A          = 0;          // Battery A voltage in milivolt
static int16_t osd_curr_A        = 0;          // Battery A current
static int batt_capacity         = 2000;       // Battery Capacity in mAh
 
#ifdef SAFETY_RADIUS
  #ifdef LT_RADIUS_ESTIMATION
  static int lt_safety_delta       = 0;
  #endif
  #ifdef ST_RADIUS_ESTIMATION
  static float st_flight_efficiency = 0;
  static int st_safety_delta       = 0;
  #endif
  #ifdef EFFICIENCY_ESTIMATION
  static float flight_efficiency = 0;
  #endif
  #ifdef BATP_RADIUS_ESTIMATION
  static float batp_flight_efficiency = 0;
  static int batp_safety_delta       = 0;
  #endif
#endif
static float osd_battery_remaining_percent_V = -1.0;     // 0 to 100
static float osd_battery_remaining_percent_A = -1.0;
//static uint8_t batt_warn_level   = 0;

static unsigned long engine_start_time = 0;    // start moment in milliseconds, used for precise remaining flight time estimation
static int flight_time = 0;                    // in milliseconds, this is the total flight duration sinse initial power burst

static uint8_t osd_mode          = 0;          // FlightMode of the FlightControl

static uint8_t osd_satellites_visible = 0;     // number of satelites
static uint8_t osd_fix_type      = 0;          // GPS lock 0-1=no fix, 2=2D, 3=3D, 4=DGps
static float osd_lat             = 0;          // latitude
static float osd_lon             = 0;          // longitude
static float osd_alt             = 0;          // altitude
static float osd_climb           = 0;          // climb rate
static float osd_groundspeed     = 0;          // ground speed
static float osd_3D_speed       = 0;           // vector speed
#ifdef FIXED_WING
static float osd_airspeed       = 0;           // vector speed
#endif
static float osd_travel_distance = 0;          // travel distance
static float osd_heading         = 0;          // ground course heading

static int16_t osd_roll          = 0;          // roll from FC
static int16_t osd_pitch         = 0;          // pitch from FC
static int16_t osd_yaw           = 0;          // relative heading from FC
static uint16_t osd_throttle     = 0;          // throttle

static uint8_t osd_alt_cnt       = 0;          // counter for stable osd_alt
static float osd_alt_prev        = 0;          // previous altitude
static uint8_t osd_got_home      = 0;          // tells if got home position or not
static float osd_home_lat        = 0;          // home latitude
static float osd_home_lon        = 0;          // home longitude
static float osd_home_alt        = 0;          // home altitude
static long osd_home_distance    = 0;          // distance from home
static long osd_direct_distance  = 0;          // distance from home in straight line, not projection over ground!
static uint8_t osd_home_direction;             // arrow direction pointing to home (1-16 to CW loop)

static int reset_home_flag       = 0;
//static int reset_trip_flag       = 0;

// OpenPilot UAVTalk:
//static uint8_t op_uavtalk_mode      = 0;       // OP UAVTalk mode, start with normal behavior
//static uint8_t op_alarm = 0;                   // OP alarm info
static uint8_t osd_armed = 0;                  // OP armed info
//#ifndef FIXED_WING
static uint8_t osd_aswa  = 0;                  // OP AlwaysStabilizeWhenArmed info
//#endif
//static uint8_t osd_time_hour        = 0;       // OP GPS time hour info
//static uint8_t osd_time_minute      = 0;       // OP GPS tiem minute info

#ifdef RSSI_ON_INPUT_CHANNEL
static int8_t rssi = -99;                        // RC Channel RSSI
#endif

static int8_t oplm_rssi = 0;                   // OPLM RSSI
static uint8_t oplm_linkquality     = 0;       // OPLM LinkQuality

//static int8_t oplm_receiver_rssi = 0;                   // OPLM Receiver RSSI
//static uint8_t oplm_receiver_linkquality     = 0;       // OPLM Receiver LinkQuality

static uint8_t osd_atti_status      = 0;       // ATTI   status 0=not found, 1=OK, 2=Warning, 3=Critical, 4=Error; arming possible when OK
static uint8_t osd_stab_status      = 0;       // STAB   status 0=not found, 1=OK, 2=Warning, 3=Critical, 4=Error; arming possible when OK

#ifdef PATHPLAN_WAYPOINTS_SUPPORT
//static uint8_t osd_pathplan_status  = 0;       // PATHPLAN	alarm status
//static uint8_t osd_pathdesired_mode;
//static uint16_t osd_pathplan_waypointcount;
//static float osd_velocitystate_north;
//static float osd_velocitystate_east;
//static float osd_waypoint_velocity;
static uint16_t osd_waypointactive_index = 0;
static float osd_pathdesired_end_north;
static float osd_pathdesired_end_east;
static float osd_positionstate_north;
static float osd_positionstate_east;
#endif

#ifndef REVO_ADD_ONS
static uint8_t osd_cpu_status       = 0;       // CPU    status 0=not found, 1=OK, 2=Warning, 3=Critical, 4=Error; arming possible when OK
static uint8_t osd_stack_status     = 0;       // STACK  status 0=not found, 1=OK, 2=Warning, 3=Critical, 4=Error; arming possible when OK
static uint8_t osd_memory_status    = 0;       // MEMORY status 0=not found, 1=OK, 2=Warning, 3=Critical, 4=Error; arming possible when OK
#endif

#ifdef REVO_ADD_ONS
static float revo_baro_alt        = 0;         // Revo baro altitude
static float osd_baro_home_alt   = -9999;      // baro home altitude
static float revo_baro_temp         = 0;       // Revo baro temperature
static uint8_t osd_mag_status       = 0;       // MAG status 0=not found, 1=OK, 2=Warning, 3=Critical, 4=Error
#endif

static float esc_temp               = 0;       // ESC temperature
static float motor_temp             = 0;       // Motor temperature
static float ambient_temp           = 0;       // Ambient temperature

//static uint8_t osd_receiver_quality = 0;       // Receiver Link Quality
//static float osd_txpid_cur[3]       = {};      // Current TXPID setting
//static uint8_t osd_temp_mode        = 0;       // Display temp or txpid values
static uint8_t batt_type            = 1;       // Battery type (1 = LiPo, 0 = LiIo)

// Flight Batt on MinimOSD:
static int volt_div_ratio    = 0;              // Volt * 100
static int curr_amp_per_volt = 0;              // Ampere * 100
static int curr_amp_offset   = 0;              // Ampere * 10000
// Flight Batt on MinimOSD and Revo
static uint16_t osd_total_A  = 0;              // Battery total current [mAh]
static uint16_t start_A = 0;                  // Consumed total current [mAh] in the start of flight
static uint16_t flight_A  = 0;                  // Consumed total current [mAh] in the start of flight
static int num_cells = 1;                      // Calculated number of batt cells. If not defined Flight Batt on MinimOSD, num_cells = 1
#ifdef LIHV_DETECTION                  
static bool LiHV_mode = false;
#endif
static float bpm = 137.08019;

static float start_batp = 99.0;
// Flight Batt on Revo
static uint16_t osd_est_flight_time = 0;       // Battery estimated flight time [sec]


// Panel BIT registers
byte panA_REG[npanels] = { 0b00000000 };
byte panB_REG[npanels] = { 0b00000000 };
byte panC_REG[npanels] = { 0b00000000 };
byte panD_REG[npanels] = { 0b00000000 };
byte panE_REG[npanels] = { 0b00000000 };


// First 8 panels and their X,Y coordinate holders
byte panCenter_XY[2][npanels];
byte panPitch_XY[2][npanels];
byte panRoll_XY[2][npanels];
byte panBatt_A_XY[2][npanels];
// byte panBatt_B_XY[2];
byte panGPSats_XY[2][npanels];
byte panGPL_XY[2][npanels];
byte panGPS_XY[2][npanels];
byte panBatteryPercent_XY[2][npanels];


// Second 8 set of panels and their X,Y coordinate holders
byte panRose_XY[2][npanels];
#ifndef COMBINED_HEADING_AND_ROSE
byte panHeading_XY[2][npanels];
#endif
byte panMavBeat_XY[2][npanels];
byte panHomeDir_XY[2][npanels];
byte panHomeDis_XY[2][npanels];
byte panWPDir_XY[2][npanels];
byte panSR_EGE_XY[2][npanels];
byte panTime_XY[2][npanels];


// Third set of panels and their X,Y coordinate holders
byte panCur_A_XY[2][npanels];
// byte panCur_B_XY[2];
byte panAlt_XY[2][npanels];
byte panHomeAlt_XY[2][npanels];
byte panVel_XY[2][npanels];
byte panAirSpeed_XY[2][npanels];
byte panThr_XY[2][npanels];
byte panFMod_XY[2][npanels];
byte panHorizon_XY[2][npanels];

// Fourth set of panels and their X,Y coordinate holders
byte panWarn_XY[2][npanels];
// byte panOff_XY[2];
byte panWindSpeed_XY[2][npanels];
byte panClimb_XY[2][npanels];
byte panTune_XY[2][npanels];
// byte panSetup_XY[2];
byte panRSSI_XY[2][npanels];
byte panDistance_XY[2][npanels];

// Fifth set of panels and their X,Y coordinate holders
byte panTemp_XY[2][npanels];

// *************************************************************************************************************
// rssi variables
//static uint8_t rssipersent     = 0;
//static uint8_t rssical         = 0;
//static uint8_t osd_rssi        = 0;            // value from PacketRxOk or analogRSSI
//static int16_t rssi            = -99;          // scaled value 0-100%
//static boolean rssiraw_on      = false;        // 0- display scale value | 1- display raw value

// raw channel variables
//static uint16_t osd_chan5_raw  = 1000;       // JDL Disabled
//static uint16_t osd_chan6_raw  = 1000;
static uint16_t osd_chan7_raw  = 1000;
//static uint16_t osd_chan8_raw  = 1000;
#ifdef RSSI_ON_INPUT_CHANNEL
static uint16_t osd_chan10_raw  = 2000;
#endif


