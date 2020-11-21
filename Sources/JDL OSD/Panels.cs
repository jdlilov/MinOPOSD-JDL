using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace OSD
{
    using uint16_t = System.UInt16;
    using uint8_t = System.Byte;
    using int8_t = System.SByte;
    using boolean = System.Byte;


    
    // this is mainly copied from osd_panels.pde


    class Panels
    {
        OSD osd;

        public Panels(OSD os)
        {
            osd = os;
        }

        string PSTR(string input)
        {
            return input;
        }

        double abs(double input)
        {
            return Math.Abs(input);
        }

        int round(double input)
        {
            return (int)Math.Round(input, 0);
        }

        double tan(double input)
        {
            return Math.Tan(input);
        }

        /*Panels variables*/
        //Will come from APM telem port

        //Config vars
        public uint8_t overspeed = 40;
        public uint8_t stall = 40;
        public uint8_t battv = 56;                //Batery (Single Cell) warning voltage - ((units Volt *10) + 3.0)
        public uint8_t batt_type = 1;            // Battery type (1 == LiPo, 0 == Li-Ion)
        public uint8_t converts = 0;                //1- Imperial; 0- Metric

        public int volt_div_ratio = 1539;
        public int curr_amp_per_volt = 2732;
        public int curr_amp_offset = 0;

        public uint8_t firmware_type = 1;            // Firmware Type: 1 = Quad(Revo); 2 = Plane(Revo); 3 = Quad(CC3D)

        public int batt_capacity = 2250;       // Battery Capacity in mAh

        static uint8_t unit_length = 0x8D;
        static uint8_t unit_speed = 0x81;


        static float osd_vbat = 15.23f;                   // voltage in milivolt
        static uint8_t num_cells = 4;

//        static uint16_t osd_battery_remaining;      // 0 to 100 <=> 0 to 1000
//        static uint8_t osd_battery_pic;                  // picture to show battery remaining

        static uint16_t osd_mode = 3;                   // Navigation mode from RC AC2 = CH5, APM = CH8
//        static uint8_t osd_nav_mode = 4;               // Navigation mode from RC AC2 = CH5, APM = CH8

        static float osd_lat = 42.044185f;                    // latidude
        static float osd_lon = 28.000488f;                    // longitude

        static uint8_t osd_satellites_visible = 21;     // number of satelites
        static uint8_t osd_fix_type = 4;               // GPS lock 0-1=no fix, 2=2D, 3=3D, 4=DGPS
//        static int start_Time = 2;

        static float lt_safety_delta = 2083f;
        static float batp_safety_delta = 2083f;
        static float st_safety_delta = 1824f;
        static float flight_efficiency = 3850f;

        static uint16_t flight_time = 312;
        static uint16_t osd_est_flight_time = 168;

        static uint16_t osd_total_A = 1218;
        static float osd_battery_remaining_percent_V = 44f;
        static float osd_battery_remaining_percent_A = 46f;

        static float revo_baro_temp = 62f;

        static float osd_direct_distance = 2285f;

        static float revo_baro_alt = 123f;
        static float osd_baro_home_alt = 67f;

        static float osd_travel_distance = 7533f;

        static float osd_3D_speed = 71;           // vector speed
        static float osd_airspeed = 58;           // vector speed

        //static uint8_t osd_got_home = 0;               // tels if got home position or not
        //static float osd_home_lat = 0;               // home latidude
        //static float osd_home_lon = 0;               // home longitude
        static float osd_home_alt = 65f;              //Home altitude
        static float osd_home_distance = 2280f;          // distance from home
//        static uint8_t osd_home_direction = 2;             // Arrow direction pointing to home (1-16 to CW loop)

        static int8_t osd_pitch = 0;                  // pitch form DCM
        static int8_t osd_roll = 0;                   // roll form DCM
        //static uint8_t osd_yaw = 0;                    // relative heading form DCM
        static float osd_heading = 172f;                // ground course heading from GPS
        static float osd_alt = 122f;                    // altitude
        static float osd_groundspeed = 67f;            // ground speed
        static uint16_t osd_throttle = 37;               // throtle
        static float osd_curr_A = 453;
        static float osd_windspeed = 10;
        static float osd_windspeedz = 2;
        static float osd_climb = -1;
        static float nav_roll = 0;
        static float nav_pitch = 0;
        static uint16_t nav_bearing = 0; // Current desired heading in degrees
        static uint16_t wp_target_bearing = 0; // Bearing to current MISSION/target in degrees
//        static uint16_t wp_dist = 9000; // Distance to active MISSION in meters
//        static uint16_t wp_number = 99; // Distance to active MISSION in meters
        static float alt_error = 0; // Current altitude error in meters
        static float aspd_error = 0; // Current airspeed error in meters/second
        static float xtrack_error = 0; // Current crosstrack error on x-y plane in meters
        static float eff = 10;

        //MAVLink session control
        static boolean mavbeat = 1;
        //static float lastMAVBeat = 0;
        //static boolean waitingMAVBeats = 1;
//        static uint8_t apm_mav_type = 2;
        //static uint8_t apm_mav_system = 7;
        //static uint8_t apm_mav_component = 0;
        //static boolean enable_mav_request = 0;
        //rssi varables
        //public uint8_t rssi = 0;
        public uint8_t rssipersent = 0;
        public uint8_t rssical = 255;
        public uint8_t rssiraw_on = 0;
        static int8_t osd_rssi = -86;
        static uint8_t osd_linkquality = 125;
        public uint8_t radio_setup_flag = 0;
        public uint8_t ch_toggle = 8; //CH8
        public boolean switch_mode = 0;
        public boolean pal_ntsc = 1; //PAL 1 - NTSC 0
        public uint8_t osd_brightness = 0; // low bright
        
        public uint8_t rssi_warn_level = 5;
//        public uint8_t batt_warn_level = 10;

        public string callsign_str = "a1b2c3d4";
        //public uint8_t[] call_sign_parse = new uint8_t[6];

        public uint8_t chan1_raw = 0;
        public uint8_t chan2_raw = 0;
        public uint8_t chan3_raw = 0;
        public uint8_t chan4_raw = 0;
        public uint8_t chan5_raw = 0;
        public uint8_t chan6_raw = 0;
        public uint8_t chan7_raw = 0;
        public uint8_t chan8_raw = 0;


        /******* PANELS - DEFINITION *******/

        /* **************************************************************** */
        // Panel  : PanCh
        // Needs  : X, Y locations
        // Output : 
        // Size   : 1 x 7Hea  (rows x chars)
        // Staus  : done

        public int panCh(int first_col, int first_line)
        {
//            osd.setPanel(first_col, first_line);
//            osd.openPanel();
//            {
//                osd.printf("%c%c%5i|%c%c%5i|%c%c%5i|%c%c%5i|%c%c%5i|%c%c%5i|%c%c%5i|%c%c%5i|", 0x43, 0x31, chan1_raw, 0x43, 0x32, chan2_raw, 0x43, 0x33, chan3_raw, 0x43, 0x34, chan4_raw, 0x43, 0x35, chan5_raw, 0x43, 0x36, chan6_raw, 0x43, 0x37, chan7_raw, 0x43, 0x38, chan8_raw);
//            }
//            osd.closePanel();
            return 0;
        }

        /* **************************************************************** */
        // Panel  : efficiency
        // Needs  : X, Y locations
        // Output : 
        // Size   : 1 x 7Hea  (rows x chars)
        // Staus  : done

        public int panEff(int first_col, int first_line)
        {
        osd.setPanel(first_col, first_line);
        osd.openPanel();
        {
        osd.printf("%c%3.0f%c", 0x17, eff, 0x82);
        }
        osd.closePanel();
         return 0;
        }

        /* **************************************************************** */
        // Panel  : panDistance     // JDL Fixed 2
        // Needs  : X, Y locations
        // Output : Odometer (Distance Travelled)
        // Size   : 1 x 7Hea  (rows x chars)
        // Staus  : done

        public int panDistance(int first_col, int first_line)
        {
            osd.setPanel(first_col, first_line);
            osd.openPanel();
            {
                osd.printf("%c%5.0f%c", 0xFE, (osd_travel_distance), unit_length);
            }
            osd.closePanel();
            return 0;
        }

        /* **************************************************************** */
 	    // Panel  : panRSSI     // JDL Fixed 2
 	    // Needs  : X, Y locations
 	    // Output : Alt symbol and altitude value in meters from MAVLink
 	    // Size   : 1 x 7Hea  (rows x chars)
  	    // Staus  : done

 	    public int panRSSI(int first_col, int first_line)
        {
 	        osd.setPanel(first_col, first_line);
 	        osd.openPanel();
 	        {
                if ((firmware_type == 1) || (firmware_type == 2))
                {
                    osd.printf("%c%4i%c %4i%c", 0xE1, osd_rssi, 0x8C, osd_linkquality, 0x8B);
                }
                else
                {
                    osd.printf("%c%3i%c", 0xE1, osd_rssi, 0x8C);
                }
 	        }
            osd.closePanel();
 	        return 0;
 	    }

        /* **************************************************************** */
        // Panel  : panBaroTemp     // JDL Fixed 2
        // Needs  : X, Y locations
        // Output : Temperarture Readings
        // Size   : 1 x 7Hea  (rows x chars)
        // Staus  : done

        public int panBaroTemp(int first_col, int first_line)
        {
            osd.setPanel(first_col, first_line);
            osd.openPanel();
            {
                if ((firmware_type == 1) || (firmware_type == 2))
                {
                    osd.printf("%c%3.0f%c", 0xE3, revo_baro_temp, 0xB0);
                }
                else
                {
                    osd.printf("no temp");
                }
            }
            osd.closePanel();
            return 0;
        }

        /* **************************************************************** */
        // Panel  : panCALLSIGN
        // Needs  : X, Y locations
        // Output : Call sign identification
        // Size   : 1 x 6Hea  (rows x chars)
        // Staus  : done

        public int panCALLSIGN(int first_col, int first_line)
        {
            osd.setPanel(first_col, first_line);
            osd.openPanel();
            osd.printf("%s", callsign_str);
            osd.closePanel();
            return 0;
        }
 
        /******* PANELS - DEFINITION *******/

        /* **************************************************************** */
        // Panel  : panTune
        // Needs  : X, Y locations
        // Output : Current symbol and altitude value in meters from MAVLink
        // Size   : 1 x 7Hea  (rows x chars)
        // Staus  : done

        public int panTune(int first_col, int first_line)
        {
            osd.setPanel(first_col, first_line);
            osd.openPanel();
            
            {
                osd.printf("%c%c%2.0f%c|%c%c%2.0f%c|%c%c%4.0i%c|%c%c%4.0i%c|%c%c%3.0f%c|%c%c%3.0f%c|%c%c%4.0f%c", 0x4E, 0x52, (nav_roll), 0xB0, 0x4E, 0x50, (nav_pitch), 0xB0, 0x4E, 0x48, (nav_bearing), 0xB0, 0x54, 0x42, (wp_target_bearing), 0xB0, 0x41, 0x45, (alt_error), 0x8D, 0x58, 0x45, (xtrack_error), 0x6D, 0x41, 0x45, ((aspd_error / 100.0) * converts), 0x88);
            }  
            osd.closePanel();
            return 0;
        }

        /* **************************************************************** */
        // Panel  : panClimb        // JDL Fixed 2
        // Needs  : X, Y locations
        // Output : Alt symbol and altitude value in meters from MAVLink
        // Size   : 1 x 7Hea  (rows x chars)
        // Staus  : done

        public int panClimb(int first_col, int first_line)
        {
            osd.setPanel(first_col, first_line);
            osd.openPanel();

            if (firmware_type == 2)
            {
                double osd_climb_m_min = abs(osd_climb * 60.0);
                uint8_t symbol_climb = 0x7D;
                if (osd_climb > 0.0)
                {
                    symbol_climb = 0x7B;
                }
                osd.printf("%c%4.0f%c", symbol_climb, osd_climb_m_min, 0x7F);
            }
            else
            {
                osd.printf("%c%3.0f%c", 0x16, osd_climb, 0x88);
//                osd.printf("%c%3.0f%c", 0x16, (double)(osd_climb), 0x88);
            }
            
            osd.closePanel();
            return 0;
        }
        
        /* **************************************************************** */
        // Panel  : pan wind speed
        // Needs  : X, Y locations
        // Output : Velocity value from MAVlink with symbols
        // Size   : 1 x 7  (rows x chars)
        // Staus  : done

        public int panWindSpeed(int first_col, int first_line)
        {
            osd.setPanel(first_col, first_line);
            osd.openPanel();
            {
                osd.printf("%c%3.0f%c|%c%c%2.0f%c", 0xFC, (double)(osd_windspeed * 3.6), 0x81, 0xA4, 0xA5, (double)(osd_windspeedz * 3.6), 0x81);
            }
            osd.closePanel();
            return 0;
        }

        /* **************************************************************** */
        // Panel  : panOff
        // Needs  : X, Y locations
        // Output : OSD off
        // Size   : 1 x 7Hea  (rows x chars)
        // Staus  : done

        public int panOff(int first_col, int first_line)
        {
            osd.setPanel(first_col, first_line);
            osd.openPanel();
            {
               // osd.printf("%c", 0x00);
            }
            osd.closePanel();
            return 0;
        }


        /* **************************************************************** */
        // Panel  : panCur_A            // JDL Fixed 2
        // Needs  : X, Y locations
        // Output : Current symbol and altitude value in meters from MAVLink
        // Size   : 1 x 7Hea  (rows x chars)
        // Staus  : done

        public int panCur_A(int first_col, int first_line)
        {
            osd.setPanel(first_col, first_line);
            osd.openPanel();
            {
                osd.printf("%c%5.2f%c", 0xE4, osd_curr_A * .01, 0x8F);
                
//                osd.printf("%c%5.2f%c%c", 0xE4, (osd_curr_A * .01), 0x8F);
            }
            osd.closePanel();
            return 0;
        }

        /* **************************************************************** */
        // Panel  : panAlt          // JDL Fixed 2
        // Needs  : X, Y locations
        // Output : Alt symbol and altitude value in meters from MAVLink
        // Size   : 1 x 7Hea  (rows x chars)
        // Staus  : done

        public int panAlt(int first_col, int first_line)
        {
            osd.setPanel(first_col, first_line);
            osd.openPanel();
            //osd.printf("%c%5.0f%c",0x85, (double)(osd_alt - osd_home_alt), 0x8D);
//            osd.printf("%c%5.0f%c", 0xE6, (double)(osd_alt), 0x8D);

            if ((firmware_type == 1) || (firmware_type == 2))
            {
                osd.printf("%c%5.0f%c %c%4.0f%c", 0x85, (float)osd_alt, unit_length, 0xE7, (float)osd_alt - osd_home_alt, unit_length);
            }
            else
            {
                osd.printf("%c%5.0f%c", 0x85, (float)osd_alt, unit_length);
            }

            osd.closePanel();
            return 0;
        }
        /* **************************************************************** */
        // Panel  : panWarning  // JDL Fixed 2
        // Needs  : X, Y locations
        // Output : Warning panel
        // Size   : 1 x 7Hea  (rows x chars)
        // Staus  : done
         public int panWarn(int first_col, int first_line){
         osd.setPanel(first_col, first_line);
         osd.openPanel();
//         osd.printf_P(PSTR("\x20\x4f\x76\x65\x72\x53\x70\x65\x65\x64\x21\x20"));
         osd.printf_P(PSTR("link quality"));
         osd.closePanel();
         return 0;
         }

        /* **************************************************************** */
        // Panel  : panHomeAlt          // JDL Fixed 2
        // Needs  : X, Y locations
        // Output : Alt symbol and home altitude value in meters from MAVLink
        // Size   : 1 x 7Hea  (rows x chars)
        // Staus  : done

        public int panHomeAlt(int first_col, int first_line)
        {
            osd.setPanel(first_col, first_line);
            osd.openPanel();

            if ((firmware_type == 1) || (firmware_type == 2))
            {
                osd.printf("%c%4.0f%c", 0xE6, (float)((revo_baro_alt - osd_baro_home_alt)), unit_length);
            }
            else
            {
                osd.printf("%c%4.0f%c", 0xE7, (float)((osd_alt - osd_home_alt)), unit_length);
            }
               
                //osd.printf("%c%5.0f%c",0x85, (double)(osd_alt - osd_home_alt), 0x8D);
//            osd.printf("%c%5.0f%c", 0xE7, (double)(osd_alt - osd_home_alt), 0x8D);
            osd.closePanel();
            return 0;
        }

        /* **************************************************************** */
        // Panel  : panVel              // JDL Fixed 2
        // Needs  : X, Y locations
        // Output : Velocity value from MAVlink with symbols
        // Size   : 1 x 7  (rows x chars)
        // Staus  : done

        public int panVel(int first_col, int first_line)
        {
            osd.setPanel(first_col, first_line);
            osd.openPanel();
            osd.printf("%c%3.0f%c", 0xE9, (float)osd_groundspeed, unit_speed);
//            osd.printf("%c%3.0f%c", 0xE9, (double)osd_groundspeed, 0x81);
            osd.closePanel();
            return 0;
        }

        /* **************************************************************** */
        // Panel  : panAirSpeed     // JDL Fixed 2
        // Needs  : X, Y locations
        // Output : Airspeed value from MAVlink with symbols
        // Size   : 1 x 7  (rows x chars)
        // Staus  : done

        public int panAirSpeed(int first_col, int first_line)
        {
            osd.setPanel(first_col, first_line);
            osd.openPanel();

            if (firmware_type == 2)
            {
                osd.printf("%c%3.0f%c|%c%3s%c", 0xE8, (float)osd_3D_speed, unit_speed, 0x89, osd_airspeed.ToString(), unit_speed);
            }
            else
            {
                osd.printf("%c%3.0f%c", 0xE8, (float)osd_3D_speed, unit_speed);
            }
//            osd.printf("%c%3.0f%c", 0xE8, (double)(osd_airspeed), 0x81);
            osd.closePanel();
            return 0;
        }

        /* **************************************************************** */
        // Panel  : panBatteryPercent       // JDL Fixed 2
        // Needs  : X, Y locations
        // Output : Battery state from MAVlink with symbols
        // Size   : 1 x 7  (rows x chars)
        // Staus  : done

        public int panBatteryPercent(int first_col, int first_line)
        {
            osd.setPanel(first_col, first_line);
            osd.openPanel();
            if ((firmware_type == 1) || (firmware_type == 2))
            {
                osd.printf("%c%5i%c|%c%2.0f %c%2.0f", 0xB9, osd_total_A, 0x82, 0xB7, osd_battery_remaining_percent_V, 0xB6, osd_battery_remaining_percent_A);
            }
            else
            {
                osd.printf("%c%2.0f%c", 0xB7, osd_battery_remaining_percent_V, 0x25);
            }
                
//            osd.printf("%c%3.0i%c", 0xB9, (osd_battery_remaining), 0x25);
            osd.closePanel();
            return 0;
        }

        /* **************************************************************** */
        // Panel  : panTime     // JDL Fixed 2
        // Needs  : X, Y locations
        // Output : Time from start with symbols
        // Size   : 1 x 7  (rows x chars)
        // Staus  : done

        public int panTime(int first_col, int first_line)
        {
            osd.setPanel(first_col, first_line);
            osd.openPanel();
            if ((firmware_type == 1) || (firmware_type == 2))
            {
                osd.printf("%c%2i%c%2i|%c%2i%c%2i", 0xB3, ((int)(flight_time / 60)) % 60, 0x3A, flight_time % 60, 0xF9, ((int)(osd_est_flight_time / 60)) % 60, 0x3A, osd_est_flight_time % 60);
            }
            else
            {
                osd.printf("%c%2i%c%2i", 0xB3, ((int)(flight_time / 60)) % 60, 0x3A, flight_time % 60);
            }
            
//            osd.printf("%c%2i%c%02i", 0xB3, (start_Time / 60) % 60, 0x3A, start_Time % 60);
            osd.closePanel();
            return 0;
        }

        /* **************************************************************** */
        // Panel  : panThr      // JDL Fixed 2
        // Needs  : X, Y locations
        // Output : Throttle value from MAVlink with symbols
        // Size   : 1 x 7  (rows x chars)
        // Staus  : done

        public int panThr(int first_col, int first_line)
        {
            osd.setPanel(first_col, first_line);
            osd.openPanel();
            osd.printf("%c%3.0i%c", 0x87, osd_throttle, 0x25);

//            osd.printf("%c%3.0i%c", 0x87, osd_throttle, 0x25);
            osd.closePanel();
            return 0;
        }

        /* **************************************************************** */
        // Panel  : panHomeDis          // JDL Fixed 2
        // Needs  : X, Y locations
        // Output : Home Symbol with distance to home in meters
        // Size   : 1 x 7  (rows x chars)
        // Staus  : done

        public int panHomeDis(int first_col, int first_line)
        {
            osd.setPanel(first_col, first_line);
            osd.openPanel();
//            osd.printf("%c%5.0f%c", 0x1F, (double)osd_home_distance, 0x8D);
            osd.printf("%c%4.0f%c", 0x12, osd_direct_distance, unit_length);
            osd.closePanel();
            return 0;
        }

        /* **************************************************************** */
        // Panel  : panCenter
        // Needs  : X, Y locations
        // Output : 2 row croshair symbol created by 2 x 4 chars
        // Size   : 2 x 4  (rows x chars)
        // Staus  : done

        public int panCenter(int first_col, int first_line)
        {
            osd.setPanel(first_col, first_line);
            osd.openPanel();
            osd.printf_P(PSTR("\x05\x03\x04\x05|\x15\x13\x14\x15"));
            osd.closePanel();
            return 0;
        }

        /* **************************************************************** */
        // Panel  : panHorizon      // JDL Fixed 2
        // Needs  : X, Y locations
        // Output : 12 x 4 Horizon line surrounded by 2 cols (left/right rules)
        // Size   : 14 x 4  (rows x chars)
        // Staus  : done

        public int panHorizon(int first_col, int first_line)
        {
            osd.setPanel(first_col, first_line);
            osd.openPanel();
            osd.printf_P(PSTR("\xc8\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\xc9|"));
            osd.printf_P(PSTR("\xc8\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\xc9|"));
            osd.printf_P(PSTR("\xd8\x0A\x0A\x0A\x0A\x0A\x0A\x0A\x0A\x0A\x0A\x0A\x0A\xd9|"));
            osd.printf_P(PSTR("\xc8\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\xc9|"));
            osd.printf_P(PSTR("\xc8\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\xc9"));
            osd.closePanel();
//            showHorizon((first_col + 1), first_line);
            return 0;
        }

        /* **************************************************************** */
        // Panel  : panPitch        // JDL Fixed 2
        // Needs  : X, Y locations
        // Output : -+ value of current Pitch from vehicle with degree symbols and pitch symbol
        // Size   : 1 x 6  (rows x chars)
        // Staus  : done

        public int panPitch(int first_col, int first_line)
        {
            osd.setPanel(first_col, first_line);
            osd.openPanel();
            osd.printf("%4i%c", osd_pitch, 0xb1);
            
            
//            osd.printf("%4i%c%c", osd_pitch, 0xb0, 0xb1);
            osd.closePanel();
            return 0;
        }

        /* **************************************************************** */
        // Panel  : panRoll         // JDL Fixed 2
        // Needs  : X, Y locations
        // Output : -+ value of current Roll from vehicle with degree symbols and roll symbol
        // Size   : 1 x 6  (rows x chars)
        // Staus  : done

        public int panRoll(int first_col, int first_line)
        {
            osd.setPanel(first_col, first_line);
            osd.openPanel();
            osd.printf("%4i%c", osd_roll, 0xb2);

//            osd.printf("%4i%c%c", osd_roll, 0xb0, 0xb2);
            osd.closePanel();
            return 0;
        }

        /* **************************************************************** */
        // Panel  : panBattery A (Voltage 1)    // JDL Fixed 2
        // Needs  : X, Y locations
        // Output : Voltage value as in XX.X and symbol of over all battery status
        // Size   : 1 x 8  (rows x chars)
        // Staus  : done

        public int panBatt_A(int first_col, int first_line)
        {
            osd.setPanel(first_col, first_line);
            osd.openPanel();

            char batsym;

            batsym = (char)0xB8;
            if ((firmware_type == 1) || (firmware_type == 2))
            {
                osd.printf("%c%5.2f%c", batsym, /*(float)*/osd_vbat, 0x8E);
            }
            else
            {
                osd.printf("%i%c%c%5.2f%c", num_cells, 0x80, batsym, (float)osd_vbat, 0x8E);
            }

            //osd.printf("%c%5.2f%c%c", 0xE2,(double)osd_vbat, 0x8E, osd_battery_pic);
//            osd.printf("%c%5.2f%c", 0xE2, (double)osd_vbat, 0x8E);
            osd.closePanel();
            return 0;
        }

        //------------------ Panel: Startup ArduCam OSD LOGO -------------------------------

        public int panLogo(int first_col, int first_line)   // JDL Fixed 2
        {
            osd.setPanel(first_col, first_line);
            osd.openPanel();
//            osd.printf_P(PSTR("\x20\x20\x20\x20\xba\xbb\xbc\xbd\xbe|\x20\x20\x20\x20\xca\xcb\xcc\xcd\xce|ArduCam OSD"));
            osd.printf_P(PSTR("minoposd jdl 2.49")); 
            osd.closePanel();
            return 0;
        }

        //------------------ Panel: Waiting for MAVLink HeartBeats -------------------------------

        public int panWaitMAVBeats(int first_col, int first_line)
        {
            panLogo(10, 5);
            osd.setPanel(first_col, first_line);
            osd.openPanel();
            osd.printf_P(PSTR("Waiting for|MAVLink heartbeats..."));
            osd.closePanel();
            return 0;
        }

        /* **************************************************************** */
        // Panel  : panGPL          // JDL Fixed 2
        // Needs  : X, Y locations
        // Output : 1 static symbol with changing FIX symbol
        // Size   : 1 x 2  (rows x chars)
        // Staus  : done

        public int panGPL(int first_col, int first_line)
        {
            osd.setPanel(first_col, first_line);
            osd.openPanel();
            osd.printf("%c", osd_fix_type <= 1 ? osd_fix_type * 0x10 : osd_fix_type - 1);
//            switch (osd_fix_type)
//            {
//                case 0:
//                    osd.printf_P(PSTR("\x10\x20"));
//                    break;
//                case 1:
//                    osd.printf_P(PSTR("\x10\x20"));
//                    break;
//                case 2:
//                    osd.printf_P(PSTR("\x11\x20"));//If not APM, x01 would show 2D fix
//                    break;
//                case 3:
//                    osd.printf_P(PSTR("\x11\x20"));//If not APM, x02 would show 3D fix
//                    break;
//            }

            /*  if(osd_fix_type <= 1) {
            osd.printf_P(PSTR("\x10"));
          } else {
            osd.printf_P(PSTR("\x11"));
          }  */
            osd.closePanel();
            return 0;
        }

        /* **************************************************************** */
        // Panel  : panGPSats   // JDL Fixed 2
        // Needs  : X, Y locations
        // Output : 1 symbol and number of locked satellites
        // Size   : 1 x 5  (rows x chars)
        // Staus  : done

        public int panGPSats(int first_col, int first_line)
        {
            osd.setPanel(first_col, first_line);
            osd.openPanel();
            osd.printf("%c%2i", 0x0f, osd_satellites_visible);
            osd.closePanel();
            return 0;
        }

        /* **************************************************************** */
        // Panel  : panGPS          // JDL Fixed 2
        // Needs  : X, Y locations
        // Output : two row numeric value of current GPS location with LAT/LON symbols as on first char
        // Size   : 2 x 12  (rows x chars)
        // Staus  : done

        public int panGPS(int first_col, int first_line)
        {
            osd.setPanel(first_col, first_line);
            osd.openPanel();
//            osd.printf("%c%11.6f|%c%11.6f", 0x83, (double)osd_lat, 0x84, (double)osd_lon);
            osd.printf("%8.5f|%8.5f", (double)osd_lat, (double)osd_lon);    // OK for Eastern Europe
            osd.closePanel();
            return 0;
        }

        /* **************************************************************** */
        // Panel  : panHeading      // JDL Fixed 2
        // Needs  : X, Y locations
        // Output : Symbols with numeric compass heading value
        // Size   : 1 x 5  (rows x chars)
        // Staus  : not ready

/*       
        public int panHeading(int first_col, int first_line)
        {
            osd.setPanel(first_col, first_line);
            osd.openPanel();
            osd.printf("%4.0f%c", (double)osd_heading, 0xb0);
            osd.closePanel();
            return 0;
        } */

        /* **************************************************************** */
        // Panel  : panRose         // JDL Fixed 2
        // Needs  : X, Y locations
        // Output : a dynamic compass rose that changes along the heading information
        // Size   : 2 x 13  (rows x chars)
        // Staus  : done

        public int panRose(int first_col, int first_line)
        {
            osd.setPanel(first_col, first_line);
            osd.openPanel();

            osd.printf("%s%3.0f%c|%c%c%c%c%c%c%c%c%c%c%c%c%c", "\x20\xc6\xc6\xc6\xc6\xc6\xc7\xc6\xc6", (double)osd_heading, 0xB0, 0xd0, buf_show[0],buf_show[1],buf_show[2],buf_show[3],buf_show[4],buf_show[5],buf_show[6],buf_show[7],buf_show[8],buf_show[9],buf_show[10], 0xd1);

//            osd.printf("%s|%c%s%c", "\x20\xc0\xc0\xc0\xc0\xc0\xc7\xc0\xc0\xc0\xc0\xc0\x20", 0xd0, Encoding.Default.GetString(buf_show), 0xd1);
            osd.closePanel();
            return 0;
        }


        /* **************************************************************** */
        // Panel  : panBoot     // JDL Fixed 2
        // Needs  : X, Y locations
        // Output : Booting up text and empty bar after that
        // Size   : 1 x 21  (rows x chars)
        // Staus  : done

        public int panBoot(int first_col, int first_line)
        {
            osd.setPanel(first_col, first_line);
            osd.openPanel();
            osd.printf_P(PSTR("booting up:\xed\xf2\xf2\xf2\xf2\xf2\xf2\xf2\xf3"));
            osd.closePanel();
            return 0;

        }

        /* **************************************************************** */
        // Panel  : panMavBeat
        // Needs  : X, Y locations
        // Output : 2 symbols, one static and one that blinks on every 50th received 
        //          mavlink packet.
        // Size   : 1 x 2  (rows x chars)
        // Staus  : done

        public int panMavBeat(int first_col, int first_line)
        {
            osd.setPanel(first_col, first_line);
            osd.openPanel();
            if (mavbeat == 1)
            {
                osd.printf_P(PSTR("\xEA\xEC"));
                mavbeat = 0;
            }
            else
            {
                osd.printf_P(PSTR("\xEA\xEB"));
            }
            osd.closePanel();
            return 0;
        }


        /* **************************************************************** */
        // Panel  : panWPDir
        // Needs  : X, Y locations
        // Output : 2 symbols that are combined as one arrow, shows direction to next waypoint
        // Size   : 1 x 2  (rows x chars)
        // Staus  : not ready

        public int panWPDir(int first_col, int first_line)
        {
            osd.setPanel(first_col, first_line);
            osd.openPanel();

            long waypoint_distance = 1620;
            uint16_t waypoint_eta_sec = 103;
            uint16_t osd_waypointactive_index = 5;

            osd.printf("%c%c%c%4.0f%c|%02i%c%02i%c%2i", 0x13, 0x14, 0x19, (float)waypoint_distance, unit_length, (osd_waypointactive_index + 1), 0x1A, ((int)(waypoint_eta_sec / 60)) % 60, 0x3A, waypoint_eta_sec % 60);
            
            osd.closePanel();
            return 0;
        }

        /* **************************************************************** */
        // Panel  : panSR_EGE   // JDL Fixed 2
        // Needs  : X, Y locations
        // Output : Safety Raduis / Efficiency/Glide Estimation
        // Size   : 1 x 2  (rows x chars)

        public int panSR_EGE(int first_col, int first_line)
        {
            osd.setPanel(first_col, first_line);
            osd.openPanel();
            if (firmware_type == 1)
            {
                osd.printf("%c%5.0f%c|%c%5.0f%c|%c%5.2f%c", 0xF7, (float)lt_safety_delta, unit_length, 0xF8, (float)st_safety_delta, unit_length, 0xFB, (float)((flight_efficiency) / 1000.0), 0xF4);
            }
            else if (firmware_type == 2)
            {
                osd.printf("%c%5.0f%c|%c%5.0f%c", 0xF7, (float)lt_safety_delta, unit_length, 0xFB, flight_efficiency, 0xFC);
            }
            else
            {
                osd.printf("%c%5.0f%c", 0xFA, (float)batp_safety_delta, unit_length);
            }

            osd.closePanel();
            return 0;
        }

        /* **************************************************************** */
        // Panel  : panHomeDir              // JDL Fixed 2
        // Needs  : X, Y locations
        // Output : 2 symbols that are combined as one arrow, shows direction to home
        // Size   : 1 x 2  (rows x chars)
        // Status : not tested

        public int panHomeDir(int first_col, int first_line)
        {
            osd.setPanel(first_col, first_line);
            osd.openPanel();
            showArrow();
            osd.closePanel();
            return 0;
        }

        /* **************************************************************** */
        // Panel  : panFlightMode       // JDL Fixed 2
        // Needs  : X, Y locations
        // Output : 2 symbols, one static name symbol and another that changes by flight modes
        // Size   : 1 x 2  (rows x chars)
        // Status : done

        public int panFlightMode(int first_col, int first_line)
        {
            osd.setPanel(first_col, first_line);
            osd.openPanel();

            string mode_str = "";

            if (osd_mode == 0)
            {
                mode_str = "man"; // MANUAL
            }
            else if (osd_mode == 1)
            {
                mode_str = "st1"; // STABILIZED1
            }
            else if (osd_mode == 2)
            {
                mode_str = "st2"; // STABILIZED2
            }
            else if (osd_mode == 3)
            {
                mode_str = "st3"; // STABILIZED3
            }
            else if (osd_mode == 4)
            {
                mode_str = "st4"; // STABILIZED4
            }
            else if (osd_mode == 5)
            {
                mode_str = "st5"; // STABILIZED5
            }
            else if (osd_mode == 6)
            {
                mode_str = "st6"; // STABILIZED6
            }
            else if (osd_mode == 7)
            {
                mode_str = "ph "; // POSITIONHOLD
            }
            else if (osd_mode == 8)
            {
                mode_str = "cl "; // COURSELOCK
            }
            else if (osd_mode == 9)
            {
                mode_str = "vr "; // VELOCITYROAM
            }
            else if (osd_mode == 10)
            {
                mode_str = "hl "; // HOMELEASH
            }
            else if (osd_mode == 11)
            {
                mode_str = "ap "; // ABSOLUTEPOSITION
            }
            else if (osd_mode == 12)
            {
                mode_str = "rtb"; // RETURNTOBASE
            }
            else if (osd_mode == 13)
            {
                mode_str = "lan"; // LAND
            }
            else if (osd_mode == 14)
            {
                mode_str = "pp "; // PATHPLANNER
            }
            else if (osd_mode == 15)
            {
                mode_str = "poi"; // POI
            }
            else if (osd_mode == 16)
            {
                mode_str = "ac "; // AUTOCRUISE
            }
            else if (osd_mode == 17)
            {
                mode_str = "at "; // AUTOTAKEOFF
            }
            char c;
            
            c = '\xE0';

            osd.printf("%c%s", c, mode_str);

            
            osd.closePanel();
            return 0;
        }


        // ---------------- EXTRA FUNCTIONS ----------------------
        // Show those fancy 2 char arrows
        public int showArrow()
        {
            osd.printf("%c%c%4.0f%c", 0x92, 0x93, (float)osd_home_distance, unit_length);
            return 0;
/*
            switch (osd_home_direction)
            {
                case 0:
                    osd.printf_P(PSTR("\x90\x91"));
                    break;
                case 1:
                    osd.printf_P(PSTR("\x90\x91"));
                    break;
                case 2:
                    osd.printf_P(PSTR("\x92\x93"));
                    break;
                case 3:
                    osd.printf_P(PSTR("\x94\x95"));
                    break;
                case 4:
                    osd.printf_P(PSTR("\x96\x97"));
                    break;
                case 5:
                    osd.printf_P(PSTR("\x98\x99"));
                    break;
                case 6:
                    osd.printf_P(PSTR("\x9A\x9B"));
                    break;
                case 7:
                    osd.printf_P(PSTR("\x9C\x9D"));
                    break;
                case 8:
                    osd.printf_P(PSTR("\x9E\x9F"));
                    break;
                case 9:
                    osd.printf_P(PSTR("\xA0\xA1"));
                    break;
                case 10:
                    osd.printf_P(PSTR("\xA2\xA3"));
                    break;
                case 11:
                    osd.printf_P(PSTR("\xA4\xA5"));
                    break;
                case 12:
                    osd.printf_P(PSTR("\xA6\xA7"));
                    break;
                case 13:
                    osd.printf_P(PSTR("\xA8\xA9"));
                    break;
                case 14:
                    osd.printf_P(PSTR("\xAA\xAB"));
                    break;
                case 15:
                    osd.printf_P(PSTR("\xAC\xAD"));
                    break;
                case 16:
                    osd.printf_P(PSTR("\xAE\xAF"));
                    break;
            }
            return 0; */
        }




        // Calculate and shows Artificial Horizon
        public int showHorizon(int start_col, int start_row)
        {

            int x, nose, row, minval, hit, subval = 0;
            int cols = 12;
            int rows = 5;
            int[] col_hit = new int[cols];
            double pitch, roll;

            if (abs(osd_pitch) == 90) { pitch = 89.99 * (90 / osd_pitch) * -0.017453293; } else { pitch = osd_pitch * -0.017453293; }
            if (abs(osd_roll) == 90) { roll = 89.99 * (90 / osd_roll) * 0.017453293; } else { roll = osd_roll * 0.017453293; }

            nose = round(tan(pitch) * (rows * 9));
            for (int col = 1; col <= cols; col++)
            {
                x = (col * 12) - (cols * 6) - 6;//center X point at middle of each col
                col_hit[col - 1] = (int)(tan(roll) * x) + nose + (rows * 9) - 1;//calculating hit point on Y plus offset to eliminate negative values
                //col_hit[(col-1)] = nose + (rows * 9);
            }

            for (int col = 0; col < cols; col++)
            {
                hit = col_hit[col];
                if (hit > 0 && hit < (rows * 18))
                {
                    row = rows - ((hit - 1) / 18);
                    minval = rows * 18 - row * 18 + 1;
                    subval = hit - minval;
                    subval = round((subval * 9) / 18);
                    if (subval == 0) subval = 1;
                    printHit((byte)(start_col + col), (byte)(start_row + row - 1), (byte)subval);
                }
            }
            return 0;
        }

        public int printHit(byte col, byte row, byte subval)
        {
            osd.openSingle(col, row);
            switch (subval)
            {
                case 1:
                    osd.printf_P(PSTR("\x06"));
                    break;
                case 2:
                    osd.printf_P(PSTR("\x07"));
                    break;
                case 3:
                    osd.printf_P(PSTR("\x08"));
                    break;
                case 4:
                    osd.printf_P(PSTR("\x09"));
                    break;
                case 5:
                    osd.printf_P(PSTR("\x0a"));
                    break;
                case 6:
                    osd.printf_P(PSTR("\x0b"));
                    break;
                case 7:
                    osd.printf_P(PSTR("\x0c"));
                    break;
                case 8:
                    osd.printf_P(PSTR("\x0d"));
                    break;
                case 9:
                    osd.printf_P(PSTR("\x0e"));
                    break;
            }
            return 0;
        }



        //------------------ Heading and Compass ----------------------------------------

        byte[] buf_show = new byte[11];
        byte[] buf_Rule = {0xc2,0xc0,0xc0,0xc1,0xc0,0xc0,0xc1,0xc0,0xc0,
                           0xc4,0xc0,0xc0,0xc1,0xc0,0xc0,0xc1,0xc0,0xc0,
                           0xc3,0xc0,0xc0,0xc1,0xc0,0xc0,0xc1,0xc0,0xc0,
                           0xc5,0xc0,0xc0,0xc1,0xc0,0xc0,0xc1,0xc0,0xc0};
        
        public void setHeadingPatern()
        {
            int start;
            start = round((osd_heading * 36) / 360);
            start -= 5;
            if (start < 0) start += 36;
            for (int x = 0; x <= 10; x++)
            {
                buf_show[x] = buf_Rule[start];
                if (++start > 35) start = 0;
            }
//             buf_show[11] = '\0';
        }

        //------------------ Battery Remaining Picture ----------------------------------

        public void setBatteryPic()
        {
            return;
/*
            if (osd_battery_remaining <= 270)
            {
                osd_battery_pic = 0xb4;
            }
            else if (osd_battery_remaining <= 300)
            {
                osd_battery_pic = 0xb5;
            }
            else if (osd_battery_remaining <= 400)
            {
                osd_battery_pic = 0xb6;
            }
            else if (osd_battery_remaining <= 500)
            {
                osd_battery_pic = 0xb7;
            }
            else if (osd_battery_remaining <= 800)
            {
                osd_battery_pic = 0xb8;
            }
            else osd_battery_pic = 0xb9; */
        }
            
    }
}