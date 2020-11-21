// JDL
// ------------------ Heading and Compass ----------------------------------------

static char buf_show[12];
const char buf_Rule[36] = { 0xc2, 0xc0, 0xc0, 0xc1, 0xc0, 0xc0, 0xc1, 0xc0, 0xc0,
                            0xc4, 0xc0, 0xc0, 0xc1, 0xc0, 0xc0, 0xc1, 0xc0, 0xc0,
                            0xc3, 0xc0, 0xc0, 0xc1, 0xc0, 0xc0, 0xc1, 0xc0, 0xc0,
                            0xc5, 0xc0, 0xc0, 0xc1, 0xc0, 0xc0, 0xc1, 0xc0, 0xc0 };
void setHeadingPattern()
{
    int start;

    start  = round((osd_heading * 36) / 360);
    start -= 5;
    if (start < 0) {
        start += 36;
    }
    for (int x = 0; x <= 10; x++) {
        buf_show[x] = buf_Rule[start];
        if (++start > 35) {
            start = 0;
        }
    }
    buf_show[11] = '\0';
}


#if defined JR_SPECIALS && not defined COMBINED_HEADING_AND_ROSE
static char CompassPoint[3];
// nst char CompassPointList[16] = {'n',  ' ', 'n',  'e',  'e',  ' ', 's',  'e',  's',  ' ', 's',  'w',  'w',  ' ', 'n',  'w'};
const char CompassPointList[16] = { 0xc2, ' ', 0xc2, 0xc4, 0xc4, ' ', 0xc3, 0xc4, 0xc3, ' ', 0xc3, 0xc5, 0xc5, ' ', 0xc2, 0xc5 };

void calculateCompassPoint()
{
    int index;

    index = (int)((osd_heading + 22.5) / 45.0);
    index = index > 7 ? 0 : index * 2;
    CompassPoint[0] = CompassPointList[index];
    CompassPoint[1] = CompassPointList[index + 1];
    CompassPoint[2] = '\0';
}
#endif


void updateTravelDistance(void)
{
    static unsigned long loopTimer = 0;

    if (loopTimer + MEASURE_PERIOD <= millis()) {
      
        osd_3D_speed = sqrt(sq(osd_groundspeed) + sq(osd_climb));
      
        if (osd_3D_speed > 0.0) {                                              // was osd_groundspeed > 1.0
            float delta_time = millis() - loopTimer; // in milliseconds
            float osd_delta_distance = osd_3D_speed * delta_time / 1000.0;
            osd_travel_distance += (osd_groundspeed * delta_time / 1000.0);
#ifdef SAFETY_RADIUS         

  #ifdef ST_RADIUS_ESTIMATION
            if (flight_A > 0) {
//                st_flight_efficiency = 0.8 * st_flight_efficiency + 0.2 * osd_delta_distance / ((float) osd_curr_A * 10.0 * (delta_time / 3600000.0));  //  meters / mah
                st_flight_efficiency = 0.8 * st_flight_efficiency + ((72000.0 * osd_delta_distance / (float) osd_curr_A) / delta_time);  //  meters / mah
                st_safety_delta = ((( (float) batt_capacity * 0.8 - osd_total_A) * st_flight_efficiency) - osd_home_distance) / 2;

                if (st_safety_delta > 99999) {
                    st_safety_delta = 99999;
                }
                if (st_safety_delta < -9999) {
                    st_safety_delta = -9999;
                }
            }
            else {
                st_safety_delta = 0.0;
            }
  #endif          
            
  #ifdef EFFICIENCY_ESTIMATION
            if ((flight_A > 0) && (osd_delta_distance > 0.0)){
//                flight_efficiency = 0.8 * flight_efficiency + 0.2 * ((float) osd_vbat_A * (float) osd_curr_A * 10.0 * (delta_time / 3600000.0)) / (osd_delta_distance / 1000.0);  //  mWh / kilometer
                  flight_efficiency = 0.8 * flight_efficiency + ((float) osd_curr_A * osd_vbat_A * delta_time / 1800.0) / osd_delta_distance;  //  mWh / kilometer

                  if (flight_efficiency > 99990) {
                      flight_efficiency = 99990;
                  }
            }
            else {
                flight_efficiency = 0.0;
            }
  #endif          

  #ifdef BATP_RADIUS_ESTIMATION
            if (osd_battery_remaining_percent_V < start_batp) {
                batp_flight_efficiency = 0.9 * batp_flight_efficiency + 0.1 * osd_travel_distance / (start_batp - osd_battery_remaining_percent_V); //  meters / delta(batt %)
                batp_safety_delta = (((osd_battery_remaining_percent_V - 20.0) * batp_flight_efficiency) - osd_home_distance) / 2;
                
                if (batp_safety_delta > 99999) {
                    batp_safety_delta = 99999;
                }
                if (batp_safety_delta < -9999) {
                    batp_safety_delta = -9999;
                }
            }
            else {
                batp_safety_delta = 0.0;
            }
  #endif
  
#endif
        }
        loopTimer = millis();
    }
}


// ------------------ Battery Remaining Picture ----------------------------------
/*
char setBatteryPic(uint16_t bat_level)
{
    if (bat_level <= 100) {
        return 0xb4;
    } else if (bat_level <= 300) {
        return 0xb5;
    } else if (bat_level <= 400) {
        return 0xb6;
    } else if (bat_level <= 500) {
        return 0xb7;
    } else if (bat_level <= 800) {
        return 0xb8;
    } else { return 0xb9; }
}
*/
// ------------------ Home Distance and Direction Calculation ----------------------------------

void setHomeVars(OSD &osd)
{
// JRChange: OpenPilot UAVTalk:
#ifdef PROTOCOL_UAVTALK
    float dstlon, dstlat;
    long bearing;

    if (osd_got_home) {
        // shrinking factor for longitude going to poles direction
        float rads = fabs(osd_home_lat) * 0.0174532925;
        double scaleLongDown = cos(rads);
        double scaleLongUp   = 1.0f / cos(rads);

        // DST to Home
        dstlat = fabs(osd_home_lat - osd_lat) * 111319.5;
        dstlon = fabs(osd_home_lon - osd_lon) * 111319.5 * scaleLongDown;
        osd_home_distance = sqrt(sq(dstlat) + sq(dstlon));

#if defined SAFETY_RADIUS && defined LT_RADIUS_ESTIMATION
    float new_est_ft;
    
    if (flight_A > 0) {
        osd_est_flight_time = (int) (0.8 * (float) osd_est_flight_time + 0.2 * (( (float) batt_capacity * 0.8 - osd_total_A) * ( (float) (millis() - engine_start_time) / flight_A) / 1000.0));
        if ( osd_total_A > (float) batt_capacity * 0.8) {
            osd_est_flight_time = 0.0;
        }
        if (osd_est_flight_time > 5999) {
            osd_est_flight_time = 5999;
        }          
        lt_safety_delta = (( (float) batt_capacity * 0.8 - osd_total_A) * (osd_travel_distance / flight_A) - osd_home_distance) / 2;

        if (lt_safety_delta > 99999) {
            lt_safety_delta = 99999;
        }
        if (lt_safety_delta < -9999) {
            lt_safety_delta = -9999;
        }
    }
    else {
        osd_est_flight_time = 0.0;
        lt_safety_delta = 0.0;
    }
#endif
//#ifdef REVO_ADD_ONS       
//        osd_direct_distance = sqrt(sq(osd_home_distance) + sq(abs(revo_baro_alt)));
//#else        
        float current_alt = osd_alt - osd_home_alt;
        osd_direct_distance = sqrt(sq(osd_home_distance) + sq(current_alt));
//#endif        

        // DIR to Home
        dstlon  = (osd_home_lon - osd_lon);                                  // OffSet X
        dstlat  = (osd_home_lat - osd_lat) * scaleLongUp;                    // OffSet Y
        bearing = 90 + (atan2(dstlat, -dstlon) * 57.295775); // absolut home direction
        if (bearing < 0) {
            bearing += 360; // normalization
        }
        bearing = bearing - 180; // absolut return direction
        if (bearing < 0) {
            bearing += 360; // normalization
        }
        bearing = bearing - osd_heading; // relative home direction
        if (bearing < 0) {
            bearing += 360; // normalization
        }
        osd_home_direction = round((float)(bearing / 360.0f) * 16.0f) + 1; // array of arrows
        if (osd_home_direction > 16) {
            osd_home_direction = 0;
        }
    } else {
        // criteria for a stable home position:
        // - GPS fix
        // - with at least 5 satellites
        // - osd_alt stable for 30 * 100ms = 3s
        // - osd_alt stable means the delta is lower 0.5m
        if (osd_fix_type > 1 && osd_satellites_visible >= 5/*7*/ && osd_alt_cnt < 30) {
            if (fabs(osd_alt_prev - osd_alt) > 0.5) {
                osd_alt_cnt  = 0;
                osd_alt_prev = osd_alt;
            } else {
                if (++osd_alt_cnt >= 30) {
                    osd_home_lat = osd_lat; // take this osd_lat as osd_home_lat
                    osd_home_lon = osd_lon; // take this osd_lon as osd_home_lon

                    osd_home_alt = osd_alt; // take this stable osd_alt as osd_home_alt (GPS only)
#ifdef REVO_ADD_ONS
                    osd_baro_home_alt = revo_baro_alt;
#endif
                    osd_got_home = 1;
                }
            }
        }
    }
#else // ifndef PROTOCOL_UAVTALK
    float dstlon, dstlat;
    long bearing;

    if (osd_got_home == 0 && osd_fix_type > 1) {
        osd_home_lat = osd_lat; // preset this osd_lat as osd_home_lat
        osd_home_lon = osd_lon; // preset this osd_lon as osd_home_lon
        // osd_home_alt = osd_alt;
        osd_got_home = 1;
    } else if (osd_got_home == 1) {
        // JRChange: osd_home_alt: check for stable osd_alt (must be stable for 25*120ms = 3s)
        if (osd_alt_cnt < 25) {
            if (fabs(osd_alt_prev - osd_alt) > 0.5) {
                osd_alt_cnt  = 0;
                osd_alt_prev = osd_alt;
            } else {
                if (++osd_alt_cnt >= 25) {
                    osd_home_alt = osd_alt; // take this stable osd_alt as osd_home_alt
                    osb_baro_home_alt = revo_baro_alt;
                }
            }
        }
        // shrinking factor for longitude going to poles direction
        float rads = fabs(osd_home_lat) * 0.0174532925;
        double scaleLongDown = cos(rads);
        double scaleLongUp   = 1.0f / cos(rads);

        // DST to Home
        dstlat = fabs(osd_home_lat - osd_lat) * 111319.5;
        dstlon = fabs(osd_home_lon - osd_lon) * 111319.5 * scaleLongDown;
        osd_home_distance = sqrt(sq(dstlat) + sq(dstlon));
        
        // DIR to Home
        dstlon  = (osd_home_lon - osd_lon); // OffSet_X
        dstlat  = (osd_home_lat - osd_lat) * scaleLongUp; // OffSet Y
        bearing = 90 + (atan2(dstlat, -dstlon) * 57.295775); // absolut home direction
        if (bearing < 0) {
            bearing += 360; // normalization
        }
        bearing = bearing - 180; // absolut return direction
        if (bearing < 0) {
            bearing += 360; // normalization
        }
        bearing = bearing - osd_heading; // relative home direction
        if (bearing < 0) {
            bearing += 360; // normalization
        }
        osd_home_direction = round((float)(bearing / 360.0f) * 16.0f) + 1; // array of arrows =)
        if (osd_home_direction > 16) {
            osd_home_direction = 0;
        }
    }
#endif // ifdef PROTOCOL_UAVTALK
}
