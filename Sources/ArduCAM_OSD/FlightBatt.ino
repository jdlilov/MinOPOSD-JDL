// JDL
/**
 ******************************************************************************
 *
 * @file       FlightBatt.ino
 * @author     Joerg-D. Rothfuchs
 * @brief      Implements voltage and current measurement of the flight battery
 *             on the Ardupilot Mega MinimOSD using built-in ADC reference.
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/> or write to the
 * Free Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */


// !!! For using this, you have to solder a little bit on the MinimOSD, see the wiki !!!


#include "FlightBatt.h"

void flight_batt_init(void)
{
    analogReference(INTERNAL); // INTERNAL: a built-in reference, equal to 1.1 volts on the ATmega168 or ATmega328
}

void flight_batt_read(void)
{
    static float voltage = -1.0;                         // battery voltage, initialized above the low voltage threshold to pre-load the filter and prevent low voltage events at startup
    static float current_amps      = 0;                  // battery instantaneous currrent draw [A]
    static float current_total     = 0;                  // totalized battery current [mAh]
    static unsigned long loopTimer = 0;
    uint16_t delta_ms;

    if (loopTimer + MEASURE_PERIOD <= millis()) {
        delta_ms   = millis() - loopTimer;
        loopTimer  = millis();

        float sampled_voltage = CURRENT_VOLTAGE(analogRead(VOLTAGE_PIN));  // reads battery voltage pin
  
        if (voltage == -1.0) {
            voltage = sampled_voltage;
        }
        else {
            if (sampled_voltage <= voltage) {
                voltage = sampled_voltage * .1 + voltage * .9;         
            }
            else {
                voltage = sampled_voltage * .3 + voltage * .7; 
            }
        }

        if (batt_type) {
            for (int cell_count=6; cell_count>1; --cell_count) {
                if ((voltage > batt_levels[cell_count-2]) && (num_cells <= cell_count)) {
                    num_cells = cell_count;
                    break;
                }
            }
        }
        else 
            num_cells = 4;

#ifdef LIHV_DETECTION                  
        LiHV_mode = LiHV_mode || ((voltage / num_cells) > 4.265);
        if (LiHV_mode) {
          bpm = 114.74469;
        }
        else {
          bpm = 137.08019;
        }
#endif

        // for LiHV:
        // 3.5V under heavy load = 95% depleted (for Graphenes), 3.4635V = 100% depleted.
        // 4.335V under light load = 0% depleted
        // sampled_batt_percent = (((osd_vbat_A / num_cells) - 3.4635) / 0.8715) * 100.0;
        // sampled_batt_percent = ((osd_vbat_A / num_cells) - 3.4635) * 114.74469;
        // sampled_batt_percent = batt_curve[(uint8_t)sampled_batt_percent];
                
        // for LiPo:
        // 3.5V under heavy load = 95% depleted (for Graphenes), 3.4635V = 100% depleted.
        // 4.193V under light load = 0% depleted
        // float sampled_batt_percent = (((voltage / num_cells) - 3.463) / 0.737) * 100.0;

        // for LiIon: 
        // 3.0V under heavy load = 100% depleted
        // 4.08V under light load = 0% depleted
        // float sampled_batt_percent = (((voltage / num_cells) - 3.0) / 1.08) * 100.0;

        float sampled_batt_percent;
        float battP_raw;
        uint8_t battP_int;

        osd_vbat_A = voltage;

        if (batt_type) {
            battP_raw = ((osd_vbat_A / num_cells) - 3.4635) * bpm;
/*            if (battP_raw < 0.0)
            {
              battP_raw = 0.0;
            }
            if (battP_raw > 99.0)
            {
              battP_raw = 99.0;
            } */
            battP_raw = constrain(battP_raw, 0.0f, 99.0f);
            battP_int = (uint8_t)battP_raw;
          
        #ifdef BAT_VOLTAGE_CURVE

          #ifndef LIHV_DETECTION
 
            sampled_batt_percent = pgm_read_byte(&batt_curve[battP_int]);        // LiPo

          #else

            if (LiHV_mode) {
                sampled_batt_percent = pgm_read_byte(&batt_curve_lihv[battP_int]);        // LiHV
            }
            else {
                sampled_batt_percent = pgm_read_byte(&batt_curve[battP_int]);        // LiPo
            }
                  
          #endif
                  
        #else                
            sampled_batt_percent = battP_int;          // LiPo
        #endif                
        }
        else {
            battP_raw = ((osd_vbat_A / num_cells) - 3.0) * 92.5925926;
/*            if (battP_raw < 0.0)
            {
              battP_raw = 0.0;
            }
            if (battP_raw > 99.0)
            {
              battP_raw = 99.0;
            } */
            battP_raw = constrain(battP_raw, 0.0f, 99.0f);
            
            sampled_batt_percent = (uint8_t)battP_raw;          // LiIon
        }

        if (osd_battery_remaining_percent_V == -1.0) {
            osd_battery_remaining_percent_V = sampled_batt_percent;
        }
        else {
            if (sampled_batt_percent <= osd_battery_remaining_percent_V) {
                osd_battery_remaining_percent_V = sampled_batt_percent * 0.05 + osd_battery_remaining_percent_V * 0.95;
            }
            else {
                osd_battery_remaining_percent_V = sampled_batt_percent * 0.3 + osd_battery_remaining_percent_V * 0.7;
            }
        }
        

        if (curr_amp_per_volt > 0) { // Consider Amp sensor disbled when Amp per Volt ratio is zero

            current_amps   = CURRENT_AMPS(analogRead(CURRENT_PIN)) * .2 + current_amps * .8;       // reads battery sensor current pin

            current_total += current_amps * (float)delta_ms * 0.0002778; // .0002778 is 1/3600 (conversion to hours)
            osd_curr_A     = current_amps * 100;
            osd_total_A    = current_total;
        }
    }
}
