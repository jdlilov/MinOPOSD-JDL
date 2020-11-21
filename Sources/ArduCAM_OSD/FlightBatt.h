// JDL
/**
 ******************************************************************************
 *
 * @file       FlightBatt.h
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
 * with this program; if not, see <http://www.gnu.org/licenses/> or write to the                  // 0.81 - 0.033 = 24.545454
 * Free Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA          // 1.02 - 0.043 = 23.720930 = 24.8542267654
 */                                                                                               // 0.71 - 0.027 = 26.296296


// !!! For using this, you have to solder a little bit on the MinimOSD, see the wiki !!!


#ifndef FLIGHT_BATT_H_
#define FLIGHT_BATT_H_

#if defined MICRO_OSD_KVTEAM

#define VOLTAGE_PIN       0                       // Micro OSD KV team : Bat1
//#define VOLTAGE_PIN       2                       // Micro OSD KV team : Bat2
#define CURRENT_PIN       1

#else

#define VOLTAGE_PIN       6                      // MinOPOSD JDL
#define CURRENT_PIN       1                      // MinOPOSD JDL

#endif

#define REF_VOLTAGE       1.1                     // INTERNAL: a built-in reference, equal to 1.1 volts on the ATmega168 or ATmega328
#define CUR_REF_VOLTAGE   4.98                     // EXTERBNAL: power supply 5.0V

#ifdef FLIGHT_BATT_ON_MINIMOSD
  #define LOW_VOLTAGE       14.0                    // filter start value for 4s LiPo
#else
  #define LOW_VOLTAGE       10.5                    // filter start value for 3s LiPo
#endif

#define VOLT_DIV_RATIO    15.14     // 15.73      // Vref 1.1V based: This is the start value for calibrating a 16k0/1k1 voltage divider usable up to 4s LiPo

// !!! for the +-50A Current Sensor(AC/DC) DFRobot SEN0098 we need approx. a 1/4 voltage divider 3k0/1k1 so that we stay below 1.1 V -> 2*50A * 0.04V/A / (4.1/1.1) = 1.073 V !!!
#define CURR_AMP_PER_VOLT 17.25                   // XXX Vref 1.1V based: This is the start value for calibrating a +-50A Current Sensor(AC/DC) DFRobot SEN0098 Sensitivity: 40 mV/A
#define CURR_AMPS_OFFSET  0.000                   // XXX Vref 1.1V based: This is the start value for calibrating a +-50A Current Sensor(AC/DC) DFRobot SEN0098 Sensitivity: 40 mV/A

#define CURRENT_VOLTAGE(x) ((x) * REF_VOLTAGE / 1024.0) * (volt_div_ratio / 100.0)
#define CURRENT_AMPS(x)    (((x) * REF_VOLTAGE / 1024.0) - (curr_amp_offset / 10000.0)) * (curr_amp_per_volt / 100.0)


void flight_batt_init(void);
void flight_batt_read(void);


#endif /* FLIGHT_BATT_H_ */
