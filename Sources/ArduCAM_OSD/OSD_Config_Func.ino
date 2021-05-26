// JDL
/* ******************************************************************/
/* *********************** GENERAL FUNCTIONS ********************** */

// JRChange: Flight Batt on MinimOSD:
#include "FlightBatt.h"

// Extract functions (get bits from the positioning bytes
#define ISa(panel, whichBit) getBit(panA_REG[panel], whichBit)
#define ISb(panel, whichBit) getBit(panB_REG[panel], whichBit)
#define ISc(panel, whichBit) getBit(panC_REG[panel], whichBit)
#define ISd(panel, whichBit) getBit(panD_REG[panel], whichBit)
#define ISe(panel, whichBit) getBit(panE_REG[panel], whichBit)


boolean getBit(byte Reg, byte whichBit)
{
  boolean State;

  State = Reg & (1 << whichBit);
  return State;
}

byte setBit(byte &Reg, byte whichBit, boolean stat)
{
  if (stat) {
    Reg = Reg | (1 << whichBit);
  } 
  else {
    Reg = Reg & ~(1 << whichBit);
  }
  return Reg;
}

// EEPROM reader/writers
// Utilities for writing and reading from the EEPROM
byte readEEPROM(int address)
{
  return EEPROM.read(address);
}

void writeEEPROM(byte value, int address)
{
  EEPROM.write(address, value);
}


// JRChange:
#if 0
void InitializeOSD()
{
  loadBar();
  delay(500);

  writeEEPROM(42, CHK1);
  writeEEPROM(VER - 42, CHK2);
  for (panel = 0; panel < npanels; panel++) {
    writeSettings();
  }

  osd.setAndOpenPanel(4, 9);
  osd.printf_P(PSTR("OSD Initialized, reboot"));
  osd.closePanel();

  // run for ever so user resets
  for (;;) {
  }
}

// Write our latest FACTORY settings to EEPROM
void writeSettings()
{
  // Writing all default parameters to EEPROM, ON = panel enabled
  // All panels have 3 values:
  // - Enable/Disable
  // - X coordinate on screen
  // - Y coordinate on screen
  uint16_t offset = OffsetBITpanel * panel;

  writeEEPROM(off, panCenter_en_ADDR + offset);
  writeEEPROM(13, panCenter_x_ADDR + offset);
  writeEEPROM(7, panCenter_y_ADDR + offset);
  writeEEPROM(on, panPitch_en_ADDR + offset);
  writeEEPROM(22, panPitch_x_ADDR + offset);
  writeEEPROM(9, panPitch_y_ADDR + offset);
  writeEEPROM(on, panRoll_en_ADDR + offset);
  writeEEPROM(11, panRoll_x_ADDR + offset);
  writeEEPROM(1, panRoll_y_ADDR + offset);
  writeEEPROM(on, panBatt_A_en_ADDR + offset);
  writeEEPROM(21, panBatt_A_x_ADDR + offset);
  writeEEPROM(1, panBatt_A_y_ADDR + offset);
  // writeEEPROM(on, panBatt_B_en_ADDR);
  // writeEEPROM(21, panBatt_B_x_ADDR);
  // writeEEPROM(3,  panBatt_B_y_ADDR);
  writeEEPROM(on, panGPSats_en_ADDR + offset);
  writeEEPROM(2, panGPSats_x_ADDR + offset);
  writeEEPROM(13, panGPSats_y_ADDR + offset);
  writeEEPROM(on, panGPL_en_ADDR + offset);
  writeEEPROM(5, panGPL_x_ADDR + offset);
  writeEEPROM(13, panGPL_y_ADDR + offset);
  writeEEPROM(on, panGPS_en_ADDR + offset);
  writeEEPROM(2, panGPS_x_ADDR + offset);
  writeEEPROM(14, panGPS_y_ADDR + offset);
  writeEEPROM(on, panRose_en_ADDR + offset);
  writeEEPROM(16, panRose_x_ADDR + offset);
  writeEEPROM(14, panRose_y_ADDR + offset);
#ifndef COMBINED_HEADING_AND_ROSE
  writeEEPROM(on, panHeading_en_ADDR + offset);
  writeEEPROM(24, panHeading_x_ADDR + offset);
  writeEEPROM(13, panHeading_y_ADDR + offset);
#endif  
  //    writeEEPROM(on, panMavBeat_en_ADDR + offset);    // JDL Changed
  //    writeEEPROM(2, panMavBeat_x_ADDR + offset);
  //    writeEEPROM(9, panMavBeat_y_ADDR + offset);
  writeEEPROM(on, panHomeDir_en_ADDR + offset);
  writeEEPROM(14, panHomeDir_x_ADDR + offset);
  writeEEPROM(3, panHomeDir_y_ADDR + offset);
  writeEEPROM(on, panHomeDis_en_ADDR + offset);
  writeEEPROM(2, panHomeDis_x_ADDR + offset);
  writeEEPROM(1, panHomeDis_y_ADDR + offset);
  
#ifdef PATHPLAN_WAYPOINTS_SUPPORT
  writeEEPROM(off, panWPDir_en_ADDR);              // JDL Changed: All Waypoint info here!
  writeEEPROM(27, panWPDir_x_ADDR);
  writeEEPROM(12, panWPDir_y_ADDR);
#endif
  
#ifdef SAFETY_RADIUS
  writeEEPROM(23, panSR_EGE_x_ADDR);
  writeEEPROM(11, panSR_EGE_y_ADDR);
  writeEEPROM(off, panSR_EGE_en_ADDR);
#endif  

  writeEEPROM(on, panRSSI_en_ADDR + offset);
  writeEEPROM(21, panRSSI_x_ADDR + offset);
  writeEEPROM(5, panRSSI_y_ADDR + offset);
  writeEEPROM(on, panCur_A_en_ADDR + offset);
  writeEEPROM(21, panCur_A_x_ADDR + offset);
  writeEEPROM(2, panCur_A_y_ADDR + offset);
  // writeEEPROM(on, panCur_B_en_ADDR);
  // writeEEPROM(21, panCur_B_x_ADDR);
  // writeEEPROM(4,  panCur_B_y_ADDR);
  writeEEPROM(on, panAlt_en_ADDR + offset);
  writeEEPROM(2, panAlt_x_ADDR + offset);
  writeEEPROM(2, panAlt_y_ADDR + offset);
  writeEEPROM(on, panHomeAlt_en_ADDR + offset);
  writeEEPROM(2, panHomeAlt_x_ADDR + offset);
  writeEEPROM(5, panHomeAlt_y_ADDR + offset);
  writeEEPROM(on, panVel_en_ADDR + offset);
  writeEEPROM(2, panVel_x_ADDR + offset);
  writeEEPROM(3, panVel_y_ADDR + offset);
  writeEEPROM(on, panAirSpeed_en_ADDR + offset);
  writeEEPROM(2, panAirSpeed_x_ADDR + offset);
  writeEEPROM(3, panAirSpeed_y_ADDR + offset);
  writeEEPROM(on, panBatteryPercent_en_ADDR + offset);
  writeEEPROM(2, panBatteryPercent_x_ADDR + offset);
  writeEEPROM(3, panBatteryPercent_y_ADDR + offset);
  writeEEPROM(on, panTemp_en_ADDR + offset);
  writeEEPROM(2, panTemp_x_ADDR + offset);
  writeEEPROM(3, panTemp_y_ADDR + offset);
  writeEEPROM(on, panTime_en_ADDR + offset);
  writeEEPROM(2, panTime_x_ADDR + offset);
  writeEEPROM(3, panTime_y_ADDR + offset);
  writeEEPROM(on, panThr_en_ADDR + offset);
  writeEEPROM(2, panThr_x_ADDR + offset);
  writeEEPROM(4, panThr_y_ADDR + offset);
  writeEEPROM(on, panFMod_en_ADDR + offset);
  writeEEPROM(17, panFMod_x_ADDR + offset);
  writeEEPROM(13, panFMod_y_ADDR + offset);
  writeEEPROM(on, panHorizon_en_ADDR + offset);
  writeEEPROM(8, panHorizon_x_ADDR + offset);
  writeEEPROM(7, panHorizon_y_ADDR + offset);
  writeEEPROM(on, panWarn_en_ADDR + offset);
  writeEEPROM(10, panWarn_x_ADDR + offset);
  writeEEPROM(4, panWarn_y_ADDR + offset);
  writeEEPROM(on, panOff_en_ADDR + offset);
  writeEEPROM(10, panOff_x_ADDR + offset);
  writeEEPROM(4, panOff_y_ADDR + offset);
  //    writeEEPROM(on, panWindSpeed_en_ADDR + offset);    // JDL Changed
  //    writeEEPROM(10, panWindSpeed_x_ADDR + offset);
  //    writeEEPROM(4, panWindSpeed_y_ADDR + offset);
  writeEEPROM(on, panClimb_en_ADDR + offset);
  writeEEPROM(10, panClimb_x_ADDR + offset);
  writeEEPROM(4, panClimb_y_ADDR + offset);
  //    writeEEPROM(on, panTune_en_ADDR + offset);        // JDL Changed
  //    writeEEPROM(10, panTune_x_ADDR + offset);
  //    writeEEPROM(4, panTune_y_ADDR + offset);

  if (panel == 0) {
    // writeEEPROM(on, panSetup_en_ADDR);

    //        writeEEPROM(30, overspeed_ADDR);                // JDL Changed
#if defined STALL_WARNING    
    writeEEPROM(40, stall_ADDR);
#endif    
    writeEEPROM(56, battv_ADDR); // 3.57Volts
    writeEEPROM(1, batt_type_ADDR); // LiPo
//    writeEEPROM(6, ch_toggle_ADDR);                        // JDL Changed
  }
}
#endif // if 0


void readSettings()
{
  //    overspeed   = EEPROM.read(overspeed_ADDR);    // JDL Changed
#if defined STALL_WARNING    
  stall_threshold = (float)EEPROM.read(stall_ADDR) / 3.6f;
#endif  

  battv       = EEPROM.read(battv_ADDR);

#if defined BATTTYPE_SELECTION
  batt_type       = EEPROM.read(batt_type_ADDR);
#endif

#if defined SAFETY_RADIUS && defined LT_RADIUS_ESTIMATION
  batt_capacity = EEPROM.read(batt_capacity_ADDR) + (EEPROM.read(batt_capacity_ADDR + 1) << 8);
  #if defined DO_ROUND_STORED_BATT_CAPACITY
  batt_capacity = round(batt_capacity / 10) * 10;
  #endif
#endif

//  switch_mode = 0; // JDL Changed // EEPROM.read(switch_mode_ADDR);

//  ch_toggle = 7; // JDL Changed // EEPROM.read(ch_toggle_ADDR);

  // JRChange: Flight Batt on MinimOSD:
#ifdef FLIGHT_BATT_ON_MINIMOSD
  volt_div_ratio    = EEPROM.read(volt_div_ratio_ADDR) + (EEPROM.read(volt_div_ratio_ADDR + 1) << 8);
  curr_amp_per_volt = EEPROM.read(curr_amp_per_volt_ADDR) + (EEPROM.read(curr_amp_per_volt_ADDR + 1) << 8);
  curr_amp_offset   = EEPROM.read(curr_amp_offset_ADDR) + (EEPROM.read(curr_amp_offset_ADDR + 1) << 8);
#endif

}

void readPanelSettings()
{
  // ****** First set of 8 Panels ******
  uint16_t offset = OffsetBITpanel * panel;

//  setBit(panA_REG[panel], Cen_BIT, readEEPROM(panCenter_en_ADDR + offset));
//  panCenter_XY[0][panel] = readEEPROM(panCenter_x_ADDR + offset);
//  panCenter_XY[1][panel] = checkPAL(readEEPROM(panCenter_y_ADDR + offset));

  setBit(panA_REG[panel], Bp_BIT, readEEPROM(panBatteryPercent_en_ADDR + offset));
  panBatteryPercent_XY[0][panel] = readEEPROM(panBatteryPercent_x_ADDR + offset);
  panBatteryPercent_XY[1][panel] = checkPAL(readEEPROM(panBatteryPercent_y_ADDR + offset));

  setBit(panA_REG[panel], Pit_BIT, readEEPROM(panPitch_en_ADDR + offset));
  panPitch_XY[0][panel]  = readEEPROM(panPitch_x_ADDR + offset);
  panPitch_XY[1][panel]  = checkPAL(readEEPROM(panPitch_y_ADDR + offset));

  setBit(panA_REG[panel], Rol_BIT, readEEPROM(panRoll_en_ADDR + offset));
  panRoll_XY[0][panel]   = readEEPROM(panRoll_x_ADDR + offset);
  panRoll_XY[1][panel]   = checkPAL(readEEPROM(panRoll_y_ADDR + offset));

  setBit(panA_REG[panel], BatA_BIT, readEEPROM(panBatt_A_en_ADDR + offset));
  panBatt_A_XY[0][panel] = readEEPROM(panBatt_A_x_ADDR + offset);
  panBatt_A_XY[1][panel] = checkPAL(readEEPROM(panBatt_A_y_ADDR + offset));

  // setBit(panA_REG, BatB_BIT, readEEPROM(panBatt_B_en_ADDR));
  // panBatt_B_XY[0] = readEEPROM(panBatt_B_x_ADDR);
  // panBatt_B_XY[1] = checkPAL(readEEPROM(panBatt_B_y_ADDR));

  setBit(panA_REG[panel], GPSats_BIT, readEEPROM(panGPSats_en_ADDR + offset));
  panGPSats_XY[0][panel] = readEEPROM(panGPSats_x_ADDR + offset);
  panGPSats_XY[1][panel] = checkPAL(readEEPROM(panGPSats_y_ADDR + offset));

  setBit(panA_REG[panel], GPL_BIT, readEEPROM(panGPL_en_ADDR + offset));
  panGPL_XY[0][panel]    = readEEPROM(panGPL_x_ADDR + offset);
  panGPL_XY[1][panel]    = checkPAL(readEEPROM(panGPL_y_ADDR + offset));

  setBit(panA_REG[panel], GPS_BIT, readEEPROM(panGPS_en_ADDR + offset));
  panGPS_XY[0][panel]    = readEEPROM(panGPS_x_ADDR + offset);
  panGPS_XY[1][panel]    = checkPAL(readEEPROM(panGPS_y_ADDR + offset));

  // ****** Second set of 8 Panels ******

  setBit(panB_REG[panel], Rose_BIT, readEEPROM(panRose_en_ADDR + offset));
  panRose_XY[0][panel]    = readEEPROM(panRose_x_ADDR + offset);
  panRose_XY[1][panel]    = checkPAL(readEEPROM(panRose_y_ADDR + offset));

#ifndef COMBINED_HEADING_AND_ROSE
  setBit(panB_REG[panel], Head_BIT, readEEPROM(panHeading_en_ADDR + offset));
  panHeading_XY[0][panel] = readEEPROM(panHeading_x_ADDR + offset);
  panHeading_XY[1][panel] = checkPAL(readEEPROM(panHeading_y_ADDR + offset));
#endif  

  //    setBit(panB_REG[panel], MavB_BIT, readEEPROM(panMavBeat_en_ADDR + offset));    // JDL Changed
  //    panMavBeat_XY[0][panel] = readEEPROM(panMavBeat_x_ADDR + offset);
  //    panMavBeat_XY[1][panel] = checkPAL(readEEPROM(panMavBeat_y_ADDR + offset));

  setBit(panB_REG[panel], HDis_BIT, readEEPROM(panHomeDis_en_ADDR + offset));
  panHomeDis_XY[0][panel] = readEEPROM(panHomeDis_x_ADDR + offset);
  panHomeDis_XY[1][panel] = checkPAL(readEEPROM(panHomeDis_y_ADDR + offset));

  setBit(panB_REG[panel], HDir_BIT, readEEPROM(panHomeDir_en_ADDR + offset));
  panHomeDir_XY[0][panel] = readEEPROM(panHomeDir_x_ADDR + offset);
  panHomeDir_XY[1][panel] = checkPAL(readEEPROM(panHomeDir_y_ADDR + offset));

#ifdef PATHPLAN_WAYPOINTS_SUPPORT
  setBit(panB_REG[panel], WDir_BIT, readEEPROM(panWPDir_en_ADDR + offset));    // JDL Changed
  panWPDir_XY[0][panel]   = readEEPROM(panWPDir_x_ADDR + offset);
  panWPDir_XY[1][panel]   = checkPAL(readEEPROM(panWPDir_y_ADDR + offset));
#endif  

#ifdef SAFETY_RADIUS
  setBit(panB_REG[panel], SR_EGE_BIT, readEEPROM(panSR_EGE_en_ADDR + offset));
  panSR_EGE_XY[0][panel]   = readEEPROM(panSR_EGE_x_ADDR + offset);
  panSR_EGE_XY[1][panel]   = checkPAL(readEEPROM(panSR_EGE_y_ADDR + offset));
#endif

  setBit(panB_REG[panel], Time_BIT, readEEPROM(panTime_en_ADDR + offset));    
  panTime_XY[0][panel]    = readEEPROM(panTime_x_ADDR + offset);
  panTime_XY[1][panel]    = checkPAL(readEEPROM(panTime_y_ADDR + offset));

  // setBit(panB_REG, RSSI_BIT, readEEPROM(panRSSI_en_ADDR));
  // panRSSI_XY[0] = readEEPROM(panRSSI_x_ADDR);
  // panRSSI_XY[1] = checkPAL(readEEPROM(panRSSI_y_ADDR));

  // ****** Third set of 8 Panels ******

  setBit(panC_REG[panel], CurA_BIT, readEEPROM(panCur_A_en_ADDR + offset));
  panCur_A_XY[0][panel] = readEEPROM(panCur_A_x_ADDR + offset);
  panCur_A_XY[1][panel] = checkPAL(readEEPROM(panCur_A_y_ADDR + offset));

  // setBit(panC_REG, CurB_BIT, readEEPROM(panCur_B_en_ADDR));
  // panCur_B_XY[0] = readEEPROM(panCur_B_x_ADDR);
  // panCur_B_XY[1] = checkPAL(readEEPROM(panCur_B_y_ADDR));

  setBit(panC_REG[panel], Alt_BIT, readEEPROM(panAlt_en_ADDR + offset));
  panAlt_XY[0][panel]      = readEEPROM(panAlt_x_ADDR + offset);
  panAlt_XY[1][panel]      = checkPAL(readEEPROM(panAlt_y_ADDR + offset));

  setBit(panC_REG[panel], Halt_BIT, readEEPROM(panHomeAlt_en_ADDR + offset));
  panHomeAlt_XY[0][panel]  = readEEPROM(panHomeAlt_x_ADDR + offset);
  panHomeAlt_XY[1][panel]  = checkPAL(readEEPROM(panHomeAlt_y_ADDR + offset));

  setBit(panC_REG[panel], As_BIT, readEEPROM(panAirSpeed_en_ADDR + offset));
  panAirSpeed_XY[0][panel] = readEEPROM(panAirSpeed_x_ADDR + offset);
  panAirSpeed_XY[1][panel] = checkPAL(readEEPROM(panAirSpeed_y_ADDR + offset));

  setBit(panC_REG[panel], Vel_BIT, readEEPROM(panVel_en_ADDR + offset));
  panVel_XY[0][panel]      = readEEPROM(panVel_x_ADDR + offset);
  panVel_XY[1][panel]      = checkPAL(readEEPROM(panVel_y_ADDR + offset));

  setBit(panC_REG[panel], Thr_BIT, readEEPROM(panThr_en_ADDR + offset));
  panThr_XY[0][panel]      = readEEPROM(panThr_x_ADDR + offset);
  panThr_XY[1][panel]      = checkPAL(readEEPROM(panThr_y_ADDR + offset));

  setBit(panC_REG[panel], FMod_BIT, readEEPROM(panFMod_en_ADDR + offset));
  panFMod_XY[0][panel]     = readEEPROM(panFMod_x_ADDR + offset);
  panFMod_XY[1][panel]     = checkPAL(readEEPROM(panFMod_y_ADDR + offset));

  setBit(panC_REG[panel], Hor_BIT, readEEPROM(panHorizon_en_ADDR + offset));
  panHorizon_XY[0][panel]  = readEEPROM(panHorizon_x_ADDR + offset);
  panHorizon_XY[1][panel]  = checkPAL(readEEPROM(panHorizon_y_ADDR + offset));

  setBit(panD_REG[panel], Warn_BIT, readEEPROM(panWarn_en_ADDR + offset));
  panWarn_XY[0][panel]     = readEEPROM(panWarn_x_ADDR + offset);
  panWarn_XY[1][panel]     = checkPAL(readEEPROM(panWarn_y_ADDR + offset));

  // setBit(panD_REG[panel], Off_BIT, readEEPROM(panOff_en_ADDR + offset));
  // panOff_XY[0] = readEEPROM(panOff_x_ADDR + offset);
  // panOff_XY[1] = checkPAL(readEEPROM(panOff_y_ADDR + offset));

  //    setBit(panD_REG[panel], WindS_BIT, readEEPROM(panWindSpeed_en_ADDR + offset));    // JDL Changed
  //    panWindSpeed_XY[0][panel] = readEEPROM(panWindSpeed_x_ADDR + offset);
  //    panWindSpeed_XY[1][panel] = checkPAL(readEEPROM(panWindSpeed_y_ADDR + offset));

  setBit(panD_REG[panel], Climb_BIT, readEEPROM(panClimb_en_ADDR + offset));
  panClimb_XY[0][panel]     = readEEPROM(panClimb_x_ADDR + offset);
  panClimb_XY[1][panel]     = checkPAL(readEEPROM(panClimb_y_ADDR + offset));

  //    setBit(panD_REG[panel], Tune_BIT, readEEPROM(panTune_en_ADDR + offset));        // JDL Changed
  //    panTune_XY[0][panel] = readEEPROM(panTune_x_ADDR + offset);
  //    panTune_XY[1][panel] = checkPAL(readEEPROM(panTune_y_ADDR + offset));

  // setBit(panD_REG[panel], Setup_BIT, readEEPROM(panSetup_en_ADDR));
  // panSetup_XY[0] = readEEPROM(panSetup_x_ADDR);
  // panSetup_XY[1] = checkPAL(readEEPROM(panSetup_y_ADDR));

  setBit(panD_REG[panel], RSSI_BIT, readEEPROM(panRSSI_en_ADDR + offset));
  panRSSI_XY[0][panel]     = readEEPROM(panRSSI_x_ADDR + offset);
  panRSSI_XY[1][panel]     = checkPAL(readEEPROM(panRSSI_y_ADDR + offset));

  setBit(panE_REG[panel], DIST_BIT, readEEPROM(panDistance_en_ADDR + offset));
  panDistance_XY[0][panel] = readEEPROM(panDistance_x_ADDR + offset);
  panDistance_XY[1][panel] = checkPAL(readEEPROM(panDistance_y_ADDR + offset));

  setBit(panE_REG[panel], TEMP_BIT, readEEPROM(panTemp_en_ADDR + offset));
  panTemp_XY[0][panel]     = readEEPROM(panTemp_x_ADDR + offset);
  panTemp_XY[1][panel]     = checkPAL(readEEPROM(panTemp_y_ADDR + offset));
}

int checkPAL(int line)
{
  if (line >= osd.getCenter() && osd.getMode() == 0) {
    line -= 3; // Cutting lines offset after center if NTSC
  }
  return line;
}

