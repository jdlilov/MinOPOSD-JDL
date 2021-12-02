// JDL

#if defined NEXT_UAVO
//----------------------------------------------------------------- START OF "NEXT_UAVO" -------------------------------------------------------------------------

/**
 ******************************************************************************
 *
 * @file       UAVTalk.h
 * @author     Joerg-D. Rothfuchs
 * @brief      Implements a subset of the telemetry communication between
 *             OpenPilot CC, CC3D, Revolution and Ardupilot Mega MinimOSD
 *             with code from OpenPilot and MinimOSD.
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


#ifndef UAVTALK_H_
#define UAVTALK_H_


#include "OSD_Config.h"


// TODO enhancement:
// Generate the following automatically out of the XML files.
//
// These object constants are version dependent!
//
// Short hints getting them manually:
// http://wiki.openpilot.org/display/Doc/Windows%3A+Building+and+Packaging
// git clone git://git.openpilot.org/OpenPilot.git OpenPilot
// QT Creator > Datei oder Projekt oeffnen... uavobjgenerator
// generate release and copy exe to <top>\ground\uavobjgenerator
// uavobjgenerator -flight ..\..\shared\uavobjectdefinition ..\..\
// //

#define ACTUATORDESIRED_OBJID					0xEAE65C28

#define ACTUATORDESIRED_OBJ_THRUST                              12


#define WAYPOINTACTIVE_OBJID					0x1EA5B19C

#define WAYPOINTACTIVE_OBJID_INDEX				0


#define PATHPLAN_OBJID						0x82F5D500

#define PATHPLAN_OBJID_WAYPOINTCOUNT				0
#define PATHPLAN_OBJID_PATHACTIONCOUNT				2
#define PATHPLAN_OBJID_CRC					4


#define PATHDESIRED_OBJID					0xBCD3B396		

#define PATHDESIRED_OBJID_START_NORTH				0
#define PATHDESIRED_OBJID_START_EAST				4
#define PATHDESIRED_OBJID_START_DOWN				8
#define PATHDESIRED_OBJID_END_NORTH				12
#define PATHDESIRED_OBJID_END_EAST				16
#define PATHDESIRED_OBJID_END_DOWN				20
#define PATHDESIRED_OBJID_STARTINGVELOCITY			24
#define PATHDESIRED_OBJID_ENDINGVELOCITY			28
#define PATHDESIRED_OBJID_MODEPARAMS_0				32
#define PATHDESIRED_OBJID_MODEPARAMS_1				36
#define PATHDESIRED_OBJID_MODEPARAMS_2				40
#define PATHDESIRED_OBJID_MODEPARAMS_3				44
#define PATHDESIRED_OBJID_UID					48
#define PATHDESIRED_OBJID_MODE					50

//#define PATHDESIRED_NUMBYTES					51

#define PATHDESIRED_MODE_GOTOENDPOINT				0
#define PATHDESIRED_MODE_FOLLOWVECTOR				1
#define PATHDESIRED_MODE_CIRCLERIGHT				2
#define PATHDESIRED_MODE_CIRCLELEFT				3
#define PATHDESIRED_MODE_FIXEDATTITUDE				4
#define PATHDESIRED_MODE_SETACCESSORY				5
#define PATHDESIRED_MODE_DISARMALARM				6
#define PATHDESIRED_MODE_LAND					7
#define PATHDESIRED_MODE_BRAKE					8
#define PATHDESIRED_MODE_VELOCITY				9
#define PATHDESIRED_MODE_AUTOTAKEOFF				10



#define FLIGHTSTATUS_OBJID_008         0x24D25E28      // RC1

#define SYSTEMALARMS_OBJID_006         0x85C20346      // 16.09-->next

#define OPLINKSTATUS_OBJID_003         0xC4A03B86      // 16.09-->next
                    
#define OPLINKSTATUS_OBJ_RSSI          90	       // 16.09-->next	
#define OPLINKSTATUS_OBJ_LINKQUALITY   14	       // 16.09-->next


#define OPLINKRECEIVER_OBJID           0x3A4DC72C      // 16.09-->next

#define OPLINKRECEIVER_OBJ_RSSI          32              // 16.09-->next	
#define OPLINKRECEIVER_OBJ_LINKQUALITY   33              // 16.09-->next

#define ATTITUDESTATE_OBJID                 0x7421C7BA      // 16.09-->next

#define GPSTIME_OBJID                       0x1E2F477E		// 16.09-->next

#define GPSVELOCITYSENSOR_OBJID             0x0BC57454      // new name since VERSION_RELEASE_14_01_1

#define FLIGHTTELEMETRYSTATS_OBJ_LEN        21
#define FLIGHTTELEMETRYSTATS_OBJ_STATUS     20
#define FLIGHTTELEMETRYSTATS_OBJ_LEN_001    37              // different since VERSION_RELEASE_14_01_1
#define FLIGHTTELEMETRYSTATS_OBJ_STATUS_001 36 // different since VERSION_RELEASE_14_01_1

#define GCSTELEMETRYSTATS_OBJ_LEN           21
#define GCSTELEMETRYSTATS_OBJ_STATUS        20
#define GCSTELEMETRYSTATS_OBJ_LEN_001       37              // different since VERSION_RELEASE_14_01_1
#define GCSTELEMETRYSTATS_OBJ_STATUS_001    36              // different since VERSION_RELEASE_14_01_1

#define ATTITUDEACTUAL_OBJ_ROLL             16
#define ATTITUDEACTUAL_OBJ_PITCH            20
#define ATTITUDEACTUAL_OBJ_YAW              24

#define FLIGHTSTATUS_OBJ_ARMED              0
#define FLIGHTSTATUS_OBJ_FLIGHTMODE         1  // in LP15_09_next_r616
#define FLIGHTSTATUS_OBJ_ASWA               2

#define MANUALCONTROLCOMMAND_OBJ_THROTTLE   0
#define MANUALCONTROLCOMMAND_OBJ_CHANNEL_0  24
#define MANUALCONTROLCOMMAND_OBJ_CHANNEL_1  26
#define MANUALCONTROLCOMMAND_OBJ_CHANNEL_2  28
#define MANUALCONTROLCOMMAND_OBJ_CHANNEL_3  30
#define MANUALCONTROLCOMMAND_OBJ_CHANNEL_4  32
#define MANUALCONTROLCOMMAND_OBJ_CHANNEL_5  34
#define MANUALCONTROLCOMMAND_OBJ_CHANNEL_6  36
#define MANUALCONTROLCOMMAND_OBJ_CHANNEL_7  38
#define MANUALCONTROLCOMMAND_OBJ_CHANNEL_8  40
#define MANUALCONTROLCOMMAND_OBJ_CHANNEL_9  42
#define MANUALCONTROLCOMMAND_OBJ_CHANNEL_10 44
#define MANUALCONTROLCOMMAND_OBJ_CHANNEL_11 46


#define GPSPOSITION_OBJ_LAT                 0
#define GPSPOSITION_OBJ_LON                 4
#define GPSPOSITION_OBJ_ALTITUDE            8
#define GPSPOSITION_OBJ_GEOIDSEPARATION     12
#define GPSPOSITION_OBJ_HEADING             16
#define GPSPOSITION_OBJ_GROUNDSPEED         20
#define GPSPOSITION_OBJ_PDOP                24
#define GPSPOSITION_OBJ_HDOP                28
#define GPSPOSITION_OBJ_VDOP                32
#define GPSPOSITION_OBJ_STATUS              36
#define GPSPOSITION_OBJ_SATELLITES          37

#define GPSTIME_OBJ_YEAR                    0
#define GPSTIME_OBJ_MONTH                   4
#define GPSTIME_OBJ_DAY                     5
#define GPSTIME_OBJ_HOUR                    6
#define GPSTIME_OBJ_MINUTE                  7
#define GPSTIME_OBJ_SECOND                  8


#define GPSVELOCITY_OBJ_NORTH               0
#define GPSVELOCITY_OBJ_EAST                4
#define GPSVELOCITY_OBJ_DOWN                8


#define SYSTEMALARMS_ALARM_OUTOFMEMORY      2
#define SYSTEMALARMS_ALARM_STACKOVERFLOW    3
#define SYSTEMALARMS_ALARM_CPUOVERLOAD      4
#define SYSTEMALARMS_ALARM_EVENTSYSTEM      5
#define SYSTEMALARMS_ALARM_MANUALCONTROL    8
#define SYSTEMALARMS_ALARM_ATTITUDE         10      
#define SYSTEMALARMS_ALARM_MAGNETOMETER     13      // 16.09-->next
#define SYSTEMALARMS_ALARM_STABILIZATION    15      // 16.09-->next
#define SYSTEMALARMS_ALARM_PATHPLAN         17	    // next_r735
#define SYSTEMALARMS_ALARM_BATTERY          18      // 16.09-->next


#define FLIGHTTELEMETRYSTATS_OBJID_001 0x6737BB5A 

#define GCSTELEMETRYSTATS_OBJID_001    0xCAD1DC0A   

#define MANUALCONTROLCOMMAND_OBJID_003 0x265BA97E // 16.09-->next

#define GPSPOSITIONSENSOR_OBJID_003    0xA529B6FE      // 16.09-->next

#if defined FLIGHT_BATT_ON_REVO

#define FLIGHTBATTERYSTATE_OBJID_001                 0x26962352   

#define FLIGHTBATTERYSTATE_OBJ_VOLTAGE               0
#define FLIGHTBATTERYSTATE_OBJ_CURRENT               4			
#define FLIGHTBATTERYSTATE_OBJ_BOARD_SUPPLY_VOLTAGE  8
#define FLIGHTBATTERYSTATE_OBJ_PEAK_CURRENT          12
#define FLIGHTBATTERYSTATE_OBJ_AVG_CURRENT           16
#define FLIGHTBATTERYSTATE_OBJ_CONSUMED_ENERGY       20
#define FLIGHTBATTERYSTATE_OBJ_ESTIMATED_FLIGHT_TIME 24

#endif // FLIGHT_BATT_ON_REVO


#ifdef FIXED_WING

#define AIRSPEEDSTATE_OBJID                0xC7009F28

#define AIRSPEEDSTATE_OBJ_CALIBRATEDAIRSPEED  0
#define AIRSPEEDSTATE_OBJ_TRUEAIRSPEED        4

#endif  // FIXED_WING


#if defined REVO_ADD_ONS

#define BAROSENSOR_OBJID                     0x48120EA6      

#define BAROALTITUDE_OBJ_ALTITUDE            0
#define BAROALTITUDE_OBJ_TEMPERATURE         4
#define BAROALTITUDE_OBJ_PRESSURE            8

#define POSITIONSTATE_OBJID                  0x4AFDB658   

#define POSITIONSTATE_OBJ_NORTH              0
#define POSITIONSTATE_OBJ_EAST               4
#define POSITIONSTATE_OBJ_DOWN               8               

#define VELOCITYSTATE_OBJID                  0xC243686C   

#define VELOCITYSTATE_OBJ_NORTH              0
#define VELOCITYSTATE_OBJ_EAST               4
#define VELOCITYSTATE_OBJ_DOWN               8   

#define MAGSTATE_OBJID                       0x9FFEAA0C      // For LibrePilot 15.09 next_r616

#define MAGSTATE_OBJ_X                       0
#define MAGSTATE_OBJ_Y                       4
#define MAGSTATE_OBJ_Z                       8
#define MAGSTATE_OBJ_SOURCE                  12

#define MAGSTATE_SOURCE_INVALID              0
#define MAGSTATE_SOURCE_ONBOARD              1
#define MAGSTATE_SOURCE_AUX                  2

#endif // REVO_ADD_ONS

#define RECEIVERSTATUS_OBJID                 0xFD24EDF2
#define RECEIVERSTATUS_OBJ_QUALITY           0

#define TXPIDSTATUS_OBJID                    0x18EDDF50
#define TXPIDSTATUS_OBJ_CURPID1              0
#define TXPIDSTATUS_OBJ_CURPID2              4
#define TXPIDSTATUS_OBJ_CURPID3              8

#define UAVTALK_MODE_PASSIVE                 0x01            // do not send any UAVTalk packets

#define FLIGHTTELEMETRYSTATS_CONNECT_TIMEOUT 10000
#define GCSTELEMETRYSTATS_SEND_PERIOD        1000

#if defined VERSION_RELEASE_12_10_1 || defined VERSION_RELEASE_12_10_2 || defined VERSION_RELEASE_13_06_1 || defined VERSION_RELEASE_13_06_2
#define HEADER_LEN                           8
#else
#define HEADER_LEN                           10
#endif

#define RESPOND_OBJ_LEN                      HEADER_LEN
#define REQUEST_OBJ_LEN                      HEADER_LEN

#define UAVTALK_SYNC_VAL                     0x3C

#define UAVTALK_TYPE_MASK                    0xF8
#define UAVTALK_TYPE_VER                     0x20

#define UAVTALK_TYPE_OBJ                     (UAVTALK_TYPE_VER | 0x00)
#define UAVTALK_TYPE_OBJ_REQ                 (UAVTALK_TYPE_VER | 0x01)
#define UAVTALK_TYPE_OBJ_ACK                 (UAVTALK_TYPE_VER | 0x02)
#define UAVTALK_TYPE_ACK                     (UAVTALK_TYPE_VER | 0x03)
#define UAVTALK_TYPE_NACK                    (UAVTALK_TYPE_VER | 0x04)


typedef enum {
    UAVTALK_PARSE_STATE_WAIT_SYNC = 0,
    UAVTALK_PARSE_STATE_GOT_SYNC,
    UAVTALK_PARSE_STATE_GOT_MSG_TYPE,
    UAVTALK_PARSE_STATE_GOT_LENGTH,
    UAVTALK_PARSE_STATE_GOT_OBJID,
    UAVTALK_PARSE_STATE_GOT_INSTID,
    UAVTALK_PARSE_STATE_GOT_DATA,
    UAVTALK_PARSE_STATE_GOT_CRC
} uavtalk_parse_state_t;


typedef enum {
    TELEMETRYSTATS_STATE_DISCONNECTED = 0,
    TELEMETRYSTATS_STATE_HANDSHAKEREQ,
    TELEMETRYSTATS_STATE_HANDSHAKEACK,
    TELEMETRYSTATS_STATE_CONNECTED
} telemetrystats_state_t;


typedef struct __uavtalk_message {
    uint8_t  Sync;
    uint8_t  MsgType;
    uint16_t Length;
    uint32_t ObjID;
    uint16_t InstID;
    uint8_t  Data[255];
    uint8_t  Crc;
} uavtalk_message_t;


int uavtalk_read(void);
int uavtalk_state(void);


#endif /* UAVTALK_H_ */

//----------------------------------------------------------------- END OF "NEXT_UAVO" -------------------------------------------------------------------------

#else // not defined NEXT_UAVO

//----------------------------------------------------------------- START OF "16.09_UAVO" -------------------------------------------------------------------------

/**
 ******************************************************************************
 *
 * @file       UAVTalk.h
 * @author     Joerg-D. Rothfuchs
 * @brief      Implements a subset of the telemetry communication between
 *             OpenPilot CC, CC3D, Revolution and Ardupilot Mega MinimOSD
 *             with code from OpenPilot and MinimOSD.
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


#ifndef UAVTALK_H_
#define UAVTALK_H_


#include "OSD_Config.h"


// TODO enhancement:
// Generate the following automatically out of the XML files.
//
// These object constants are version dependent!
//
// Short hints getting them manually:
// http://wiki.openpilot.org/display/Doc/Windows%3A+Building+and+Packaging
// git clone git://git.openpilot.org/OpenPilot.git OpenPilot
// QT Creator > Datei oder Projekt oeffnen... uavobjgenerator
// generate release and copy exe to <top>\ground\uavobjgenerator
// uavobjgenerator -flight ..\..\shared\uavobjectdefinition ..\..\
// //

#define ACTUATORDESIRED_OBJID					0xEAE65C28

#define ACTUATORDESIRED_OBJ_THRUST                              12


#define WAYPOINTACTIVE_OBJID					0x1EA5B19C

#define WAYPOINTACTIVE_OBJID_INDEX				0


#define PATHPLAN_OBJID						0x82F5D500

#define PATHPLAN_OBJID_WAYPOINTCOUNT				0
#define PATHPLAN_OBJID_PATHACTIONCOUNT				2
#define PATHPLAN_OBJID_CRC					4


#define PATHDESIRED_OBJID					0xBCD3B396		

#define PATHDESIRED_OBJID_START_NORTH				0
#define PATHDESIRED_OBJID_START_EAST				4
#define PATHDESIRED_OBJID_START_DOWN				8
#define PATHDESIRED_OBJID_END_NORTH				12
#define PATHDESIRED_OBJID_END_EAST				16
#define PATHDESIRED_OBJID_END_DOWN				20
#define PATHDESIRED_OBJID_STARTINGVELOCITY			24
#define PATHDESIRED_OBJID_ENDINGVELOCITY			28
#define PATHDESIRED_OBJID_MODEPARAMS_0				32
#define PATHDESIRED_OBJID_MODEPARAMS_1				36
#define PATHDESIRED_OBJID_MODEPARAMS_2				40
#define PATHDESIRED_OBJID_MODEPARAMS_3				44
#define PATHDESIRED_OBJID_UID					48
#define PATHDESIRED_OBJID_MODE					50

//#define PATHDESIRED_NUMBYTES					51

#define PATHDESIRED_MODE_GOTOENDPOINT				0
#define PATHDESIRED_MODE_FOLLOWVECTOR				1
#define PATHDESIRED_MODE_CIRCLERIGHT				2
#define PATHDESIRED_MODE_CIRCLELEFT				3
#define PATHDESIRED_MODE_FIXEDATTITUDE				4
#define PATHDESIRED_MODE_SETACCESSORY				5
#define PATHDESIRED_MODE_DISARMALARM				6
#define PATHDESIRED_MODE_LAND					7
#define PATHDESIRED_MODE_BRAKE					8
#define PATHDESIRED_MODE_VELOCITY				9
#define PATHDESIRED_MODE_AUTOTAKEOFF				10



#ifdef NEXT_R616

  #define FLIGHTSTATUS_OBJID_008         0x6E3BDB60      // VERSION_RELEASE_LP15_09_next_r616

  #define SYSTEMALARMS_OBJID_006         0x6B7639EC      // different ID for VERSION_RELEASE_LP15_09

  #define OPLINKSTATUS_OBJID_003         0xCC2F6630      // r616
                    
  #define OPLINKSTATUS_OBJ_RSSI          72
  #define OPLINKSTATUS_OBJ_LINKQUALITY   73

#else  

  #define FLIGHTSTATUS_OBJID_008         0x24D25E28      // RC1
  #define SYSTEMALARMS_OBJID_006         0x20E6CAAC      // RC1

  #define OPLINKSTATUS_OBJID_003         0xDED43774      // different ID for VERSION_RELEASE_LP_15_09_next_r957
                    
  #define OPLINKSTATUS_OBJ_RSSI          102
  #define OPLINKSTATUS_OBJ_LINKQUALITY   26 

#endif  

/*  r616
    typedef struct {
        quint32 DeviceID;				0		
        quint32 PairIDs[4];				4		
        quint16 BoardRevision;			        20		
        quint16 HeapRemaining;			        22		
        quint16 UAVTalkErrors;			        24
    uint16_t LinkQuality;                                      26
        quint16 TXRate;					26     28 
        quint16 RXRate;					28     30
        quint16 TXSeq;					30     32
        quint16 RXSeq;					32     34
        quint16 TXPacketRate;			        34     36
        quint16 RXPacketRate;			        36     38 
        quint8 Description[40];			        38     40
        quint8 CPUSerial[12];			        78     80
        quint8 BoardType;				90     92
        quint8 RxGood;					91     93
        quint8 RxCorrected;				92     94
        quint8 RxErrors;				93     95
        quint8 RxMissed;				94     96
        quint8 RxFailure;				95     97
        quint8 TxDropped;				96     98
        quint8 TxFailure;				97     99
        quint8 Resets;					98     100
        quint8 Timeouts;				99     101
        qint8 RSSI;					100    102
        quint8 LinkState;				103
        qint8 PairSignalStrengths[4];	                104
							108
    } __attribute__((packed)) DataFields;
*/


/*  RC1
    typedef struct {
        quint32 DeviceID;				0		
        quint32 PairIDs[4];				4		
        quint16 BoardRevision;			        20		
        quint16 HeapRemaining;			        22		
        quint16 UAVTalkErrors;			        24
    uint16_t LinkQuality;                                      26
        quint16 TXRate;					26     28 
        quint16 RXRate;					28     30
        quint16 TXSeq;					30     32
        quint16 RXSeq;					32     34
        quint16 TXPacketRate;			        34     36
        quint16 RXPacketRate;			        36     38 
        quint8 Description[40];			        38     40
        quint8 CPUSerial[12];			        78     80
        quint8 BoardType;				90     92
        quint8 RxGood;					91     93
        quint8 RxCorrected;				92     94
        quint8 RxErrors;				93     95
        quint8 RxMissed;				94     96
        quint8 RxFailure;				95     97
        quint8 TxDropped;				96     98
        quint8 TxFailure;				97     99
        quint8 Resets;					98     100
        quint8 Timeouts;				99     101
        qint8 RSSI;					100    102
        quint8 LinkState;				103
        qint8 PairSignalStrengths[4];	                104
							108
    } __attribute__((packed)) DataFields;
*/

#define ATTITUDESTATE_OBJID                 0xD7E0D964      // new name since VERSION_RELEASE_14_01_1

#define GPSTIME_OBJID                       0xD4478084

#define GPSVELOCITYSENSOR_OBJID             0x0BC57454      // new name since VERSION_RELEASE_14_01_1

#define FLIGHTTELEMETRYSTATS_OBJ_LEN        21
#define FLIGHTTELEMETRYSTATS_OBJ_STATUS     20
#define FLIGHTTELEMETRYSTATS_OBJ_LEN_001    37              // different since VERSION_RELEASE_14_01_1
#define FLIGHTTELEMETRYSTATS_OBJ_STATUS_001 36 // different since VERSION_RELEASE_14_01_1

#define GCSTELEMETRYSTATS_OBJ_LEN           21
#define GCSTELEMETRYSTATS_OBJ_STATUS        20
#define GCSTELEMETRYSTATS_OBJ_LEN_001       37              // different since VERSION_RELEASE_14_01_1
#define GCSTELEMETRYSTATS_OBJ_STATUS_001    36              // different since VERSION_RELEASE_14_01_1

#define ATTITUDEACTUAL_OBJ_ROLL             16
#define ATTITUDEACTUAL_OBJ_PITCH            20
#define ATTITUDEACTUAL_OBJ_YAW              24

#define FLIGHTSTATUS_OBJ_ARMED              0
#define FLIGHTSTATUS_OBJ_FLIGHTMODE         1  // in LP15_09_next_r616
#define FLIGHTSTATUS_OBJ_ASWA               2

#define MANUALCONTROLCOMMAND_OBJ_THROTTLE   0
#define MANUALCONTROLCOMMAND_OBJ_CHANNEL_0  24
#define MANUALCONTROLCOMMAND_OBJ_CHANNEL_1  26
#define MANUALCONTROLCOMMAND_OBJ_CHANNEL_2  28
#define MANUALCONTROLCOMMAND_OBJ_CHANNEL_3  30
#define MANUALCONTROLCOMMAND_OBJ_CHANNEL_4  32
#define MANUALCONTROLCOMMAND_OBJ_CHANNEL_5  34
#define MANUALCONTROLCOMMAND_OBJ_CHANNEL_6  36
#define MANUALCONTROLCOMMAND_OBJ_CHANNEL_7  38
#define MANUALCONTROLCOMMAND_OBJ_CHANNEL_8  40
#define MANUALCONTROLCOMMAND_OBJ_CHANNEL_9  42
#define MANUALCONTROLCOMMAND_OBJ_CHANNEL_10 44
#define MANUALCONTROLCOMMAND_OBJ_CHANNEL_11 46


#define GPSPOSITION_OBJ_LAT                 0
#define GPSPOSITION_OBJ_LON                 4
#define GPSPOSITION_OBJ_ALTITUDE            8
#define GPSPOSITION_OBJ_GEOIDSEPARATION     12
#define GPSPOSITION_OBJ_HEADING             16
#define GPSPOSITION_OBJ_GROUNDSPEED         20
#define GPSPOSITION_OBJ_PDOP                24
#define GPSPOSITION_OBJ_HDOP                28
#define GPSPOSITION_OBJ_VDOP                32
#define GPSPOSITION_OBJ_STATUS              36
#define GPSPOSITION_OBJ_SATELLITES          37

#define GPSTIME_OBJ_YEAR                    0
#define GPSTIME_OBJ_MONTH                   2
#define GPSTIME_OBJ_DAY                     3
#define GPSTIME_OBJ_HOUR                    4
#define GPSTIME_OBJ_MINUTE                  5
#define GPSTIME_OBJ_SECOND                  6

#define GPSVELOCITY_OBJ_NORTH               0
#define GPSVELOCITY_OBJ_EAST                4
#define GPSVELOCITY_OBJ_DOWN                8


#define SYSTEMALARMS_ALARM_OUTOFMEMORY      2
#define SYSTEMALARMS_ALARM_STACKOVERFLOW    3
#define SYSTEMALARMS_ALARM_CPUOVERLOAD      4
#define SYSTEMALARMS_ALARM_EVENTSYSTEM      5
#define SYSTEMALARMS_ALARM_MANUALCONTROL    8
#define SYSTEMALARMS_ALARM_MAGNETOMETER     12

#define SYSTEMALARMS_ALARM_ATTITUDE         10
#define SYSTEMALARMS_ALARM_STABILIZATION    14

#define FLIGHTTELEMETRYSTATS_OBJID_001 0x6737BB5A // different ID for VERSION_RELEASE_14_01_1 and VERSION_RELEASE_14_06_1 and VERSION_RELEASE_14_10_1 and VERSION_RELEASE_15_01_1 and VERSION_RELEASE_15_02_1

#define GCSTELEMETRYSTATS_OBJID_001    0xCAD1DC0A      // different ID for VERSION_RELEASE_14_01_1 and VERSION_RELEASE_14_06_1 and VERSION_RELEASE_14_10_1 and VERSION_RELEASE_15_01_1 and VERSION_RELEASE_15_02_1

#define MANUALCONTROLCOMMAND_OBJID_003 0xC4107480 // different for VERSION_RELEASE_LP15_09

#define GPSPOSITIONSENSOR_OBJID_003    0x9DF1F67A      // different ID for VERSION_RELEASE_LP15_09_next_r616

#if defined FLIGHT_BATT_ON_REVO

#define FLIGHTBATTERYSTATE_OBJID_001                 0x26962352      // different ID for VERSION_RELEASE_14_06_1 and VERSION_RELEASE_14_10_1 and VERSION_RELEASE_15_01_1 and VERSION_RELEASE_15_02_1

#define FLIGHTBATTERYSTATE_OBJ_VOLTAGE               0
#define FLIGHTBATTERYSTATE_OBJ_CURRENT               4				
#define FLIGHTBATTERYSTATE_OBJ_BOARD_SUPPLY_VOLTAGE  8
#define FLIGHTBATTERYSTATE_OBJ_PEAK_CURRENT          12
#define FLIGHTBATTERYSTATE_OBJ_AVG_CURRENT           16
#define FLIGHTBATTERYSTATE_OBJ_CONSUMED_ENERGY       20
#define FLIGHTBATTERYSTATE_OBJ_ESTIMATED_FLIGHT_TIME 24

#endif // FLIGHT_BATT_ON_REVO


#ifdef FIXED_WING

#define AIRSPEEDSTATE_OBJID                0xC7009F28

#define AIRSPEEDSTATE_OBJ_CALIBRATEDAIRSPEED  0
#define AIRSPEEDSTATE_OBJ_TRUEAIRSPEED        4

#endif  // FIXED_WING


#if defined REVO_ADD_ONS

#define BAROSENSOR_OBJID                     0x48120EA6      // new name since VERSION_RELEASE_14_01_1 and VERSION_RELEASE_14_10_1 and VERSION_RELEASE_15_01_1 and VERSION_RELEASE_15_02_1

#define BAROALTITUDE_OBJ_ALTITUDE            0
#define BAROALTITUDE_OBJ_TEMPERATURE         4
#define BAROALTITUDE_OBJ_PRESSURE            8

#define POSITIONSTATE_OBJID                  0x4AFDB658      // VERSION_RELEASE_LP15_09

#define POSITIONSTATE_OBJ_NORTH              0
#define POSITIONSTATE_OBJ_EAST               4
#define POSITIONSTATE_OBJ_DOWN               8               

#define VELOCITYSTATE_OBJID                  0xC243686C      // VERSION_RELEASE_LP15_09

#define VELOCITYSTATE_OBJ_NORTH              0
#define VELOCITYSTATE_OBJ_EAST               4
#define VELOCITYSTATE_OBJ_DOWN               8   

#define MAGSTATE_OBJID                       0x9FFEAA0C      // For LibrePilot 15.09 next_r616

#define MAGSTATE_OBJ_X                       0
#define MAGSTATE_OBJ_Y                       4
#define MAGSTATE_OBJ_Z                       8
#define MAGSTATE_OBJ_SOURCE                  12

#define MAGSTATE_SOURCE_INVALID              0
#define MAGSTATE_SOURCE_ONBOARD              1
#define MAGSTATE_SOURCE_AUX                  2

#endif // REVO_ADD_ONS

#define RECEIVERSTATUS_OBJID                 0xFD24EDF2
#define RECEIVERSTATUS_OBJ_QUALITY           0

#define TXPIDSTATUS_OBJID                    0x18EDDF50
#define TXPIDSTATUS_OBJ_CURPID1              0
#define TXPIDSTATUS_OBJ_CURPID2              4
#define TXPIDSTATUS_OBJ_CURPID3              8

#define UAVTALK_MODE_PASSIVE                 0x01            // do not send any UAVTalk packets

#define FLIGHTTELEMETRYSTATS_CONNECT_TIMEOUT 10000
#define GCSTELEMETRYSTATS_SEND_PERIOD        1000

#if defined VERSION_RELEASE_12_10_1 || defined VERSION_RELEASE_12_10_2 || defined VERSION_RELEASE_13_06_1 || defined VERSION_RELEASE_13_06_2
#define HEADER_LEN                           8
#else
#define HEADER_LEN                           10
#endif

#define RESPOND_OBJ_LEN                      HEADER_LEN
#define REQUEST_OBJ_LEN                      HEADER_LEN

#define UAVTALK_SYNC_VAL                     0x3C

#define UAVTALK_TYPE_MASK                    0xF8
#define UAVTALK_TYPE_VER                     0x20

#define UAVTALK_TYPE_OBJ                     (UAVTALK_TYPE_VER | 0x00)
#define UAVTALK_TYPE_OBJ_REQ                 (UAVTALK_TYPE_VER | 0x01)
#define UAVTALK_TYPE_OBJ_ACK                 (UAVTALK_TYPE_VER | 0x02)
#define UAVTALK_TYPE_ACK                     (UAVTALK_TYPE_VER | 0x03)
#define UAVTALK_TYPE_NACK                    (UAVTALK_TYPE_VER | 0x04)


typedef enum {
    UAVTALK_PARSE_STATE_WAIT_SYNC = 0,
    UAVTALK_PARSE_STATE_GOT_SYNC,
    UAVTALK_PARSE_STATE_GOT_MSG_TYPE,
    UAVTALK_PARSE_STATE_GOT_LENGTH,
    UAVTALK_PARSE_STATE_GOT_OBJID,
    UAVTALK_PARSE_STATE_GOT_INSTID,
    UAVTALK_PARSE_STATE_GOT_DATA,
    UAVTALK_PARSE_STATE_GOT_CRC
} uavtalk_parse_state_t;


typedef enum {
    TELEMETRYSTATS_STATE_DISCONNECTED = 0,
    TELEMETRYSTATS_STATE_HANDSHAKEREQ,
    TELEMETRYSTATS_STATE_HANDSHAKEACK,
    TELEMETRYSTATS_STATE_CONNECTED
} telemetrystats_state_t;


typedef struct __uavtalk_message {
    uint8_t  Sync;
    uint8_t  MsgType;
    uint16_t Length;
    uint32_t ObjID;
    uint16_t InstID;
    uint8_t  Data[255];
    uint8_t  Crc;
} uavtalk_message_t;


int uavtalk_read(void);
int uavtalk_state(void);


#endif /* UAVTALK_H_ */

//----------------------------------------------------------------- END OF "16.09_UAVO" -------------------------------------------------------------------------

#endif // NEXT_UAVO
