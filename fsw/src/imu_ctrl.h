/*
**  Copyright 2022 bitValence, Inc.
**  All Rights Reserved.
**
**  This program is free software; you can modify and/or redistribute it
**  under the terms of the GNU Affero General Public License
**  as published by the Free Software Foundation; version 3 with
**  attribution addendums as found in the LICENSE.txt
**
**  This program is distributed in the hope that it will be useful,
**  but WITHOUT ANY WARRANTY; without even the implied warranty of
**  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
**  GNU Affero General Public License for more details.
**
**  Purpose:
**    Define IMU Controller class
**
**  Notes:
**    None 
**
**  References:
**    1. OpenSatKit Object-based Application Developer's Guide
**    2. cFS Application Developer's Guide
**
*/

#ifndef _imu_ctrl_
#define _imu_ctrl_

/*
** Includes
*/

#include "app_cfg.h"
#include "imu_i2c.h"
#include "mqtt_gw_eds_typedefs.h"

/***********************/
/** Macro Definitions **/
/***********************/


/*
** Event Message IDs
*/

#define IMU_CTRL_CONSTRUCTOR_EID                     (IMU_CTRL_BASE_EID + 0)
#define IMU_CTRL_SET_SENSOR_DELTA_TIME_EID           (IMU_CTRL_BASE_EID + 1)
#define IMU_CTRL_SET_ACCELEROMETER_SCALE_FACTOR_EID  (IMU_CTRL_BASE_EID + 2)
#define IMU_CTRL_SET_FILTER_CONSTANT_EID             (IMU_CTRL_BASE_EID + 3)
#define IMU_CTRL_SET_GYRO_SCALE_FACTOR_EID           (IMU_CTRL_BASE_EID + 4)


/**********************/
/** Type Definitions **/
/**********************/


/******************************************************************************
** Command Packets
*/

/* Defined in EDS */

/******************************************************************************
** Telmetery Packets
*/

/* Defined in EDS */

/******************************************************************************
** IMU_CTRL_Class
*/

typedef struct
{

   /*
   ** Framework References
   */
   
   INITBL_Class_t*  IniTbl;

   /*
   ** Contained Objects
   */

   IMU_I2C_Class_t  ImuI2c;

   /*
   ** Class State Data
   */

   bool    InitCycle;
   int     SampleStartMs;
   uint32  SensorDeltaTimeMs;           /* Time (ms) between sensor readings  */
   float   DeltaTimeSec;                /* Time in seconds between samples    */
   float   ComplimentaryFilterConstant; /* See algorithm in imu_ctrl.c        */
   float   AccelerometerScaleFactor;    /* Degree/LSB (Least Significant Bit) */
   float   GyroScaleFactor;             /* Degree/LSB (Least Significant Bit) */

   int  AccelerometerRaw[3];
   int  GyroRaw[3];
   int  MagnetometerRaw[3];

   float GyroRateX;
   float GyroRateY;
   float GyroRateZ;

   float GyroAngleX;
   float GyroAngleY;
   float GyroAngleZ;

   float AccelerometerAngleX;
   float AccelerometerAngleY;
   float FilterAngleX;
   float FilterAngleY;
   float PrevFilterAngleX;
   float PrevFilterAngleY;

   /*
   ** Telemetry Packets
   */
   
   CFE_SB_MsgId_t      RateMid;
   MQTT_GW_RateTlm_t   RateTlm;      //BERRY_IMU_RateTlm_t  RateTlm;

} IMU_CTRL_Class_t;



/************************/
/** Exported Functions **/
/************************/


/******************************************************************************
** Function: IMU_CTRL_Constructor
**
** Initialize the IMU Controller object to a known state
**
** Notes:
**   1. This must be called prior to any other function.
**
*/
void IMU_CTRL_Constructor(IMU_CTRL_Class_t *ImuCtrlPtr, INITBL_Class_t* IniTbl);


/******************************************************************************
** Function: IMU_CTRL_ChildTask
**
*/
bool IMU_CTRL_ChildTask(CHILDMGR_Class_t* ChildMgr);


/******************************************************************************
** Function: IMU_CTRL_InitImuInterfaceCmd
**
** Notes:
**   1. Currently uses the default device filename defined in the JSON, but
**      a parameter may be added.
*/
bool IMU_CTRL_InitImuInterfaceCmd(void* DataObjPtr, const CFE_MSG_Message_t *MsgPtr);


/******************************************************************************
** Function: IMU_CTRL_ResetStatus
**
** Reset counters and status flags to a known reset state.
**
** Notes:
**   1. Any counter or variable that is reported in HK telemetry that doesn't
**      change the functional behavior should be reset.
**
*/
void IMU_CTRL_ResetStatus(void);


/******************************************************************************
** Function: IMU_CTRL_SetAccelerometerScaleFactorCmd
**
** Notes:
**   1. No limits placed on commanded value.
**
*/
bool IMU_CTRL_SetAccelerometerScaleFactorCmd(void* DataObjPtr, const CFE_MSG_Message_t *MsgPtr);


/******************************************************************************
** Function: IMU_CTRL_SetFilterConstantCmd
**
** Notes:
**   1. No limits placed on commanded value.
**
*/
bool IMU_CTRL_SetFilterConstantCmd(void* DataObjPtr, const CFE_MSG_Message_t *MsgPtr);


/******************************************************************************
** Function: IMU_CTRL_SetGyroScaleFactorCmd
**
** Notes:
**   1. No limits placed on commanded value.
**
*/
bool IMU_CTRL_SetGyroScaleFactorCmd(void* DataObjPtr, const CFE_MSG_Message_t *MsgPtr);


/******************************************************************************
** Function: IMU_CTRL_SetSensorDeltaTimeCmd
**
** Notes:
**   1. No limits placed on commanded value.
**
*/
bool IMU_CTRL_SetSensorDeltaTimeCmd(void* DataObjPtr, const CFE_MSG_Message_t *MsgPtr);


#endif /* _imu_ctrl_ */
