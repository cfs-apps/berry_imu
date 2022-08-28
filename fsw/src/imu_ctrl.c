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
**    Implement the Berry IMU control and data processing
**
**  Notes:
**    1. 
**
**  References:
**    1. OpenSatKit Object-based Application Developer's Guide
**    2. cFS Application Developer's Guide
**
*/

/*
** Include Files:
*/

#include <string.h>
#include <math.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include "app_cfg.h"
#include "imu_i2c.h"
#include "imu_ctrl.h"

/***********************/
/** Macro Definitions **/
/***********************/

#define IMU_MOUNT_UP 1  // Up is 'normal' mounting. Set to 0 for upside down mount

#define RAD_TO_DEG 57.29578
#define DEG_TO_RAD  0.0174533
#define PI          3.14159265358979323846


/**********************/
/** Global File Data **/
/**********************/

static IMU_CTRL_Class_t*  ImuCtrl = NULL;


/*******************************/
/** Local Function Prototypes **/
/*******************************/

static int CurrentMilliSec(void);

/******************************************************************************
** Function: IMU_CTRL_Constructor
**
** Initialize the IMU Controller object to a known state
**
** Notes:
**   1. This must be called prior to any other function.
**
*/
void IMU_CTRL_Constructor(IMU_CTRL_Class_t *ImuCtrlPtr, INITBL_Class_t* IniTbl)
{
   
   ImuCtrl = ImuCtrlPtr;
   ImuCtrl->IniTbl = IniTbl;
   
   memset(ImuCtrl, 0, sizeof(IMU_CTRL_Class_t));

   IMU_I2C_Constructor(&ImuCtrl->ImuI2c, INITBL_GetStrConfig(IniTbl, CFG_IMU_DEVICE_FILE));
   
   ImuCtrl->InitCycle = true;
   ImuCtrl->SensorDeltaTimeMs = INITBL_GetIntConfig(IniTbl, CFG_IMU_SENSOR_DELTA_TIME);
   ImuCtrl->DeltaTimeSec = (float)ImuCtrl->SensorDeltaTimeMs / 1000.0; 
   ImuCtrl->RateTlm.Payload.Z = 0.0;
   ImuCtrl->FilterAngleX = 0.0;
   ImuCtrl->FilterAngleY = 0.0; 

   ImuCtrl->AccelerometerScaleFactor    = INITBL_GetFltConfig(IniTbl, CFG_IMU_ACCEL_SCALE_FACTOR);
   ImuCtrl->GyroScaleFactor             = INITBL_GetFltConfig(IniTbl, CFG_IMU_GYRO_SCALE_FACTOR);
   ImuCtrl->ComplimentaryFilterConstant = INITBL_GetFltConfig(IniTbl, CFG_IMU_COMP_FILTER_CONSTANT);

   CFE_MSG_Init(CFE_MSG_PTR(ImuCtrl->RateTlm.TelemetryHeader), 
                            CFE_SB_ValueToMsgId(INITBL_GetIntConfig(IniTbl, CFG_BERRY_IMU_RATE_TLM_TOPICID)),
                            sizeof(BERRY_IMU_RateTlm_t));

   CFE_EVS_SendEvent (IMU_CTRL_CONSTRUCTOR_EID, CFE_EVS_EventType_INFORMATION, 
                      "IMU sensor sampling delta time set to %0.6f", ImuCtrl->DeltaTimeSec);

} /* End IMU_CTRL_Constructor() */


/******************************************************************************
** Function: IMU_CTRL_ChildTask
**
** Notes:
**   1. This is a callback from an infinite loop so it needs to have some mechanism
**      to release control so it doesn't hog the CPU
*/
bool IMU_CTRL_ChildTask(CHILDMGR_Class_t* ChildMgr)
{

   ImuCtrl->SampleStartMs = CurrentMilliSec();
      
   IMU_I2C_ReadAccelerometer(ImuCtrl->AccelerometerRaw);
   IMU_I2C_ReadGyroscope(ImuCtrl->GyroRaw);
OS_printf("Gyro raw Y = %d\n", ImuCtrl->GyroRaw[1]);
   /* Convert Gyro raw to degrees per second */
   ImuCtrl->GyroRateX = (float) ImuCtrl->GyroRaw[0] * ImuCtrl->GyroScaleFactor;
   ImuCtrl->GyroRateY = (float) ImuCtrl->GyroRaw[1] * ImuCtrl->GyroScaleFactor;
   ImuCtrl->GyroRateZ = (float) ImuCtrl->GyroRaw[2] * ImuCtrl->GyroScaleFactor;
OS_printf("Gyro rate Y = %0.3f\n", ImuCtrl->GyroRateY);
   /* Calculate the angles from the gyro */
   ImuCtrl->GyroAngleX += ImuCtrl->GyroRateX * ImuCtrl->DeltaTimeSec;
   ImuCtrl->GyroAngleY += ImuCtrl->GyroRateY * ImuCtrl->DeltaTimeSec;
   ImuCtrl->GyroAngleZ += ImuCtrl->GyroRateZ * ImuCtrl->DeltaTimeSec;
OS_printf("Gyro Y angle = %0.3f\n", ImuCtrl->GyroAngleY);
   /* TODO - Very algorithm. The raw counts have not been scaled prior to use below */
   /* Convert Accelerometer values to degrees */
   ImuCtrl->AccelerometerAngleX = (float) (atan2(ImuCtrl->AccelerometerRaw[1], ImuCtrl->AccelerometerRaw[2]) + PI) * RAD_TO_DEG;
   ImuCtrl->AccelerometerAngleY = (float) (atan2(ImuCtrl->AccelerometerRaw[2], ImuCtrl->AccelerometerRaw[0]) + PI) * RAD_TO_DEG;
OS_printf("AccYangle = %0.3f\n", ImuCtrl->AccelerometerAngleY);
   /* Account for potential different mountings */
   #if IMU_MOUNT_UP == 1
   
      /* IMU is mounted up the correct way */
      ImuCtrl->AccelerometerAngleX -= (float)180.0;
      if (ImuCtrl->AccelerometerAngleY > 90.0)
         ImuCtrl->AccelerometerAngleY -= (float)270.0;
      else
         ImuCtrl->AccelerometerAngleY += (float)90.0;
   
   #else
	
      /* IMU is mounted upside down */
      /* Change the rotation value of the accelerometer to -/+ 180 and move the Y axis '0' point to up  */
      
      if (ImuCtrl->AccelerometerAngleX > 180)
         ImuCtrl->AccelerometerAngleX -= (float)360.0;

      ImuCtrl->AccelerometerAngleY -= 90.0;
      if (ImuCtrl->AccelerometerAngleY > 180.0)
         ImuCtrl->AccelerometerAngleY -= (float)360.0;

   #endif

   /* Complementary filter used to combine the accelerometer and gyro values */
   ImuCtrl->FilterAngleX = ImuCtrl->ComplimentaryFilterConstant * (ImuCtrl->FilterAngleX + ImuCtrl->GyroRateX * ImuCtrl->DeltaTimeSec) + 
                           (1.0 - ImuCtrl->ComplimentaryFilterConstant) * ImuCtrl->AccelerometerAngleX;
                           
   ImuCtrl->FilterAngleY = ImuCtrl->ComplimentaryFilterConstant * (ImuCtrl->FilterAngleY + ImuCtrl->GyroRateY * ImuCtrl->DeltaTimeSec) + 
                           (1.0 - ImuCtrl->ComplimentaryFilterConstant) * ImuCtrl->AccelerometerAngleY;
OS_printf("CFangleY = %0.3f\n", ImuCtrl->FilterAngleY);

   /* Load and send rate telemetry */
   
   if (ImuCtrl->InitCycle)
   {
      ImuCtrl->InitCycle = false;
      ImuCtrl->RateTlm.Payload.X = 0.0;
      ImuCtrl->RateTlm.Payload.Y = 0.0;
   }
   else
   {
      ImuCtrl->RateTlm.Payload.X = (ImuCtrl->FilterAngleX-ImuCtrl->PrevFilterAngleX)*DEG_TO_RAD/ImuCtrl->DeltaTimeSec;
      ImuCtrl->RateTlm.Payload.Y = (ImuCtrl->FilterAngleY-ImuCtrl->PrevFilterAngleY)*DEG_TO_RAD/ImuCtrl->DeltaTimeSec;;
   }
   ImuCtrl->PrevFilterAngleX = ImuCtrl->FilterAngleX;
   ImuCtrl->PrevFilterAngleY = ImuCtrl->FilterAngleY;
   
   
   CFE_SB_TimeStampMsg(CFE_MSG_PTR(ImuCtrl->RateTlm.TelemetryHeader));
   CFE_SB_TransmitMsg(CFE_MSG_PTR(ImuCtrl->RateTlm.TelemetryHeader), true);

   OS_printf("***BERRY_IMU*** Rate  = %0.3f(%0.3f), %0.3f(%0.3f)\n", 
             ImuCtrl->RateTlm.Payload.X*RAD_TO_DEG, ImuCtrl->RateTlm.Payload.X,
             ImuCtrl->RateTlm.Payload.Y*RAD_TO_DEG, ImuCtrl->RateTlm.Payload.Y);
//OS_printf("***BERRY_IMU*** Rate  = %0.3f, %0.3f, %0.3f\n", ImuCtrl->GyroRateX, ImuCtrl->GyroRateY, ImuCtrl->GyroRateZ);
//OS_printf("***BERRY_IMU*** Angle = %0.3f, %0.3f, %0.3f\n", ImuCtrl->GyroAngleX, ImuCtrl->GyroAngleY, ImuCtrl->GyroAngleZ);
   while((CurrentMilliSec() - ImuCtrl->SampleStartMs) < ImuCtrl->SensorDeltaTimeMs)
   {
      usleep(100);
   }
   OS_printf("Loop Time %d\n\n", (CurrentMilliSec() - ImuCtrl->SampleStartMs));
		
   return true;

} /* End IMU_CTRL_ChildTask() */


/******************************************************************************
** Function: IMU_CTRL_InitImuInterfaceCmd
**
** Notes:
**   1. Currently uses the default device filename defined in the JSON, but
**      a parameter may be added.
*/
bool IMU_CTRL_InitImuInterfaceCmd(void* DataObjPtr, const CFE_MSG_Message_t *MsgPtr)
{

   return IMU_I2C_InitializeInterface(INITBL_GetStrConfig(ImuCtrl->IniTbl, CFG_IMU_DEVICE_FILE));

} /* End IMU_CTRL_InitImuInterfaceCmd() */


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
void IMU_CTRL_ResetStatus(void)
{

   IMU_I2C_ResetStatus();
   
   return;

} /* End IMU_CTRL_ResetStatus() */


/******************************************************************************
** Function: IMU_CTRL_SetAccelerometerScaleFactorCmd
**
** Notes:
**   1. No limits placed on commanded value.
**
*/
bool IMU_CTRL_SetAccelerometerScaleFactorCmd(void* DataObjPtr, const CFE_MSG_Message_t *MsgPtr)
{
   
   const BERRY_IMU_SetAccelerometerScaleFactor_Payload_t *Cmd = CMDMGR_PAYLOAD_PTR(MsgPtr, BERRY_IMU_SetAccelerometerScaleFactor_t);
  
   CFE_EVS_SendEvent (IMU_CTRL_SET_ACCELEROMETER_SCALE_FACTOR_EID, CFE_EVS_EventType_INFORMATION, 
                      "IMU accelerometer scale factor changed from %0.6f to %0.6f milliseconds", 
                      ImuCtrl->AccelerometerScaleFactor, Cmd->ScaleFactor);
                      
   ImuCtrl->AccelerometerScaleFactor = Cmd->ScaleFactor;
  
   return true;
   
} /* End IMU_CTRL_SetAccelerometerScaleFactorCmd() */


/******************************************************************************
** Function: IMU_CTRL_SetFilterConstantCmd
**
** Notes:
**   1. No limits placed on commanded value.
**
*/
bool IMU_CTRL_SetFilterConstantCmd(void* DataObjPtr, const CFE_MSG_Message_t *MsgPtr)
{
   
   const BERRY_IMU_SetFilterConstant_Payload_t *Cmd = CMDMGR_PAYLOAD_PTR(MsgPtr, BERRY_IMU_SetFilterConstant_t);
  
   CFE_EVS_SendEvent (IMU_CTRL_SET_FILTER_CONSTANT_EID, CFE_EVS_EventType_INFORMATION,
                     "IMU complimentary filter constant changed from  %0.6f to %0.6f", 
                     ImuCtrl->ComplimentaryFilterConstant, Cmd->ComplimentaryConstant);

   ImuCtrl->ComplimentaryFilterConstant = Cmd->ComplimentaryConstant;  
  
   return true;
   
} /* End IMU_CTRL_SetFilterConstantCmd() */


/******************************************************************************
** Function: IMU_CTRL_SetGyroScaleFactorCmd
**
** Notes:
**   1. No limits placed on commanded value.
**
*/
bool IMU_CTRL_SetGyroScaleFactorCmd(void* DataObjPtr, const CFE_MSG_Message_t *MsgPtr)
{
   
   const BERRY_IMU_SetGyroScaleFactor_Payload_t *Cmd = CMDMGR_PAYLOAD_PTR(MsgPtr, BERRY_IMU_SetGyroScaleFactor_t);
  
   CFE_EVS_SendEvent (IMU_CTRL_SET_GYRO_SCALE_FACTOR_EID, CFE_EVS_EventType_INFORMATION, 
                      "IMU gyro scale factor changed from %0.6f to %0.6f milliseconds", 
                      ImuCtrl->GyroScaleFactor, Cmd->ScaleFactor);
                      
   ImuCtrl->GyroScaleFactor = Cmd->ScaleFactor;
  
   return true;   
   
} /* End IMU_CTRL_SetGyroSCaleFactorCmd() */


/******************************************************************************
** Function: IMU_CTRL_SetSensorDeltaTimeCmd
**
** Notes:
**   1. No limits placed on commanded value.
**
*/
bool IMU_CTRL_SetSensorDeltaTimeCmd(void* DataObjPtr, const CFE_MSG_Message_t *MsgPtr)
{
   
   const BERRY_IMU_SetSensorDeltaTime_Payload_t *Cmd = CMDMGR_PAYLOAD_PTR(MsgPtr, BERRY_IMU_SetSensorDeltaTime_t);
  
   CFE_EVS_SendEvent (IMU_CTRL_SET_SENSOR_DELTA_TIME_EID, CFE_EVS_EventType_INFORMATION, 
                      "IMU sensor sampling delta time changed from %u to %u milliseconds", ImuCtrl->SensorDeltaTimeMs, Cmd->SensorDeltaTime);

   ImuCtrl->SensorDeltaTimeMs = Cmd->SensorDeltaTime;
   ImuCtrl->DeltaTimeSec = (float)ImuCtrl->SensorDeltaTimeMs / 1000.0; 
  
   return true;
   
} /* End IMU_CTRL_SetSensorDeltaTimeCmd() */


/******************************************************************************
** Function: CurrentMilliSec
**
** Notes:
**   None
**
*/
static int CurrentMilliSec(void)
{
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return (tv.tv_sec) * 1000 + (tv.tv_usec)/1000;
}


