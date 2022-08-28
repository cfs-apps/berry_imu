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
**    Implement the Berry IMU application
**
**  Notes:
**    1. See berry_imu_app.h for details.
**
**  References:
**    1. OpenSatKit Object-based Application Developer's Guide
**    2. cFS Application Developer's Guide
**
*/

/*
** Includes
*/

#include <string.h>
#include "berry_imu_app.h"
#include "berry_imu_eds_cc.h"

/***********************/
/** Macro Definitions **/
/***********************/

/* Convenience macros */
#define  INITBL_OBJ    (&(BerryImu.IniTbl))
#define  CMDMGR_OBJ    (&(BerryImu.CmdMgr))
#define  CHILDMGR_OBJ  (&(BerryImu.ChildMgr))
#define  IMU_CTRL_OBJ  (&(BerryImu.ImuCtrl))


/*******************************/
/** Local Function Prototypes **/
/*******************************/

static int32 InitApp(void);
static int32 ProcessCommands(void);
static void SendHousekeepingPkt(void);


/**********************/
/** File Global Data **/
/**********************/

/* 
** Must match DECLARE ENUM() declaration in app_cfg.h
** Defines "static INILIB_CfgEnum IniCfgEnum"
*/
DEFINE_ENUM(Config,APP_CONFIG)  


/*****************/
/** Global Data **/
/*****************/

BERRY_IMU_Class_t  BerryImu;


/******************************************************************************
** Function: BERRY_IMU_AppMain
**
*/
void BERRY_IMU_AppMain(void)
{

   uint32 RunStatus = CFE_ES_RunStatus_APP_ERROR;


   CFE_EVS_Register(NULL, 0, CFE_EVS_NO_FILTER);

   if (InitApp() == CFE_SUCCESS) /* Performs initial CFE_ES_PerfLogEntry() call */
   {  
   
      RunStatus = CFE_ES_RunStatus_APP_RUN;
      
   }
   
   /*
   ** Main process loop
   */
   while (CFE_ES_RunLoop(&RunStatus))
   {

      RunStatus = ProcessCommands(); /* Pends indefinitely & manages CFE_ES_PerfLogEntry() calls */

   } /* End CFE_ES_RunLoop */

   CFE_ES_WriteToSysLog("BERRY_IMU App terminating, err = 0x%08X\n", RunStatus);   /* Use SysLog, events may not be working */

   CFE_EVS_SendEvent(BERRY_IMU_EXIT_EID, CFE_EVS_EventType_CRITICAL, "BERRY_IMU App terminating, err = 0x%08X", RunStatus);

   CFE_ES_ExitApp(RunStatus);  /* Let cFE kill the task (and any child tasks) */

} /* End of BERRY_IMU_AppMain() */


/******************************************************************************
** Function: BERRY_IMU_NoOpCmd
**
*/

bool BERRY_IMU_NoOpCmd(void* ObjDataPtr, const CFE_MSG_Message_t *MsgPtr)
{

   CFE_EVS_SendEvent (BERRY_IMU_NOOP_EID, CFE_EVS_EventType_INFORMATION,
                      "No operation command received for BERRY_IMU App version %d.%d.%d",
                      BERRY_IMU_MAJOR_VER, BERRY_IMU_MINOR_VER, BERRY_IMU_PLATFORM_REV);

   return true;


} /* End BERRY_IMU_NoOpCmd() */


/******************************************************************************
** Function: BERRY_IMU_ResetAppCmd
**
** Notes:
**   1. No need to pass an object reference to contained objects becuase they
**      already have a reference from when they were constructed
**
*/

bool BERRY_IMU_ResetAppCmd(void* ObjDataPtr, const CFE_MSG_Message_t *MsgPtr)
{

   CMDMGR_ResetStatus(CMDMGR_OBJ);
   CHILDMGR_ResetStatus(CHILDMGR_OBJ);
   
   IMU_CTRL_ResetStatus();
	  
   return true;

} /* End _ResetAppCmd() */


/******************************************************************************
** Function: InitApp
**
*/
static int32 InitApp(void)
{

   int32 Status = OSK_C_FW_CFS_ERROR;
   
   CHILDMGR_TaskInit_t ChildTaskInit;


   /*
   ** Initialize objects 
   */

   if (INITBL_Constructor(&BerryImu.IniTbl, BERRY_IMU_INI_FILENAME, &IniCfgEnum))
   {
   
      BerryImu.PerfId    = INITBL_GetIntConfig(INITBL_OBJ, CFG_APP_PERF_ID);
      BerryImu.CmdMid    = CFE_SB_ValueToMsgId(INITBL_GetIntConfig(INITBL_OBJ, CFG_BERRY_IMU_CMD_TOPICID));
      BerryImu.SendHkMid = CFE_SB_ValueToMsgId(INITBL_GetIntConfig(INITBL_OBJ, CFG_BERRY_IMU_SEND_HK_TOPICID));

      CFE_ES_PerfLogEntry(BerryImu.PerfId);

      /* Constructor sends error events */    
      ChildTaskInit.TaskName  = INITBL_GetStrConfig(INITBL_OBJ, CFG_CHILD_NAME);
      ChildTaskInit.PerfId    = INITBL_GetIntConfig(INITBL_OBJ, CHILD_PERF_ID);
      ChildTaskInit.StackSize = INITBL_GetIntConfig(INITBL_OBJ, CFG_CHILD_STACK_SIZE);
      ChildTaskInit.Priority  = INITBL_GetIntConfig(INITBL_OBJ, CFG_CHILD_PRIORITY);
      Status = CHILDMGR_Constructor(CHILDMGR_OBJ, 
                                    ChildMgr_TaskMainCallback,
                                    IMU_CTRL_ChildTask, 
                                    &ChildTaskInit); 
  
   } /* End if INITBL Constructed */
  
   if (Status == CFE_SUCCESS)
   {

      IMU_CTRL_Constructor(IMU_CTRL_OBJ, &BerryImu.IniTbl);

      /*
      ** Initialize app level interfaces
      */
      
      CFE_SB_CreatePipe(&BerryImu.CmdPipe, INITBL_GetIntConfig(INITBL_OBJ, CFG_CMD_PIPE_DEPTH), INITBL_GetStrConfig(INITBL_OBJ, CFG_CMD_PIPE_NAME));  
      CFE_SB_Subscribe(BerryImu.CmdMid,    BerryImu.CmdPipe);
      CFE_SB_Subscribe(BerryImu.SendHkMid, BerryImu.CmdPipe);

      CMDMGR_Constructor(CMDMGR_OBJ);
      CMDMGR_RegisterFunc(CMDMGR_OBJ, CMDMGR_NOOP_CMD_FC,   NULL, BERRY_IMU_NoOpCmd,     0);
      CMDMGR_RegisterFunc(CMDMGR_OBJ, CMDMGR_RESET_CMD_FC,  NULL, BERRY_IMU_ResetAppCmd, 0);

      CMDMGR_RegisterFunc(CMDMGR_OBJ, BERRY_IMU_SET_SENSOR_DELTA_TIME_CC,          IMU_CTRL_OBJ, IMU_CTRL_SetSensorDeltaTimeCmd,  sizeof(BERRY_IMU_SetSensorDeltaTime_Payload_t));
      CMDMGR_RegisterFunc(CMDMGR_OBJ, BERRY_IMU_SET_ACCELEROMETER_SCALE_FACTOR_CC, IMU_CTRL_OBJ, IMU_CTRL_SetAccelerometerScaleFactorCmd,  sizeof(BERRY_IMU_SetAccelerometerScaleFactor_Payload_t));
      CMDMGR_RegisterFunc(CMDMGR_OBJ, BERRY_IMU_SET_FILTER_CONSTANT_CC,            IMU_CTRL_OBJ, IMU_CTRL_SetFilterConstantCmd,   sizeof(BERRY_IMU_SetFilterConstant_Payload_t));
      CMDMGR_RegisterFunc(CMDMGR_OBJ, BERRY_IMU_SET_GYRO_SCALE_FACTOR_CC,          IMU_CTRL_OBJ, IMU_CTRL_SetGyroScaleFactorCmd,  sizeof(BERRY_IMU_SetGyroScaleFactor_Payload_t));
      CMDMGR_RegisterFunc(CMDMGR_OBJ, BERRY_IMU_INITIALIZE_IMU_INTERFACE_CC,       IMU_CTRL_OBJ, IMU_CTRL_InitImuInterfaceCmd,    0);
      
      CFE_MSG_Init(CFE_MSG_PTR(BerryImu.HkTlm.TelemetryHeader), CFE_SB_ValueToMsgId(INITBL_GetIntConfig(INITBL_OBJ, CFG_BERRY_IMU_HK_TLM_TOPICID)), sizeof(BERRY_IMU_HkTlm_t));
   
      /*
      ** Application startup event message
      */
      CFE_EVS_SendEvent(BERRY_IMU_INIT_APP_EID, CFE_EVS_EventType_INFORMATION,
                        "BERRY_IMU App Initialized. Version %d.%d.%d",
                        BERRY_IMU_MAJOR_VER, BERRY_IMU_MINOR_VER, BERRY_IMU_PLATFORM_REV);
                        
   } /* End if CHILDMGR constructed */
   
   return(Status);

} /* End of InitApp() */


/******************************************************************************
** Function: ProcessCommands
**
*/
static int32 ProcessCommands(void)
{

   int32  RetStatus = CFE_ES_RunStatus_APP_RUN;
   int32  SysStatus;

   CFE_SB_Buffer_t  *SbBufPtr;
   CFE_SB_MsgId_t   MsgId = CFE_SB_INVALID_MSG_ID;
   

   CFE_ES_PerfLogExit(BerryImu.PerfId);
   SysStatus = CFE_SB_ReceiveBuffer(&SbBufPtr, BerryImu.CmdPipe, CFE_SB_PEND_FOREVER);
   CFE_ES_PerfLogEntry(BerryImu.PerfId);

   if (SysStatus == CFE_SUCCESS)
   {
      
      SysStatus = CFE_MSG_GetMsgId(&SbBufPtr->Msg, &MsgId);
   
      if (SysStatus == CFE_SUCCESS)
      {
  
         if (CFE_SB_MsgId_Equal(MsgId, BerryImu.CmdMid)) 
         {
            
            CMDMGR_DispatchFunc(CMDMGR_OBJ, &SbBufPtr->Msg);
         
         } 
         else if (CFE_SB_MsgId_Equal(MsgId, BerryImu.SendHkMid))
         {

            SendHousekeepingPkt();
            
         }
         else
         {
            
            CFE_EVS_SendEvent(BERRY_IMU_INVALID_MID_EID, CFE_EVS_EventType_ERROR,
                              "Received invalid command packet, MID = 0x%08X",
                              CFE_SB_MsgIdToValue(MsgId));
         } 

      }
      else
      {
         
         CFE_EVS_SendEvent(BERRY_IMU_INVALID_MID_EID, CFE_EVS_EventType_ERROR,
                           "CFE couldn't retrieve message ID from the message, Status = %d", SysStatus);
      }
      
   } /* Valid SB receive */ 
   else 
   {
   
         CFE_ES_WriteToSysLog("BERRY_IMU software bus error. Status = 0x%08X\n", SysStatus);   /* Use SysLog, events may not be working */
         RetStatus = CFE_ES_RunStatus_APP_ERROR;
   }  
      
   return RetStatus;

} /* End ProcessCommands() */


/******************************************************************************
** Function: SendHousekeepingPkt
**
*/
static void SendHousekeepingPkt(void)
{
   
   BERRY_IMU_HkTlm_Payload_t *HkTlmPayload = &BerryImu.HkTlm.Payload;
   
   HkTlmPayload->ValidCmdCnt   = BerryImu.CmdMgr.ValidCmdCnt;
   HkTlmPayload->InvalidCmdCnt = BerryImu.CmdMgr.InvalidCmdCnt;

   /*
   ** Controller 
   */ 
   
   HkTlmPayload->ImuVersion     = BerryImu.ImuCtrl.ImuI2c.Version;
   HkTlmPayload->ImuEnabled     = BerryImu.ImuCtrl.ImuI2c.Enabled;
   HkTlmPayload->DeltaTime      = BerryImu.ImuCtrl.SensorDeltaTimeMs;
   HkTlmPayload->FilterConstant = BerryImu.ImuCtrl.ComplimentaryFilterConstant;
   HkTlmPayload->AccScaleFactor = BerryImu.ImuCtrl.AccelerometerScaleFactor;
   HkTlmPayload->GyrScaleFactor = BerryImu.ImuCtrl.GyroScaleFactor;
   
   CFE_SB_TimeStampMsg(CFE_MSG_PTR(BerryImu.HkTlm.TelemetryHeader));
   CFE_SB_TransmitMsg(CFE_MSG_PTR(BerryImu.HkTlm.TelemetryHeader), true);
   
} /* End SendHousekeepingPkt() */



