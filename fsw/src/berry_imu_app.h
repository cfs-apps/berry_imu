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
**    Define the Berry IMU application
**
**  Notes:
**    1. This app uses code is based on the ozzmaker Berry IMU code. See ref #1.
**    2. I2C must be enabled on the Pi
**
**  References:
**    1. https://github.com/ozzmaker/BerryIMU
**    2. OpenSatKit Object-based Application Developer's Guide
**    3. cFS Application Developer's Guide
**
*/

#ifndef _berry_imu_app_
#define _berry_imu_app_

/*
** Includes
*/

#include "app_cfg.h"
#include "childmgr.h"
#include "initbl.h"
#include "imu_ctrl.h"

/***********************/
/** Macro Definitions **/
/***********************/

/*
** Events
*/

#define BERRY_IMU_INIT_APP_EID    (BERRY_IMU_BASE_EID + 0)
#define BERRY_IMU_NOOP_EID        (BERRY_IMU_BASE_EID + 1)
#define BERRY_IMU_EXIT_EID        (BERRY_IMU_BASE_EID + 2)
#define BERRY_IMU_INVALID_MID_EID (BERRY_IMU_BASE_EID + 3)


/**********************/
/** Type Definitions **/
/**********************/


/******************************************************************************
** Command Packets
*/

/* Defined in EDS */

/******************************************************************************
** Telemetry Packets
*/

/* Defined in EDS */

/******************************************************************************
** BERRY_IMU_Class
*/
typedef struct 
{

   /* 
   ** App Framework
   */ 
   
   INITBL_Class_t     IniTbl; 
   CFE_SB_PipeId_t    CmdPipe;
   CMDMGR_Class_t     CmdMgr;
   CHILDMGR_Class_t   ChildMgr;   
   
   /*
   ** Telemetry Packets
   */
   
   BERRY_IMU_HkTlm_t  HkTlm;

   /*
   ** App State & Objects
   */ 
       
   uint32             PerfId;
   CFE_SB_MsgId_t     CmdMid;
   CFE_SB_MsgId_t     SendHkMid;
   
   IMU_CTRL_Class_t   ImuCtrl;
 
} BERRY_IMU_Class_t;


/*******************/
/** Exported Data **/
/*******************/

extern BERRY_IMU_Class_t  BerryImu;


/************************/
/** Exported Functions **/
/************************/


/******************************************************************************
** Function: BERRY_IMU_AppMain
**
*/
void BERRY_IMU_AppMain(void);


/******************************************************************************
** Function: BERRY_IMU_NoOpCmd
**
*/
bool BERRY_IMU_NoOpCmd(void* ObjDataPtr, const CFE_MSG_Message_t *MsgPtr);


/******************************************************************************
** Function: BERRY_IMU_ResetAppCmd
**
*/
bool BERRY_IMU_ResetAppCmd(void* ObjDataPtr, const CFE_MSG_Message_t *MsgPtr);



#endif /* _berry_imu_app_ */
