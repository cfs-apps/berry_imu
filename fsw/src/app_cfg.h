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
**    Define application configurations for the Berry IMU application
**
**  Notes:
**    1. These macros can only be built with the application and can't
**       have a platform scope because the same app_cfg.h filename is used for
**       all applications following the object-based application design.
**
**  References:
**    1. OpenSatKit Object-based Application Developer's Guide
**    2. cFS Application Developer's Guide
**
*/

#ifndef _app_cfg_
#define _app_cfg_

/*
** Includes
*/

#include "berry_imu_eds_typedefs.h"
//#include "berry_imu_eds_designparameters.h"

#include "berry_imu_platform_cfg.h"
#include "osk_c_fw.h"


/******************************************************************************
** Application Macros
*/

/*
** Versions:
** 1.0 - Initial release
*/

#define  BERRY_IMU_MAJOR_VER   0
#define  BERRY_IMU_MINOR_VER   9


/******************************************************************************
** Init File declarations create:
**
**  typedef enum {
**     CMD_PIPE_DEPTH,
**     CMD_PIPE_NAME
**  } INITBL_ConfigEnum;
**    
**  typedef struct {
**     CMD_PIPE_DEPTH,
**     CMD_PIPE_NAME
**  } INITBL_ConfigStruct;
**
**   const char *GetConfigStr(value);
**   ConfigEnum GetConfigVal(const char *str);
**
** XX(name,type)
*/

#define CFG_APP_CFE_NAME               APP_CFE_NAME
#define CFG_APP_PERF_ID                APP_PERF_ID

#define CFG_CMD_PIPE_NAME              APP_CMD_PIPE_NAME
#define CFG_CMD_PIPE_DEPTH             APP_CMD_PIPE_DEPTH

#define CFG_BERRY_IMU_CMD_TOPICID      BERRY_IMU_CMD_TOPICID
#define CFG_BERRY_IMU_SEND_HK_TOPICID  BERRY_IMU_SEND_HK_TOPICID
#define CFG_BERRY_IMU_HK_TLM_TOPICID   BERRY_IMU_HK_TLM_TOPICID
#define CFG_BERRY_IMU_RATE_TLM_TOPICID BERRY_IMU_RATE_TLM_TOPICID

#define CFG_CHILD_NAME                 CHILD_NAME
#define CFG_CHILD_PERF_ID              CHILD_PERF_ID
#define CFG_CHILD_STACK_SIZE           CHILD_STACK_SIZE
#define CFG_CHILD_PRIORITY             CHILD_PRIORITY

#define CFG_IMU_DEVICE_FILE            IMU_DEVICE_FILE
#define CFG_IMU_SENSOR_DELTA_TIME      IMU_SENSOR_DELTA_TIME 

#define CFG_IMU_ACCEL_SCALE_FACTOR     IMU_ACCEL_SCALE_FACTOR 
#define CFG_IMU_GYRO_SCALE_FACTOR      IMU_GYRO_SCALE_FACTOR 
#define CFG_IMU_COMP_FILTER_CONSTANT   IMU_COMP_FILTER_CONSTANT 

#define APP_CONFIG(XX) \
   XX(APP_CFE_NAME,char*) \
   XX(APP_PERF_ID,uint32) \
   XX(APP_CMD_PIPE_NAME,char*) \
   XX(APP_CMD_PIPE_DEPTH,uint32) \
   XX(BERRY_IMU_CMD_TOPICID,uint32) \
   XX(BERRY_IMU_SEND_HK_TOPICID,uint32) \
   XX(BERRY_IMU_HK_TLM_TOPICID,uint32) \
   XX(BERRY_IMU_RATE_TLM_TOPICID,uint32) \
   XX(CHILD_NAME,char*) \
   XX(CHILD_PERF_ID,uint32) \
   XX(CHILD_STACK_SIZE,uint32) \
   XX(CHILD_PRIORITY,uint32) \
   XX(IMU_DEVICE_FILE,char*) \
   XX(IMU_SENSOR_DELTA_TIME,uint32) \
   XX(IMU_ACCEL_SCALE_FACTOR,float) \
   XX(IMU_GYRO_SCALE_FACTOR,float) \
   XX(IMU_COMP_FILTER_CONSTANT,float) \
      
DECLARE_ENUM(Config,APP_CONFIG)


/******************************************************************************
** Command Macros
** - Load/dump table definitions are placeholders for a JSON table
*/

#define BERRY_IMU_TBL_LOAD_CMD_FC      (CMDMGR_APP_START_FC + 0)
#define BERRY_IMU_TBL_DUMP_CMD_FC      (CMDMGR_APP_START_FC + 1)


/******************************************************************************
** Event Macros
**
** Define the base event message IDs used by each object/component used by the
** application. There are no automated checks to ensure an ID range is not
** exceeded so it is the developer's responsibility to verify the ranges. 
*/

#define BERRY_IMU_BASE_EID  (OSK_C_FW_APP_BASE_EID +  0)
#define IMU_CTRL_BASE_EID   (OSK_C_FW_APP_BASE_EID + 20)
#define IMU_I2C_BASE_EID    (OSK_C_FW_APP_BASE_EID + 40)

#endif /* _app_cfg_ */
