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
**    Define the imu_i2c class. 
**
**  Notes:
**    1. The code is based on the ozzmaker Berry IMU code. See ref #1. The original
**       code didn't have a header file and was not organized into an object
**    2. TODO - Consider switching to OSK's pi_iolib for I2C interface
**
**  References:
**    1. https://github.com/ozzmaker/BerryIMU
**    2. OpenSatKit Object-based Application Developer's Guide
**    3. cFS Application Developer's Guide
**
*/

#ifndef _imu_i2c_
#define _imu_i2c_

/*
** Includes
*/

#include "app_cfg.h"


/***********************/
/** Macro Definitions **/
/***********************/

/*
** Event Message IDs
*/

#define IMU_I2C_FILE_OPEN_ERR_EID      (IMU_I2C_BASE_EID + 0)
#define IMU_I2C_IMU_DETECTED_EID       (IMU_I2C_BASE_EID + 1)
#define IMU_I2C_IMU_DETECTION_ERR_EID  (IMU_I2C_BASE_EID + 2)
#define IMU_I2C_IMU_ENABLED_EID        (IMU_I2C_BASE_EID + 3)
#define IMU_I2C_IMU_ENABLED_ERR_EID    (IMU_I2C_BASE_EID + 4)

/**********************/
/** Type Definitions **/
/**********************/

/*
** Enumerations are used as indices so their values start at 
** zero and are sequential. Zero is not always a good choice
** but okay for this small scope usage.
*/

typedef enum 
{

   IMU_I2C_VERSION_1 = 0,  /* BerryIMUv1/LSM9DS0 */
   IMU_I2C_VERSION_2 = 1,  /* BerryIMUv2/LSM9DS1 */
   IMU_I2C_VERSION_3 = 2,  /* BerryIMUv3/LSM6DSL/LIS3MDL */
   
   IMU_I2C_VERSION_COUNT = 3,
   IMU_I2C_VERSION_UNDEF = 4
    
} IMU_I2C_Version_t;

typedef struct
{

   int      Address;
   uint8_t  ReadCmd;

} IMU_I2C_Interface_t;


/******************************************************************************
** IMU_I2C_Class
*/

typedef struct
{

   int   File;
   bool  Enabled;
   IMU_I2C_Version_t  Version;
      
   IMU_I2C_Interface_t  Accelerometer;
   IMU_I2C_Interface_t  Gyroscope;
   IMU_I2C_Interface_t  Magnetometer;
   
} IMU_I2C_Class_t;


/************************/
/** Exported Functions **/
/************************/


/******************************************************************************
** Function: IMU_I2C_Constructor
**
** Initialize the GPIO Controller object to a known state
**
** Notes:
**   1. This must be called prior to any other function.
**
*/
void IMU_I2C_Constructor(IMU_I2C_Class_t *ImuI2cPtrr, const char *DevFilename);


/******************************************************************************
** Function: IMU_I2C_InitializeInterface
**
** Detect and enable the I2C interface for each sensor
**
** Notes:
**   1. Always outputs an event message
**
*/
bool IMU_I2C_InitializeInterface(const char *DevFilename);


/******************************************************************************
** Function: IMU_I2C_ReadAccelerometer
**
** Read Accelerometer data
**
** Notes:
**   None
**
*/
bool IMU_I2C_ReadAccelerometer(int AccData[]);


/******************************************************************************
** Function: IMU_I2C_ReadGyroscope
**
** Read Gyroscope data
**
** Notes:
**   None
**
*/
bool IMU_I2C_ReadGyroscope(int GyroData[]);


/******************************************************************************
** Function: IMU_I2C_ReadMagnetometer
**
** Read Magnetometer data
**
** Notes:
**   None
**
*/
bool IMU_I2C_ReadMagnetometer(int MagData[]);


/******************************************************************************
** Function: IMU_I2C_ResetStatus
**
** Reset counters and status flags to a known reset state.
**
** Notes:
**   1. Any counter or variable that is reported in HK telemetry that doesn't
**      change the functional behavior should be reset.
**
*/
void IMU_I2C_ResetStatus(void);


#endif /* _imu_i2c_ */

