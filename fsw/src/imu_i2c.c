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
**    Interface to ozzmaker Berry IMU using I2C
**
**  Notes:
**    1. This code is based on the ozzmaker Berry IMU code. See ref #1.
**    2. The Sensor llokup table is used to simplify and optimize the code
**       that is executed each execution cycle.  
**
**  References:
**    1. https://github.com/ozzmaker/BerryIMU
**    2. OpenSatKit Object-based Application Developer's Guide
**    3. cFS Application Developer's Guide
**
*/
#include <stdint.h>
#include <fcntl.h>
#include "i2c-dev.h"
#include "LSM9DS0.h"
#include "LSM9DS1.h"
#include "LSM6DSL.h"
#include "LIS3MDL.h"
#include "imu_i2c.h"


/***********************/
/** Macro Definitions **/
/***********************/

#define ACC_DATA_BLOCK_LEN 6
#define GYR_DATA_BLOCK_LEN 6
#define MAG_DATA_BLOCK_LEN 6


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

   SENSOR_ACC = 0,  /* Accelerometer */
   SENSOR_GYR = 1,  /* Gyroscope     */
   SENSOR_MAG = 2,  /* Magnetometer  */
   
   SENSOR_COUNT = 3,
   SENSOR_UNDEF = 4

} SENSOR_Enum_t;


typedef struct
{

   IMU_I2C_Interface_t  Interface[IMU_I2C_VERSION_COUNT];

} SENSOR_Version_t;


/**********************/
/** Global File Data **/
/**********************/

static IMU_I2C_Class_t*  ImuI2c = NULL;

static SENSOR_Version_t  Sensor[SENSOR_COUNT] =
{
   /* Accelerometer */
   {{
      { LSM9DS0_ACC_ADDRESS, (0x80 |  LSM9DS0_OUT_X_L_A) },  /* Version 1 */
      { LSM9DS1_ACC_ADDRESS, LSM9DS1_OUT_X_L_XL          },  /* Version 2 */
      { LSM6DSL_ADDRESS,     LSM6DSL_OUTX_L_XL           }   /* Version 3 */   
   }},
   
   /* Gyroscope */
   {{
      { LSM9DS0_GYR_ADDRESS, (0x80 |  LSM9DS0_OUT_X_L_G) },  /* Version 1 */
      { LSM9DS1_GYR_ADDRESS, LSM9DS1_OUT_X_L_G           },  /* Version 2 */
      { LSM6DSL_ADDRESS,     LSM6DSL_OUTX_L_G            }   /* Version 3 */   
   }},

   /* Magnetometer */
   {{
      { LSM9DS0_MAG_ADDRESS, (0x80 |  LSM9DS0_OUT_X_L_M) },  /* Version 1 */
      { LSM9DS1_MAG_ADDRESS, LSM9DS1_OUT_X_L_M           },  /* Version 2 */
      { LIS3MDL_ADDRESS,     LIS3MDL_OUT_X_L             }   /* Version 3 */   
   }}
};


/*******************************/
/** Local Function Prototypes **/
/*******************************/

static bool DetectImu(const char *DevFilename);
static bool EnableImu(void);
static bool ReadDataBlock(uint8_t Command, uint8_t Size, uint8_t *Data);
static bool SelectDevice(int Address);
static bool WriteAccReg(uint8_t Register, uint8_t Value);
static bool WriteGyrReg(uint8_t Register, uint8_t Value);
static bool WriteMagReg(uint8_t Register, uint8_t Value);


/******************************************************************************
** Function: IMU_I2C_Constructor
**
** Initialize the IMU I2C object to a known state
**
** Notes:
**   1. This must be called prior to any other function.
**
*/
void IMU_I2C_Constructor(IMU_I2C_Class_t *ImuI2cPtr, const char *DevFilename)
{

   ImuI2c = ImuI2cPtr;
   
   memset(ImuI2c, 0, sizeof(IMU_I2C_Class_t));
  
   IMU_I2C_InitializeInterface(DevFilename);
  
} /* End IMU_I2C_Constructor() */


/******************************************************************************
** Function: IMU_I2C_InitializeInterface
**
** Detect and enable the I2C interface for each sensor
**
** Notes:
**   1. Always outputs an event message
**
*/
bool IMU_I2C_InitializeInterface(const char *DevFilename)
{
   
   bool RetStatus = false;

   /* DetectImu() loads Address/ReadCmd values and sends a status event message */
   if (DetectImu(DevFilename))
   {
      RetStatus = EnableImu();
   }
   
   return RetStatus;

} /* End IMU_I2C_InitializeInterface() */


/******************************************************************************
** Function: IMU_I2C_ReadAccelerometer
**
** Read Accelerometer data
**
** Notes:
**   None
**
*/
bool IMU_I2C_ReadAccelerometer(int AccData[])
{

   bool RetStatus = false;
   uint8_t DataBlock[ACC_DATA_BLOCK_LEN];
	
   if (SelectDevice(ImuI2c->Accelerometer.Address))
   {
      if (ReadDataBlock(ImuI2c->Accelerometer.ReadCmd, sizeof(DataBlock), DataBlock))
      {
	     RetStatus = true;
         // Combine readings for each axis.
         AccData[0] = (int16_t)(DataBlock[0] | DataBlock[1] << 8);
         AccData[1] = (int16_t)(DataBlock[2] | DataBlock[3] << 8);
         AccData[2] = (int16_t)(DataBlock[4] | DataBlock[5] << 8);
      }
   }
 
   return RetStatus;

} /* IMU_I2C_ReadAccelerometer() */


/******************************************************************************
** Function: IMU_I2C_ReadGyroscope
**
** Read Gyroscope data
**
** Notes:
**   None
**
*/
bool IMU_I2C_ReadGyroscope(int GyroData[])
{

   bool RetStatus = false;
   uint8_t DataBlock[GYR_DATA_BLOCK_LEN];

   if (SelectDevice(ImuI2c->Gyroscope.Address))
   {
      if (ReadDataBlock(ImuI2c->Gyroscope.ReadCmd, sizeof(DataBlock), DataBlock))
      {
	      RetStatus = true;   
         // Combine readings for each axis.
         GyroData[0] = (int16_t)(DataBlock[0] | DataBlock[1] << 8);
         GyroData[1] = (int16_t)(DataBlock[2] | DataBlock[3] << 8);
         GyroData[2] = (int16_t)(DataBlock[4] | DataBlock[5] << 8);
      }
   }
 
   return RetStatus;
   
} /* End IMU_I2C_ReadGyroscope() */


/******************************************************************************
** Function: IMU_I2C_ReadMagnetometer
**
** Read Magnetometer data
**
** Notes:
**   None
**
*/
bool IMU_I2C_ReadMagnetometer(int MagData[])
{

   bool RetStatus = false;
   uint8_t DataBlock[MAG_DATA_BLOCK_LEN];

   if (SelectDevice(ImuI2c->Magnetometer.Address))
   {
      if (ReadDataBlock(ImuI2c->Magnetometer.ReadCmd, sizeof(DataBlock), DataBlock))
      {
	      RetStatus = true;         
         // Combine readings for each axis.
         MagData[0] = (int16_t)(DataBlock[0] | DataBlock[1] << 8);
         MagData[1] = (int16_t)(DataBlock[2] | DataBlock[3] << 8);
         MagData[2] = (int16_t)(DataBlock[4] | DataBlock[5] << 8);
      }
   }
 
   return RetStatus;

} /* End IMU_I2C_ReadMagnetometer() */


/******************************************************************************
** Function: IMU_I2C_ResetStatus
**
** Reset counters and status flags to a known reset state.
**
** Notes:IMU_CTRL_InitImuInterfaceCmd
**   1. Any counter or variable that is reported in HK telemetry that doesn't
**      change the functional behavior should be reset.
**
*/
void IMU_I2C_ResetStatus(void)
{

   return;
   
} /* End IMU_I2C_ResetStatus() */


/******************************************************************************
** Function: DetectImu
**
** Detect which version of the Berry IMU is present
**
** Notes:
**   None
**
*/
static bool DetectImu(const char *DevFilename)
{

   bool RetStatus = false;
   char ImuDetectedStr[32];

   ImuI2c->Version = IMU_I2C_VERSION_UNDEF;

   ImuI2c->File = open(DevFilename, O_RDWR);
   if (ImuI2c->File < 0)
   {
      CFE_EVS_SendEvent (IMU_I2C_FILE_OPEN_ERR_EID, CFE_EVS_EventType_ERROR,
                          "Error opening I2C bus %s", DevFilename);
   }
   else
   {

      //Detect if BerryIMUv1 (Which uses a LSM9DS0) is connected
      SelectDevice(LSM9DS0_ACC_ADDRESS);
      int LSM9DS0_WHO_XM_response = i2c_smbus_read_byte_data(ImuI2c->File, LSM9DS0_WHO_AM_I_XM);

      SelectDevice(LSM9DS0_GYR_ADDRESS);	
      int LSM9DS0_WHO_G_response = i2c_smbus_read_byte_data(ImuI2c->File, LSM9DS0_WHO_AM_I_G);

      if (LSM9DS0_WHO_G_response == 0xd4 && LSM9DS0_WHO_XM_response == 0x49)
      {
         sprintf(ImuDetectedStr, "%s", "BerryIMUv1/LSM9DS0");
         ImuI2c->Version = IMU_I2C_VERSION_1;
      }


      //Detect if BerryIMUv2 (Which uses a LSM9DS1) is connected
      SelectDevice(LSM9DS1_MAG_ADDRESS);
      int LSM9DS1_WHO_M_response = i2c_smbus_read_byte_data(ImuI2c->File, LSM9DS1_WHO_AM_I_M);

      SelectDevice(LSM9DS1_GYR_ADDRESS);	
      int LSM9DS1_WHO_XG_response = i2c_smbus_read_byte_data(ImuI2c->File, LSM9DS1_WHO_AM_I_XG);

      if (LSM9DS1_WHO_XG_response == 0x68 && LSM9DS1_WHO_M_response == 0x3d){
         sprintf(ImuDetectedStr, "%s", "BerryIMUv2/LSM9DS1");
         ImuI2c->Version = IMU_I2C_VERSION_2;
      }

      //Detect if BerryIMUv3 (Which uses a LSM6DSL and LIS3MDL) is connected
      SelectDevice(LSM6DSL_ADDRESS);
      int LSM6DSL_WHO_M_response = i2c_smbus_read_byte_data(ImuI2c->File, LSM6DSL_WHO_AM_I);

      SelectDevice(LIS3MDL_ADDRESS);	
      int LIS3MDL_WHO_XG_response = i2c_smbus_read_byte_data(ImuI2c->File, LIS3MDL_WHO_AM_I);

      if (LSM6DSL_WHO_M_response == 0x6A && LIS3MDL_WHO_XG_response == 0x3D)
      {
         sprintf(ImuDetectedStr, "%s", "BerryIMUv3/LSM6DSL/LIS3MDL");
         ImuI2c->Version = IMU_I2C_VERSION_3;
      }

      OS_TaskDelay(2000);
      
      if (ImuI2c->Version == IMU_I2C_VERSION_UNDEF)
      {
         CFE_EVS_SendEvent (IMU_I2C_IMU_DETECTION_ERR_EID, CFE_EVS_EventType_ERROR,
                            "Error detecting an IMU");
      }
      else
      {
         
         RetStatus = true;
         ImuI2c->Accelerometer.Address = Sensor[SENSOR_ACC].Interface[ImuI2c->Version].Address;
         ImuI2c->Accelerometer.ReadCmd = Sensor[SENSOR_ACC].Interface[ImuI2c->Version].ReadCmd;
         ImuI2c->Gyroscope.Address     = Sensor[SENSOR_GYR].Interface[ImuI2c->Version].Address;
         ImuI2c->Gyroscope.ReadCmd     = Sensor[SENSOR_GYR].Interface[ImuI2c->Version].ReadCmd;
         ImuI2c->Magnetometer.Address  = Sensor[SENSOR_MAG].Interface[ImuI2c->Version].Address;
         ImuI2c->Magnetometer.ReadCmd  = Sensor[SENSOR_MAG].Interface[ImuI2c->Version].ReadCmd;

         CFE_EVS_SendEvent (IMU_I2C_IMU_DETECTION_ERR_EID, CFE_EVS_EventType_INFORMATION,
                            "%s detected. Addresses[ACC.GYR,MAG] = [0x%04X,0x%04X,0x%04X]", 
                            ImuDetectedStr,ImuI2c->Accelerometer.Address,ImuI2c->Gyroscope.Address,
                            ImuI2c->Magnetometer.Address);      
      }
      
	} /* End if open I2C file */
	
	return RetStatus;
	
} /* End DetectImu() */


/******************************************************************************
** Function: EnableImu
**
** Enable the gyro, accelerometer, and magnetometer
**
** Notes:
**   1. Assumes a Berry IMU version has been detected
**
*/
static bool EnableImu(void)
{

   bool Enabled = true;
   
   switch (ImuI2c->Version)
   {
      case IMU_I2C_VERSION_1:
      
         // Enable Gyroscope
         WriteGyrReg(LSM9DS0_CTRL_REG1_G, 0x0F);  // 0b00001111 - Normal power mode, all axes enabled
         WriteGyrReg(LSM9DS0_CTRL_REG4_G, 0x30);  // 0b00110000 - Continuos update, 2000 dps full scale

         // Enable Accelerometer.
         WriteAccReg(LSM9DS0_CTRL_REG1_XM, 0x67); // 0b01100111 - z,y,x axis enabled, continuous update,  100Hz data rate
         WriteAccReg(LSM9DS0_CTRL_REG2_XM, 0x20); // 0b00100000 - +/- 16G full scale

         //Enable Magnetometer
         WriteMagReg(LSM9DS0_CTRL_REG5_XM, 0xF0); // 0b11110000 - Temp enable, M data rate = 50Hz
         WriteMagReg(LSM9DS0_CTRL_REG6_XM, 0x60); // 0b01100000 - +/-12gauss
         WriteMagReg(LSM9DS0_CTRL_REG7_XM, 0x00); // 0b00000000 - Continuous-conversion mode
       
         break;

      case IMU_I2C_VERSION_2:

         // Enable Gyroscope
         WriteGyrReg(LSM9DS1_CTRL_REG4,    0x38);  // 0b00111000 - z, y, x axis enabled for gyro
         WriteGyrReg(LSM9DS1_CTRL_REG1_G,  0xB8);  // 0b10111000 - Gyro ODR = 476Hz, 2000 dps
         WriteGyrReg(LSM9DS1_ORIENT_CFG_G, 0xB8);  // 0b10111000 - Swap orientation 

         // Enable Accelerometer
         WriteAccReg(LSM9DS1_CTRL_REG5_XL, 0x38);  // 0b00111000 - z, y, x axis enabled for accelerometer
         WriteAccReg(LSM9DS1_CTRL_REG6_XL, 0x28);  // 0b00101000 - +/- 16g

         //Enable Magnetometer
         WriteMagReg(LSM9DS1_CTRL_REG1_M, 0x9C);   // 0b10011100 - Temp compensation enabled,Low power mode mode,80Hz ODR
         WriteMagReg(LSM9DS1_CTRL_REG2_M, 0x40);   // 0b01000000 - +/-12gauss
         WriteMagReg(LSM9DS1_CTRL_REG3_M, 0x00);   // 0b00000000 - continuos update
         WriteMagReg(LSM9DS1_CTRL_REG4_M, 0x00);   // 0b00000000 - lower power mode for Z axis
       
         break;
        	
      case IMU_I2C_VERSION_3:

         //Enable Gyroscope
         Enabled &= WriteGyrReg(LSM6DSL_CTRL2_G, 0x9C);    // 0b10011100 - ODR 3.3 kHz, 2000 dps

         // Enable Accelerometer
         Enabled &= WriteAccReg(LSM6DSL_CTRL1_XL, 0x9F);   // 0b10011111 - ODR 3.33 kHz, +/- 8g , BW = 400hz
         Enabled &= WriteAccReg(LSM6DSL_CTRL8_XL, 0xC8);   // 0b11001000 - Low pass filter enabled, BW9, composite filter
         Enabled &= WriteAccReg(LSM6DSL_CTRL3_C,  0x44);   // 0b01000100 - Enable Block Data update, increment during multi byte read

         //Enable Magnetometer
         Enabled &= WriteMagReg(LIS3MDL_CTRL_REG1, 0xDC);  // 0b11011100 - Temp sensor enabled, High performance, ODR 80 Hz, FAST ODR disabled and Selft test disabled.
         Enabled &= WriteMagReg(LIS3MDL_CTRL_REG2, 0x20);  // 0b00100000 - +/- 8 gauss
         Enabled &= WriteMagReg(LIS3MDL_CTRL_REG3, 0x00);  // 0b00000000 - Continuous-conversion mode
		
		 break;

      default:   
         Enabled = false;
         break;
         
   } /* End version switch */

   ImuI2c->Enabled = Enabled;
   if (Enabled)
   {
       CFE_EVS_SendEvent (IMU_I2C_IMU_ENABLED_EID, CFE_EVS_EventType_INFORMATION,
                          "Successfully enabled IMU sensors");
   }
   else
   {
       CFE_EVS_SendEvent (IMU_I2C_IMU_ENABLED_ERR_EID, CFE_EVS_EventType_ERROR,
                          "Error enabling IMU sensors");
   }
   
   return Enabled;
   
   
} /* End EnableImu() */


/******************************************************************************
** Function: ReadDataBlock
**
** Read a block of data from a sensor
**
** Notes:
**   None
**
*/
static bool ReadDataBlock(uint8_t Command, uint8_t Size, uint8_t *Data)
{

   return (i2c_smbus_read_i2c_block_data(ImuI2c->File, Command, Size, Data) == Size);

} /* End ReadDataBlock() */


/******************************************************************************
** Function: SelectDevice
**
** Selects a device using the supplied address
**
** Notes:
**   1. A negative ioctl() value is an error
**
*/
static bool SelectDevice(int Address)
{

   return (ioctl(ImuI2c->File, I2C_SLAVE, Address) >= 0);

} /* End SelectDevice() */


/******************************************************************************
** Function: WriteAcceReg
**
** Write to an accelerometer register
**
** Notes:
**   1. A negative i2c_smbus_write_byte_data() value is an error
**
*/
static bool WriteAccReg(uint8_t Register, uint8_t Value)
{

   bool RetStatus = false;
   
   if (SelectDevice(ImuI2c->Accelerometer.Address))
   {

      RetStatus = (i2c_smbus_write_byte_data(ImuI2c->File, Register, Value) >= 0);
   }

   return RetStatus;
   
} /* End WriteAccReg() */


/******************************************************************************
** Function: WriteGyrReg
**
** Write to an gyroscope register
**
** Notes:
**   1. A negative i2c_smbus_write_byte_data() value is an error
**
*/
static bool WriteGyrReg(uint8_t Register, uint8_t Value)
{

   bool RetStatus = false;
   
   if (SelectDevice(ImuI2c->Gyroscope.Address))
   {

      RetStatus = (i2c_smbus_write_byte_data(ImuI2c->File, Register, Value) >=0);
   }

   return RetStatus;

} /* End WriteGyrReg() */


/******************************************************************************
** Function: WriteMagReg
**
** Write to a magnetometer register
**
** Notes:
**   1. A negative i2c_smbus_write_byte_data() value is an error
**
*/
static bool WriteMagReg(uint8_t Register, uint8_t Value)
{

   bool RetStatus = false;
   
   if (SelectDevice(ImuI2c->Magnetometer.Address))
   {

      RetStatus = (i2c_smbus_write_byte_data(ImuI2c->File, Register, Value) >=0);
   }
	
   return RetStatus;

} /* End WriteMagReg() */

