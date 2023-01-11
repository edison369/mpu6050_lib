/************************************************************************
 * NASA Docket No. GSC-18,719-1, and identified as “core Flight System: Bootes”
 *
 * Copyright (c) 2020 United States Government as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License. You may obtain
 * a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ************************************************************************/

/**
 * @file
 *  An example of an internal (private) header file for SAMPLE Lib
 */
#ifndef MPU6050_INTERNAL_H
#define MPU6050_INTERNAL_H

/* Include all external/public definitions */
#include "mpu6050.h"

/*************************************************************************
** Macro Definitions
*************************************************************************/

#define MPU6050_BUFFER_SIZE 16

/*************************************************************************
** Internal Data Structures
*************************************************************************/
extern char MPU6050_Buffer[MPU6050_BUFFER_SIZE];

/*************************************************************************
** Function Declarations
*************************************************************************/

/**
 * Library initialization routine/entry point
 */
int32 MPU6050_Init(void);

int sensor_mpu6050_ioctl(i2c_dev *dev, ioctl_command_t command, void *arg);
int read_bytes(int fd, uint16_t i2c_address, uint8_t data_address, uint16_t nr_bytes, uint8_t **buff);

int sensor_mpu6050_set_reg_8(i2c_dev *dev, int ptr, uint8_t val);
int sensor_mpu6050_get_reg_8(uint8_t register_add, uint8_t **buff);

int sensor_mpu6050_get_gyro_axis(uint8_t **buff, sensor_mpu6050_axis axis);
int sensor_mpu6050_get_accel_axis(uint8_t **buff, sensor_mpu6050_axis axis);

float wrap(float angle,float limit);
void setGyroOffsets(float x, float y, float z);
void setAccOffsets(float x, float y, float z);
void setFilterGyroCoef(float gyro_coeff);

void sensor_mpu6050_init_data(void);

int sensor_mpu6050_update_accel(void);

int sensor_mpu6050_update_gyro(void);

int sensor_mpu6050_update_temp(void);

#endif
