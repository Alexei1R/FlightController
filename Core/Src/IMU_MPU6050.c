/*
 * mpu6050.c
 *
 *  Created on: Nov 13, 2019
 *      Author: Bulanov Konstantin
 *
 *  Contact information
 *  -------------------
 *
 * e-mail   :  leech001@gmail.com
 */

/*
 * |---------------------------------------------------------------------------------
 * | Copyright (C) Bulanov Konstantin,2021
 * |
 * | This program is free software: you can redistribute it and/or modify
 * | it under the terms of the GNU General Public License as published by
 * | the Free Software Foundation, either version 3 of the License, or
 * | any later version.
 * |
 * | This program is distributed in the hope that it will be useful,
 * | but WITHOUT ANY WARRANTY; without even the implied warranty of
 * | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * | GNU General Public License for more details.
 * |
 * | You should have received a copy of the GNU General Public License
 * | along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * |
 * | Kalman filter algorithm used from https://github.com/TKJElectronics/KalmanFilter
 * |---------------------------------------------------------------------------------
 */

#include <math.h>
#include "IMU_MPU6050.h"

/*
#define RAD_TO_DEG 57.295779513082320876798154814105

#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_CONFIG_REG 0x1B
#define GYRO_XOUT_H_REG 0x43
*/




uint8_t IMU_Init(I2C_HandleTypeDef *I2Cx)
{
    uint8_t check;
    uint8_t Data;

    HAL_I2C_Mem_Read(I2Cx, IMU_ADDR, IMU_WHO_AM_I, 1, &check, 1, HAL_MAX_DELAY);

    if (check == 104)
    {
        Data = 0;
        HAL_I2C_Mem_Write(I2Cx, IMU_ADDR, IMU_PWR_MGMT_1, 1, &Data, 1, HAL_MAX_DELAY);
        Data = 0x07;
        HAL_I2C_Mem_Write(I2Cx, IMU_ADDR, IMU_SMPLRT_DIV, 1, &Data, 1, HAL_MAX_DELAY);
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, IMU_ADDR, IMU_ACCEL_CONFIG, 1, &Data, 1, HAL_MAX_DELAY);
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, IMU_ADDR, IMU_GYRO_CONFIG, 1, &Data, 1, HAL_MAX_DELAY);
        return 0;
    }
    return 1;
}

void IMU_Update(I2C_HandleTypeDef *I2Cx,IMU_DATA* data,uint32_t dt)
{
    uint8_t Rec_Data[14];
    int16_t temp;

    // Read 14 BYTES of data starting from ACCEL_XOUT_H register

    HAL_I2C_Mem_Read(I2Cx, IMU_ADDR, 0x3b, 1, Rec_Data, 14, HAL_MAX_DELAY);

    data->IMU_row_Acc_X = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    data->IMU_row_Acc_Y = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    data->IMU_row_Acc_Z = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
    data->IMU_row_Temp  = (int16_t)(Rec_Data[6] << 8 | Rec_Data[7]);
    data->IMU_row_Gyro_X = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
    data->IMU_row_Gyro_Y = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
    data->IMU_row_Gyro_Z = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);

    data->IMU_Acc_X = data->IMU_row_Acc_X / 16384.0;
    data->IMU_Acc_Y = data->IMU_row_Acc_Y / 16384.0;
    data->IMU_Acc_Z = data->IMU_row_Acc_Z / 16384.0;
    data->IMU_Temp = (float)((int16_t)data->IMU_row_Temp / (float)340.0 + (float)36.53);
    data->IMU_Gyro_X = data->IMU_row_Gyro_X / 131.0;
    data->IMU_Gyro_Y = data->IMU_row_Gyro_Y / 131.0;
    data->IMU_Gyro_Z = data->IMU_row_Gyro_Z / 131.0;


};
