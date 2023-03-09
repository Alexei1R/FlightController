


#ifndef IMU_MPU6050
#define IMU_MPU6050


#include <stdint.h>


#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_i2c.h"


#define IMU_ADDR 			(0x68 << 1) + 0 //MPU6050 Addres
#define IMU_SMPLRT_DIV 		0x19
#define IMU_CONFIG 			0x1a
#define IMU_GYRO_CONFIG 	0x1b
#define IMU_ACCEL_CONFIG 	0x1c
#define IMU_WHO_AM_I 		0x75
#define IMU_PWR_MGMT_1 		0x6b
#define IMU_TEMP_H 			0x41
#define IMU_TEMP_L 			0x42

#define IMU_ADD_CORECT 		104 //Status when write IMU Who i am
#define IMU_OK   			1
#define IMU_ERROR 			0


// MPU6050 structure
typedef struct {
	int16_t IMU_row_Acc_X;
	int16_t IMU_row_Acc_Y;
	int16_t IMU_row_Acc_Z;
	int16_t IMU_row_Gyro_X;
	int16_t IMU_row_Gyro_Y;
	int16_t IMU_row_Gyro_Z;
	int16_t IMU_row_Temp;

	//Correct data

	float IMU_Acc_X;
	float IMU_Acc_Y;
	float IMU_Acc_Z;
	float IMU_Gyro_X;
	float IMU_Gyro_Y;
	float IMU_Gyro_Z;
	float IMU_Temp;
}IMU_DATA;





uint8_t IMU_Init(I2C_HandleTypeDef *I2Cx);
void IMU_Update(I2C_HandleTypeDef *I2Cx,IMU_DATA* data,uint32_t dt);
IMU_DATA  IMU_Get_Data(uint32_t dt);




#endif
