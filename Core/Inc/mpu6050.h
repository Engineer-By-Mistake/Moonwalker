#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_
#include "stm32f4xx.h"
extern I2C_HandleTypeDef hi2c1;
typedef struct{
	int16_t gyro[3];
	int16_t temp;
	int16_t axceloro[3];
	int32_t base_value[7];
}raw;

typedef struct{
	float gyro[3];
	float temp;
	float axceloro[3];
	float base_value[7];
	int some_thing_is_wrong[1];
}reading;

extern raw raw_values;
extern reading true_reading;

void MPU6050_Init(void);
void MPU6050_read_data(void);
void data_convert(void);

#endif
