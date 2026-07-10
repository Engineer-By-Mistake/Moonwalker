#include"stm32f4xx.h"
#include "mpu6050.h"

extern I2C_HandleTypeDef hi2c1;
raw raw_values;
reading true_reading;

#define MPU6050_ADDR (0x68 << 1)
#define MPU6050_REG_PWR_MGMT1 0x6B
#define MPU6050_REG_WHO_AM_I  0x75
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define REG_ACCEL_CONFIG 0x1C
#define REG_GYRO_CONFIG 0x1B


void MPU6050_Init(void)
{
    uint8_t data = 0x00; // clear sleep bit
    HAL_I2C_Mem_Write(&hi2c1,
                      MPU6050_ADDR,
                      MPU6050_REG_PWR_MGMT1,
                      I2C_MEMADD_SIZE_8BIT,
                      &data,
                      1,
                      HAL_MAX_DELAY);
    data = 0x18;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, REG_ACCEL_CONFIG, 1, &data, 1, 100);
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, REG_GYRO_CONFIG, 1, &data, 1, 100);

    uint8_t whoami = 0;
    HAL_I2C_Mem_Read(&hi2c1,MPU6050_ADDR,MPU6050_REG_WHO_AM_I,I2C_MEMADD_SIZE_8BIT,
                     &whoami,1,HAL_MAX_DELAY);
    if (whoami != 0x68)
      {
          true_reading.some_thing_is_wrong[0]=1;
      }
      else
      {
          true_reading.some_thing_is_wrong[0]=0;
      }
    
    HAL_Delay(2000);
    for(int i=0;i<=200;i++){
    	MPU6050_read_data();
    	raw_values.base_value[0]+=raw_values.axceloro[0];
    	raw_values.base_value[1]+=raw_values.axceloro[1];
    	raw_values.base_value[2]+=raw_values.axceloro[2];
    	raw_values.base_value[3]+=raw_values.temp;
    	raw_values.base_value[4]+=raw_values.gyro[0];
    	raw_values.base_value[5]+=raw_values.gyro[1];
    	raw_values.base_value[6]+=raw_values.gyro[2];
    	HAL_Delay(5);
    }
    true_reading.base_value[0]=(raw_values.base_value[0]/200.0f)/2048.0f;
    true_reading.base_value[1]=(raw_values.base_value[1]/200.0f)/2048.0f;
    true_reading.base_value[2]=(raw_values.base_value[2]/200.0f)/2048.0f;
    true_reading.base_value[3]=(raw_values.base_value[3]/200.0f/340.0f)+36.53f;
    true_reading.base_value[4]=(raw_values.base_value[4]/200.0f)/16.4f;
    true_reading.base_value[5]=(raw_values.base_value[5]/200.0f)/16.4f;
    true_reading.base_value[6]=(raw_values.base_value[6]/200.0f)/16.4f;
}

void MPU6050_read_data (void)
{
    uint8_t buf[14];
    HAL_I2C_Mem_Read(&hi2c1,
                     MPU6050_ADDR,
                     MPU6050_REG_ACCEL_XOUT_H,
                     I2C_MEMADD_SIZE_8BIT,
                     buf,
                     14,
                     HAL_MAX_DELAY);

    raw_values.axceloro[0]= (int16_t)( (buf[0] << 8) | buf[1] );
    raw_values.axceloro[1]= (int16_t)( (buf[2] << 8) | buf[3] );
    raw_values.axceloro[2]= (int16_t)( (buf[4] << 8) | buf[5] );
    raw_values.temp= (int16_t)((buf[6] << 8) | buf[7]);
    raw_values.gyro[0]= (int16_t)( (buf[8]  << 8) | buf[9]  );
    raw_values.gyro[1] = (int16_t)( (buf[10] << 8) | buf[11] );
    raw_values.gyro[2]= (int16_t)( (buf[12] << 8) | buf[13] );
}
void data_convert(void){
	true_reading.temp=(raw_values.temp/340.0f)+36.53f;
    true_reading.axceloro[0]=raw_values.axceloro[0]/2048.0f;
    true_reading.axceloro[1]=raw_values.axceloro[1]/2048.0f;
    true_reading.axceloro[2]=raw_values.axceloro[2]/2048.0f;
    true_reading.gyro[0]=raw_values.gyro[0]/16.4f;
    true_reading.gyro[1]=raw_values.gyro[1]/16.4f;
    true_reading.gyro[2]=raw_values.gyro[2]/16.4f;
}


