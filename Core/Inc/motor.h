#ifndef INC_motor_H_
#define INC_motor_H_
#include "stm32f4xx.h"
#include "stm32f411xe.h"
#include "stm32f4xx_hal.h" 
#include "stm32f4xx_hal_gpio.h"
#include <stdint.h>
#include "math.h"
#define portA GPIOA
#define portB GPIOB
#define ani1 GPIO_PIN_10
#define ani2 GPIO_PIN_9
#define bni1 GPIO_PIN_15
#define bni2 GPIO_PIN_14
void motor_control (int32_t a,int32_t b,TIM_HandleTypeDef *c);
void motor_pin_set (TIM_HandleTypeDef *c);
#endif
