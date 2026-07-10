#include "motor.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h" 
#include "stm32f411xe.h"
#include "stm32f4xx_hal_gpio.h"
#include <stdint.h>
#include "stdlib.h"
#include "stm32f4xx_hal_tim.h"
void motor_pin_set (TIM_HandleTypeDef *c){
    HAL_GPIO_WritePin(portA, ani1,0);
    HAL_GPIO_WritePin(portA, ani2,0);
    HAL_GPIO_WritePin(portB, bni1,0);
    HAL_GPIO_WritePin(portB, bni2,0);
    HAL_TIM_PWM_Start(c,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(c,TIM_CHANNEL_4);

}

void motor_control (int32_t a, int32_t b,TIM_HandleTypeDef *c){

   if (a > 0) { HAL_GPIO_WritePin(portA, ani1, 1); HAL_GPIO_WritePin(portA, ani2, 0); }
    else if (a < 0) { HAL_GPIO_WritePin(portA, ani1, 0); HAL_GPIO_WritePin(portA, ani2, 1); }
    else { HAL_GPIO_WritePin(portA, ani1, 0); HAL_GPIO_WritePin(portA, ani2, 0); }
    __HAL_TIM_SET_COMPARE(c, TIM_CHANNEL_1, abs(a));

    // Right motor
    if (b > 0) { HAL_GPIO_WritePin(portB, bni1, 1); HAL_GPIO_WritePin(portB, bni2, 0); }
    else if (b < 0) { HAL_GPIO_WritePin(portB, bni1, 0); HAL_GPIO_WritePin(portB, bni2, 1); }
    else { HAL_GPIO_WritePin(portB, bni1, 0); HAL_GPIO_WritePin(portB, bni2, 0); }
    __HAL_TIM_SET_COMPARE(c, TIM_CHANNEL_4, abs(b));
}