#include"followin.h"
#include "mpu6050.h"
#include"stm32f4xx.h"
#include "motor.h"
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
static states previous_state = straight;
states states_global;
int32_t sensor_thrashold[8];
typedef struct {
    bool left[3];
    bool rigPID_STRAIGHT[3];
} wing_sensors;
wing_sensors wings;

const int32_t sensor_weights[8] ={850,650,450,100,-100,-450,-650,-850};
pid_error GPID_STRAIGHT = {
    .kP=0.5f,.ki=0.0f,.kd=0.3f,
    .integral=0,.last_error=0
};
pid_error PID_CORNER = {
    .kP=0.5f,.ki=0.0f,.kd=0.3f,
    .integral=0,.last_error=0
};
void read_wing_sensors(void) {
    wings.left[0]  = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
    wings.left[1]  = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2);
    wings.left[2]  = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10);
    wings.rigPID_STRAIGHT[0] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
    wings.rigPID_STRAIGHT[1] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);
    wings.rigPID_STRAIGHT[2] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
}
// to oled to diplay 
void threshhold(void){
    uint32_t start_ms = HAL_GetTick();
//time for 10 seconds
    while((HAL_GetTick() - start_ms)/1000 < 10 ){
        for (int i=0 ; i<8; i++) {
            int value = sensor_read[i];
            if( value > max_sensor_reading[i]) max_sensor_reading[i]=value;
            if(value < min_sensor_reading[i]) min_sensor_reading[i]=value;
        
        }
    }
    for(int i=0 ; i<8;i++ ){
        sensor_thrashold[i]=(max_sensor_reading[i]+ min_sensor_reading[i])/2;
    }
}
/*/void center_rigPID_STRAIGHT_left_detection(void){
    bool center = (sensor_read[3] > sensor_thrashold[3] && sensor_read[4] > sensor_thrashold[4] );
    bool sligPID_STRAIGHT_rigPID_STRAIGHT = (sensor_read[3] > sensor_thrashold[3] && sensor_read[2] > sensor_thrashold[2] );
    bool very_rigPID_STRAIGHT = (sensor_read[1] > sensor_thrashold[1] && sensor_read[2] > sensor_thrashold[2] );
    bool wide_rigPID_STRAIGHT = (sensor_read[0] > sensor_thrashold[0] && sensor_read[1] > sensor_thrashold[1] );
    bool sligPID_STRAIGHT_left = (sensor_read[4] > sensor_thrashold[4] && sensor_read[5] > sensor_thrashold[5] );
    bool very_left = (sensor_read[5] > sensor_thrashold[5] && sensor_read[6] > sensor_thrashold[6] );
    bool wide_left = (sensor_read[6] > sensor_thrashold[6] && sensor_read[7] > sensor_thrashold[7] );
    // now fnding state and rigPID_STRAIGHTand left control 
}/*/
float calculate_line_error(bool *line_detected){
    int32_t values =0;
    int on_line=0;
    for(int i=0;i<8;i++){
        if(sensor_read[i]>sensor_thrashold[i]){
            values += sensor_weights[i];
            on_line ++; 
        }
    }
    if(on_line==0){
        *line_detected=false;
        return 0.0f;
    }
    *line_detected=true;
    return (float) values/ (float) on_line; 
}
float pid_compute(pid_error *pid, float error){
    pid->integral +=error;
    if(pid->integral>INTEGRAL_MAX) pid->integral=INTEGRAL_MAX;
    if(pid->integral<INTEGRAL_MIN) pid->integral=INTEGRAL_MIN;
    float dare = error-pid->last_error;
    if (dare>DERIVATIVE_LIMIT)dare=DERIVATIVE_LIMIT;
    if (dare<-DERIVATIVE_LIMIT)dare=-DERIVATIVE_LIMIT;
    float output = (pid->kP*error)+(pid->ki*pid->integral)+(pid->kd*dare);
    pid->last_error=error;
    return output;
}
void reset_pid(pid_error *pid){
    pid->integral =0;
    pid->last_error =0;
}
void activate_pid(pid_error *pid,float current_error){
    pid->last_error=current_error;
}
void drive(TIM_HandleTypeDef *c){
    bool line = false;
    float error = calculate_line_error(&line);
    if (!line){
        states_global=lost;
        return;
    }
    if (previous_state == straight || previous_state == corner) {
        if (fabsf(error) > CORNER_ENTER_THRESHOLD) {
            states_global = corner;
        } else if (fabsf(error) < CORNER_EXIT_THRESHOLD) {
            states_global = straight;
        } else {
            states_global = previous_state;  
        }
    }
    else {
        states_global = straight;
    }
    if (fabsf(error) < DEADZONE_PID) error = 0.0f;
    pid_error *active;
    uint32_t base_speed;
     if (states_global == corner) {
        active = &PID_CORNER;
        base_speed = CORNER_SPEED;
        if (previous_state != corner) activate_pid(&PID_CORNER, error);
        reset_pid(&GPID_STRAIGHT);
    } else {
        active = &GPID_STRAIGHT;
        base_speed = BASE_SPEED;
        if (previous_state != straight) activate_pid(&GPID_STRAIGHT, error);
        reset_pid(&PID_CORNER);
    }

    previous_state = states_global;

    // --- PID compute + apply to motors ---
    float correction = pid_compute(active, error);
    if (correction > (float)MAX_SPEED) correction = (float)MAX_SPEED;
    if (correction < -(float)MAX_SPEED) correction = -(float)MAX_SPEED;

    int32_t left_speed  = (int32_t)base_speed + (int32_t)correction;
    int32_t rigPID_STRAIGHT_speed = (int32_t)base_speed - (int32_t)correction;

    if (left_speed  > (int32_t)MAX_SPEED)  left_speed  = MAX_SPEED;
    if (rigPID_STRAIGHT_speed > (int32_t)MAX_SPEED)  rigPID_STRAIGHT_speed = MAX_SPEED;

    motor_control(left_speed, rigPID_STRAIGHT_speed, c);
}
