#include"followin.h"
#include "mpu6050.h"
#include"stm32f4xx.h"
#include "motor.h"
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
static states previous_state = straight;
states states_global;
biease biease_global = straight_biase; 
uint32_t sensor_thrashold[8];
typedef struct {
    bool left[3];
    bool right[3];
} wing_sensors;
wing_sensors wings;
static uint8_t intersection_counter = 0;
static uint16_t intersection_maneuver_timer = 0;
static uint16_t intersection_cooldown = 0;
uint32_t max_sensor_reading[8] = {0, 0, 0, 0, 0, 0, 0, 0};
uint32_t min_sensor_reading[8] = {4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095};
const int32_t sensor_weights[8] ={850,650,450,100,-100,-450,-650,-850};
pid_error PID_STRAIGHT = {
    .kP=0.3f,.ki=0.0f,.kd=0.1f,
    .integral=0,.last_error=0
};
pid_error PID_CORNER = {
    .kP=0.2f,.ki=0.0f,.kd=0.1f,
    .integral=0,.last_error=0
};
void read_wing_sensors(void) {
    wings.left[0]  = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
    wings.left[1]  = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2);
    wings.left[2]  = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10);
    wings.right[0] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
    wings.right[1] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);
    wings.right[2] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
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
/*/void center_riPID_STRAIGHT_left_detection(void){
    bool center = (sensor_read[3] > sensor_thrashold[3] && sensor_read[4] > sensor_thrashold[4] );
    bool sliPID_STRAIGHT_riPID_STRAIGHT = (sensor_read[3] > sensor_thrashold[3] && sensor_read[2] > sensor_thrashold[2] );
    bool very_riPID_STRAIGHT = (sensor_read[1] > sensor_thrashold[1] && sensor_read[2] > sensor_thrashold[2] );
    bool wide_riPID_STRAIGHT = (sensor_read[0] > sensor_thrashold[0] && sensor_read[1] > sensor_thrashold[1] );
    bool sliPID_STRAIGHT_left = (sensor_read[4] > sensor_thrashold[4] && sensor_read[5] > sensor_thrashold[5] );
    bool very_left = (sensor_read[5] > sensor_thrashold[5] && sensor_read[6] > sensor_thrashold[6] );
    bool wide_left = (sensor_read[6] > sensor_thrashold[6] && sensor_read[7] > sensor_thrashold[7] );
    // now fnding state and riPID_STRAIGHTand left control 
}/*/
bool center_black_most(void){
    int sensor_on_line_count =0;
    for (int i=0; i<8; i++) {
        if (sensor_read[i]>sensor_thrashold[i]) {
            sensor_on_line_count++;
        }
    }
   return sensor_on_line_count>=6;
}
bool detect_intersection(){
    bool right_side = wings.right[1] || wings.right[2];
    bool left_side = wings.left[1] || wings.left[2];
    if((right_side && left_side) || center_black_most() ){
        intersection_counter++;
    }
    else {
        intersection_counter=0;
    }
    return (intersection_counter>=TURN_CONFIRM_CYCLES);
}
void run_intersection_maneuver(TIM_HandleTypeDef *c) {
    switch (biease_global) {
        case right:
            motor_control(BASE_SPEED, -(int32_t)INTERSECTION_SPEED, c);
            break;
        case left:
            motor_control(-(int32_t)INTERSECTION_SPEED, BASE_SPEED, c);
            break;
        case straight_biase:
        default:
            motor_control(BASE_SPEED, BASE_SPEED, c);
            break;
    }

    intersection_maneuver_timer++;

    if (intersection_maneuver_timer >= INTERSECTION_TURN_CYCLES) {
        intersection_maneuver_timer = 0;
        intersection_counter = 0;
        intersection_cooldown = INTERSECTION_COOLDOWN_CYCLES;   // prevent immediate re-trigger
        states_global = straight;
        previous_state = straight;
        activate_pid(&PID_STRAIGHT, 0.0f);
        reset_pid(&PID_CORNER);
    }
}

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
void drive(TIM_HandleTypeDef *c) {
    read_wing_sensors();

    bool line = false;
    float error = calculate_line_error(&line);

    // --- Priority 1: mid-maneuver, keep executing it ---
    if (states_global == intersection_pending) {
        run_intersection_maneuver(c);
        return;
    }

    // --- Priority 2: line lost — active recovery, not silent motor freeze ---
    if (!line) {
        states_global = lost;
        int dir = (PID_STRAIGHT.last_error >= 0) ? 1 : -1;
        motor_control(dir * (int32_t)PIVOT_SPEED, -dir * (int32_t)PIVOT_SPEED, c);
        return;
    }

    // --- Priority 3: check for new intersection (respecting cooldown) ---
    if (intersection_cooldown > 0) {
        intersection_cooldown--;
    } else if (detect_intersection()) {
        states_global = intersection_pending;
        intersection_maneuver_timer = 0;
        return;   // maneuver starts next cycle
    }

    // --- Priority 4: normal corner/straight PID with hysteresis ---
    if (previous_state == straight || previous_state == corner) {
        if (fabsf(error) > CORNER_ENTER_THRESHOLD) {
            states_global = corner;
        } else if (fabsf(error) < CORNER_EXIT_THRESHOLD) {
            states_global = straight;
        } else {
            states_global = previous_state;
        }
    } else {
        states_global = straight;
    }

    if (fabsf(error) < DEADZONE_PID) error = 0.0f;

    pid_error *active;
    uint32_t base_speed;
    if (states_global == corner) {
        active = &PID_CORNER;
        base_speed = CORNER_SPEED;
        if (previous_state != corner) activate_pid(&PID_CORNER, error);
        reset_pid(&PID_STRAIGHT);
    } else {
        active = &PID_STRAIGHT;
        base_speed = BASE_SPEED;
        if (previous_state != straight) activate_pid(&PID_STRAIGHT, error);
        reset_pid(&PID_CORNER);
    }

    previous_state = states_global;

    float correction = pid_compute(active, error);
    if (correction > (float)MAX_SPEED) correction = (float)MAX_SPEED;
    if (correction < -(float)MAX_SPEED) correction = -(float)MAX_SPEED;

    int32_t left_speed  = (int32_t)base_speed + (int32_t)correction;
    int32_t right_speed = (int32_t)base_speed - (int32_t)correction;

    if (left_speed  > (int32_t)MAX_SPEED) left_speed  = MAX_SPEED;
    if (right_speed > (int32_t)MAX_SPEED) right_speed = MAX_SPEED;

    motor_control(left_speed, right_speed, c);
}