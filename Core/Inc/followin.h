#ifndef INC_followin_H_
#define INC_followin_H_
#include <stdint.h>
#include <sys/types.h>
#include "motor.h"
#include <stdbool.h>
#define BASE_SPEED        700u
#define MAX_SPEED         950u
#define CORNER_SPEED      ((uint32_t)(0.8f * BASE_SPEED))
#define CORRECTION_SPEED  ((uint32_t)(0.6f * BASE_SPEED))
#define PIVOT_SPEED       ((uint32_t)(0.5f * BASE_SPEED))
#define INTERSECTION_SPEED ((uint32_t)(0.55f * BASE_SPEED))
#define CORNER_ENTER_THRESHOLD   1500.0f
#define CORNER_EXIT_THRESHOLD    1100.0f   // hysteresis band, lower than enter
#define DEADZONE_PID             150.0f
#define DERIVATIVE_LIMIT         3000.0f
#define INTEGRAL_MAX             5000.0f
#define INTEGRAL_MIN            -5000.0f
typedef struct{
    float kP;
    float kd;
    float ki;
    float integral;
    float last_error;
    float error_max;


}pid_error ;
typedef enum{
    straight_biase = 0,
    left = 1,
    right = 2,
} biease;

typedef enum {
    Ideal=0,
    straight,
    corner,
    // wing sensor 
    right_pendidng,
    left_pending,
    intersection_pending,
    //sharp turns
    right_sharp,
    left_sharp,
    //lost
    lost,
    finished

} states;
extern uint32_t max_sensor_reading[8];
extern uint32_t min_sensor_reading[8];
extern int32_t sensor_thrashold[8];
extern const uint32_t sensor_weights[8];
extern volatile uint32_t sensor_read[9]; 
extern pid_error PID_STRAIGHT;
extern pid_error PID_CORNER;
extern biease biease_global;
extern states states_global;
float calculate_line_error(bool *line_detected);
float pid_compute(pid_error *pid, float error);
void activate_pid(pid_error *pid, float current_error);
void reset_pid(pid_error *pid);
void drive(TIM_HandleTypeDef *c);
void threshhold(void);
void _right_left_detection(void);
#endif 
