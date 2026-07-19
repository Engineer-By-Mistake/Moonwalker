#ifndef INC_followin_H_
#define INC_followin_H_
#include <stdint.h>
#include <sys/types.h>
#include "motor.h"
#include <stdbool.h>
#define SPEED_SMOOTHING   0.2f   // 0.0-1.0, higher = snappier/less smooth, lower = smoother/slower response
#define BASE_SPEED        800u
#define MAX_SPEED         999u
#define CORNER_SPEED      ((uint32_t)(0.75f * BASE_SPEED))
#define CORRECTION_SPEED  ((uint32_t)(0.65f * BASE_SPEED))
#define PIVOT_SPEED       ((uint32_t)(0.75f * BASE_SPEED))
#define INTERSECTION_SPEED ((uint32_t)(0.65f * BASE_SPEED))
#define CORNER_ENTER_THRESHOLD   500.0f
#define CORNER_EXIT_THRESHOLD    100.0f   // hysteresis band, lower than enter
#define DEADZONE_PID             150.0f
#define DERIVATIVE_LIMIT         3000.0f
#define INTEGRAL_MAX             5000.0f
#define INTEGRAL_MIN            -5000.0f
#define TURN_CONFIRM_CYCLES         3
#define INTERSECTION_TURN_CYCLES    120
#define INTERSECTION_COOLDOWN_CYCLES 50
#define PID_RESET_DELAY_CYCLES   15
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
typedef struct {
    uint8_t left[3];
    uint8_t right[3];
} wing_sensors;
extern uint32_t max_sensor_reading[8];
extern uint32_t min_sensor_reading[8];
extern uint32_t sensor_thrashold[8];
extern wing_sensors wings;
extern const int32_t sensor_weights[8];
extern volatile uint16_t sensor_read[10]; 
extern pid_error PID_STRAIGHT;
extern pid_error PID_CORNER;
extern biease biease_global;
extern states states_global;

void read_wing_sensors(void);
bool center_black_most(void);
bool detect_intersection(void);
void run_intersection_maneuver(TIM_HandleTypeDef *c);
float calculate_line_error(bool *line_detected);
float pid_compute(pid_error *pid, float error);
void activate_pid(pid_error *pid, float current_error);
void motor_control_smooth(int32_t target_left, int32_t target_right, TIM_HandleTypeDef *c);
void reset_pid(pid_error *pid);
void threshhold(void);
void _right_left_detection(void);
void drive(TIM_HandleTypeDef *c );
void drive_test(TIM_HandleTypeDef *c);
#endif 
