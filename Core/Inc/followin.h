#ifndef INC_followin_H_
#define INC_followin_H_
#include <stdint.h>
#include <sys/types.h>
typedef struct{
    float kP;
    float kd;
    float ki;
    float integral;
    float last_error;
    float error_max;


}pid_error ;
uint32_t base_speed = 700;
uint32_t maxs_speed = 950;
unit32_t corner_speed = .8*base_speed;
uint32_t corrention_speed =.6*base_speed;
uint32_t pviot_speed =.5*base_speed;
uint32_t crocksection_speed=.55*base_speed;
typedef enum{
    straight = 0;
    left = 1;
    right = 2;
} biease;
uint32_t max_sensor_reading[8]={0,0,0,0,0,0,0,0};
uint32_t min_sensor_reading[8]={0,0,0,0,0,0,0,0};
uint32_t sensor_thrashold[8]={0,0,0,0,0,0,0,0};
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
extern pid_error pid_global;
extern biease biease_global;
extern states states_global;
void threshhold(void);
void _right_left_detection(void);
void drive (void);
#endif 
