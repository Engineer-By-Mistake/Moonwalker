#include"followin.h"
#include"stm32f4xx.h"
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
        sensor_thrashold[i]=(max_sensor_reading[1]+ min_sensor_reading[i])/2
    }
}
void center_right_left_detection(void){
    bool center = (sensor_read[4] > sensor_thrashold[4] && sensor_read[5] > sensor_thrashold[5] );
    bool slight_right = (sensor_read[4] > sensor_thrashold[4] && sensor_read[3] > sensor_thrashold[3] );
    bool very_right = (sensor_read[2] > sensor_thrashold[2] && sensor_read[3] > sensor_thrashold[3] );
    bool wide_right = (sensor_read[2] > sensor_thrashold[2] && sensor_read[1] > sensor_thrashold[1] );
    bool slight_left = (sensor_read[5] > sensor_thrashold[5] && sensor_read[6] > sensor_thrashold[6] );
    bool very_left = (sensor_read[7] > sensor_thrashold[7] && sensor_read[6] > sensor_thrashold[6] );
    bool wide_left = (sensor_read[8] > sensor_thrashold[8] && sensor_read[7] > sensor_thrashold[7] );
    // now fnding state and rightand left control 





}