#ifndef INC_oled_H_
#define INC_oled_H_
#include "followin.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal_tim.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"

// Add to followin.h
typedef enum {
    MODE_SENSOR_TEST = 0,
    MODE_RUN,
    MODE_TELEMETRY,
    MODE_BIAS_SELECT,
    MODE_COUNT   // keep last — used for wraparound
} display_mode;

extern display_mode current_mode;
void oled_show_telemetry(void);
void oled_mode_handler(void);
void oled_show_sensor_test(void);
void oled_show_run_mode(void);
void oled_show_bias_select(void);
#endif