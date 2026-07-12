#include "oled.h"
#include "followin.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal_tim.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include <stdio.h>

extern TIM_HandleTypeDef htim1;
extern float last_known_error;
extern states states_global;


display_mode current_mode = MODE_SENSOR_TEST;

static bool oled_enabled = true;
static bool oled_was_cleared = false;

// ==================== KILL SWITCH ====================
void check_oled_kill_switch(void) {
    bool pressed = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_RESET);
    if (pressed) {
        oled_enabled = false;
    }
}

// ==================== MODE HANDLER ====================
void oled_mode_handler(void) {
    check_oled_kill_switch();

    if (!oled_enabled) {
        if (!oled_was_cleared) {
            ssd1306_Fill(Black);
            ssd1306_UpdateScreen();
            oled_was_cleared = true;
        }
        return;   // no drawing, no I2C traffic at all
    }

    switch (current_mode) {
        case MODE_SENSOR_TEST:
            oled_show_sensor_test();
            break;
        case MODE_RUN:
            oled_show_run_mode();
            break;
        case MODE_TELEMETRY:
            oled_show_telemetry();
            break;
        case MODE_BIAS_SELECT:
            oled_show_bias_select();
            break;
        default:
            current_mode = MODE_SENSOR_TEST;
            break;
    }
}

void oled_show_sensor_test(void) {
    char buf[32];
    char wing_buf[32];

    ssd1306_Fill(Black);
    ssd1306_SetCursor(2, 2);
    ssd1306_WriteString("SENSOR DEBUG", Font_7x10, White);
    ssd1306_Line(0, 13, 128, 13, White);

    for(int i = 0; i < 4; i++) {
        ssd1306_SetCursor(2, 17 + (i * 9));
        sprintf(buf, "S%d: %4d", i + 1, sensor_read[i]); 
        ssd1306_WriteString(buf, Font_6x8, White);
    }
    for(int i = 4; i < 8; i++) {
        ssd1306_SetCursor(66, 17 + ((i - 4) * 9));
        sprintf(buf, "S%d: %4d", i + 1, sensor_read[i]);
        ssd1306_WriteString(buf, Font_6x8, White);
    }

    sprintf(wing_buf, "WNG L:%d%d%d R:%d%d%d",
            wings.left[0], wings.left[1], wings.left[2],
            wings.right[0], wings.right[1], wings.right[2]);
    ssd1306_SetCursor(2, 54);
    ssd1306_WriteString(wing_buf, Font_6x8, White);
    read_wing_sensors();
    ssd1306_UpdateScreen();
}

// ==================== RUN MODE — intentionally minimal ====================
void oled_show_run_mode(void) {
    static bool drawn_once = false;
    if (drawn_once) return;   // draw once, then leave the screen alone — zero I2C overhead while racing

    ssd1306_Fill(Black);
    ssd1306_SetCursor(20, 28);
    ssd1306_WriteString("RUNNING", Font_11x18, White);
    ssd1306_UpdateScreen();
    drawn_once = true;
}

// ==================== TELEMETRY — all the debug info lives here now ====================
void oled_show_telemetry(void) {
    char line1[24], line2[24], line3[32];
    ssd1306_Fill(Black);

    ssd1306_SetCursor(2, 2);
    ssd1306_WriteString("TELEMETRY", Font_7x10, White);
    ssd1306_Line(0, 13, 128, 13, White);

    const char* state_name;
    switch (states_global) {
        case straight: state_name = "STRAIGHT"; break;
        case corner: state_name = "CORNER"; break;
        case intersection_pending: state_name = "INTERSECT"; break;
        case lost: state_name = "LOST"; break;
        default: state_name = "?"; break;
    }
    sprintf(line1, "St:%s", state_name);
    ssd1306_SetCursor(2, 18);
    ssd1306_WriteString(line1, Font_6x8, White);

    sprintf(line2, "Error: %d", (int)last_known_error);
    ssd1306_SetCursor(2, 30);
    ssd1306_WriteString(line2, Font_6x8, White);

   float vbat = 4*sensor_read[8];
    vbat=3.3*(vbat/4095);
    vbat=vbat*((float)58/(float)11);
    char vbat_str[20];
    snprintf(vbat_str, sizeof(vbat_str), "%f", vbat);
    sprintf(line3, "Batt:%d V",(int)vbat);
    ssd1306_SetCursor(2, 42);
    ssd1306_WriteString(line3, Font_6x8, White);
    drive_test(&htim1);
    ssd1306_UpdateScreen();
    
}

void oled_show_bias_select(void) {
    ssd1306_Fill(Black);
    ssd1306_SetCursor(2, 2);
    ssd1306_WriteString("BIAS SELECT", Font_7x10, White);
    
    ssd1306_SetCursor(2, 20);
    if (biease_global == straight_biase) {
        ssd1306_WriteString("> Straight", Font_7x10, White);
    } else if (biease_global == left) {
        ssd1306_WriteString("> Left", Font_7x10, White);
    } else if (biease_global == right) {
        ssd1306_WriteString("> Right", Font_7x10, White);
    }
    
    ssd1306_SetCursor(2, 42);
    ssd1306_WriteString("PB4: Change", Font_6x8, White);
    ssd1306_SetCursor(2, 52);
    ssd1306_WriteString("PB3: Back", Font_6x8, White);
    
    ssd1306_UpdateScreen();
}