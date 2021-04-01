#pragma once

#include "driver/i2c.h"

#include "vl53l1_api.h"


#define COLOR_BLK "\e[40m"
#define COLOR_MAG "\e[45m"
#define COLOR_BLU "\e[44m"
#define COLOR_GRN "\e[42m"
#define COLOR_YEL "\e[43m"
#define COLOR_RED "\e[41m"


VL53L1_Dev_t Dev;


typedef void (*pb4_sensor_callback_t)(uint16_t, uint16_t);


void vSensorInit(void);

_Noreturn void tSensorPoll(pb4_sensor_callback_t);
