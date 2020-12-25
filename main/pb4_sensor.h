#pragma once


typedef void (*pb4_sensor_callback_t)();


void vSensorInit(void);

_Noreturn void tSensorPoll(pb4_sensor_callback_t);
