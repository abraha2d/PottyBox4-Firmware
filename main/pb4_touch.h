#pragma once

#include "hal/touch_sensor_types.h"
#include "soc/touch_sensor_channel.h"


#define PB4_TOUCH_CAL_PERIOD_MS     1000
#define PB4_TOUCH_CAL_WINDOW        10
#define PB4_TOUCH_CAL_MARGIN        2

#define PB4_TOUCH_POLL_PERIOD_MS    100

#define PB4_TOUCH_1    TOUCH_PAD_GPIO32_CHANNEL
#define PB4_TOUCH_2    TOUCH_PAD_GPIO33_CHANNEL


typedef void (*pb4_touch_callback_t)(touch_pad_t, bool);


void vTouchInit(void);

_Noreturn void tTouchCalibrate(void *);

_Noreturn void tTouchPoll(pb4_touch_callback_t pxTouchCallback);
