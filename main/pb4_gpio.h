#pragma once

#include "hal/gpio_types.h"


#define PB4_GPIO_TOUCH_1    32
#define PB4_GPIO_TOUCH_2    33

#define PB4_GPIO_S1_INT     4
#define PB4_GPIO_S1_LED     2
#define PB4_GPIO_S1_SHUT    5

#define PB4_GPIO_S2_INT     19
#define PB4_GPIO_S2_LED     18
#define PB4_GPIO_S2_SHUT    23

#define PB4_GPIO_LED_R      27
#define PB4_GPIO_LED_G      14
#define PB4_GPIO_LED_B      12

#define PB4_GPIO_EXHAUST    13
#define PB4_GPIO_FLUSH_1    25
#define PB4_GPIO_FLUSH_2    26


void vGpioInit(void);

void vGpioWrite(gpio_num_t, bool);
