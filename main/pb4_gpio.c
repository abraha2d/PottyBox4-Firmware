#include "driver/gpio.h"

#include "pb4_gpio.h"


static void prvGpioConfig(gpio_num_t eGpioNum, bool bIsInput) {
    gpio_config_t gpioConfig = {
            .pin_bit_mask = 1ULL << eGpioNum,
            .mode = bIsInput ? GPIO_MODE_INPUT : GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&gpioConfig));
}


void vGpioInit(void) {
    prvGpioConfig(PB4_GPIO_TOUCH_1, true);
    prvGpioConfig(PB4_GPIO_TOUCH_2, true);

    prvGpioConfig(PB4_GPIO_S1_INT, true);
    prvGpioConfig(PB4_GPIO_S1_LED, false);
    prvGpioConfig(PB4_GPIO_S1_SHUT, false);

    prvGpioConfig(PB4_GPIO_S2_INT, true);
    prvGpioConfig(PB4_GPIO_S2_LED, false);
    prvGpioConfig(PB4_GPIO_S2_SHUT, false);

    prvGpioConfig(PB4_GPIO_LED_R, false);
    GPIO.func_out_sel_cfg[PB4_GPIO_LED_R].inv_sel = true;
    prvGpioConfig(PB4_GPIO_LED_G, false);
    GPIO.func_out_sel_cfg[PB4_GPIO_LED_G].inv_sel = true;
    prvGpioConfig(PB4_GPIO_LED_B, false);
    GPIO.func_out_sel_cfg[PB4_GPIO_LED_B].inv_sel = true;

    prvGpioConfig(PB4_GPIO_EXHAUST, false);
    prvGpioConfig(PB4_GPIO_FLUSH_1, false);
    prvGpioConfig(PB4_GPIO_FLUSH_2, false);
}


void vGpioWrite(gpio_num_t eGpioNum, bool bState) {
    ESP_ERROR_CHECK(gpio_set_level(eGpioNum, bState));
}
