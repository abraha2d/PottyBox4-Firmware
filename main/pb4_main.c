#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "pb4_gpio.h"
#include "pb4_touch.h"


void vMainTouchCallback(touch_pad_t eTouchPad, bool bTouchState) {
    if (eTouchPad == PB4_TOUCH_1) {
        vGpioWrite(PB4_GPIO_FLUSH_1, bTouchState);
    } else if (eTouchPad == PB4_TOUCH_2) {
        vGpioWrite(PB4_GPIO_FLUSH_2, bTouchState);
    }
}


_Noreturn void app_main(void) {
    vGpioInit();
    vTouchInit();

    xTaskCreate(
            tTouchCalibrate,
            "touchCalibrate",
            2048,
            NULL,
            1,
            NULL
    );

    xTaskCreate(
            (TaskFunction_t) tTouchPoll,
            "touchPoll",
            2048,
            vMainTouchCallback,
            1,
            NULL
    );

    int cnt = 0;
    for (;;) {
        vGpioWrite(PB4_GPIO_S1_LED, cnt++ % 2);
        vGpioWrite(PB4_GPIO_S1_SHUT, cnt++ % 2);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
