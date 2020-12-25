#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "pb4_gpio.h"
#include "pb4_touch.h"


#define PB4_FLUSH_MIN_US    4000000


int64_t llTouch1Time, llTouch2Time;
esp_timer_handle_t xFlush1Timer, xFlush2Timer;


static void vMainFlushStop(const gpio_num_t *eGpioNum) {
    vGpioWrite(*eGpioNum, false);
}


static void vMainTouchCallback(touch_pad_t eTouchPad, bool bTouchState) {
    if (eTouchPad == PB4_TOUCH_1) {
        if (bTouchState || esp_timer_get_time() - llTouch1Time > PB4_FLUSH_MIN_US) {
            if (bTouchState) llTouch1Time = esp_timer_get_time();
            esp_timer_stop(xFlush1Timer);
            vGpioWrite(PB4_GPIO_FLUSH_1, bTouchState);
        } else {
            esp_timer_start_once(
                    xFlush1Timer,
                    llTouch1Time + PB4_FLUSH_MIN_US - esp_timer_get_time()
            );
        }
    } else if (eTouchPad == PB4_TOUCH_2) {
        if (bTouchState || esp_timer_get_time() - llTouch2Time > PB4_FLUSH_MIN_US) {
            if (bTouchState) llTouch2Time = esp_timer_get_time();
            esp_timer_stop(xFlush2Timer);
            vGpioWrite(PB4_GPIO_FLUSH_2, bTouchState);
        } else {
            esp_timer_start_once(
                    xFlush2Timer,
                    llTouch2Time + PB4_FLUSH_MIN_US - esp_timer_get_time()
            );
        }
    }
}


_Noreturn void app_main(void) {
    vGpioInit();
    vTouchInit();

    gpio_num_t eGpioFlush1 = PB4_GPIO_FLUSH_1;
    esp_timer_create(&(esp_timer_create_args_t) {
            .callback = (esp_timer_cb_t) vMainFlushStop,
            .arg = &eGpioFlush1,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "Flush1",
    }, &xFlush1Timer);

    gpio_num_t eGpioFlush2 = PB4_GPIO_FLUSH_2;
    esp_timer_create(&(esp_timer_create_args_t) {
            .callback = (esp_timer_cb_t) vMainFlushStop,
            .arg = &eGpioFlush2,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "Flush2",
    }, &xFlush2Timer);

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
