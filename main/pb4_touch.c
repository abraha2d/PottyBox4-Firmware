#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/touch_sensor.h"

#include "pb4_touch.h"


static void prvTouchConfig(touch_pad_t eTouchPad) {
    // Use a threshold of 0, since we have no baseline yet.
    ESP_ERROR_CHECK(touch_pad_config(eTouchPad, 0));

    // Wait for IIR filter to start returning values.
    // touch_pad_read_filtered returns 0 until the first x msecs have
    // passed, where x is the sampling period of the filter (set during
    // touch_pad_filter_start).
    uint16_t usTouchValue = 0;
    esp_err_t lTouchError;
    do {
        lTouchError = touch_pad_read_filtered(eTouchPad, &usTouchValue);
        if (lTouchError != ESP_ERR_INVALID_STATE) ESP_ERROR_CHECK(lTouchError);
    } while (usTouchValue == 0);

    // Set the touch threshold somewhat lower than the current
    ESP_ERROR_CHECK(touch_pad_set_thresh(eTouchPad, usTouchValue * 2 / 3));
}


static void prvTouchCalibrate(
        touch_pad_t eTouchPad,
        uint16_t *pusTouchValue,
        uint16_t *pusTouchBaseline,
        uint8_t *pucTouchCount
) {
    esp_err_t lTouchError = touch_pad_read_filtered(eTouchPad, pusTouchValue);
    if (lTouchError != ESP_ERR_INVALID_STATE) ESP_ERROR_CHECK(lTouchError);
    if (abs((int32_t) *pusTouchValue - (int32_t) *pusTouchBaseline) < PB4_TOUCH_CAL_MARGIN) {
        if (*pucTouchCount < PB4_TOUCH_CAL_WINDOW) {
            ++*pucTouchCount;
        } else if (*pucTouchCount != UINT8_MAX) {
            ESP_ERROR_CHECK(touch_pad_set_thresh(eTouchPad, *pusTouchBaseline * 2 / 3));
            *pucTouchCount = UINT8_MAX;
        }
    } else {
        *pusTouchBaseline = *pusTouchValue;
        *pucTouchCount = 0;
    }
}


static void prvTouchPoll(
        touch_pad_t eTouchPad,
        bool *bTouchState,
        pb4_touch_callback_t pxTouchCallback
) {
    uint16_t usTouchValue, usTouchThreshold;
    ESP_ERROR_CHECK(touch_pad_get_thresh(eTouchPad, &usTouchThreshold));
    esp_err_t lTouchError = touch_pad_read_filtered(eTouchPad, &usTouchValue);
    if (lTouchError != ESP_ERR_INVALID_STATE) ESP_ERROR_CHECK(lTouchError);
    bool bIsTouched = usTouchValue < usTouchThreshold;
    if (bIsTouched != *bTouchState) {
        pxTouchCallback(eTouchPad, bIsTouched);
        *bTouchState = bIsTouched;
    }
}


void vTouchInit(void) {
    ESP_ERROR_CHECK(touch_pad_init());
    ESP_ERROR_CHECK(touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER));
    ESP_ERROR_CHECK(touch_pad_config(PB4_TOUCH_1, 0));
    ESP_ERROR_CHECK(touch_pad_config(PB4_TOUCH_2, 0));
    ESP_ERROR_CHECK(touch_pad_filter_start(10));
    prvTouchConfig(PB4_TOUCH_1);
    prvTouchConfig(PB4_TOUCH_2);
}


void tTouchCalibrate(__unused void *pvParams) {
    uint16_t usTouch1Value = 0, usTouch2Value = 0;
    uint16_t usTouch1Baseline = 0, usTouch2Baseline = 0;
    uint8_t ucTouch1Count = 0, ucTouch2Count = 0;
    for (;;) {
        prvTouchCalibrate(PB4_TOUCH_1, &usTouch1Value, &usTouch1Baseline, &ucTouch1Count);
        prvTouchCalibrate(PB4_TOUCH_2, &usTouch2Value, &usTouch2Baseline, &ucTouch2Count);
        vTaskDelay(PB4_TOUCH_CAL_PERIOD_MS / portTICK_PERIOD_MS);
    }
}


void tTouchPoll(pb4_touch_callback_t pxTouchCallback) {
    bool bTouch1State = false, bTouch2State = false;
    for (;;) {
        prvTouchPoll(PB4_TOUCH_1, &bTouch1State, pxTouchCallback);
        prvTouchPoll(PB4_TOUCH_2, &bTouch2State, pxTouchCallback);
        vTaskDelay(PB4_TOUCH_POLL_PERIOD_MS / portTICK_PERIOD_MS);
    }
}
