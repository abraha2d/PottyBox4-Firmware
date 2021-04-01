#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "pb4_sensor.h"


void vSensorInit(void) {
    ESP_ERROR_CHECK(gpio_reset_pin(5));
    ESP_ERROR_CHECK(gpio_set_direction(5, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(5, 1));
    i2c_config_t sConfig = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = 21,
            .scl_io_num = 22,
            .sda_pullup_en = true,
            .scl_pullup_en = true,
            .master.clk_speed = 400000,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &sConfig));
    ESP_ERROR_CHECK(i2c_driver_install(
            I2C_NUM_0,
            sConfig.mode,
            0, 0,
            ESP_INTR_FLAG_IRAM
    ));
    Dev.I2cDevAddr = 0x29;
    VL53L1_Error err;
    err = VL53L1_WaitDeviceBooted(&Dev);
    if (err != VL53L1_ERROR_NONE) printf("WaitDeviceBooted err %i\n", err);
    err = VL53L1_DataInit(&Dev);
    if (err != VL53L1_ERROR_NONE) printf("DataInit err %i\n", err);
    err = VL53L1_StaticInit(&Dev);
    if (err != VL53L1_ERROR_NONE) printf("StaticInit err %i\n", err);
    err = VL53L1_PerformRefSpadManagement(&Dev);
    if (err != VL53L1_ERROR_NONE) printf("PerformRefSpadManagement err %i\n", err);
    err = VL53L1_SetMeasurementTimingBudgetMicroSeconds(&Dev, 4529);
    if (err != VL53L1_ERROR_NONE) printf("SetMeasurementTimingBudgetMicroSeconds err %i\n", err);
    err = VL53L1_SetInterMeasurementPeriodMilliSeconds(&Dev, 8);
    if (err != VL53L1_ERROR_NONE) printf("SetInterMeasurementPeriodMilliSeconds err %i\n", err);
    err = VL53L1_SetDistanceMode(&Dev, VL53L1_DISTANCEMODE_SHORT);
    if (err != VL53L1_ERROR_NONE) printf("SetDistanceMode err %i\n", err);
}


char color[17];
char* getColor(int16_t range) {
    sprintf(color, "\e[48;2;%u;0;0m", range < 1000 ? 255 - range / 4 : 0);
    return color;
    if (range < 20) {
        return COLOR_RED;
    } else if (20 < range && range < 55) {
        return COLOR_YEL;
    } else if (55 < range && range < 148) {
        return COLOR_GRN;
    } else if (148 < range && range < 403) {
        return COLOR_BLU;
    } else if (403 < range && range < 1097) {
        return COLOR_MAG;
    } else {
        return COLOR_BLK;
    }
}


void tSensorPoll(pb4_sensor_callback_t pxSensorCallback) {
    VL53L1_Error err;
    uint8_t measurementDataReady;
    VL53L1_RangingMeasurementData_t rangingMeasurementData;

    int16_t image[13][13];

    VL53L1_UserRoi_t roiConfig;

    for (;;) {
        for (int y = 0; y < 13; ++y) {
            for (int x = 0; x < 13; ++x) {
                vTaskDelay(0);

                roiConfig.TopLeftX = x;
                roiConfig.TopLeftY = 15 - y;
                roiConfig.BotRightX = x + 3;
                roiConfig.BotRightY = 12 - y;
                err = VL53L1_SetUserROI(&Dev, &roiConfig);
                if (err != VL53L1_ERROR_NONE) printf("SetUserROI err %i\n", err);

                err = VL53L1_StartMeasurement(&Dev);
                if (err != VL53L1_ERROR_NONE) printf("StartMeasurement err %i\n", err);

                do {
                    vTaskDelay(0);
                    err = VL53L1_GetMeasurementDataReady(&Dev, &measurementDataReady);
                    if (err != VL53L1_ERROR_NONE) printf("GetMeasurementDataReady err %i\n", err);
                } while (measurementDataReady != 1);

                err = VL53L1_GetRangingMeasurementData(&Dev, &rangingMeasurementData);
                if (err != VL53L1_ERROR_NONE) printf("GetRangingMeasurementData err %i\n", err);

                image[y][12-x] = rangingMeasurementData.RangeMilliMeter;

                err = VL53L1_StopMeasurement(&Dev);
                if (err != VL53L1_ERROR_NONE) printf("StopMeasurement err %i\n", err);
            }
        }

        printf("\e[H\e[2J\e[3J");

        for (int y = 0; y < 13; ++y) {
            for (int x = 0; x < 13; ++x) {
                printf("%s  %s", getColor(image[y][x]), "\e[0m");
            }
            printf("\n");
        }
    }
}
