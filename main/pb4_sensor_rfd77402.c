#include <ctype.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "pb4_sensor.h"


static void prvSensorWriteReg(uint8_t ucReg, uint8_t *pucData, size_t uxDataLen) {
    i2c_cmd_handle_t pvCmdHandle = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(pvCmdHandle));
    ESP_ERROR_CHECK(i2c_master_write_byte(pvCmdHandle, (PB4_SENSOR_I2C_ADDR << 1) | I2C_MASTER_WRITE, true));
    ESP_ERROR_CHECK(i2c_master_write_byte(pvCmdHandle, ucReg, true));
    ESP_ERROR_CHECK(i2c_master_write(pvCmdHandle, pucData, uxDataLen, false));
    ESP_ERROR_CHECK(i2c_master_stop(pvCmdHandle));

    ESP_ERROR_CHECK(i2c_master_cmd_begin(PB4_SENSOR_I2C_PORT, pvCmdHandle, 10 / portTICK_PERIOD_MS));

    i2c_cmd_link_delete(pvCmdHandle);
}


static void prvSensorReadReg(uint8_t ucReg, uint8_t *pucData, size_t uxDataLen) {
    i2c_cmd_handle_t pvCmdHandle = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(pvCmdHandle));
    ESP_ERROR_CHECK(i2c_master_write_byte(pvCmdHandle, (PB4_SENSOR_I2C_ADDR << 1) | I2C_MASTER_WRITE, true));
    ESP_ERROR_CHECK(i2c_master_write_byte(pvCmdHandle, ucReg, true));
    ESP_ERROR_CHECK(i2c_master_start(pvCmdHandle));
    ESP_ERROR_CHECK(i2c_master_write_byte(pvCmdHandle, (PB4_SENSOR_I2C_ADDR << 1) | I2C_MASTER_READ, true));
    ESP_ERROR_CHECK(i2c_master_read(pvCmdHandle, pucData, uxDataLen, I2C_MASTER_LAST_NACK));
    ESP_ERROR_CHECK(i2c_master_stop(pvCmdHandle));

    ESP_ERROR_CHECK(i2c_master_cmd_begin(PB4_SENSOR_I2C_PORT, pvCmdHandle, 10 / portTICK_PERIOD_MS));

    i2c_cmd_link_delete(pvCmdHandle);
}


static void vSensorPowerOn() {
    // Reset
    pb4_reg_cmd.data[0] = 0;
    prvSensorWriteReg(PB4_SENSOR_REG_CMD_ADR, pb4_reg_cmd.data, PB4_SENSOR_REG_CMD_LEN);
    pb4_reg_cmd.reset = 1;
    prvSensorWriteReg(PB4_SENSOR_REG_CMD_ADR, pb4_reg_cmd.data, PB4_SENSOR_REG_CMD_LEN);

    // Check Standby Status
    do {
        prvSensorReadReg(PB4_SENSOR_REG_DEVSTATUS_ADR, pb4_reg_devstatus.data, PB4_SENSOR_REG_DEVSTATUS_LEN);
    } while (pb4_reg_devstatus.pmStatus != PB4_SENSOR_PMSTATUS_STANDBY);

    // Configure I2C Interface
    pb4_reg_i2cinitcfg.data[0] = 0;
    pb4_reg_i2cinitcfg.autoInc = 1;
    pb4_reg_i2cinitcfg.MCPUDebugAccess = 1;
    prvSensorWriteReg(PB4_SENSOR_REG_I2CINITCFG_ADR, pb4_reg_i2cinitcfg.data, PB4_SENSOR_REG_I2CINITCFG_LEN);

    // Set MCPU_OFF
    pb4_reg_cmd.data[0] = 0;
    pb4_reg_cmd.cmdOpcode = PB4_SENSOR_CMDOPCODE_MCPUOFF;
    pb4_reg_cmd.valid = 1;
    prvSensorWriteReg(PB4_SENSOR_REG_CMD_ADR, pb4_reg_cmd.data, PB4_SENSOR_REG_CMD_LEN);

    do {
        prvSensorReadReg(PB4_SENSOR_REG_CMD_ADR, pb4_reg_cmd.data, PB4_SENSOR_REG_CMD_LEN);
    } while (pb4_reg_cmd.valid == 1);

    // Check MCPU_OFF Status
    do {
        prvSensorReadReg(PB4_SENSOR_REG_DEVSTATUS_ADR, pb4_reg_devstatus.data, PB4_SENSOR_REG_DEVSTATUS_LEN);
    } while (pb4_reg_devstatus.pmStatus != PB4_SENSOR_PMSTATUS_MCPUOFF);

    // Set Initialization
    pb4_reg_pmucfg.data[0] = 0;
    pb4_reg_pmucfg.data[1] = 0;
    pb4_reg_pmucfg.MCPUInitState = 1;
    pb4_reg_pmucfg.reserved3 = 1;
    prvSensorWriteReg(PB4_SENSOR_REG_PMUCFG_ADR, pb4_reg_pmucfg.data, PB4_SENSOR_REG_PMUCFG_LEN);

    // Set MCPU_ON
    pb4_reg_cmd.cmdOpcode = PB4_SENSOR_CMDOPCODE_MCPUON;
    pb4_reg_cmd.valid = 1;
    prvSensorWriteReg(PB4_SENSOR_REG_CMD_ADR, pb4_reg_cmd.data, PB4_SENSOR_REG_CMD_LEN);

    do {
        prvSensorReadReg(PB4_SENSOR_REG_CMD_ADR, pb4_reg_cmd.data, PB4_SENSOR_REG_CMD_LEN);
    } while (pb4_reg_cmd.valid == 1);

    // Check MCPU_ON Status
    do {
        prvSensorReadReg(PB4_SENSOR_REG_DEVSTATUS_ADR, pb4_reg_devstatus.data, PB4_SENSOR_REG_DEVSTATUS_LEN);
    } while (pb4_reg_devstatus.pmStatus != PB4_SENSOR_PMSTATUS_MCPUON);
}


void vSensorInit(void) {
    i2c_config_t sConfig = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = 21,
            .scl_io_num = 22,
            .sda_pullup_en = true,
            .scl_pullup_en = true,
            .master.clk_speed = 100000,
    };
    ESP_ERROR_CHECK(i2c_param_config(PB4_SENSOR_I2C_PORT, &sConfig));
    ESP_ERROR_CHECK(i2c_driver_install(
            PB4_SENSOR_I2C_PORT,
            sConfig.mode,
            0, 0,
            ESP_INTR_FLAG_IRAM
    ));
}


void tSensorPoll(pb4_sensor_callback_t pxSensorCallback) {
    vSensorPowerOn();
    for (;;) {
        // Single Measure Command
        pb4_reg_cmd.cmdOpcode = PB4_SENSOR_CMDOPCODE_SINGLEMEASURE;
        pb4_reg_cmd.valid = 1;
        prvSensorWriteReg(PB4_SENSOR_REG_CMD_ADR, pb4_reg_cmd.data, PB4_SENSOR_REG_CMD_LEN);

        do {
            prvSensorReadReg(PB4_SENSOR_REG_CMD_ADR, pb4_reg_cmd.data, PB4_SENSOR_REG_CMD_LEN);
        } while (pb4_reg_cmd.valid == 1);

        // Check Data Ready
        do {
            prvSensorReadReg(PB4_SENSOR_REG_ICSR_ADR, pb4_reg_icsr.data, PB4_SENSOR_REG_ICSR_LEN);
        } while (pb4_reg_icsr.intStatus_Data != 1);

        // Read Result Distance, Amplitude
        prvSensorReadReg(PB4_SENSOR_REG_RESULT_ADR, pb4_reg_result.data, PB4_SENSOR_REG_RESULT_LEN);
        prvSensorReadReg(PB4_SENSOR_REG_RSLTCNFD_ADR, pb4_reg_rsltcnfd.data, PB4_SENSOR_REG_RSLTCNFD_LEN);

        pxSensorCallback(pb4_reg_result.distance, pb4_reg_rsltcnfd.vecAmpl);
    }
}
