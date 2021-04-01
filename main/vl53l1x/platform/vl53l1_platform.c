
/*
* Copyright (c) 2017, STMicroelectronics - All Rights Reserved
*
* This file is part of VL53L1 Core and is dual licensed,
* either 'STMicroelectronics
* Proprietary license'
* or 'BSD 3-clause "New" or "Revised" License' , at your option.
*
********************************************************************************
*
* 'STMicroelectronics Proprietary license'
*
********************************************************************************
*
* License terms: STMicroelectronics Proprietary in accordance with licensing
* terms at www.st.com/sla0081
*
* STMicroelectronics confidential
* Reproduction and Communication of this document is strictly prohibited unless
* specifically authorized in writing by STMicroelectronics.
*
*
********************************************************************************
*
* Alternatively, VL53L1 Core may be distributed under the terms of
* 'BSD 3-clause "New" or "Revised" License', in which case the following
* provisions apply instead of the ones mentioned above :
*
********************************************************************************
*
* License terms: BSD 3-clause "New" or "Revised" License.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
* may be used to endorse or promote products derived from this software
* without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*
********************************************************************************
*
*/


#include "vl53l1_platform.h"

#include "driver/i2c.h"

VL53L1_Error convertError(esp_err_t err) {
    if (err == ESP_OK) {
        return VL53L1_ERROR_NONE;
    } else if (err == ESP_ERR_INVALID_ARG) {
        // i2c num invalid
        // i2c cmd link not initialized
        // i2c buffer not in iram
        return VL53L1_ERROR_INVALID_PARAMS;
    } else if (err == ESP_ERR_INVALID_STATE) {
        // i2c driver not installed
        // i2c not in master mode
        return VL53L1_ERROR_CONTROL_INTERFACE;
    } else if (err == ESP_ERR_TIMEOUT) {
        // fsm stuck
        // i2c timeout
        return VL53L1_ERROR_TIME_OUT;
    } else {
        return VL53L1_ERROR_UNDEFINED;
    }
}

VL53L1_Error VL53L1_WriteMulti(VL53L1_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count) {
    esp_err_t err;
    i2c_cmd_handle_t cmdLink;

    cmdLink = i2c_cmd_link_create();
    i2c_master_start(cmdLink);
    i2c_master_write_byte(cmdLink, (Dev->I2cDevAddr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmdLink, (index >> 8) & 0xFF, true);
    i2c_master_write_byte(cmdLink, (index >> 0) & 0xFF, true);
    i2c_master_write(cmdLink, pdata, count, true);
    i2c_master_stop(cmdLink);

    err = i2c_master_cmd_begin(I2C_NUM_0, cmdLink, 10 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmdLink);
    return convertError(err);
}

VL53L1_Error VL53L1_ReadMulti(VL53L1_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count) {
    esp_err_t err;
    i2c_cmd_handle_t cmdLink;

    cmdLink = i2c_cmd_link_create();
    i2c_master_start(cmdLink);
    i2c_master_write_byte(cmdLink, (Dev->I2cDevAddr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmdLink, (index >> 8) & 0xFF, true);
    i2c_master_write_byte(cmdLink, (index >> 0) & 0xFF, true);
    i2c_master_start(cmdLink);
    i2c_master_write_byte(cmdLink, (Dev->I2cDevAddr << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmdLink, pdata, count, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmdLink);

    err = i2c_master_cmd_begin(I2C_NUM_0, cmdLink, 10 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmdLink);
    return convertError(err);
}

VL53L1_Error VL53L1_WrByte(VL53L1_DEV Dev, uint16_t index, uint8_t data) {
    return VL53L1_WriteMulti(Dev, index, &data, 1);
}

VL53L1_Error VL53L1_WrWord(VL53L1_DEV Dev, uint16_t index, uint16_t data) {
    uint8_t pdata[] = {
        (data >> 8) & 0xff,
        (data >> 0) & 0xff,
    };
    return VL53L1_WriteMulti(Dev, index, pdata, 2);
}

VL53L1_Error VL53L1_RdByte(VL53L1_DEV Dev, uint16_t index, uint8_t *data) {
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    Status = VL53L1_ReadMulti(Dev, index, data, 1);
    return Status;
}

VL53L1_Error VL53L1_RdWord(VL53L1_DEV Dev, uint16_t index, uint16_t *data) {
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    uint8_t pdata[2];
    Status = VL53L1_ReadMulti(Dev, index, pdata, 2);
    *data = pdata[0] << 8 | pdata[1];
    return Status;
}

VL53L1_Error VL53L1_GetTickCount(
	uint32_t *ptick_count_ms)
{
	VL53L1_Error status = VL53L1_ERROR_NONE;
    *ptick_count_ms = esp_timer_get_time() / 1000;
	return status;
}

VL53L1_Error VL53L1_WaitMs(VL53L1_Dev_t *pdev, int32_t wait_ms){
	VL53L1_Error status = VL53L1_ERROR_NONE;
    vTaskDelay(wait_ms / portTICK_PERIOD_MS);
	return status;
}

VL53L1_Error VL53L1_WaitUs(VL53L1_Dev_t *pdev, int32_t wait_us){
	VL53L1_Error status = VL53L1_ERROR_NONE;
    vTaskDelay((wait_us / 1000) / portTICK_PERIOD_MS);
    return status;
}

VL53L1_Error VL53L1_WaitValueMaskEx(
	VL53L1_Dev_t *pdev,
	uint32_t      timeout_ms,
	uint16_t      index,
	uint8_t       value,
	uint8_t       mask,
	uint32_t      poll_delay_ms)
{
	VL53L1_Error status = VL53L1_ERROR_NONE;
	TickType_t timeoutTicks = timeout_ms / portTICK_PERIOD_MS;
	TickType_t pollDelayTicks = poll_delay_ms / portTICK_PERIOD_MS;
    TickType_t startTick = xTaskGetTickCount();
    uint8_t rValue;
    do {
        vTaskDelay(pollDelayTicks);
        status = VL53L1_ReadMulti(pdev, index, &rValue, 1);
        if (status != VL53L1_ERROR_NONE) return status;
        if (xTaskGetTickCount() - startTick > timeoutTicks) return VL53L1_ERROR_TIME_OUT;
    } while ((value & mask) != (rValue & mask));
	return status;
}
