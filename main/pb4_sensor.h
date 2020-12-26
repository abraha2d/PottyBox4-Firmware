#pragma once

#include "driver/i2c.h"


#define PB4_SENSOR_I2C_PORT                 I2C_NUM_0
#define PB4_SENSOR_I2C_ADDR                 0x4C


typedef void (*pb4_sensor_callback_t)(uint16_t, uint16_t);


#define PB4_SENSOR_REG_ICSR_ADR              0x00
#define PB4_SENSOR_REG_ICSR_LEN              1
union pb4_reg_icsr_t {
    uint8_t data[PB4_SENSOR_REG_ICSR_LEN];
    struct {
        uint8_t intClrCfg: 1;       // 0
        uint8_t intClrType: 1;      // 1
        uint8_t intMode: 1;         // 2
        uint8_t intPolarity: 1;     // 3
        uint8_t intStatus_Data: 1;  // 4
        uint8_t intStatus_M2H: 1;   // 5
        uint8_t intStatus_H2M: 1;   // 6
        uint8_t intStatus_Reset: 1; // 7
    };
} pb4_reg_icsr;


#define PB4_SENSOR_REG_CMD_ADR              0x04
#define PB4_SENSOR_REG_CMD_LEN              1
union pb4_reg_cmd_t {
    uint8_t data[PB4_SENSOR_REG_CMD_LEN];
    struct {
        uint8_t cmdOpcode: 6;       // 5:0
        uint8_t reset: 1;           // 6
        uint8_t valid: 1;           // 7
    };
} pb4_reg_cmd;
#define PB4_SENSOR_CMDOPCODE_SINGLEMEASURE  0x01
#define PB4_SENSOR_CMDOPCODE_STANDBY        0x10
#define PB4_SENSOR_CMDOPCODE_MCPUOFF        0x11
#define PB4_SENSOR_CMDOPCODE_MCPUON         0x12


#define PB4_SENSOR_REG_DEVSTATUS_ADR        0x06
#define PB4_SENSOR_REG_DEVSTATUS_LEN        2
union pb4_reg_devstatus_t {
    uint8_t data[PB4_SENSOR_REG_DEVSTATUS_LEN];
    struct {
        uint8_t pmStatus: 5;        // 4:0
        uint8_t oscStatus: 1;       // 5
        uint8_t cpuStatus: 1;       // 6
        uint8_t pwrStatus: 1;       // 7
        uint8_t reserved: 8;        // 15:8
    };
} pb4_reg_devstatus;
#define PB4_SENSOR_PMSTATUS_STANDBY         0x00
#define PB4_SENSOR_PMSTATUS_MCPUOFF         0x10
#define PB4_SENSOR_PMSTATUS_MCPUON          0x18


#define PB4_SENSOR_REG_RESULT_ADR           0x08
#define PB4_SENSOR_REG_RESULT_LEN           2
union pb4_reg_result_t {
    uint8_t data[PB4_SENSOR_REG_RESULT_LEN];
    struct {
        uint8_t reserved: 2;        // 1:0
        uint16_t distance: 11;      // 12:2
        uint8_t errCode: 2;         // 14:13
        uint8_t valid: 1;           // 15
    };
} pb4_reg_result;
#define PB4_SENSOR_ERRCODE_NONE             0x00
#define PB4_SENSOR_ERRCODE_NEAR             0x01
#define PB4_SENSOR_ERRCODE_FAR              0x10
#define PB4_SENSOR_ERRCODE_GENERAL          0x11


#define PB4_SENSOR_REG_RSLTCNFD_ADR         0x0A
#define PB4_SENSOR_REG_RSLTCNFD_LEN         2
union pb4_reg_rsltcnfd_t {
    uint8_t data[PB4_SENSOR_REG_RSLTCNFD_LEN];
    struct {
        uint8_t valPixels: 4;       // 3:0
        uint16_t vecAmpl: 11;       // 14:4
        uint8_t reserved: 1;        // 15
    };
} pb4_reg_rsltcnfd;


#define PB4_SENSOR_REG_PMUCFG_ADR           0x14
#define PB4_SENSOR_REG_PMUCFG_LEN           2
union pb4_reg_pmucfg_t {
    uint8_t data[PB4_SENSOR_REG_PMUCFG_LEN];
    struct {
        uint8_t reserved4: 4;       // 3:0
        uint8_t patchMemDvd: 4;     // 7:4
        uint8_t patchCodeLdEn: 1;   // 8
        uint8_t MCPUInitState: 1;   // 9
        uint8_t reserved3: 1;       // 10
        uint8_t reserved2: 1;       // 11
        uint8_t reserved1: 4;       // 15:12
    };
} pb4_reg_pmucfg;


#define PB4_SENSOR_REG_MODCHIPID_ADR        0x14
#define PB4_SENSOR_REG_MODCHIPID_LEN        2
union pb4_reg_modchipid_t {
    uint8_t data[PB4_SENSOR_REG_MODCHIPID_LEN];
    struct {
        uint16_t revisionId: 16;    // 15:0
    };
} pb4_reg_modchipid;


#define PB4_SENSOR_REG_I2CINITCFG_ADR        0x1C
#define PB4_SENSOR_REG_I2CINITCFG_LEN        1
union pb4_reg_i2cinitcfg_t {
    uint8_t data[PB4_SENSOR_REG_I2CINITCFG_LEN];
    struct {
        uint8_t autoInc: 1;         // 0
        uint8_t wordAccess: 1;      // 1
        uint8_t autoIncData: 1;     // 2
        uint8_t wordAccessData: 1;  // 3
        uint8_t MCPUDebugI2C: 1;    // 4
        uint8_t hostDebugAccess: 1; // 5
        uint8_t MCPUDebugAccess: 1; // 6
        uint8_t reserved: 1;        // 7
    };
} pb4_reg_i2cinitcfg;


void vSensorInit(void);

_Noreturn void tSensorPoll(pb4_sensor_callback_t);
