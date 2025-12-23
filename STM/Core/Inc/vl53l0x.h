#ifndef __VL53L0X_H
#define __VL53L0X_H

#include "main.h"

// I2C address (default 0x29) shifted left for HAL (0x29 << 1 = 0x52)
#define VL53L0X_ADDR  (0x29 << 1)

// Register addresses
#define VL53L0X_REG_IDENTIFICATION_MODEL_ID         0xC0
#define VL53L0X_REG_IDENTIFICATION_REVISION_ID      0xC2
#define VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD   0x50
#define VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD 0x70
#define VL53L0X_REG_SYSRANGE_START                  0x00
#define VL53L0X_REG_RESULT_INTERRUPT_STATUS         0x13
#define VL53L0X_REG_RESULT_RANGE_STATUS             0x14

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t address;
    uint8_t io_timeout;
    uint8_t did_timeout;
    int16_t offset_mm;
    uint8_t stop_variable; // Added for correct continuous mode management
} VL53L0X_Dev_t;

// Function Prototypes
uint8_t VL53L0X_Init(VL53L0X_Dev_t *dev, I2C_HandleTypeDef *hi2c);
void VL53L0X_SetOffset(VL53L0X_Dev_t *dev, int16_t offset);
uint16_t VL53L0X_ReadDistance(VL53L0X_Dev_t *dev);
uint8_t VL53L0X_GetRangeStatus(VL53L0X_Dev_t *dev);


#endif /* __VL53L0X_H */