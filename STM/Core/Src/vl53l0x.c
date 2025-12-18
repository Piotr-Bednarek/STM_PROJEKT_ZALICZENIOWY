#include "vl53l0x.h"

// Basic Registers
#define VL53L0X_REG_IDENTIFICATION_MODEL_ID         0xC0
#define VL53L0X_REG_SYSRANGE_START                  0x00
#define VL53L0X_REG_RESULT_INTERRUPT_STATUS         0x13
#define VL53L0X_REG_RESULT_RANGE_STATUS             0x14

// Helpers
static void WriteByte(VL53L0X_Dev_t *dev, uint8_t reg, uint8_t val) {
    HAL_I2C_Mem_Write(dev->hi2c, dev->address, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 1000);
}

static uint8_t ReadByte(VL53L0X_Dev_t *dev, uint8_t reg) {
    uint8_t val = 0;
    HAL_I2C_Mem_Read(dev->hi2c, dev->address, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 1000);
    return val;
}

static void WriteMulti(VL53L0X_Dev_t *dev, uint8_t reg, uint8_t *data, uint16_t size) {
	HAL_I2C_Mem_Write(dev->hi2c, dev->address, reg, I2C_MEMADD_SIZE_8BIT, data, size, 1000);
}

// Default tuning settings
uint8_t vl53l0x_tuning[] = {
  0xFF, 0x01, 0x00, 0x00, 0xFF, 0x00, 0x09, 0x00, 0x10, 0x00, 0x11, 0x00, 0x24, 0x01, 0x25, 0xFF,
  0x75, 0x00, 0xFF, 0x01, 0x4E, 0x2C, 0x48, 0x00, 0x30, 0x20, 0xFF, 0x00, 0x30, 0x09, 0x54, 0x00,
  0x31, 0x04, 0x32, 0x03, 0x40, 0x83, 0x46, 0x25, 0x60, 0x00, 0x27, 0x00, 0x50, 0x06, 0x51, 0x00,
  0x52, 0x96, 0x56, 0x08, 0x57, 0x30, 0x61, 0x00, 0x62, 0x00, 0x64, 0x00, 0x65, 0x00, 0x66, 0xA0,
  0xFF, 0x01, 0x22, 0x32, 0x47, 0x14, 0x49, 0xFF, 0x4A, 0x00, 0xFF, 0x00, 0x7A, 0x0A, 0x7B, 0x00,
  0x78, 0x21, 0xFF, 0x01, 0x23, 0x34, 0x42, 0x00, 0x44, 0xFF, 0x45, 0x26, 0x46, 0x05, 0x40, 0x40,
  0x0E, 0x06, 0x20, 0x1A, 0x43, 0x40, 0xFF, 0x00, 0x34, 0x03, 0x35, 0x44, 0xFF, 0x01, 0x31, 0x04,
  0x4B, 0x09, 0x4C, 0x05, 0x4D, 0x04, 0xFF, 0x00, 0x44, 0x00, 0x45, 0x20, 0x47, 0x08, 0x48, 0x28,
  0x67, 0x00, 0x70, 0x04, 0x71, 0x01, 0x72, 0xFE, 0x76, 0x00, 0x77, 0x00, 0xFF, 0x01, 0x0D, 0x01,
  0xFF, 0x00, 0x80, 0x01, 0x01, 0xF8, 0xFF, 0x01, 0x8E, 0x01, 0x00, 0x01, 0xFF, 0x00, 0x80, 0x00
};

static void VL53L0X_LoadTuning(VL53L0X_Dev_t *dev) {
	for(int i = 0; i < sizeof(vl53l0x_tuning); i+=2) {
		WriteByte(dev, vl53l0x_tuning[i], vl53l0x_tuning[i+1]);
	}
}

// Perform Reference Calibration (VHV + Phase)
static uint8_t VL53L0X_PerformRefCalibration(VL53L0X_Dev_t *dev) {
    WriteByte(dev, 0x00, 0x01); // SYSRANGE_START to 0x01 
    
    // Set Sequence Config to 0x01 (VHV + Phase Cal only)
    WriteByte(dev, 0x01, 0x01);
    
    // Start
    WriteByte(dev, 0x00, 0x01);
    
    // Wait for Interrupt
    uint32_t start = HAL_GetTick();
    while ((ReadByte(dev, 0x13) & 0x07) == 0) {
        if (HAL_GetTick() - start > 500) return 0; // Timeout 500ms
    }
    
    // Clear Interrupt
    WriteByte(dev, 0x0B, 0x01);
    
    // Restore Sequence Config 
    WriteByte(dev, 0x01, 0xE8);
    
    return 1;
}

// Ref SPAD Management function
static uint8_t VL53L0X_PerformRefSpadManagement(VL53L0X_Dev_t *dev, uint8_t *count, uint8_t *is_aperture) {
    WriteByte(dev, 0xFF, 0x01);
    WriteByte(dev, 0x00, 0x00);
    
    WriteByte(dev, 0xFF, 0x06);
    WriteByte(dev, 0x83, ReadByte(dev, 0x83) | 0x04);
    WriteByte(dev, 0xFF, 0x07);
    WriteByte(dev, 0x81, 0x01);
    
    WriteByte(dev, 0x80, 0x01);
    
    WriteByte(dev, 0x94, 0x6b);
    WriteByte(dev, 0x83, 0x00);
    
    uint32_t start = HAL_GetTick();
    while (ReadByte(dev, 0x83) == 0x00) {
        if (HAL_GetTick() - start > 500) return 0; // Timeout 500ms
    }
    
    WriteByte(dev, 0x83, 0x01);
    uint8_t val = ReadByte(dev, 0x92);
    
    *count = val & 0x7f;
    *is_aperture = (val >> 7) & 0x01;
    
    WriteByte(dev, 0x81, 0x00);
    WriteByte(dev, 0xFF, 0x06);
    WriteByte(dev, 0x83, ReadByte(dev, 0x83) & ~0x04);
    WriteByte(dev, 0xFF, 0x01);
    WriteByte(dev, 0x00, 0x01);
    
    WriteByte(dev, 0xFF, 0x00);
    WriteByte(dev, 0x80, 0x00);
    
    return 1;
}

static void VL53L0X_SetReferenceSpads(VL53L0X_Dev_t *dev, uint8_t count, uint8_t is_aperture) {
    WriteByte(dev, 0xFF, 0x01);
    WriteByte(dev, 0x00, 0x00);
    
    WriteByte(dev, 0xFF, 0x00);
    WriteByte(dev, 0x80, 0x00);
    WriteByte(dev, 0xFF, 0x01);
    WriteByte(dev, 0xFF, 0x00);
    WriteByte(dev, 0x00, 0x00);
    
    WriteByte(dev, 0xFF, 0x01); 
    WriteByte(dev, 0x00, 0x00); 
    WriteByte(dev, 0xFF, 0x06); 
    
    uint8_t refConfig = ReadByte(dev, 0x83);
    WriteByte(dev, 0x83, refConfig & ~0x04); // Disable SPADs
    
    uint8_t first_spad_to_enable = (is_aperture) ? 12 : 0;
    uint8_t spads_enabled = 0;
    uint8_t spad_map[6] = {0,0,0,0,0,0};
    
    for(uint8_t i=0; i<48; i++) {
        if(i >= first_spad_to_enable && spads_enabled < count) {
            spad_map[i/8] |= (1 << (i%8));
            spads_enabled++;
        }
    }
    
    WriteByte(dev, 0xFF, 0x01);
    WriteByte(dev, 0x00, 0x00);
    WriteByte(dev, 0xFF, 0x00); 
    
    WriteMulti(dev, 0x80, spad_map, 6); // Write all 6 bytes
    
    // Restore
    WriteByte(dev, 0xFF, 0x01);
    WriteByte(dev, 0x00, 0x00);
    WriteByte(dev, 0xFF, 0x00);
    
    // Original restore
    WriteByte(dev, 0x80, 0x00);
}


uint8_t VL53L0X_Init(VL53L0X_Dev_t *dev, I2C_HandleTypeDef *hi2c) {
    dev->hi2c = hi2c;
    dev->address = VL53L0X_ADDR; // 0x52
    dev->offset_mm = 0;

    // Soft Reset
    WriteByte(dev, 0x80, 0x00); 
    HAL_Delay(5);
    WriteByte(dev, 0x80, 0x01);
    HAL_Delay(5);
    
    // Check ID
    uint8_t model_id = ReadByte(dev, VL53L0X_REG_IDENTIFICATION_MODEL_ID);
    if (model_id != 0xEE) {
        return 0; // Device not found or ID mismatch
    }

    // Data Init
    WriteByte(dev, 0x88, 0x00);
    WriteByte(dev, 0x80, 0x01);
    WriteByte(dev, 0xFF, 0x01);
    WriteByte(dev, 0x00, 0x00);
    
    ReadByte(dev, 0x91); 
    
    WriteByte(dev, 0x00, 0x01);
    WriteByte(dev, 0xFF, 0x00);
    WriteByte(dev, 0x80, 0x00);
    
    // SPAD Management
    uint8_t spad_count;
    uint8_t spad_type_is_aperture;
    if (!VL53L0X_PerformRefSpadManagement(dev, &spad_count, &spad_type_is_aperture)) {
    	// Fallback to reasonable defaults?
    	spad_count = 5; 
        spad_type_is_aperture = 0;
    }

    // Load Tuning Settings
    VL53L0X_LoadTuning(dev);
    
    // Apply SPADs
    VL53L0X_SetReferenceSpads(dev, spad_count, spad_type_is_aperture);

    // Perform Reference Calibration (VHV + Phase)
    VL53L0X_PerformRefCalibration(dev);

    // Set Signal Rate Limit to 0.25 MCPS
    WriteByte(dev, 0x44, 0x00);
    WriteByte(dev, 0x45, 0x20);
    
    // Set System Sequence Config
    WriteByte(dev, 0x01, 0xE8);

    // Set interrupt config to new sample ready
    WriteByte(dev, 0x0A, 0x04);

    // Set Measurement Timing Budget (Enable sequence steps)
    // Start Continuous Back-to-Back Mode
    WriteByte(dev, 0x80, 0x01);
    WriteByte(dev, 0xFF, 0x01);
    WriteByte(dev, 0x00, 0x00);
    WriteByte(dev, 0x91, 0x3C);
    WriteByte(dev, 0x00, 0x01);
    WriteByte(dev, 0xFF, 0x00);
    WriteByte(dev, 0x80, 0x00);

    WriteByte(dev, VL53L0X_REG_SYSRANGE_START, 0x02); // 0x02 = Continuous, 0x01 = Single
    
    return 1;
}

void VL53L0X_SetOffset(VL53L0X_Dev_t *dev, int16_t offset) {
    dev->offset_mm = offset;
}

uint16_t VL53L0X_ReadDistance(VL53L0X_Dev_t *dev) {
    // In Continuous mode, we just polling for interrupt flag
    
    int timeout = 0;
    while (timeout < 1000) { // Safety timeout
    	uint8_t val = ReadByte(dev, VL53L0X_REG_RESULT_INTERRUPT_STATUS);
        if (val & 0x07) break; 
        // HAL_Delay(1); // Wait for next sample (~33ms in default mode)
        // With loop overhead, checking continuously is fine or small delay
        timeout++;
        if (timeout % 100 == 0) HAL_Delay(1);
    }
    
    if (timeout >= 1000) return 0xFFFF;

    // Read distance
    uint8_t high = ReadByte(dev, 0x1E);
    uint8_t low = ReadByte(dev, 0x1F);
    uint16_t dist = (high << 8) | low;
    
    // Clear Interrupt
    WriteByte(dev, 0x0B, 0x01);

    if (dist != 0xFFFF && dist != 8190 && dist != 8191) {
        dist = (uint16_t)((int16_t)dist + dev->offset_mm);
    }

    return dist;
}

uint8_t VL53L0X_GetRangeStatus(VL53L0X_Dev_t *dev) {
    return (ReadByte(dev, VL53L0X_REG_RESULT_RANGE_STATUS) >> 3) & 0x0F;
}

// Internal function to perform Reference Calibration (VHV/Phase)
// This is critical for avoiding "Phase Fail" or constant error readings