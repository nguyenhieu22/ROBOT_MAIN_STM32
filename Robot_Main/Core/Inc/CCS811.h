/*
 * CCS811.h
 *
 *  Created on: Apr 8, 2024
 *      Author: User
 */

#ifndef INC_CCS811_H_
#define INC_CCS811_H_
/*I2C ADDRESS*/
#define CCS811_I2C_ADDRESS1                      0x5A
#define CCS811_I2C_ADDRESS2                      0x5B

#define CCS811_REG_STATUS                        0x00
#define CCS811_REG_MEAS_MODE                     0x01
#define CCS811_REG_ALG_RESULT_DATA               0x02
#define CCS811_REG_RAW_DATA                      0x03
#define CCS811_REG_ENV_DATA                      0x05
#define CCS811_REG_NTC                           0x06
#define CCS811_REG_THRESHOLDS                    0x10
#define CCS811_REG_BASELINE                      0x11
#define CCS811_REG_HW_ID                         0x20
#define CCS811_REG_HW_VERSION                    0x21
#define CCS811_REG_FW_BOOT_VERSION               0x23
#define CCS811_REG_FW_APP_VERSION                0x24
#define CCS811_REG_INTERNAL_STATE                0xA0
#define CCS811_REG_ERROR_ID                      0xE0
#define CCS811_REG_SW_RESET                      0xFF

#define CCS811_BOOTLOADER_APP_ERASE              0xF1
#define CCS811_BOOTLOADER_APP_DATA               0xF2
#define CCS811_BOOTLOADER_APP_VERIFY             0xF3
#define CCS811_BOOTLOADER_APP_START              0xF4

#define CCS811_HW_ID                             0x81

/*  ------------------------------------Mo so struct -------------------------------------------  */
typedef struct {
	unsigned int CO2;
	unsigned int TVOC;
}CCS811_Data;

/*  -------------------------------------------------------------------------------  */
typedef enum{
        eClosed,      /**< Idle (Measurements are disabled in this mode) */
        eCycle_1s,    /**< Constant power mode, IAQ measurement every second */
        eCycle_10s,   /**< Pulse heating mode IAQ measurement every 10 seconds */
        eCycle_60s,   /**< Low power pulse heating mode IAQ measurement every 60 seconds */
        eCycle_250ms  /**< Constant power mode, sensor measurement every 250ms 1xx: Reserved modes (For future use) */
    }eCycle_t;


void writeReg(uint8_t reg, uint8_t *pBuf, size_t size);
void CCS811_softReset(void);
uint8_t readReg(uint8_t reg, uint8_t *pBuf, size_t size);
void setMeasurementMode(eCycle_t mode, uint8_t thresh, uint8_t interrupt);
void setInTempHum(float temperature, float humidity);
uint8_t CCS811_Init(void);
uint8_t CCS811_checkDataReady();
uint16_t CCS811_getCO2PPM();
uint16_t CCS811_getTVOCPPB();
void CCS811_writeBaseLine(uint16_t baseLine);


#endif /* INC_CCS811_H_ */
