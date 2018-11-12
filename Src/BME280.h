#ifndef BME280_H
#define BME280_H

// Include Your device specific libraries here
#include "main.h"
#include "stm32f0xx_hal.h"

#define BME280_HUM_LSB      0xFE
#define BME280_HUM_MSB      0xFD

#define BME280_TEMP_XLSB    0xFC
#define BME280_TEMP_LSB     0xFB
#define BME280_TEMP_MSB     0xFA

#define BME280_PRESS_XLSB   0xF9
#define BME280_PRESS_LSB    0xF8
#define BME280_PRESS_MSB    0xF7

#define BME280_CONFIG       0xF5
#define BME280_CTRL_MEAS    0xF4
#define BME280_STATUS       0xF3
#define BME280_CTRL_HUM     0xF2

#define BME280_RESET        0xE0

#define BME280_ID           0xD0


#define BME280_ID_READ_VALUE 0x60
#define BME280_RESET_BYTE    0xE0

#define BME280_COMPENASATION_T_1_3 0x88
#define BME280_COMPENASATION_P_1_9 0x8E

#define BME280_COMPENASATION_H_1   0xA1
#define BME280_COMPENASATION_H_2_6 0xE3

#define BME280_CTRL_HUM_DISABLE              0b000
#define BME280_CTRL_HUM_OVERSAMPLING_1       0b001
#define BME280_CTRL_HUM_OVERSAMPLING_2       0b010
#define BME280_CTRL_HUM_OVERSAMPLING_4       0b011
#define BME280_CTRL_HUM_OVERSAMPLING_8       0b100
#define BME280_CTRL_HUM_OVERSAMPLING_16      0b101

#define BME280_STATUS_MEASURING_MASK (1 << 3)
#define BME280_STATUS_IM_UPDATE_MASK (1 << 0)

#define BME280_CTRL_MEAS_PRESS_DISABLE              (0b000 << 2)
#define BME280_CTRL_MEAS_PRESS_OVERSAMPLING_1       (0b001 << 2)
#define BME280_CTRL_MEAS_PRESS_OVERSAMPLING_2       (0b010 << 2)
#define BME280_CTRL_MEAS_PRESS_OVERSAMPLING_4       (0b011 << 2)
#define BME280_CTRL_MEAS_PRESS_OVERSAMPLING_8       (0b100 << 2)
#define BME280_CTRL_MEAS_PRESS_OVERSAMPLING_16      (0b101 << 2)

#define BME280_CTRL_MEAS_TEMP_DISABLE               (0b000 << 5)
#define BME280_CTRL_MEAS_TEMP_OVERSAMPLING_1        (0b001 << 5)
#define BME280_CTRL_MEAS_TEMP_OVERSAMPLING_2        (0b010 << 5)
#define BME280_CTRL_MEAS_TEMP_OVERSAMPLING_4        (0b011 << 5)
#define BME280_CTRL_MEAS_TEMP_OVERSAMPLING_8        (0b100 << 5)
#define BME280_CTRL_MEAS_TEMP_OVERSAMPLING_16       (0b101 << 5)

#define BME280_CTRL_MEAS_MODE_MASK   0b11
#define BME280_CTRL_MEAS_MODE_SLEEP  0b00
#define BME280_CTRL_MEAS_MODE_FORCED 0b01
#define BME280_CTRL_MEAS_MODE_NORMAL 0b11

#define BME280_CONFIG_T_SB_0_5MS  (0b000 << 5)
#define BME280_CONFIG_T_SB_10MS   (0b110 << 5)
#define BME280_CONFIG_T_SB_20MS   (0b111 << 5)
#define BME280_CONFIG_T_SB_62_5MS (0b001 << 5)
#define BME280_CONFIG_T_SB_125MS  (0b010 << 5)
#define BME280_CONFIG_T_SB_250MS  (0b011 << 5)
#define BME280_CONFIG_T_SB_500MS  (0b100 << 5)
#define BME280_CONFIG_T_SB_1000MS (0b101 << 5)

#define BME280_CONFIG_FILTER_OFF  (0b000 << 2)
#define BME280_CONFIG_FILTER_X2   (0b001 << 2)
#define BME280_CONFIG_FILTER_X4   (0b010 << 2)
#define BME280_CONFIG_FILTER_X8   (0b011 << 2)
#define BME280_CONFIG_FILTER_X16  (0b100 << 2)

#define BME280_CONFIG_SPI3W_ENABLE  1
#define BME280_CONFIG_SPI3W_DISABLE 0


uint8_t BME280whoAmI(uint8_t device_id);
void BME280setMod(uint8_t device_id, uint8_t mod);

void BME280initTemperatureCompensationTable(uint8_t device_id);
void BME280initPressureCompensationTable(uint8_t device_id);
void BME280initHumidityCompensationTable(uint8_t device_id);
void BME280initallCompensationTable(uint8_t device_id);

float BME280readTemperature(uint8_t device_id, uint8_t reg_addr);
uint32_t BME280readTemperatureData(I2C_HandleTypeDef *hi2c, uint8_t device_id, uint8_t reg_addr);

float BME280readPressure(uint8_t device_id);
uint32_t BME280readPressureData(I2C_HandleTypeDef *hi2c, uint8_t device_id);

float BME280readHumidity(I2C_HandleTypeDef *hi2c, uint8_t device_id);
uint16_t BME280readHumidityData(I2C_HandleTypeDef *hi2c, uint8_t device_id);



uint8_t BME280readByte(uint8_t device_addr, uint8_t reg_addr);
uint16_t BME280readWord(uint8_t device_addr, uint8_t reg_addr);
uint32_t BME280readDWord(I2C_HandleTypeDef *hi2c,uint8_t device_addr, uint8_t reg_addr);

void BME280writeByte(uint8_t device_addr, uint8_t reg_addr, uint8_t data);

#endif /* BME280_H */
