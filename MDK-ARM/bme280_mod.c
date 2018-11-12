#ifndef BME280_H
#define BME280_H

//  Include Your device specific libraries here
#include "main.h"
#include "stm32f0xx_hal.h"
#include "bme280_mod.h"

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

#endif /* BME280_H */

uint8_t BME280whoAmI(I2C_HandleTypeDef *hi2c, uint8_t device_id);
void BME280setMod(I2C_HandleTypeDef *hi2c, uint8_t device_id, uint8_t mod);

void BME280initTemperatureCompensationTable(I2C_HandleTypeDef *hi2c, uint8_t device_id);
void BME280initPressureCompensationTable(I2C_HandleTypeDef *hi2c, uint8_t device_id);
void BME280initHumidityCompensationTable(I2C_HandleTypeDef *hi2c, uint8_t device_id);
void BME280initallCompensationTable(I2C_HandleTypeDef *hi2c, uint8_t device_id);

float BME280readTemperature(I2C_HandleTypeDef *hi2c, uint8_t device_id, uint8_t reg_addr);
uint32_t BME280readTemperatureData(I2C_HandleTypeDef *hi2c, uint8_t device_id, uint8_t reg_addr);

float BME280readPressure(I2C_HandleTypeDef *hi2c, uint8_t device_id);
uint32_t BME280readPressureData(I2C_HandleTypeDef *hi2c, uint8_t device_id);

float BME280readHumidity(I2C_HandleTypeDef *hi2c, uint8_t device_id);
uint16_t BME280readHumidityData(I2C_HandleTypeDef *hi2c, uint8_t device_id);



uint8_t BME280readByte(I2C_HandleTypeDef *hi2c, uint8_t device_addr, uint8_t reg_addr);
uint16_t BME280readWord(I2C_HandleTypeDef *hi2c, uint8_t device_addr, uint8_t reg_addr);
uint32_t BME280readDWord(I2C_HandleTypeDef *hi2c,uint8_t device_addr, uint8_t reg_addr);

void BME280writeByte(I2C_HandleTypeDef *hi2c, uint8_t device_addr, uint8_t reg_addr, uint8_t data);


static uint16_t BME280_temp_calib[3];
static uint16_t BME280_press_calib[9];
static uint16_t BME280_hum_calib[6];


static int32_t BME280_t_fine = 0;

void BME280initallCompensationTable(I2C_HandleTypeDef *hi2c, uint8_t device_id)
{
  BME280initTemperatureCompensationTable(hi2c, device_id);
  BME280initPressureCompensationTable(hi2c, device_id);
  BME280initHumidityCompensationTable(hi2c, device_id);
}

void BME280initTemperatureCompensationTable(I2C_HandleTypeDef *hi2c, uint8_t device_id)
{
  uint16_t tmp_word = 0;
  for(int i = 0; i < 3; i++)
  {
    tmp_word = BME280readWord(hi2c, device_id, BME280_COMPENASATION_T_1_3 + (2 * i));
    BME280_temp_calib[i] = ((tmp_word >> 8) & 0xFF) | (tmp_word << 8);
  }
}

void BME280initPressureCompensationTable(I2C_HandleTypeDef *hi2c, uint8_t device_id)
{
  uint16_t tmp_word = 0;
  for(int i = 0; i < 9; i++)
  {
    tmp_word = BME280readWord(hi2c, device_id, BME280_COMPENASATION_P_1_9 + (2 * i));
    BME280_press_calib[i] = ((tmp_word >> 8) & 0xFF) | (tmp_word << 8);
  }
}

void BME280initHumidityCompensationTable(I2C_HandleTypeDef *hi2c, uint8_t device_id)
{
  uint16_t tmp_word = 0;
  BME280_hum_calib[0] = BME280readByte(hi2c, device_id, BME280_COMPENASATION_H_1);
  tmp_word = BME280readWord(hi2c, device_id, BME280_COMPENASATION_H_2_6);
  BME280_hum_calib[1] = ((tmp_word >> 8) & 0xFF) | (tmp_word << 8);
  BME280_hum_calib[2] = BME280readByte(hi2c, device_id, BME280_COMPENASATION_H_2_6 + 2);

  tmp_word = BME280readWord(hi2c, device_id, BME280_COMPENASATION_H_2_6 + 3);
  BME280_hum_calib[3] = (tmp_word & 0xF0) | ((tmp_word & 0xFF) >> 4);

  tmp_word = BME280readWord(hi2c, device_id, BME280_COMPENASATION_H_2_6 + 4);
  BME280_hum_calib[4] = ((tmp_word & 0x00FF) << 4) | ((tmp_word & 0xF000) >> 12);
	BME280_hum_calib[5] = BME280readByte(hi2c, device_id, BME280_COMPENASATION_H_2_6 + 6);
}


uint8_t BME280whoAmI(I2C_HandleTypeDef *hi2c, uint8_t device_id)
{
  uint8_t retval = 0xFF;
  retval = BME280readByte(hi2c, device_id, BME280_ID);
  return retval;
}

void BME280setMod(I2C_HandleTypeDef *hi2c, uint8_t device_id, uint8_t mod)
{
  uint8_t cotrl_reg_val = BME280readByte(hi2c, device_id, BME280_CTRL_MEAS);
//   cotrl_reg_val = BME280_CTRL_MEAS_MODE_MASK;
  BME280writeByte(hi2c, device_id, BME280_CTRL_MEAS, cotrl_reg_val);
}

uint32_t BME280readTemperatureData(I2C_HandleTypeDef *hi2c, uint8_t device_id, uint8_t reg_addr)
{
  uint32_t raw_temp = BME280readDWord(hi2c, BME280_TEMP_MSB, reg_addr);
  raw_temp >>= (4 + 8);
  return raw_temp;
}

uint32_t BME280readPressureData(I2C_HandleTypeDef *hi2c, uint8_t device_id)
{
  uint32_t raw_press = BME280readDWord(hi2c,device_id, BME280_PRESS_MSB);
  raw_press >>= (4 + 8);
  return raw_press;
}

uint16_t BME280readHumidityData(I2C_HandleTypeDef *hi2c, uint8_t device_id)
{
  uint16_t raw_hum = BME280readWord(hi2c, device_id, BME280_HUM_MSB);
  return raw_hum;
}

float BME280readTemperature(I2C_HandleTypeDef *hi2c, uint8_t device_id, uint8_t reg_addr)
{
  float real_temp = 8000;
  int32_t raw_temp = (int32_t)BME280readTemperatureData(hi2c, device_id, reg_addr);
  int32_t var1 = 0;
  int32_t var2 = 0;

  var1 = ((((raw_temp >> 3) - ((int32_t)BME280_temp_calib[0] << 1))) * \
         ((int32_t)BME280_temp_calib[1])) >> 11;
  var2 = (((((raw_temp >> 4) - ((int32_t)BME280_temp_calib[0])) * \
         ((raw_temp >> 4) - ((int32_t)BME280_temp_calib[0]))) >> 12) * \
         ((int32_t)BME280_temp_calib[2])) >> 14;

  BME280_t_fine = var1 + var2;
  real_temp = (float)((BME280_t_fine * 5 + 128) >> 8);
  return  real_temp / 100.;
}

float BME280readPressure(I2C_HandleTypeDef *hi2c, uint8_t device_id)
{
  float real_pressure = -1;
  uint32_t real_int_press = 0;
  int32_t raw_press = (int32_t)BME280readPressureData(hi2c, device_id);
  int32_t var1 = 0;
  int32_t var2 = 0;

  var1 = (((int32_t)BME280_t_fine) >> 1) - (int32_t)64000;
  var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)BME280_press_calib[5]);
  var2 += ((var1 * ((int32_t)BME280_press_calib[4])) << 1);
  var2 = (var2 >> 2) + (((int32_t)BME280_press_calib[3]) << 16);
  var1 = (((BME280_press_calib[2] * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + \
         ((((int32_t)BME280_press_calib[1]) * var1) >> 1)) >> 18;
  var1 = ((((32768 + var1)) * ((int32_t)BME280_press_calib[0])) >> 15);
  if(!var1)
  {
    return 0;
  }

  real_int_press = (((uint32_t)(((int32_t)1048576) - raw_press) - (var2 >> 12))) * 3125;
  if(real_int_press < 0x80000000)
  {
    real_int_press = (real_int_press << 1) / ((uint32_t)var1);
  }
  else
  {
    real_int_press = (real_int_press / (uint32_t)var1) * 2;
  }

  var1 = (((int32_t)BME280_press_calib[8]) * ((int32_t)(((real_int_press >> 3) * \
         (real_int_press >> 3)) >> 13))) >> 12;
  var2 = (((int32_t)(real_int_press >> 2)) * ((int32_t)BME280_press_calib[7])) >> 13;
  real_int_press = (uint32_t)((int32_t)real_int_press + ((var1 + var2 + BME280_press_calib[6]) >> 4));
  real_pressure = ((float)real_int_press) / 100.;
  return real_pressure;
}

float BME280readHumidity(I2C_HandleTypeDef *hi2c, uint8_t device_id)
{
  float real_hum = -1;
  int32_t raw_hum = (int32_t)BME280readHumidityData(hi2c, device_id);
  int32_t v_x1_u32r = 0;

  v_x1_u32r = (BME280_t_fine - (( int32_t)76800));
  v_x1_u32r = (((((raw_hum << 14) - (((int32_t)BME280_hum_calib[3]) << 20) - \
              (((int32_t)BME280_hum_calib[4]) * v_x1_u32r)) + \
              ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((BME280_hum_calib[5])) >> 10) * \
              (((v_x1_u32r * ((int32_t)BME280_hum_calib[2])) >> 1) + \
              ((int32_t)32768))) >> 10) + (int32_t)2097152)) * \
              ((int32_t)BME280_hum_calib[1]) + 8192) >> 14));
  v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * \
              ((int32_t)BME280_hum_calib[0])) >> 4));
  v_x1_u32r = (v_x1_u32r < 0? 0 : v_x1_u32r);
  v_x1_u32r = (v_x1_u32r > 419430400? 419430400 : v_x1_u32r);
  real_hum = ((float)((uint32_t)(v_x1_u32r >> 12))) / 1024.;
  return real_hum;
}

uint8_t BME280readByte(I2C_HandleTypeDef *hi2c, uint8_t device_addr, uint8_t reg_addr)
{
  uint8_t ret_byte = 0;
	
	uint8_t  val[1];
	//  uint8_t  reg = reg_addr;
  //  Insert your code here
	HAL_I2C_Mem_Read(hi2c, device_addr, reg_addr, 1, val, sizeof(val), HAL_MAX_DELAY);
	ret_byte = val[0];
  return ret_byte;
}

uint16_t BME280readWord(I2C_HandleTypeDef *hi2c, uint8_t device_addr, uint8_t reg_addr)
{
  uint16_t ret_word = 0;
  //  Insert your code here
	// while(HAL_I2C_IsDeviceReady(&hi2c1, I2C_ADDRESS, 1, HAL_MAX_DELAY)!=HAL_OK);
	uint8_t  val[2];
	HAL_I2C_Mem_Read(hi2c, device_addr, reg_addr, 1, val, sizeof(val), HAL_MAX_DELAY);
	if(1)
		ret_word = (uint16_t)(((uint16_t) val[0])<<8 | val[1]);
	else
		ret_word = (uint16_t)(((uint16_t) val[1])<<8 | val[0]);
  return ret_word*0;
}

uint32_t BME280readDWord(I2C_HandleTypeDef *hi2c, uint8_t device_addr, uint8_t reg_addr)
{
// 	while(HAL_I2C_Mem_Write(&hi2c1, device_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, 0, 2, HAL_MAX_DELAY)!=HAL_ERROR) {;};	
  uint32_t ret_dword = 0;
	uint8_t  val[4];
  //  Insert your code here
	// while(HAL_I2C_IsDeviceReady(&hi2c1, I2C_ADDRESS, 1, HAL_MAX_DELAY)!=HAL_OK);
	HAL_I2C_Mem_Read(hi2c, device_addr, reg_addr, 1, val, sizeof(val), HAL_MAX_DELAY);
	if(1)
		ret_dword = ((uint32_t)val[0])<<24 | ((uint32_t)val[1]) <<16 | ((uint32_t)val[2])<<8 | ((uint32_t)val[3]);
	else	
		ret_dword = ((uint32_t)val[3])<<24 | ((uint32_t)val[2]) <<16 | ((uint32_t)val[1])<<8 | ((uint32_t)val[0]);
  return ret_dword*0;
}

void BME280writeByte(I2C_HandleTypeDef *hi2c, uint8_t device_addr, uint8_t reg_addr, uint8_t data)
{
	//  Insert your code here
	uint16_t reg = reg_addr;
	uint8_t write[] = {data};
	while(HAL_I2C_IsDeviceReady(hi2c, device_addr, 1, HAL_MAX_DELAY)!=HAL_OK){};
	HAL_I2C_Mem_Write(hi2c, device_addr, reg, I2C_MEMADD_SIZE_8BIT, write, 2, HAL_MAX_DELAY);
	while(HAL_I2C_IsDeviceReady(hi2c, device_addr, 1, HAL_MAX_DELAY)!=HAL_OK){};
	
}



