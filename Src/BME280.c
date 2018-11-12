#include "BME280.h"
// #include "main.h"
 #include "stm32f0xx_hal.h"
 #include "stm32f0xx_hal_i2c.h"

static uint16_t BME280_temp_calib[3];
static uint16_t BME280_press_calib[9];
static uint16_t BME280_hum_calib[6];

//I2C_HandleTypeDef hi2c1;

static int32_t BME280_t_fine = 0;

void BME280initallCompensationTable(uint8_t device_id)
{
  BME280initTemperatureCompensationTable(device_id);
  BME280initPressureCompensationTable(device_id);
  BME280initHumidityCompensationTable(device_id);
}

void BME280initTemperatureCompensationTable(uint8_t device_id)
{
  uint16_t tmp_word = 0;
  for(int i = 0; i < 3; i++)
  {
    tmp_word = BME280readWord(device_id, BME280_COMPENASATION_T_1_3 + (2 * i));
    BME280_temp_calib[i] = ((tmp_word >> 8) & 0xFF) | (tmp_word << 8);
  }
}

void BME280initPressureCompensationTable(uint8_t device_id)
{
  uint16_t tmp_word = 0;
  for(int i = 0; i < 9; i++)
  {
    tmp_word = BME280readWord(device_id, BME280_COMPENASATION_P_1_9 + (2 * i));
    BME280_press_calib[i] = ((tmp_word >> 8) & 0xFF) | (tmp_word << 8);
  }
}

void BME280initHumidityCompensationTable(uint8_t device_id)
{
  uint16_t tmp_word = 0;
  BME280_hum_calib[0] = BME280readByte(device_id, BME280_COMPENASATION_H_1);
  tmp_word = BME280readWord(device_id, BME280_COMPENASATION_H_2_6);
  BME280_hum_calib[1] = ((tmp_word >> 8) & 0xFF) | (tmp_word << 8);;
  BME280_hum_calib[2] = BME280readByte(device_id, BME280_COMPENASATION_H_2_6 + 2);

  tmp_word = BME280readWord(device_id, BME280_COMPENASATION_H_2_6 + 3);
  BME280_hum_calib[3] = (tmp_word & 0xF0) | ((tmp_word & 0xFF) >> 4);

  tmp_word = BME280readWord(device_id, BME280_COMPENASATION_H_2_6 + 4);
  BME280_hum_calib[4] = ((tmp_word & 0x00FF) << 4) | ((tmp_word & 0xF000) >> 12);
	BME280_hum_calib[5] = BME280readByte(device_id, BME280_COMPENASATION_H_2_6 + 6);
}


uint8_t BME280whoAmI(uint8_t device_id)
{
  uint8_t retval = 0xFF;
  retval = BME280readByte(device_id, BME280_ID);
  return retval;
}

void BME280setMod(uint8_t device_id, uint8_t mod)
{
  uint8_t cotrl_reg_val = BME280readByte(device_id, BME280_CTRL_MEAS);
  // cotrl_reg_val |= mod & BME280_CTRL_MEAS_MODE_MASK;
  BME280writeByte(device_id, BME280_CTRL_MEAS, cotrl_reg_val);
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
  uint16_t raw_hum = BME280readWord(device_id, BME280_HUM_MSB);
  return raw_hum;
}

float BME280readTemperature(uint8_t device_id, uint8_t reg_addr)
{
  float real_temp = 8000;
  int32_t raw_temp = (int32_t)BME280readTemperatureData(&hi2c1, device_id, reg_addr);
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

float BME280readPressure(uint8_t device_id)
{
  float real_pressure = -1;
  uint32_t real_int_press = 0;
  int32_t raw_press = (int32_t)BME280readPressureData(&hi2c1, device_id);
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
  int32_t raw_hum = (int32_t)BME280readHumidityData(&hi2c1, device_id);
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

uint8_t BME280readByte(uint8_t device_addr, uint8_t reg_addr)
{
  uint8_t ret_byte = 0;
  // Insert your code here
  return ret_byte;
}

uint16_t BME280readWord(uint8_t device_addr, uint8_t reg_addr)
{
  uint16_t ret_word = 0;
  // Insert your code here
  return ret_word;
}

uint32_t BME280readDWord(I2C_HandleTypeDef *hi2c,uint8_t device_addr, uint8_t reg_addr)
{
//	while(HAL_I2C_Mem_Write(&hi2c1, device_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, 0, 2, HAL_MAX_DELAY)!=HAL_ERROR) {;};	
  uint32_t ret_dword = 0;
  // Insert your code here
	//while(HAL_I2C_Mem_Read(&hi2c1, device_addr, 0x0, I2C_MEMADD_SIZE_8BIT, (uint8_t *) ret_dword, 6, HAL_MAX_DELAY)!=HAL_ERROR) {;};
  return ret_dword;
}

void BME280writeByte(uint8_t device_addr, uint8_t reg_addr, uint8_t data)
{
	// while(HAL_I2C_Mem_Write(&hi2c1, device_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, &data, 2, HAL_MAX_DELAY)!=HAL_ERROR) {;};	
  // Insert your code here
}
