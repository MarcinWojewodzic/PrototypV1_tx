/*
 * BME280.c
 *
 *  Created on: 2 kwi 2021
 *      Author: Marcin
 */
#include "main.h"
#include "BME280.h"
static void BME280_Write8(BME280_t *bme,uint8_t address,uint8_t data)
{
	HAL_I2C_Mem_Write(bme->bme_i2c, (bme->addr<<1), address, 1, &data, 1, BME_MAX_TIMEOUT);
}
static uint8_t BME280_Read8(BME280_t *bme,uint8_t address)
{
	uint8_t data;
	HAL_I2C_Mem_Read(bme->bme_i2c, (bme->addr<<1), address, 1, &data, 1, BME_MAX_TIMEOUT);
	return data;
}

uint32_t BME280_Read24(BME280_t *bme,uint8_t address)
{
	uint8_t data[3];
	HAL_I2C_Mem_Read(bme->bme_i2c, (bme->addr<<1), address, 1, data, 3, BME_MAX_TIMEOUT);
	return ((data[0] << 16) | (data[1] << 8) | data[2]);
}
static uint16_t BME280_Read16LE(BME280_t *bme,uint8_t address)
{
	uint8_t data[2];
	data[0]=BME280_Read8(bme, address);
	data[1]=BME280_Read8(bme, address+1);
	return ((data[1]<<8)|data[0]);
}
uint16_t BME280_Read16(BME280_t *bme,uint8_t address)
{
	uint8_t data[2];
	data[0]=BME280_Read8(bme, address);
	data[1]=BME280_Read8(bme, address+1);
	return ((data[0]<<8)|data[1]);
}
static float BME280_Read_temp(BME280_t *bme)
{
	int32_t var1, var2;
	  int32_t adc_T = BME280_Read24(bme,BME280_TEMPDATA);
	  if (adc_T == 0x800000)
		  return -99;

	  adc_T >>= 4;

	  var1  = ((((adc_T>>3) - ((int32_t)bme->t1 <<1))) *
			  ((int32_t)bme->t2)) >> 11;

	  var2  = (((((adc_T>>4) - ((int32_t)bme->t1)) *
			  ((adc_T>>4) - ((int32_t)bme->t1))) >> 12) *
			  ((int32_t)bme->t3)) >> 14;

	  bme->t_fine = var1 + var2;

	  float T  = (bme->t_fine * 5 + 128) >> 8;
	  return T/100;

	  return -99;

}
static float BME280_Read_hum(BME280_t *bme)
{

	int32_t adc_H = BME280_Read16(bme,BME280_HUMIDDATA);
	if (adc_H == 0x8000) // value in case humidity measurement was disabled
	   return -99; //error
	int32_t v_x1_u32r;

	v_x1_u32r = (bme->t_fine - ((int32_t)76800));

	  v_x1_u32r = (((((adc_H << 14) - (((int32_t)bme->h4) << 20) -
					  (((int32_t)bme->h5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
				   (((((((v_x1_u32r * ((int32_t)bme->h6)) >> 10) *
						(((v_x1_u32r * ((int32_t)bme->h3)) >> 11) + ((int32_t)32768))) >> 10) +
					  ((int32_t)2097152)) * ((int32_t)bme->h2) + 8192) >> 14));

	  v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
								 ((int32_t)bme->h1)) >> 4));

	  v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
	  v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;
	  float h = (v_x1_u32r>>12);
	  return  h / 1024.0;
}
static float BME280_Read_press(BME280_t *bme)
{
	int64_t var1, var2, p;

	// Must be done first to get the t_fine variable set up

	int32_t adc_P = BME280_Read24(bme,BME280_PRESSUREDATA);
	adc_P >>= 4;

	var1 = ((int64_t)bme->t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)bme->p6;
	var2 = var2 + ((var1*(int64_t)bme->p5)<<17);
	var2 = var2 + (((int64_t)bme->p4)<<35);
	var1 = ((var1 * var1 * (int64_t)bme->p3)>>8) +
			((var1 * (int64_t)bme->p2)<<12);
	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)bme->p1)>>33;

	if (var1 == 0) {
		return 0;  // avoid exception caused by division by zero
	}
	p = 1048576 - adc_P;
	p = (((p<<31) - var2)*3125) / var1;
	var1 = (((int64_t)bme->p9) * (p>>13) * (p>>13)) >> 25;
	var2 = (((int64_t)bme->p8) * p) >> 19;

	p = ((p + var1 + var2) >> 8) + (((int64_t)bme->p7)<<4);
	return (int32_t)p/256;
}
void BME280_Read_all(BME280_t *bme, float *T,float *P,float *H)
{
	*T=BME280_Read_temp(bme);
	*P=BME280_Read_press(bme)/100;
	*H=BME280_Read_hum(bme);
}
void BME280_Init(BME280_t *bme,I2C_HandleTypeDef *i2c,uint8_t address)
{
	bme->bme_i2c=i2c;
	bme->addr=address;

	bme->t1 = BME280_Read16LE(bme,BME280_DIG_T1);
	bme->t2 = BME280_Read16LE(bme,BME280_DIG_T2);
	bme->t3 = BME280_Read16LE(bme,BME280_DIG_T3);

	bme->p1 = BME280_Read16LE(bme,BME280_DIG_P1);
	bme->p2 = BME280_Read16LE(bme,BME280_DIG_P2);
	bme->p3 = BME280_Read16LE(bme,BME280_DIG_P3);
	bme->p4 = BME280_Read16LE(bme,BME280_DIG_P4);
	bme->p5 = BME280_Read16LE(bme,BME280_DIG_P5);
	bme->p6 = BME280_Read16LE(bme,BME280_DIG_P6);
	bme->p7 = BME280_Read16LE(bme,BME280_DIG_P7);
	bme->p8 = BME280_Read16LE(bme,BME280_DIG_P8);
	bme->p9 = BME280_Read16LE(bme,BME280_DIG_P9);

	bme->h1 = BME280_Read8(bme,BME280_DIG_H1);
	bme->h2 = BME280_Read16LE(bme,BME280_DIG_H2);
	bme->h3 = BME280_Read8(bme,BME280_DIG_H3);
	bme->h4 = ((BME280_Read8(bme,BME280_DIG_H4) << 4 ) | (BME280_Read8(bme,BME280_DIG_H4+1) & 0xF));
	bme->h5 = ((BME280_Read8(bme,BME280_DIG_H5+1) << 4) | (BME280_Read8(bme,BME280_DIG_H5) >> 4));
	bme->h6 = (int8_t)BME280_Read8(bme,BME280_DIG_H6);
	BME280_Write8(bme, BME280_HUM_CONTROL, 0b00000100);
	BME280_Write8(bme, BME280_CONTROL, 0b10110111);
	BME280_Write8(bme, BME280_CONFIG, 0b00000000);
}
