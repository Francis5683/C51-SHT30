#ifndef __SHT3x_H__
#define __SHT3x_H__
//#include "stm32l0xx_hal.h"
#include "stm8l15x.h"
#define MPS_05HZ  	1			//2S1次
#define MPS_1HZ  	2			//1S1次
#define MPS_2HZ 	3			//1S2次
#define MPS_4HZ  	4			//1S4次
#define MPS_10HZ  	5			//1S10次

#define REFRESH_HIGH	1		//刷新率高(重复率高)
#define REFRESH_MID 	2		//刷新率中(重复率中)
#define REFRESH_LOW 	3		//刷新率低(重复率低)

typedef void (*sht_read)(uint8_t i2caddr,uint16_t regaddr,uint8_t*databuf,uint8_t datalen);
typedef void (*sht_write)(uint8_t i2caddr,uint16_t regaddr,uint8_t*databuf,uint8_t datalen);
typedef enum {DEV0X44 = 0X44,DEV0X45 = 0X45}SHT3X_DEVADDR;
typedef struct SHT3X_DEV
{
	sht_read read;
	sht_write write;
	SHT3X_DEVADDR dev_addr;
	float temperature;
	float humidity;
	uint16_t badrh_crc_count;
	uint16_t goodrh_crc_count;
	uint16_t badtp_crc_count;
	uint16_t goodtp_crc_count;
}SHT3X_DEV;
//------------------------------------------------------------------------
uint8_t SHT3X_init(SHT3X_DEV* base,
	void (*read)(uint8_t,uint16_t,uint8_t*,uint8_t),
	void (*write)(uint8_t,uint16_t,uint8_t*,uint8_t),
	SHT3X_DEVADDR dev_addr
	);
uint8_t check_sensor_SHT(SHT3X_DEV* base);
void SHT3X_soft_reset(SHT3X_DEV* base);
int SHT3X_temperature_humidity(SHT3X_DEV* base,float *temp_adcval,float *rh_adcval);
void set_SHT3x_Periodic_mode(SHT3X_DEV* base,uint8_t mps,uint8_t refresh);			//设置周期性输出方式;


//------------------------------------------------------------------------

#endif

