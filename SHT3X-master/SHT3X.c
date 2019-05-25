/*************************************************************************************
Srh读值，RH转换后的湿度值
Tsr读值, T(C)摄氏度,T(F)华氏度

RH= 100*Srh / 65535    

T(C) = -45 + 175 * Tsr / 65535
T(F) = -49 + 315 * Tsr / 65535

温湿度传感器都使用I2C1进行通迅 通信scl速率设置成50hz比较合适，很低,即使如此也有10%的错误率

***********************************************************************************/
#include "SHT3X.h"

#define SHTX_TC(St)	(175 * (float)St / 65535 -45)
#define SHTX_TF(St)  (315 * (float)St / 65535 -49)
#define SHTX_RH(St)  (100 * (float)St / 65535)

const uint8_t Measurementcommands[] = {0x2032,0x2024,0x202F,0x2130,0x2126,0x212D,0x2236,0x2220,0x222B,0x2334,0x2322,0x2329,0x2737,0x2721,0x272A};
static uint16_t SHT3X_crc8(unsigned char *addr,uint8_t num);

/***********************************************************************
* 描    述: SHT3X温湿度传感初始化
* 入口参数: read 函数指针（该函数第一个参数是传入i2c器件地址，第二个参数是传入寄存器地址，8位数据指针，数据长度）使用i2c读取一段数据的函数
		   write 函数指针使用i2c写入一段数据，
* 出口参数: 
* 附加信息: 
* 说    明: 由于SHTX是使用双字节地址，所以需要实现双地址的i2c接口，
  			不采用crc校验。降低i2c通信速率可以得到很好的数据
************************************************************************/
uint8_t SHT3X_init(SHT3X_DEV* base,
	void (*read)(uint8_t,uint16_t,uint8_t*,uint8_t),
	void (*write)(uint8_t,uint16_t,uint8_t*,uint8_t),
	SHT3X_DEVADDR dev_addr
	)
{
	base->read = read;
	base->write = write;
	base->badrh_crc_count = base->goodrh_crc_count = base->badtp_crc_count = base->goodtp_crc_count = 0;
	SHT3X_soft_reset(base);
	set_SHT3x_Periodic_mode(base, MPS_1HZ,REFRESH_HIGH);			//设置温湿度传感器输出方式		
	base->dev_addr = dev_addr;
	return 0;
}

//***********************************************************************************************
// 函 数 名 : Check_SHT3X
// 输入参数 : NONE
// 返回参数 : stauts 在线TRUE,不在线FALSH
// 说    明 : 查找温湿度传感器是否在线
//            随便读取一个SHT中的寄存器，若有返回 0：在线
//***********************************************************************************************
uint8_t Check_SHT3X(SHT3X_DEV* base)
{
	uint8_t ReadBuf[2];
	return 0;
	base->read(base->dev_addr,0xE000,ReadBuf,2);	
	if((ReadBuf[0] != 0) && (ReadBuf[0] != 0xff))
	{
		return 0;
	}
	return 1;
}

//***********************************************************************************************
// 函 数 名 : SHT3X_soft_reset
// 输入参数 : InExt(内外部湿度)
// 返回参数 : NONE
// 说    明 : 软件复位
//***********************************************************************************************
void SHT3X_soft_reset(SHT3X_DEV* base)
{
	uint8_t buf;
	base->write(base->dev_addr,0x30A2,&buf,0);
}
/**********************************************************************************************************
*	函 数 名: void SHT3X_temperature_humidity(SHT3X_DEV* base,float *TEMP_ADCVal,float *RH_ADCVal)
*	功能说明: 读取温度函数
*	传    参: float *TEMP_ADCVal 温度比如23.9摄氏度
			  float *RH_ADCVal 89 百分之89湿度
*	返 回 值: 0:成功采集数据1：失败
*   说    明: 
*********************************************************************************************************/
int SHT3X_temperature_humidity(SHT3X_DEV* base,float *temp_adcval,float *rh_adcval)
{
	uint8_t ReadBuf[8] = {0x00};	
	uint16_t temp = 0;
	uint16_t remp = 0;
	base->read(base->dev_addr,0xE000,ReadBuf,6);	
// (SHT3X_crc8(&ReadBuf[3],2) == ReadBuf[5])&&(SHT3X_crc8(&ReadBuf[0],2) == ReadBuf[2])

	temp = (ReadBuf[0]<<8) + ReadBuf[1];
	remp = (ReadBuf[3]<<8) + ReadBuf[4];

	if((SHT3X_crc8(&ReadBuf[0],2) == ReadBuf[2])){
		*temp_adcval = base->temperature = SHTX_TC(temp);
		base->goodtp_crc_count++;
	}else{
		base->badtp_crc_count++;

		*temp_adcval = 0;
		temp = 1;
	}
	if(SHT3X_crc8(&ReadBuf[3],2) == ReadBuf[5]){		
		*rh_adcval = base->humidity = SHTX_RH(remp);
		base->goodrh_crc_count++;
	}
	else{
		base->badrh_crc_count++;
		*rh_adcval = 0;
		temp = 1;
	}
	return temp;
}

//***********************************************************************************************
// 函 数 名 : set_SHT3x_Periodic_mode
// 输入参数 : mps(采样率),refresh(刷新率),InExt(内外部湿度)
// 返回参数 : NONE
// 说    明 : 设置周期性输出方式
//            刷新率越低,数值跳变越明显
//            mps,采样率有:0.5Hz,1Hz,2Hz,4Hz,10Hz(MPS_05HZ,MPS_1HZ,MPS_2HZ...)
//            refresh刷新率,高中低(REFRESH_HIGH,REFRESH_MID,REFRESH_LOW)
//***********************************************************************************************
void set_SHT3x_Periodic_mode(SHT3X_DEV* base,uint8_t mps,uint8_t refresh)			//设置周期性输出方式
{
	uint8_t buf[1];	
	uint8_t index = 0;
	index = (mps-MPS_05HZ)*3;
	index = index + refresh;
	if(index >= sizeof(Measurementcommands))
	{
		index = 0;
	}

	base->write(base->dev_addr,Measurementcommands[index],buf,0);
}


//*************************************************************************************************
// 函 数 名 : SHT3X_crc8
// 输入参数 : 参与运算的值addr,长度
// 返回参数 : CRC
// 说    明 : SHT3X CRC校验
// 						P(x) = x^8 + x^5 + x^4 + 1
//此CRC计算方式仅适用小数据量运算,多数据运算时,请使用查表方式
//***********************************************************************************************
static uint16_t SHT3X_crc8(unsigned char *addr,uint8_t num) 
{  
	uint8_t  i;  
	uint8_t crc =  0xFF;
	
	for (; num > 0; num--)              // Step through bytes in memory   
	{  
		crc ^= (*addr++);
		for (i = 0; i < 8; i++)           // Prepare to rotate 8 bits   
		{  
			if (crc & 0x80)            			// b7 is set...   
				crc = (crc << 1) ^  0x31; // P(x) = x^8 + x^5 + x^4 + 1 = 100110001
			else                          	// b7 is clear...   
				crc <<= 1;                  // just rotate   
		}                             		// Loop for 8 bits   
		crc &= 0xFF;                  		// Ensure CRC remains 16-bit value   
	}                               		// Loop until num=0   
	return(crc);                    		// Return updated CRC   
}

