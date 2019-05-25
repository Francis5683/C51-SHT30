/* Includes ------------------------------------------------------------------*/
#include "main.h"

static __IO uint32_t msCounter;
uint32_t msCounterBak;
uint32_t oneSecond = 0;

//-- Global variables ----------------------------------------------------------
u8t _i2cWriteHeader;
u8t _i2cReadHeader;

uint32_t GPIO_Pin;
GPIO_InitTypeDef GPIO_InitStructure;

union{
  float f;
  char  buf[4];
}temperature;

uint32_t temperature32;
uint8_t temperatureS[9];
union{
  float f;
  char  buf[4];
}humidity;
uint32_t humidity32;
uint8_t humidityS[9];


uint8_t UART1TxBuf[256];

uint8_t UART1Value;
uint16_t UART1TxIn = 0;
uint16_t UART1TxOut = 0;

void DelayMicroSeconds(u32t nbrOfUs)
{   /* -- adapt this delay for your uC -- */
//==============================================================================
  u32t i;
  //for(i = 0; i < nbrOfUs; i++)
  //{  
  //  NOP();  // nop's may be added or removed for timing adjustment
  //  NOP();
  //  NOP();
  //  NOP();
  //}
  
	static uint32_t delayTime;
	while (nbrOfUs > 0)
	{
  	for (delayTime = 0; delayTime < DELAY_1US_CLOCK; delayTime++)
  	{
  		NOP();
  	}
  	nbrOfUs--;
  }
}

//==============================================================================
void I2c_StartCondition(void){
//==============================================================================
  SDA_OPEN();
  DelayMicroSeconds(1);
  SCL_OPEN();
  DelayMicroSeconds(1);
  SDA_LOW();
  DelayMicroSeconds(10);  // hold time start condition (t_HD;STA)
  SCL_LOW();
  DelayMicroSeconds(10);
}

//==============================================================================
void I2c_StopCondition(void){
//==============================================================================
  SCL_LOW();
  DelayMicroSeconds(1);
  SDA_LOW();
  DelayMicroSeconds(1);
  SCL_OPEN();
  DelayMicroSeconds(10);  // set-up time stop condition (t_SU;STO)
  SDA_OPEN();
  DelayMicroSeconds(10);
}

//==============================================================================
etError I2c_WriteByte(u8t txByte){
//==============================================================================
  etError error = NO_ERROR;
  u8t     mask;
  for(mask = 0x80; mask > 0; mask >>= 1)// shift bit for masking (8 times)
  {
    if((mask & txByte) == 0) SDA_LOW(); // masking txByte, write bit to SDA-Line
    else                     SDA_OPEN();
    DelayMicroSeconds(1);               // data set-up time (t_SU;DAT)
    SCL_OPEN();                         // generate clock pulse on SCL
    DelayMicroSeconds(5);               // SCL high time (t_HIGH)
    SCL_LOW();
    DelayMicroSeconds(1);               // data hold time(t_HD;DAT)
  }
  SDA_OPEN();                           // release SDA-line
  SCL_OPEN();                           // clk #9 for ack
  DelayMicroSeconds(1);                 // data set-up time (t_SU;DAT)
  if(SDA_READ) error = ACK_ERROR;       // check ack from i2c slave
  SCL_LOW();
  DelayMicroSeconds(20);                // wait to see byte package on scope
  return error;                         // return error code
}

//==============================================================================
etError I2c_ReadByte(u8t *rxByte, etI2cAck ack, u8t timeout){
//==============================================================================
  etError error = NO_ERROR;
  u8t mask;
  *rxByte = 0x00;
  SDA_OPEN();                            // release SDA-line
  for(mask = 0x80; mask > 0; mask >>= 1) // shift bit for masking (8 times)
  { 
    SCL_OPEN();                          // start clock on SCL-line
    DelayMicroSeconds(1);                // clock set-up time (t_SU;CLK)
		error = I2c_WaitWhileClockStreching(timeout);// wait while clock streching
    DelayMicroSeconds(3);                // SCL high time (t_HIGH)
    if(SDA_READ) *rxByte |= mask;        // read bit
    SCL_LOW();
    DelayMicroSeconds(1);                // data hold time(t_HD;DAT)
  }
  if(ack == ACK) SDA_LOW();              // send acknowledge if necessary
  else           SDA_OPEN();
  DelayMicroSeconds(1);                  // data set-up time (t_SU;DAT)
  SCL_OPEN();                            // clk #9 for ack
  DelayMicroSeconds(5);                  // SCL high time (t_HIGH)
  SCL_LOW();
  SDA_OPEN();                            // release SDA-line
  DelayMicroSeconds(20);                 // wait to see byte package on scope
	
	return error;                          // return with no error
}

//==============================================================================
etError I2c_WaitWhileClockStreching(u8t timeout){
//==============================================================================
	etError error = NO_ERROR;
	
  while(SCL_READ == 0)
  {
    if(timeout-- == 0)	return TIMEOUT_ERROR;
    //DelayMicroSeconds(1000);
    DelayMs(1);
  }
	
	return error;
}

//==============================================================================
etError I2c_GeneralCallReset(void){
//==============================================================================
	etError error;
	
	I2c_StartCondition();
	                      error = I2c_WriteByte(0x00);
	if(error == NO_ERROR) error = I2c_WriteByte(0x06);
	
	return error;
}


//==============================================================================
void SHT3X_Init(u8t i2cAdr){              /* -- adapt the init for your uC -- */
//==============================================================================
	// init I/O-pins
//	RCC->APB2ENR |= 0x00000008;  // I/O port B clock enabled
	
//  // Alert on port B, bit 10
//	GPIOB->CRH   &= 0xFFFFF0FF;  // set floating input for Alert-Pin
//  GPIOB->CRH   |= 0x00000400;  //
	
  // Reset on port B, bit 12
//	GPIOB->CRH   &= 0xFFF0FFFF;  // set push-pull output for Reset pin
//  GPIOB->CRH   |= 0x00010000;  //
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  
	RESET_LOW();
	
  //I2c_Init(); // init I2C
        
  
  // SDA on port B, bit 14
  // SCL on port B, bit 13
  GPIO_InitStructure.GPIO_Pin = (GPIO_Pin_13 | GPIO_Pin_14);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  SDA_OPEN();                  // I2C-bus idle mode SDA released
  SCL_OPEN();                  // I2C-bus idle mode SCL released
  SHT3X_SetI2cAdr(i2cAdr);
	
	// release reset
	RESET_HIGH();
}

//==============================================================================
void SHT3X_SetI2cAdr(u8t i2cAdr){
//==============================================================================
  _i2cWriteHeader = i2cAdr << 1;
  _i2cReadHeader = _i2cWriteHeader | 0x01;
}

//==============================================================================
etError SHT3x_ReadSerialNumber(u32t *serialNbr){
//==============================================================================
  etError error; // error code
  u16t serialNumWords[2];

  error = SHT3X_StartWriteAccess();

  // write "read serial number" command
  error |= SHT3X_WriteCommand(CMD_READ_SERIALNBR);
  // if no error, start read access
  if(error == NO_ERROR) error = SHT3X_StartReadAccess();
  // if no error, read first serial number word
  if(error == NO_ERROR) error = SHT3X_Read2BytesAndCrc(&serialNumWords[0], ACK, 100);
  // if no error, read second serial number word
  if(error == NO_ERROR) error = SHT3X_Read2BytesAndCrc(&serialNumWords[1], NACK, 0);

  SHT3X_StopAccess();
  
  // if no error, calc serial number as 32-bit integer
  if(error == NO_ERROR)
  {
    *serialNbr = (serialNumWords[0] << 16) | serialNumWords[1];
  }

  return error;
}

//==============================================================================
etError SHT3X_ReadStatus(u16t *status){
//==============================================================================
  etError error; // error code

  error = SHT3X_StartWriteAccess();

  // if no error, write "read status" command
  if(error == NO_ERROR) error = SHT3X_WriteCommand(CMD_READ_STATUS);
  // if no error, start read access
  if(error == NO_ERROR) error = SHT3X_StartReadAccess(); 
  // if no error, read status
  if(error == NO_ERROR) error = SHT3X_Read2BytesAndCrc(status, NACK, 0);

  SHT3X_StopAccess();

  return error;
}

//==============================================================================
etError SHT3X_ClearAllAlertFlags(void){
//==============================================================================
  etError error; // error code

  error = SHT3X_StartWriteAccess();

  // if no error, write clear status register command
  if(error == NO_ERROR) error = SHT3X_WriteCommand(CMD_CLEAR_STATUS);

  SHT3X_StopAccess();

  return error;
}

//==============================================================================
etError SHT3X_GetTempAndHumi(ft         *temp,
                             ft         *humi,
                             etRepeatab repeatab,
                             etMode     mode,
                             u8t        timeout){
//==============================================================================
  etError error;
                               
  switch(mode)
  {    
    case MODE_CLKSTRETCH: // get temperature with clock stretching mode
      error = SHT3X_GetTempAndHumiClkStretch(temp, humi, repeatab, timeout); break;
    case MODE_POLLING:    // get temperature with polling mode
      error = SHT3X_GetTempAndHumiPolling(temp, humi, repeatab, timeout); break;
    default:              
      error = PARM_ERROR; break;
  }
  
  return error;
}


//==============================================================================
etError SHT3X_GetTempAndHumiClkStretch(ft         *temp,
                                       ft         *humi,
                                       etRepeatab repeatab,
                                       u8t        timeout){
//==============================================================================
  etError error;        // error code
  u16t    rawValueTemp; // temperature raw value from sensor
  u16t    rawValueHumi; // humidity raw value from sensor

  error = SHT3X_StartWriteAccess();

  // if no error ...
  if(error == NO_ERROR)
  {
    // start measurement in clock stretching mode
    // use depending on the required repeatability, the corresponding command
    switch(repeatab)
    {
      case REPEATAB_LOW:    error = SHT3X_WriteCommand(CMD_MEAS_CLOCKSTR_L); break;
      case REPEATAB_MEDIUM: error = SHT3X_WriteCommand(CMD_MEAS_CLOCKSTR_M); break;
      case REPEATAB_HIGH:   error = SHT3X_WriteCommand(CMD_MEAS_CLOCKSTR_H); break;
      default:              error = PARM_ERROR; break;
    }
  }

  // if no error, start read access
  if(error == NO_ERROR) error = SHT3X_StartReadAccess();
  // if no error, read temperature raw values
  if(error == NO_ERROR) error = SHT3X_Read2BytesAndCrc(&rawValueTemp, ACK, timeout);
  // if no error, read humidity raw values
  if(error == NO_ERROR) error = SHT3X_Read2BytesAndCrc(&rawValueHumi, NACK, 0);

  SHT3X_StopAccess();

  // if no error, calculate temperature in 癈 and humidity in %RH
  if(error == NO_ERROR)
  {
    *temp = SHT3X_CalcTemperature(rawValueTemp);
    *humi = SHT3X_CalcHumidity(rawValueHumi);
  }

  return error;
}

//==============================================================================
etError SHT3X_GetTempAndHumiPolling(ft         *temp,
                                    ft         *humi,
                                    etRepeatab repeatab,
                                    u8t        timeout){
//==============================================================================
  etError error;           // error code
  u16t    rawValueTemp;    // temperature raw value from sensor
  u16t    rawValueHumi;    // humidity raw value from sensor

  error  = SHT3X_StartWriteAccess();

  // if no error ...
  if(error == NO_ERROR)
  {
    // start measurement in polling mode
    // use depending on the required repeatability, the corresponding command
    switch(repeatab)
    {
      case REPEATAB_LOW:    error = SHT3X_WriteCommand(CMD_MEAS_POLLING_L); break;
      case REPEATAB_MEDIUM: error = SHT3X_WriteCommand(CMD_MEAS_POLLING_M); break;
      case REPEATAB_HIGH:		error = SHT3X_WriteCommand(CMD_MEAS_POLLING_H); break;
      default:         			error = PARM_ERROR; break;
    }
  }

  // if no error, wait until measurement ready
  if(error == NO_ERROR)
  {
    // poll every 1ms for measurement ready until timeout
    while(timeout--)
    {
      // check if the measurement has finished
      error = SHT3X_StartReadAccess();

      // if measurement has finished -> exit loop
      if(error == NO_ERROR) break;

      // delay 1ms
      DelayMicroSeconds(1000);
    }
		
		// check for timeout error
		if(timeout == 0) error = TIMEOUT_ERROR;
  }
	
	// if no error, read temperature and humidity raw values
  if(error == NO_ERROR)
  {
    error |= SHT3X_Read2BytesAndCrc(&rawValueTemp, ACK, 0);
    error |= SHT3X_Read2BytesAndCrc(&rawValueHumi, NACK, 0);
  }

  SHT3X_StopAccess();

  // if no error, calculate temperature in 癈 and humidity in %RH
  if(error == NO_ERROR)
  {
    *temp = SHT3X_CalcTemperature(rawValueTemp);
    *humi = SHT3X_CalcHumidity(rawValueHumi);
  }

  return error;
}

//==============================================================================
etError SHT3X_StartPeriodicMeasurment(etRepeatab  repeatab,
                                      etFrequency freq){
//==============================================================================
  etError error;        // error code

  error = SHT3X_StartWriteAccess();

	// if no error, start periodic measurement 
	if(error == NO_ERROR)
	{
    // use depending on the required repeatability and frequency,
	  // the corresponding command
		switch(repeatab)
		{
			case REPEATAB_LOW: // low repeatability
				switch(freq)
				{
					case FREQUENCY_HZ5:  // low repeatability,  0.5 Hz
						error |= SHT3X_WriteCommand(CMD_MEAS_PERI_05_L); break;					
					case FREQUENCY_1HZ:  // low repeatability,  1.0 Hz
						error |= SHT3X_WriteCommand(CMD_MEAS_PERI_1_L); break;					
					case FREQUENCY_2HZ:  // low repeatability,  2.0 Hz
						error |= SHT3X_WriteCommand(CMD_MEAS_PERI_2_L); break;					
					case FREQUENCY_4HZ:  // low repeatability,  4.0 Hz
						error |= SHT3X_WriteCommand(CMD_MEAS_PERI_4_L); break;					
					case FREQUENCY_10HZ: // low repeatability, 10.0 Hz
						error |= SHT3X_WriteCommand(CMD_MEAS_PERI_10_L); break;					
					default:
						error |= PARM_ERROR; break;
				}
			  break;
				
			case REPEATAB_MEDIUM: // medium repeatability
				switch(freq)
				{
					case FREQUENCY_HZ5:  // medium repeatability,  0.5 Hz
						error |= SHT3X_WriteCommand(CMD_MEAS_PERI_05_M); break;
					case FREQUENCY_1HZ:  // medium repeatability,  1.0 Hz
						error |= SHT3X_WriteCommand(CMD_MEAS_PERI_1_M); break;				
					case FREQUENCY_2HZ:  // medium repeatability,  2.0 Hz
						error |= SHT3X_WriteCommand(CMD_MEAS_PERI_2_M); break;				
					case FREQUENCY_4HZ:  // medium repeatability,  4.0 Hz
						error |= SHT3X_WriteCommand(CMD_MEAS_PERI_4_M); break;			
					case FREQUENCY_10HZ: // medium repeatability, 10.0 Hz
						error |= SHT3X_WriteCommand(CMD_MEAS_PERI_10_M); break;
					default:
						error |= PARM_ERROR; break;
				}
			  break;
				
			case REPEATAB_HIGH: // high repeatability
				switch(freq)
				{
					case FREQUENCY_HZ5:  // high repeatability,  0.5 Hz
						error |= SHT3X_WriteCommand(CMD_MEAS_PERI_05_H); break;
					case FREQUENCY_1HZ:  // high repeatability,  1.0 Hz
						error |= SHT3X_WriteCommand(CMD_MEAS_PERI_1_H); break;
					case FREQUENCY_2HZ:  // high repeatability,  2.0 Hz
						error |= SHT3X_WriteCommand(CMD_MEAS_PERI_2_H); break;
					case FREQUENCY_4HZ:  // high repeatability,  4.0 Hz
						error |= SHT3X_WriteCommand(CMD_MEAS_PERI_4_H); break;
					case FREQUENCY_10HZ: // high repeatability, 10.0 Hz
						error |= SHT3X_WriteCommand(CMD_MEAS_PERI_10_H); break;
					default:
						error |= PARM_ERROR; break;
				}
			  break;
			default:
				error |= PARM_ERROR; break;
		}
	}

  SHT3X_StopAccess();

  return error;
}

//==============================================================================
etError SHT3X_ReadMeasurementBuffer(ft *temp, ft *humi){
//==============================================================================
  etError  error;        // error code
	u16t     rawValueTemp; // temperature raw value from sensor
  u16t     rawValueHumi; // humidity raw value from sensor

	error = SHT3X_StartWriteAccess();

	// if no error, read measurements
	if(error == NO_ERROR)	error = SHT3X_WriteCommand(CMD_FETCH_DATA);
	if(error == NO_ERROR)	error = SHT3X_StartReadAccess();	
	if(error == NO_ERROR)	error = SHT3X_Read2BytesAndCrc(&rawValueTemp, ACK, 0);
	if(error == NO_ERROR) error = SHT3X_Read2BytesAndCrc(&rawValueHumi, NACK, 0);
	
	// if no error, calculate temperature in 癈 and humidity in %RH
	if(error == NO_ERROR)
	{
		*temp = SHT3X_CalcTemperature(rawValueTemp);
		*humi = SHT3X_CalcHumidity(rawValueHumi);
	}
	
	SHT3X_StopAccess();
	
	return error;
}

//==============================================================================
etError SHT3X_EnableHeater(void){
//==============================================================================
  etError error; // error code

  error = SHT3X_StartWriteAccess();

  // if no error, write heater enable command
  if(error == NO_ERROR) error = SHT3X_WriteCommand(CMD_HEATER_ENABLE);

  SHT3X_StopAccess();

  return error;
}

//==============================================================================
etError SHT3X_DisbaleHeater(void){
//==============================================================================
  etError error; // error code

  error = SHT3X_StartWriteAccess();

  // if no error, write heater disable command
  if(error == NO_ERROR) error = SHT3X_WriteCommand(CMD_HEATER_DISABLE);

  SHT3X_StopAccess();

  return error;
}

//==============================================================================
etError SHT3X_SoftReset(void){
//==============================================================================
  etError error; // error code

  error = SHT3X_StartWriteAccess();

  // write reset command
  error |= SHT3X_WriteCommand(CMD_SOFT_RESET);

  SHT3X_StopAccess();
	
	// if no error, wait 50 ms after reset
	//if(error == NO_ERROR) DelayMicroSeconds(50000); 
	if(error == NO_ERROR) DelayMs(50); 

  return error;
}

//==============================================================================
void SHT3X_HardReset(void){
//==============================================================================
	// set reset low
	RESET_LOW();

	// wait 100 ms
	//DelayMicroSeconds(100000);
	DelayMs(100);
	
	// release reset
	RESET_HIGH();
	
	// wait 50 ms after reset
	//DelayMicroSeconds(50000);
	DelayMs(50);
}

//==============================================================================
etError SHT3X_StartWriteAccess(void){
//==============================================================================
  etError error; // error code

  // write a start condition
  I2c_StartCondition();

  // write the sensor I2C address with the write flag
  error = I2c_WriteByte(_i2cWriteHeader);

  return error;
}

//==============================================================================
etError SHT3X_StartReadAccess(void){
//==============================================================================
  etError error; // error code

  // write a start condition
  I2c_StartCondition();

  // write the sensor I2C address with the read flag
  error = I2c_WriteByte(_i2cReadHeader);

  return error;
}

//==============================================================================
void SHT3X_StopAccess(void){
//==============================================================================
  // write a stop condition
  I2c_StopCondition();
}

//==============================================================================
etError SHT3X_WriteCommand(etCommands cmd){
//==============================================================================
  etError error; // error code

  // write the upper 8 bits of the command to the sensor
  error  = I2c_WriteByte(cmd >> 8);

  // write the lower 8 bits of the command to the sensor
  error |= I2c_WriteByte(cmd & 0xFF);

  return error;
}

//==============================================================================
etError SHT3X_Read2BytesAndCrc(u16t *data, etI2cAck finaleAckNack, u8t timeout){
//==============================================================================
  etError error;    // error code
  u8t     bytes[2]; // read data array
  u8t     checksum; // checksum byte
 
  // read two data bytes and one checksum byte
	                      error = I2c_ReadByte(&bytes[0], ACK, timeout);
	if(error == NO_ERROR) error = I2c_ReadByte(&bytes[1], ACK, 0);
  if(error == NO_ERROR) error = I2c_ReadByte(&checksum, finaleAckNack, 0);
  
  // verify checksum
  if(error == NO_ERROR) error = SHT3X_CheckCrc(bytes, 2, checksum);
  
  // combine the two bytes to a 16-bit value
  *data = (bytes[0] << 8) | bytes[1];
  
  return error;
}

//==============================================================================
etError SHT3X_Write2BytesAndCrc(u16t data){
//==============================================================================
  etError error;    // error code
  u8t     bytes[2]; // read data array
  u8t     checksum; // checksum byte
	
	bytes[0] = data >> 8;
	bytes[1] = data & 0xFF;
	checksum = SHT3X_CalcCrc(bytes, 2);
 
	// write two data bytes and one checksum byte
	                      error = I2c_WriteByte(bytes[0]); // write data MSB
	if(error == NO_ERROR) error = I2c_WriteByte(bytes[1]); // write data LSB
	if(error == NO_ERROR) error = I2c_WriteByte(checksum); // write checksum
  
  return error;
}

//==============================================================================
u8t SHT3X_CalcCrc(u8t data[], u8t nbrOfBytes){
//==============================================================================
	u8t bit;        // bit mask
  u8t crc = 0xFF; // calculated checksum
  u8t byteCtr;    // byte counter
  
  // calculates 8-Bit checksum with given polynomial
  for(byteCtr = 0; byteCtr < nbrOfBytes; byteCtr++)
  {
    crc ^= (data[byteCtr]);
    for(bit = 8; bit > 0; --bit)
    {
      if(crc & 0x80) crc = (crc << 1) ^ POLYNOMIAL;
      else           crc = (crc << 1);
    }
  }
	
	return crc;
}

//==============================================================================
etError SHT3X_CheckCrc(u8t data[], u8t nbrOfBytes, u8t checksum){
//==============================================================================
  u8t crc;     // calculated checksum
  
	// calculates 8-Bit checksum
	crc = SHT3X_CalcCrc(data, nbrOfBytes);
  
  // verify checksum
  if(crc != checksum) return CHECKSUM_ERROR;
  else                return NO_ERROR;
}

//==============================================================================
ft SHT3X_CalcTemperature(u16t rawValue){
//==============================================================================
  // calculate temperature [癈]
  // T = -45 + 175 * rawValue / (2^16-1)
  return 175 * (ft)rawValue / 65535 - 45;
}

//==============================================================================
ft SHT3X_CalcHumidity(u16t rawValue){
//==============================================================================
  // calculate relative humidity [%RH]
  // RH = rawValue / (2^16-1) * 100
  return 100 * (ft)rawValue / 65535;
}

//==============================================================================
u16t SHT3X_CalcRawTemperature(ft temperature){
//==============================================================================
  // calc raw value from a temperature [癈]
  // rawValue = (T + 45) / 175 * (2^16-1)
	return (u16t)((temperature + 45) / 175 * 65535);
}

//==============================================================================
u16t SHT3X_CalcRawHumidity(ft humidity){
//==============================================================================
  // calc raw value from a relative humidity [%RH]
  // rawValue = RH / 100 * (2^16-1)
	return (u16t)(humidity / 100 * 65535);
}

//==============================================================================
bt SHT3X_ReadAlert(void){
//==============================================================================
  // read alert pin
  return (ALERT_READ != 0) ? TRUE : FALSE;
}


void lcd_busy(void)
{                          
  GPIO_InitStructure.GPIO_Pin = LCD_D7_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//GPIO_PuPd_UP;
  GPIO_Init(LCD_D7_PORT, &GPIO_InitStructure);

  GPIO_ResetBits(LCD_RS_PORT,LCD_RS_PIN);
  GPIO_SetBits(LCD_RW_PORT,LCD_RW_PIN);
  GPIO_ResetBits(LCD_E_PORT,LCD_E_PIN);
  NOP();
  //DelayMs(1);
  GPIO_SetBits(LCD_E_PORT,LCD_E_PIN);
  NOP();
  //DelayMs(1);
  //Delay10Us(2);
  while ((LCD_D7_PORT->IDR & LCD_D7_PIN) != RESET)
  {
  }
  GPIO_ResetBits(LCD_E_PORT,LCD_E_PIN);

  GPIO_InitStructure.GPIO_Pin = (LCD_D7_PIN);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(LCD_D7_PORT, &GPIO_InitStructure);
  GPIO_ResetBits(LCD_D7_PORT,(LCD_D7_PIN));
}

void write_cmd(uint8_t cmd)
{                          
    lcd_busy();
  GPIO_ResetBits(LCD_E_PORT,LCD_E_PIN);
    //LCD_RS = 0;  //di
    GPIO_ResetBits(LCD_RS_PORT,LCD_RS_PIN);
    //LCD_RW = 0;
    GPIO_ResetBits(LCD_RW_PORT,LCD_RW_PIN);
    //e=1;   
    GPIO_SetBits(LCD_E_PORT,LCD_E_PIN);
    NOP();
    //P0 = cmd;
    LCD_DATA_PORT->BSRR = GPIO_Pin;
    LCD_DATA_PORT->ODR = LCD_DATA_PORT->ODR & 0xff00 | cmd;
  NOP();
  GPIO_SetBits(LCD_E_PORT,LCD_E_PIN);
  NOP();
  //DelayMs(1);
  // Delay();
  //Delay10Us(1);
    GPIO_ResetBits(LCD_E_PORT,LCD_E_PIN);
}

void write_data(uint8_t dat)
{                          
   lcd_busy();
    
  GPIO_ResetBits(LCD_E_PORT,LCD_E_PIN);
    //LCD_RS = 0;  //di
    GPIO_SetBits(LCD_RS_PORT,LCD_RS_PIN);
    //LCD_RW = 0;
    GPIO_ResetBits(LCD_RW_PORT,LCD_RW_PIN);
    //e=1;   
    GPIO_SetBits(LCD_E_PORT,LCD_E_PIN);
    NOP();
    //P0 = cmd;
    LCD_DATA_PORT->BSRR = GPIO_Pin;
    LCD_DATA_PORT->ODR = LCD_DATA_PORT->ODR & 0xff00 | dat;
  NOP();
  GPIO_SetBits(LCD_E_PORT,LCD_E_PIN);
  NOP();
  //DelayMs(1);
  // Delay();
  //Delay10Us(1);
    GPIO_ResetBits(LCD_E_PORT,LCD_E_PIN);
}

void display_string(uint8_t addr,uint8_t *p)
{ 
   write_cmd(addr);
   while(*p>0)    
   {  
     write_data(*p);
      p++;
   }
}


void LCD_Init(void)
{

  GPIO_InitStructure.GPIO_Pin = LCD_RST_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(LCD_RST_PORT, &GPIO_InitStructure);
  GPIO_SetBits(LCD_RST_PORT,LCD_RST_PIN);
  

  GPIO_InitStructure.GPIO_Pin = LCD_PSB_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(LCD_PSB_PORT, &GPIO_InitStructure);
  GPIO_SetBits(LCD_PSB_PORT,LCD_PSB_PIN);
  
  GPIO_InitStructure.GPIO_Pin = LCD_RS_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(LCD_RS_PORT, &GPIO_InitStructure);
  GPIO_ResetBits(LCD_RS_PORT,LCD_RS_PIN);

  GPIO_InitStructure.GPIO_Pin = (LCD_D0_PIN | LCD_D1_PIN | LCD_D2_PIN | LCD_D3_PIN | LCD_D4_PIN | LCD_D5_PIN | LCD_D6_PIN);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(LCD_D0_PORT, &GPIO_InitStructure);
  GPIO_ResetBits(LCD_D0_PORT,(LCD_D0_PIN | LCD_D1_PIN | LCD_D2_PIN | LCD_D3_PIN | LCD_D4_PIN | LCD_D5_PIN | LCD_D6_PIN));

  GPIO_InitStructure.GPIO_Pin = (LCD_D7_PIN);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(LCD_D7_PORT, &GPIO_InitStructure);
  GPIO_ResetBits(LCD_D7_PORT,LCD_D7_PIN);

  GPIO_InitStructure.GPIO_Pin = LCD_RW_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(LCD_RW_PORT, &GPIO_InitStructure);
  GPIO_ResetBits(LCD_RW_PORT,LCD_RW_PIN);

  GPIO_InitStructure.GPIO_Pin = LCD_E_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(LCD_E_PORT, &GPIO_InitStructure);
  GPIO_ResetBits(LCD_E_PORT,LCD_E_PIN);
 
    write_cmd(0x36);      //扩充指令操作
    write_cmd(0x30);      //基本指令操作
    write_cmd(0x0C);      //显示开，关光标
    write_cmd(0x01);      //清除LCD的显示内容
    write_cmd(0x06);//指定在资料的读取及写入时，设定游标的移动方向及指定显示的移位

}



void lcd_clear(void)
{
     uint8_t i,j ;
     write_cmd(0x34) ;      //8Bit扩充指令集,即使是36H也要写两次
     write_cmd(0x36) ;      //绘图ON,基本指令集里面36H不能开绘图
     for(i=0 ;i<32 ;i++)            //12864实际为256x32
     {
           write_cmd(0x80|i) ;      //行位置
           write_cmd(0x80) ;      //列位置
           for(j=0 ;j<32 ;j++)            //256/8=32 byte
                write_data(0x00) ;
     }
     write_cmd(0x30) ;      //8BitMCU,基本指令集合
     write_cmd(0x80) ;      //AC归起始位
     for(i=0 ;i<64 ;i++)
        write_data(0x20) ;
}
void USART1_PutChar(uint8_t c)
{
	UART1TxBuf[UART1TxIn] = c;
	UART1TxIn = (UART1TxIn + 1);
}

int main(void)
{
  
  etError   error;       // error code
  u32t      serialNumber;// serial number
  regStatus status;      // sensor status
  //ft        temperature; // temperature [癈]
  //ft        humidity;    // relative humidity [%RH]
  bt        heater;      // heater, false: off, true: on
  
  SysTickConfig();
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF, ENABLE);
  DelayMs(1000);
  USART1_Config();
  
        //USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
		/*
                                USART1_PutChar('1');
                                USART1_PutChar('2');
                                USART1_PutChar('3');
                                USART1_PutChar('4');
                                USART1_PutChar(0x0D);
                                USART1_PutChar(0x0A);
                                USART1_PutChar(0x0A);
								*/
                                //UART1RxOut = UART1RxIn = 0;
                                USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
								
     
	 
  
  SHT3X_Init(0x44); // Address: 0x44 = Sensor on EvalBoard connector
                    //          0x45 = Sensor on EvalBoard
  
  /*LCD_Init();                //初始化LCD    
  
  DelayMs(10); 
  lcd_clear();  
  lcd_clear();  
  lcd_clear();  
  
  DelayMs(10);  
  display_string(0x80,"温度："); 
  DelayMs(10);  
  display_string(0x90,"        摄氏度  ");  
  DelayMs(10);  
  display_string(0x88,"湿度：          "); 
  DelayMs(10);  
  display_string(0x98,"        %       ");
  // wait 50ms after power on
  //DelayMicroSeconds(50000);   
  */
  DelayMs(50);   
  
  error = SHT3x_ReadSerialNumber(&serialNumber);
  DelayMs(50);   
  SHT3X_DisbaleHeater();
  //if(error != NO_ERROR){}
  
  // wait 50ms after serial number read
  //DelayMicroSeconds(50000);   
  DelayMs(50);   
  
  // demonstrate a single shot measurement with clock-stretching
  error = SHT3X_GetTempAndHumi(&temperature.f, &humidity.f, REPEATAB_HIGH, MODE_CLKSTRETCH, 50);
  temperature32 = (u32t)(temperature.f * 1000);
  humidity32 = (u32t)(humidity.f * 1000);
  //if(error != NO_ERROR){}
  
  // wait 50ms after measurment
  //DelayMicroSeconds(50000);   
  DelayMs(50);   
  
  // demonstrate a single shot measurement with polling and 50ms timeout
  // 一种简单的方式是在主循环中调用这个函数读取温湿度
  error = SHT3X_GetTempAndHumi(&temperature.f, &humidity.f, REPEATAB_HIGH, MODE_POLLING, 50);
  //if(error != NO_ERROR){}
  // 另一种方法用芯片自动周期产生数据
  // wait 50ms after measurment
  //DelayMicroSeconds(50000);   
  DelayMs(50);   
  
	
  temperature32 = (u32t)(temperature.f * 1000);
  humidity32 = (u32t)(humidity.f * 1000);
  
  temperatureS[0] = '+';
  temperatureS[1] = '0';
  temperatureS[2] = '0';
  temperatureS[3] = '.';
  temperatureS[4] = '0';
  temperatureS[5] = '0';
  temperatureS[6] = '0';
  temperatureS[7] = ' ';
  temperatureS[8] = 0;
  
  humidityS[0] = '+';
  humidityS[1] = '0';
  humidityS[2] = '0';
  humidityS[3] = '.';
  humidityS[4] = '0';
  humidityS[5] = '0';
  humidityS[6] = '0';
  humidityS[7] = ' ';
  humidityS[8] =  0;
	// loop forever
	while(1)
	{
		error = NO_ERROR;
		
		// loop while no error
		while(error == NO_ERROR)
		{
      // read status register
			error |= SHT3X_ReadStatus(&status.u16);
      if(error != NO_ERROR) break;
      
      // check if the reset bit is set after a reset or power-up
      if(status.bit.ResetDetected)
      {
        // clear reset flag
        error = SHT3X_ClearAllAlertFlags();
        if(error != NO_ERROR) break;
        
  DelayMs(50);   
        //start periodic measurement, with high repeatability and 1 measurements per second
        error = SHT3X_StartPeriodicMeasurment(REPEATAB_HIGH, FREQUENCY_1HZ);
        if(error != NO_ERROR) break;
  DelayMs(50);   
        
        //switch green LED on
        //LedGreenOn();
      }
				
			// read measurment buffer
 			error = SHT3X_ReadMeasurementBuffer(&temperature.f, &humidity.f);
		  if(error == NO_ERROR)
			{
        // flash blue LED to signalise new temperature and humidity values
        //LedBlueOn();
        //DelayMicroSeconds(10000);
                          
  temperature32 = (u32t)(temperature.f * 1000);
  humidity32 = (u32t)(humidity.f * 1000);
  
  temperatureS[1] = ((temperature32 / 10000) % 10 ) + '0';
  temperatureS[2] = ((temperature32 / 1000) % 10 ) + '0';
  temperatureS[4] = ((temperature32 / 100) % 10 ) + '0';
  temperatureS[5] = ((temperature32 / 10) % 10 ) + '0';
  temperatureS[6] = ((temperature32) % 10 ) + '0';
  
  humidityS[1] = ((humidity32 / 10000) % 10 ) + '0';
  humidityS[2] = ((humidity32 / 1000) % 10 ) + '0';
  humidityS[4] = ((humidity32 / 100) % 10 ) + '0';
  humidityS[5] = ((humidity32 / 10) % 10 ) + '0';
  humidityS[6] = ((humidity32) % 10 ) + '0';
  
  
UART1TxIn = 0;
UART1TxOut = 0;
  
                                USART1_PutChar(temperatureS[0]);
                                USART1_PutChar(temperatureS[1]);
                                USART1_PutChar(temperatureS[2]);
                                USART1_PutChar(temperatureS[3]);
                                USART1_PutChar(temperatureS[4]);
                                USART1_PutChar(temperatureS[5]);
                                USART1_PutChar(temperatureS[6]);
                                //USART1_PutChar(temperatureS[7]);
                                //USART1_PutChar(0xa7);
                                USART1_PutChar('C');
                                USART1_PutChar(' ');
                                USART1_PutChar(' ');
								
								
                                USART1_PutChar(humidityS[0]);
                                USART1_PutChar(humidityS[1]);
                                USART1_PutChar(humidityS[2]);
                                USART1_PutChar(humidityS[3]);
                                USART1_PutChar(humidityS[4]);
                                USART1_PutChar(humidityS[5]);
                                USART1_PutChar(humidityS[6]);
                                //USART1_PutChar(humidityS[7]);
                                USART1_PutChar('%');
								
                                USART1_PutChar(0x0D);
                                USART1_PutChar(0x0A);
                                USART1_PutChar(0x0D);
                                USART1_PutChar(0x0A);
                                USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
  
  //display_string(0x90,temperatureS); 
  //display_string(0x98,humidityS);
        DelayMs(10);
        //LedBlueOff();
      }
      else if (error == ACK_ERROR)
      {
				// there were no new values in the buffer -> ignore this error
				error = NO_ERROR;
			}
      else break;
      /*
      // read heater status
      heater = status.bit.HeaterStatus ? TRUE : FALSE;
	
      
      
      // if the user button is not pressed ...
			if(ReadUserButton() == 0)
			{ 
         // ... and the heater is on
				 if(heater)
				 {
					 // switch off the sensor internal heater
					 error |= SHT3X_DisbaleHeater();
           if(error != NO_ERROR) break;
				 }
			}
			else
      // if the user button is pressed ...
			{
         // ... and the heater is off
				 if(!heater)
				 {
					 // switch on the sensor internal heater
					 error |= SHT3X_EnableHeater();
           if(error != NO_ERROR) break;
				 }
			}
			*/
			// wait 100ms
      //DelayMicroSeconds(100000);
      DelayMs(100);
		}
		
		// in case of an error ...
		
		// ... switch green and blue LED off
		//LedGreenOff();
		//LedBlueOff();
		
		// ... try first a soft reset ...
		error = SHT3X_SoftReset();
		
    // ... if the soft reset fails, do a hard reset
		if(error != NO_ERROR)
		{
			SHT3X_HardReset();
    }
    
    // flash green LED to signalise an error
    //LedGreenOn();
    //DelayMicroSeconds(10000);
    DelayMs(10);
    //LedGreenOff();
	}
  while(1);
}

void USART1_Config(void)
{
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  //GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable GPIO clock */
  RCC_AHBPeriphClockCmd(USART1_TX_GPIO_CLK | USART1_RX_GPIO_CLK, ENABLE);
  //RCC_AHBPeriphClockCmd(USART1_TX_GPIO_CLK, ENABLE);

  /* Enable USART clock */
  USART1_APBPERIPHCLOCK(USART1_CLK, ENABLE);

  /* Connect PXx to USART1_Tx */
  GPIO_PinAFConfig(USART1_TX_GPIO_PORT, USART1_TX_SOURCE, USART1_TX_AF);

  /* Connect PXx to USART1_Rx */
  GPIO_PinAFConfig(USART1_RX_GPIO_PORT, USART1_RX_SOURCE, USART1_RX_AF);

  /* Configure USART Tx and Rx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Pin = USART1_TX_PIN;
  GPIO_Init(USART1_TX_GPIO_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = USART1_RX_PIN;
  GPIO_Init(USART1_RX_GPIO_PORT, &GPIO_InitStructure);

  /* USART1 configuration ----------------------------------------------------*/
  /* USART1 configured as follow:
  - BaudRate = 230400 baud
  - Word Length = 8 Bits
  - One Stop Bit
  - No parity
  - Hardware flow control disabled (RTS and CTS signals)
  - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART1, &USART_InitStructure);

  /* NVIC configuration */
  /* Enable the USART1 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Enable USART */
  USART_Cmd(USART1, ENABLE);

}

void SysTickConfig(void)
{
  /* Setup SysTick Timer for 10ms interrupts  */
  if (SysTick_Config(SystemCoreClock / 1000))
  {
    /* Capture error */
    while (1);
  }
  /* Configure the SysTick handler priority */
  NVIC_SetPriority(SysTick_IRQn, 0x0);
}

void Delay(uint32_t nTime)
{
  uint32_t msCounterBak = msCounter;
  while(msCounter - msCounterBak < nTime);
}

void DelayMs(uint32_t mS)
{
	static uint32_t delayTime;
	while (mS > 0)
	{
  	for (delayTime = 0; delayTime < DELAY_MS_CLOCK; delayTime++)
  	{
  		NOP();
  	}
  	mS--;
  }
}
void Delay100Us(uint32_t time)
{
	static uint32_t delayTime;
	while (time > 0)
	{
  	for (delayTime = 0; delayTime < DELAY_100US_CLOCK; delayTime++)
  	{
  		NOP();
  	}
  	time--;
  }
}
void Delay10Us(uint32_t time)
{
	static uint32_t delayTime;
	while (time > 0)
	{
  	for (delayTime = 0; delayTime < DELAY_10US_CLOCK; delayTime++)
  	{
  		NOP();
  	}
  	time--;
  }
}
void Delay1Us(uint32_t time)
{
	static uint32_t delayTime;
	while (time > 0)
	{
  	for (delayTime = 0; delayTime < DELAY_1US_CLOCK; delayTime++)
  	{
  		NOP();
  	}
  	time--;
  }
}


void SysTick_Handler(void)
{
  msCounter++;
}


void USART1_IRQHandler(void)
{
  if (USART_GetITStatus(USART1, USART_IT_TXE) == SET)
  {
    if (UART1TxIn != UART1TxOut)
    {
		USART_SendData(USART1, UART1TxBuf[UART1TxOut]);
		UART1TxOut = (UART1TxOut + 1);
    }
    else
    {
		USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
    }
  
  }

  /* USART in mode Receiver --------------------------------------------------*/
  if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
  {
  	USART_ReceiveData(USART1);
  }
  USART1->ICR = ~((uint32_t)0x01 << (uint32_t)6);
}


