/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
#include "stm32f0xx.h"

//-- Defines -------------------------------------------------------------------
//Processor endian system
//#define BIG ENDIAN   //e.g. Motorola (not tested at this time)
#define LITTLE_ENDIAN  //e.g. PIC, 8051, NEC V850
//==============================================================================
// basic types: making the size of types clear
//==============================================================================
typedef unsigned char   u8t;      ///< range: 0 .. 255
typedef signed char     i8t;      ///< range: -128 .. +127
                                      
typedef unsigned short  u16t;     ///< range: 0 .. 65535
typedef signed short    i16t;     ///< range: -32768 .. +32767
                                      
typedef unsigned long   u32t;     ///< range: 0 .. 4'294'967'295
typedef signed long     i32t;     ///< range: -2'147'483'648 .. +2'147'483'647
                                      
typedef float           ft;       ///< range: +-1.18E-38 .. +-3.39E+38
typedef double          dt;       ///< range:            .. +-1.79E+308

typedef enum{
  FALSE     = 0,
  TRUE      = 1
}bt;

typedef union {
  u16t u16;               // element specifier for accessing whole u16
  i16t i16;               // element specifier for accessing whole i16
  struct {
    #ifdef LITTLE_ENDIAN  // Byte-order is little endian
    u8t u8L;              // element specifier for accessing low u8
    u8t u8H;              // element specifier for accessing high u8
    #else                 // Byte-order is big endian
    u8t u8H;              // element specifier for accessing low u8
    u8t u8L;              // element specifier for accessing high u8
    #endif
  } s16;                  // element spec. for acc. struct with low or high u8
} nt16;

typedef union {
  u32t u32;               // element specifier for accessing whole u32
  i32t i32;               // element specifier for accessing whole i32
 struct {
    #ifdef LITTLE_ENDIAN  // Byte-order is little endian
    u16t u16L;            // element specifier for accessing low u16
    u16t u16H;            // element specifier for accessing high u16
    #else                 // Byte-order is big endian
    u16t u16H;            // element specifier for accessing low u16
    u16t u16L;            // element specifier for accessing high u16
    #endif
  } s32;                  // element spec. for acc. struct with low or high u16
} nt32;
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;


//-- Enumerations --------------------------------------------------------------
// Error codes
typedef enum{
  NO_ERROR       = 0x00, // no error
  ACK_ERROR      = 0x01, // no acknowledgment error
  CHECKSUM_ERROR = 0x02, // checksum mismatch error
  TIMEOUT_ERROR  = 0x04, // timeout error
  PARM_ERROR     = 0x80, // parameter out of range error
}etError;

// I2C acknowledge
typedef enum{
  ACK  = 0,
  NACK = 1,
}etI2cAck;


//-- Enumerations --------------------------------------------------------------
// Sensor Commands
typedef enum{
  CMD_READ_SERIALNBR  = 0x3780, // read serial number
  CMD_READ_STATUS     = 0xF32D, // read status register
	CMD_CLEAR_STATUS    = 0x3041, // clear status register
	CMD_HEATER_ENABLE   = 0x306D, // enabled heater
	CMD_HEATER_DISABLE  = 0x3066, // disable heater
  CMD_SOFT_RESET      = 0x30A2, // soft reset
	CMD_MEAS_CLOCKSTR_H = 0x2C06, // meas. clock stretching, high rep.
	CMD_MEAS_CLOCKSTR_M = 0x2C0D, // meas. clock stretching, medium rep.
	CMD_MEAS_CLOCKSTR_L = 0x2C10, // meas. clock stretching, low rep.
	CMD_MEAS_POLLING_H  = 0x2400, // meas. no clock stretching, high rep.
	CMD_MEAS_POLLING_M  = 0x240B, // meas. no clock stretching, medium rep.
	CMD_MEAS_POLLING_L  = 0x2416, // meas. no clock stretching, low rep.
	CMD_MEAS_PERI_05_H  = 0x2032, // meas. periodic 0.5 mps, high rep.
	CMD_MEAS_PERI_05_M  = 0x2024, // meas. periodic 0.5 mps, medium rep.
	CMD_MEAS_PERI_05_L  = 0x202F, // meas. periodic 0.5 mps, low rep.
	CMD_MEAS_PERI_1_H   = 0x2130, // meas. periodic 1 mps, high rep.
	CMD_MEAS_PERI_1_M   = 0x2126, // meas. periodic 1 mps, medium rep.
	CMD_MEAS_PERI_1_L   = 0x212D, // meas. periodic 1 mps, low rep.
	CMD_MEAS_PERI_2_H   = 0x2236, // meas. periodic 2 mps, high rep.
	CMD_MEAS_PERI_2_M   = 0x2220, // meas. periodic 2 mps, medium rep.
	CMD_MEAS_PERI_2_L   = 0x222B, // meas. periodic 2 mps, low rep.
	CMD_MEAS_PERI_4_H   = 0x2334, // meas. periodic 4 mps, high rep.
	CMD_MEAS_PERI_4_M   = 0x2322, // meas. periodic 4 mps, medium rep.
	CMD_MEAS_PERI_4_L   = 0x2329, // meas. periodic 4 mps, low rep.
	CMD_MEAS_PERI_10_H  = 0x2737, // meas. periodic 10 mps, high rep.
	CMD_MEAS_PERI_10_M  = 0x2721, // meas. periodic 10 mps, medium rep.
	CMD_MEAS_PERI_10_L  = 0x272A, // meas. periodic 10 mps, low rep.
	CMD_FETCH_DATA      = 0xE000, // readout measurements for periodic mode
	CMD_R_AL_LIM_LS     = 0xE102, // read alert limits, low set
	CMD_R_AL_LIM_LC     = 0xE109, // read alert limits, low clear
	CMD_R_AL_LIM_HS     = 0xE11F, // read alert limits, high set
	CMD_R_AL_LIM_HC     = 0xE114, // read alert limits, high clear
	CMD_W_AL_LIM_LS     = 0x6100, // write alert limits, low set
	CMD_W_AL_LIM_LC     = 0x610B, // write alert limits, low clear
	CMD_W_AL_LIM_HS     = 0x611D, // write alert limits, high set
	CMD_W_AL_LIM_HC     = 0x6116, // write alert limits, high clear
  CMD_NO_SLEEP        = 0x303E,
}etCommands;

typedef enum{
	REPEATAB_HIGH,   // high repeatability
	REPEATAB_MEDIUM, // medium repeatability
	REPEATAB_LOW,    // low repeatability
}etRepeatab;

typedef enum{
	MODE_CLKSTRETCH, // clock stretching
	MODE_POLLING,    // polling
}etMode;

typedef enum{
	FREQUENCY_HZ5,  //  0.5 measurements per seconds
	FREQUENCY_1HZ,  //  1.0 measurements per seconds
	FREQUENCY_2HZ,  //  2.0 measurements per seconds
	FREQUENCY_4HZ,  //  4.0 measurements per seconds
	FREQUENCY_10HZ, // 10.0 measurements per seconds
}etFrequency;

//-- Typedefs ------------------------------------------------------------------
// Status-Register
typedef union {
	u16t u16;
	struct{
		#ifdef LITTLE_ENDIAN  // bit-order is little endian
		u16t CrcStatus     : 1; // write data checksum status
		u16t CmdStatus     : 1; // command status
		u16t Reserve0      : 2; // reserved
		u16t ResetDetected : 1; // system reset detected
		u16t Reserve1      : 5; // reserved
		u16t T_Alert       : 1; // temperature tracking alert
		u16t RH_Alert      : 1; // humidity tracking alert
		u16t Reserve2      : 1; // reserved
		u16t HeaterStatus  : 1; // heater status
		u16t Reserve3      : 1; // reserved
		u16t AlertPending  : 1; // alert pending status 
		#else                 // bit-order is big endian
		u16t AlertPending  : 1;
		u16t Reserve3      : 1;
		u16t HeaterStatus  : 1;
		u16t Reserve2      : 1;
		u16t RH_Alert      : 1;
		u16t T_Alert       : 1;
		u16t Reserve1      : 5;
		u16t ResetDetected : 1;
		u16t Reserve0      : 2;
		u16t CmdStatus     : 1;
		u16t CrcStatus     : 1;
		#endif
	}bit;
} regStatus;


//-- Defines -------------------------------------------------------------------
// CRC
#define POLYNOMIAL  0x131 // P(x) = x^8 + x^5 + x^4 + 1 = 100110001

//------------------------------------------------------------------------------
// IO-Pins                             /* -- adapt the defines for your uC -- */
//------------------------------------------------------------------------------
// Reset on port B, bit 12
//#define RESET_LOW()  (GPIOB->BSRR = 0x10000000) // set Reset to low
//#define RESET_HIGH() (GPIOB->BSRR = 0x00001000) // set Reset to high

#define RESET_LOW()  (GPIOB->BRR = GPIO_Pin_12) // set Reset to low
#define RESET_HIGH() (GPIOB->BSRR = GPIO_Pin_12) // set Reset to high


// Alert on port B, bit 10
#define ALERT_READ   (GPIOB->IDR  & 0x0400)     // read Alert
//------------------------------------------------------------------------------
//==============================================================================
// Initializes the I2C bus for communication with the sensor.
//------------------------------------------------------------------------------
// input:  i2cAdr        I2C address, 0x44 ADDR pin low / 0x45 ADDR pin high
//------------------------------------------------------------------------------
void SHT3X_Init(u8t i2cAdr);


//==============================================================================
// Sets the I2C address.
//------------------------------------------------------------------------------
// input:  i2cAdr       I2C address, 0x44 ADDR pin low / 0x45 ADDR pin high
//------------------------------------------------------------------------------
void SHT3X_SetI2cAdr(u8t i2cAdr);


//==============================================================================
// Reads the serial number from sensor.
//------------------------------------------------------------------------------
// input:  *serialNbr   pointer to a variable to store the serialNumber
//
// return: error:       ACK_ERROR      = no acknowledgment from sensor
//                      CHECKSUM_ERROR = checksum mismatch
//                      TIMEOUT_ERROR  = timeout
//                      NO_ERROR       = no error
//------------------------------------------------------------------------------
etError SHT3x_ReadSerialNumber(u32t *serialNbr);


//==============================================================================
// Reads the status register from the sensor.
//------------------------------------------------------------------------------
// input:  *status      pointer to a variable to store the status
//
// return: error:       ACK_ERROR      = no acknowledgment from sensor
//                      CHECKSUM_ERROR = checksum mismatch
//                      TIMEOUT_ERROR  = timeout
//                      NO_ERROR       = no error
//------------------------------------------------------------------------------
etError SHT3X_ReadStatus(u16t *status);


//==============================================================================
// Clears all alert flags in status register from sensor.
//------------------------------------------------------------------------------
// return: error:       ACK_ERROR      = no acknowledgment from sensor
//                      CHECKSUM_ERROR = checksum mismatch
//                      TIMEOUT_ERROR  = timeout
//                      NO_ERROR       = no error
//------------------------------------------------------------------------------
etError SHT3X_ClearAllAlertFlags(void);


//==============================================================================
// Gets the temperature [°C] and the relative humidity [%RH] from the sensor.
//------------------------------------------------------------------------------
// input:  *temp        pointer to a variable to store the temperature
//         *humi        pointer to a variable to store the humidity
//         repeatab     repeatability for the measurement [low, medium, high]
//         mode         command mode [clock stretching, polling]
//         timeout      timeout in milliseconds
//
// return: error:       ACK_ERROR      = no acknowledgment from sensor
//                      CHECKSUM_ERROR = checksum mismatch
//                      TIMEOUT_ERROR  = timeout
//                      PARM_ERROR     = parameter out of range
//                      NO_ERROR       = no error
//------------------------------------------------------------------------------
etError SHT3X_GetTempAndHumi(ft         *temp,
                             ft         *humi,
                             etRepeatab repeatab,
                             etMode     mode,
                             u8t        timeout);


//==============================================================================
// Gets the temperature [°C] and the relative humidity [%RH] from the sensor.
// This function uses the i2c clock stretching for waiting until measurement is
// ready.
//------------------------------------------------------------------------------
// input:  *temp        pointer to a variable to store the temperature
//         *humi        pointer to a variable to store the humidity
//         repeatab     repeatability for the measurement [low, medium, high]
//         timeout      clock stretching timeout in milliseconds
//
// return: error:       ACK_ERROR      = no acknowledgment from sensor
//                      CHECKSUM_ERROR = checksum mismatch
//                      TIMEOUT_ERROR  = timeout
//                      PARM_ERROR     = parameter out of range
//                      NO_ERROR       = no error
//------------------------------------------------------------------------------
etError SHT3X_GetTempAndHumiClkStretch(ft         *temp,
                                       ft         *humi,
                                       etRepeatab repeatab,
                                       u8t        timeout);


//==============================================================================
// Gets the temperature [°C] and the relative humidity [%RH] from the sensor.
// This function polls every 1ms until measurement is ready.
//------------------------------------------------------------------------------
// input:  *temp        pointer to a variable to store the temperature
//         *humi        pointer to a variable to store the humidity
//         repeatab     repeatability for the measurement [low, medium, high]
//         timeout      polling timeout in milliseconds
//
// return: error:       ACK_ERROR      = no acknowledgment from sensor
//                      CHECKSUM_ERROR = checksum mismatch
//                      TIMEOUT_ERROR  = timeout
//                      PARM_ERROR     = parameter out of range
//                      NO_ERROR       = no error
//------------------------------------------------------------------------------
etError SHT3X_GetTempAndHumiPolling(ft         *temp,
                                    ft         *humi,
                                    etRepeatab repeatab,
                                    u8t        timeout);


//==============================================================================
// Starts periodic measurement.
//------------------------------------------------------------------------------
// input:  repeatab     repeatability for the measurement [low, medium, high]
//         freq         measurement frequency [0.5, 1, 2, 4, 10] Hz
//
// return: error:       ACK_ERROR      = no acknowledgment from sensor
//                      CHECKSUM_ERROR = checksum mismatch
//                      TIMEOUT_ERROR  = timeout
//                      PARM_ERROR     = parameter out of range
//                      NO_ERROR       = no error
//------------------------------------------------------------------------------
etError SHT3X_StartPeriodicMeasurment(etRepeatab  repeatab, etFrequency freq);


//==============================================================================
// Reads last measurement from the sensor buffer
//------------------------------------------------------------------------------
// input:  *temp        pointer to a variable to store the temperature
//         *humi        pointer to a variable to store the humidity
//
// return: error:       ACK_ERROR      = no acknowledgment from sensor
//                      CHECKSUM_ERROR = checksum mismatch
//                      TIMEOUT_ERROR  = timeout
//                      NO_ERROR       = no error
//------------------------------------------------------------------------------
etError SHT3X_ReadMeasurementBuffer(ft *temp, ft *humi);


//==============================================================================
// Enables the heater on sensor
//------------------------------------------------------------------------------
// return: error:       ACK_ERROR      = no acknowledgment from sensor
//                      CHECKSUM_ERROR = checksum mismatch
//                      TIMEOUT_ERROR  = timeout
//                      NO_ERROR       = no error
//------------------------------------------------------------------------------
etError SHT3X_EnableHeater(void);


//==============================================================================
// Disables the heater on sensor
//------------------------------------------------------------------------------
// return: error:       ACK_ERROR      = no acknowledgment from sensor
//                      CHECKSUM_ERROR = checksum mismatch
//                      TIMEOUT_ERROR  = timeout
//                      NO_ERROR       = no error
//------------------------------------------------------------------------------
etError SHT3X_DisbaleHeater(void);


//==============================================================================
// Calls the soft reset mechanism that forces the sensor into a well-defined
// state without removing the power supply.
//------------------------------------------------------------------------------
// return: error:       ACK_ERROR      = no acknowledgment from sensor
//                      CHECKSUM_ERROR = checksum mismatch
//                      TIMEOUT_ERROR  = timeout
//                      NO_ERROR       = no error
//------------------------------------------------------------------------------
etError SHT3X_SoftReset(void);


//==============================================================================
// Resets the sensor by pulling down the reset pin.
//------------------------------------------------------------------------------
void SHT3X_HardReset(void);


//==============================================================================
// Writes a start condition and the sensor I2C address with the write flag.
//------------------------------------------------------------------------------
// return: error:       ACK_ERROR      = no acknowledgment from sensor
//                      NO_ERROR       = no error
//------------------------------------------------------------------------------
etError SHT3X_StartWriteAccess(void);


//==============================================================================
// Writes a start condition and the sensor I2C address with the read flag.
//------------------------------------------------------------------------------
// return: error:       ACK_ERROR      = no acknowledgment from sensor
//                      NO_ERROR       = no error
//------------------------------------------------------------------------------
etError SHT3X_StartReadAccess(void);

//==============================================================================
// Writes a stop condition.
//------------------------------------------------------------------------------
void SHT3X_StopAccess(void);


//==============================================================================
// Reads two bytes plus the checksum and verifies this. If the checksum
// verification is successful, then the two bytes stored in a 16-bit integer. 
//------------------------------------------------------------------------------
// input:  *data        pointer to a variable to store the data
//
//         finaleAck    set ACK to continue reading further bytes
//                      set NACK to stop reading
//
//         timeout      clock stretching timeout in milliseconds
//
// return: error:       CHECKSUM_ERROR = checksum mismatch
//                      TIMEOUT_ERROR  = timeout
//                      NO_ERROR       = no error
//------------------------------------------------------------------------------
etError SHT3X_Read2BytesAndCrc(u16t *data, etI2cAck finaleAck, u8t timeout);


//==============================================================================
// Writes two bytes plus the checksum.
//------------------------------------------------------------------------------
// input:  data         write data: 2 bytes in a 16-bit variable
//
// return: error:       ACK_ERROR      = no acknowledgment from sensor
//                      CHECKSUM_ERROR = checksum mismatch
//                      TIMEOUT_ERROR  = timeout
//                      NO_ERROR       = no error
//------------------------------------------------------------------------------
etError SHT3X_Write2BytesAndCrc(u16t data);

//==============================================================================
// Writes command to the sensor.
//------------------------------------------------------------------------------
// input:  cmd          sensor command
//
// return: error:       ACK_ERROR      = no acknowledgment from sensor
//                      TIMEOUT_ERROR  = timeout
//                      NO_ERROR       = no error
//------------------------------------------------------------------------------
etError SHT3X_WriteCommand(etCommands cmd);


//==============================================================================
// Calculates the crc from a byte array
//------------------------------------------------------------------------------
// input:  data[]       checksum is built based on this data
//         nbrOfBytes   checksum is built for n bytes of data
//
// return:              crc, checksum
//------------------------------------------------------------------------------
u8t SHT3X_CalcCrc(u8t data[], u8t nbrOfBytes);


//==============================================================================
// Calculates checksum for n bytes of data and compares it with expected
// checksum. 
//------------------------------------------------------------------------------
// input:  data[]       checksum is built based on this data
//         nbrOfBytes   checksum is built for n bytes of data
//         checksum     expected checksum
//
// return: error:       CHECKSUM_ERROR = checksum does not match
//                      NO_ERROR       = checksum matches
//------------------------------------------------------------------------------
etError SHT3X_CheckCrc(u8t data[], u8t nbrOfBytes, u8t checksum);


//==============================================================================
// Calculates the temperature [°C] as a floating point value from the raw data
// that are read from the sensor.
//------------------------------------------------------------------------------
// input:  rawValue     temperature raw value (16bit scaled)
//
// return:              temperature [°C] as a floating point value
//------------------------------------------------------------------------------
ft SHT3X_CalcTemperature(u16t rawValue);

//==============================================================================
// Calculates the relative humidity [%RH] as a floating point value from the raw
// data that are read from the sensor.
//------------------------------------------------------------------------------
// input:  rawValue     humidity raw value (16-bit scaled)
//
// return:              relative humidity [%RH] as a floating point value
//------------------------------------------------------------------------------
ft SHT3X_CalcHumidity(u16t rawValue);


//==============================================================================
// Calculates the raw data as 16-bit value from temperature [°C].
//------------------------------------------------------------------------------
// input:  temperature  temperature [°C] as a floating point value
//
// return:              temperature raw value (16-bit scaled)
//------------------------------------------------------------------------------
u16t SHT3X_CalcRawTemperature(ft temperature);


//==============================================================================
// Calculates the raw data as 16-bit value from relative humidity [%RH].
//------------------------------------------------------------------------------
// input:  humidity     humidity [%RH] as a floating point value
//
// return:              humidity raw value (16-bit scaled)
u16t SHT3X_CalcRawHumidity(ft humidity);


//==============================================================================
// Returns the state of the Alert-Pin.
//------------------------------------------------------------------------------
// return:              true:  Alert-Pin is high
//                      false: Alter-Pin is low
//------------------------------------------------------------------------------
bt SHT3X_ReadAlert(void);



void I2c_Init(void);
void I2c_StartCondition(void);
void I2c_StopCondition(void);
etError I2c_WriteByte(u8t txByte);
etError I2c_ReadByte(u8t *rxByte, etI2cAck ack, u8t timeout);
etError I2c_WaitWhileClockStreching(u8t timeout);
etError I2c_GeneralCallReset(void);

#define SYSTEM_CORE_CLOCK 48000000 
#define DBG_TIME
#define DELAY_MS_CLOCK    (SYSTEM_CORE_CLOCK - 18) / 11000
#define DELAY_100US_CLOCK    (SYSTEM_CORE_CLOCK) / 120600
#define DELAY_10US_CLOCK    (SYSTEM_CORE_CLOCK) / 1200000
#define DELAY_1US_CLOCK    (SYSTEM_CORE_CLOCK) / 12000000
#define NOP() __ASM volatile ("nop")


#define LCD_DATA_PORT  (GPIOB)             //Êý¾Ý¿Ú
#define LCD_E_PORT  (GPIOA)
#define LCD_E_PIN  (GPIO_Pin_7)
#define LCD_RS_PORT  (GPIOA)
#define LCD_RS_PIN  (GPIO_Pin_5)
#define LCD_RW_PORT  (GPIOA)
#define LCD_RW_PIN  (GPIO_Pin_6)
#define LCD_PSB_PORT  (GPIOB)
#define LCD_PSB_PIN  (GPIO_Pin_11)
#define LCD_RST_PORT  (GPIOB)
#define LCD_RST_PIN  (GPIO_Pin_9)

#define LCD_D0_PORT  (GPIOB)
#define LCD_D0_PIN  (GPIO_Pin_0)
#define LCD_D1_PORT  (GPIOB)
#define LCD_D1_PIN  (GPIO_Pin_1)
#define LCD_D2_PORT  (GPIOB)
#define LCD_D2_PIN  (GPIO_Pin_2)
#define LCD_D3_PORT  (GPIOB)
#define LCD_D3_PIN  (GPIO_Pin_3)
#define LCD_D4_PORT  (GPIOB)
#define LCD_D4_PIN  (GPIO_Pin_4)
#define LCD_D5_PORT  (GPIOB)
#define LCD_D5_PIN  (GPIO_Pin_5)
#define LCD_D6_PORT  (GPIOB)
#define LCD_D6_PIN  (GPIO_Pin_6)
#define LCD_D7_PORT  (GPIOB)
#define LCD_D7_PIN  (GPIO_Pin_7)

// SDA on port B, bit 14
#define SDA_LOW()  (GPIOB->BRR = GPIO_Pin_14) // set SDA to low
#define SDA_OPEN() (GPIOB->BSRR = GPIO_Pin_14) // set SDA to open-drain
#define SDA_READ   (GPIOB->IDR  & 0x4000)     // read SDA

// SCL on port B, bit 13               /* -- adapt the defines for your uC -- */
#define SCL_LOW()  (GPIOB->BRR = GPIO_Pin_13) // set SCL to low
#define SCL_OPEN() (GPIOB->BSRR = GPIO_Pin_13) // set SCL to open-drain
#define SCL_READ   (GPIOB->IDR  & 0x2000)     // read SCL

void DelayMicroSeconds(u32t nbrOfUs);
void Delay(uint32_t nTime);
void DelayMs(uint32_t mS);
void Delay1Us(uint32_t time);
void Delay10Us(uint32_t time);
void Delay100Us(uint32_t time);
void USART_Config(void);
void SysTickConfig(void);

void SysTick_Handler(void);
void write_cmd(uint8_t cmd);
void LCD_Init(void);
void lcd_busy(void);
void write_data(uint8_t dat);
void lcd_clear(void);
void display_string(uint8_t addr,uint8_t *p);
void display(uint8_t add,uint8_t dat);


#define USART1_CLK                       RCC_APB2Periph_USART1
#define USART1_APBPERIPHCLOCK            RCC_APB2PeriphClockCmd
#define USART1_IRQn                      USART1_IRQn
#define USART1_IRQHandler                USART1_IRQHandler

#define USART1_TX_PIN                    GPIO_Pin_6
#define USART1_TX_GPIO_PORT              GPIOB
#define USART1_TX_GPIO_CLK               RCC_AHBPeriph_GPIOB
#define USART1_TX_SOURCE                 GPIO_PinSource6
#define USART1_TX_AF                     GPIO_AF_0

#define USART1_RX_PIN                    GPIO_Pin_7
#define USART1_RX_GPIO_PORT              GPIOB
#define USART1_RX_GPIO_CLK               RCC_AHBPeriph_GPIOB
#define USART1_RX_SOURCE                 GPIO_PinSource7
#define USART1_RX_AF                     GPIO_AF_0
#define UART1_RX_BUF_SIZE 256
#define UART1_TX_BUF_SIZE 256


void USART1_Config(void);
void USART1_IRQHandler(void);
void USART1_PutChar(uint8_t c);


#endif /* __MAIN_H */

