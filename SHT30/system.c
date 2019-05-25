//==============================================================================
//    S E N S I R I O N   AG,  Laubisruetistr. 44, CH-8712 Staefa, Switzerland
//==============================================================================
// Project   :  SHT3x Sample Code (V1.0)
// File      :  system.c (V1.0)
// Author    :  RFU
// Date      :  16-Jun-2014
// Controller:  STM32F100RB
// IDE       :  µVision V4.71.2.0
// Compiler  :  Armcc
// Brief     :  System functions
//==============================================================================

//-- Includes ------------------------------------------------------------------
#include "system.h"

//==============================================================================
void SystemInit(void){
//==============================================================================
  // no initialization required
}

//==============================================================================
void DelayMicroSeconds(u32t nbrOfUs){   /* -- adapt this delay for your uC -- */
//==============================================================================
  u32t i;
  for(i = 0; i < nbrOfUs; i++)
  {  
    __nop();  // nop's may be added or removed for timing adjustment
    __nop();
    __nop();
    __nop();
  }
}
