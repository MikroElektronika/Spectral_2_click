/*
    __spectral2_driver.h

-----------------------------------------------------------------------------

  This file is part of mikroSDK.
  
  Copyright (c) 2017, MikroElektonika - http://www.mikroe.com

  All rights reserved.

----------------------------------------------------------------------------- */

/**
@file   __spectral2_driver.h
@brief    Spectral_2 Driver
@mainpage Spectral_2 Click
@{

@image html libstock_fb_view.jpg

@}

@defgroup   SPECTRAL2
@brief      Spectral_2 Click Driver
@{

| Global Library Prefix | **SPECTRAL2** |
|:---------------------:|:-----------------:|
| Version               | **1.0.0**    |
| Date                  | **Jan 2018.**      |
| Developer             | **MikroE Team**     |

*/
/* -------------------------------------------------------------------------- */

#include "stdint.h"

#ifndef _SPECTRAL2_H_
#define _SPECTRAL2_H_

/** 
 * @macro T_SPECTRAL2_P
 * @brief Driver Abstract type 
 */
#define T_SPECTRAL2_P    const uint8_t*

/** @defgroup SPECTRAL2_COMPILE Compilation Config */              /** @{ */

//  #define   __SPECTRAL2_DRV_SPI__                            /**<     @macro __SPECTRAL2_DRV_SPI__  @brief SPI driver selector */
   #define   __SPECTRAL2_DRV_I2C__                            /**<     @macro __SPECTRAL2_DRV_I2C__  @brief I2C driver selector */                                          
// #define   __SPECTRAL2_DRV_UART__                           /**<     @macro __SPECTRAL2_DRV_UART__ @brief UART driver selector */ 

                                                                       /** @} */
/** @defgroup SPECTRAL2_VAR Variables */                           /** @{ */

// Setup Configuration
extern const uint8_t _SPECTRAL2_SOFT_RESET;
extern const uint8_t _SPECTRAL2_NORMAL_OPERATION;
extern const uint8_t _SPECTRAL2_INT_ENABLE;
extern const uint8_t _SPECTRAL2_INT_DISABLE;
extern const uint8_t _SPECTRAL2_GAIN_1X;
extern const uint8_t _SPECTRAL2_GAIN_3_7X;
extern const uint8_t _SPECTRAL2_GAIN_16X;
extern const uint8_t _SPECTRAL2_GAIN_64X;
extern const uint8_t _SPECTRAL2_MODE_0;
extern const uint8_t _SPECTRAL2_MODE_1;
extern const uint8_t _SPECTRAL2_MODE_2;
extern const uint8_t _SPECTRAL2_MODE_3;

// Led Control
extern const uint8_t _SPECTRAL2_LED_DRV_CURRENT_12_5mA ;
extern const uint8_t _SPECTRAL2_LED_DRV_CURRENT_25mA   ;
extern const uint8_t _SPECTRAL2_LED_DRV_CURRENT_50mA   ;
extern const uint8_t _SPECTRAL2_LED_DRV_CURRENT_100mA  ;
extern const uint8_t _SPECTRAL2_LED_DRV_ENABLE         ;
extern const uint8_t _SPECTRAL2_LED_DRV_DISABLE        ;
extern const uint8_t _SPECTRAL2_LED_IND_CURRENT_1mA    ;
extern const uint8_t _SPECTRAL2_LED_IND_CURRENT_2mA    ;
extern const uint8_t _SPECTRAL2_LED_IND_CURRENT_4mA    ;
extern const uint8_t _SPECTRAL2_LED_IND_CURRENT_8mA    ;
extern const uint8_t _SPECTRAL2_LED_IND_ENABLE         ;
extern const uint8_t _SPECTRAL2_LED_IND_DISABLE        ;

extern const uint8_t _SPECTRAL2_CALIBRATED_DATA_V;
extern const uint8_t _SPECTRAL2_CALIBRATED_DATA_B;
extern const uint8_t _SPECTRAL2_CALIBRATED_DATA_G;
extern const uint8_t _SPECTRAL2_CALIBRATED_DATA_Y;
extern const uint8_t _SPECTRAL2_CALIBRATED_DATA_O;
extern const uint8_t _SPECTRAL2_CALIBRATED_DATA_R;

extern const uint8_t _SPECTRAL2_DATA_V ;
extern const uint8_t _SPECTRAL2_DATA_B ;
extern const uint8_t _SPECTRAL2_DATA_G ;
extern const uint8_t _SPECTRAL2_DATA_Y ;
extern const uint8_t _SPECTRAL2_DATA_O ;
extern const uint8_t _SPECTRAL2_DATA_R ;



                                                                       /** @} */

#ifdef __cplusplus
extern "C"{
#endif

/** @defgroup SPECTRAL2_INIT Driver Initialization */              /** @{ */

#ifdef   __SPECTRAL2_DRV_SPI__
void spectral2_spiDriverInit(T_SPECTRAL2_P gpioObj, T_SPECTRAL2_P spiObj);
#endif
#ifdef   __SPECTRAL2_DRV_I2C__
void spectral2_i2cDriverInit(T_SPECTRAL2_P gpioObj, T_SPECTRAL2_P i2cObj, uint8_t slave);
#endif
#ifdef   __SPECTRAL2_DRV_UART__
void spectral2_uartDriverInit(T_SPECTRAL2_P gpioObj, T_SPECTRAL2_P uartObj);
#endif

// GPIO Only Drivers - remove in other cases
void spectral2_gpioDriverInit(T_SPECTRAL2_P gpioObj);
                                                                       /** @} */
/** @defgroup SPECTRAL2_FUNC Driver Functions */                   /** @{ */

/**
 * @brief Function for reset chip
 */
void spectral2_reset();

/**
 * @brief Function for reads one byte
 *
 * @param[in] virtualReg   virtual registry from which one byte is read
 *
 * @return one byte, which is read from the register
 */
uint8_t spectral2_readByte(uint8_t virtualReg);

/**
 * @brief Function for configuration measurements
 *
 * @param[in] _data   data that will be entered in the configuration register
 *
 * Options for Configuration:
     Soft Reset ( 0 - automatically after the reset / 1 - soft reset )
     Enable interrupt pin output ( 1 - Enable / 0 - Disable )
     Sensor Channel Gain Setting ( 1x, 3.7x, 16x or  64x)
     Data Conversion Type:
        Mode 0 -  reads V, B, G and Y data
        Mode 1 -  reads G, Y, O and R data
        Mode 2 -  reads all data
        Mode 3 -  reads V, B, G, Y, O and R in One-Shot mode
 */
void spectral2_Configuration(uint8_t _data);

/**
 * @brief Function for settings integration time
 *
 * @param[in] _time   data that will be written in the integration time register
 *
 * Integration time = <value> * 2.8ms (default 0xFF)
 */
void spectral2_setIntegrationTime(uint8_t _time);

/**
 * @brief Function for reading device temperature
 *
 * @return Device temperature data byte in (°C)
 */
uint8_t spectral2_getTemperature();

/**
 * @brief Function for led control and settings led
 *
 * @param[in] _data   data that will be written in the LED control register
 *
 * Options:
     LED_DRV current limit ( 12.5mA, 25mA, 50mA or 100mA )
     Enable LED_DRV ( 1 - Enable / 0 - Disable )
     LED_IND current limit ( 1mA, 2mA, 4mA or 8mA )
     Enable LED_IND ( 1 - Enable / 0 - Disable )
 */
void spectral2_ledControl(uint8_t _data);

/**
 * @brief Function for reads Data
 *
 * @param[in] dataReg   value from which the filter will be read
 *
 * @return 16 bit data that is read from the register
 *
 */
uint16_t spectral2_getData(uint8_t dataReg);

/**
 * @brief Function for reads calibrated data
 *
 * @param[in] dataReg   value from which the filter will be read
 *
 * @return float data that is read from the register
 *
 */
float spectral2_getCalibratedData(uint8_t dataReg);


                                                                       /** @} */
#ifdef __cplusplus
} // extern "C"
#endif
#endif

/**
    @example Click_Spectral_2_STM.c
    @example Click_Spectral_2_TIVA.c
    @example Click_Spectral_2_CEC.c
    @example Click_Spectral_2_KINETIS.c
    @example Click_Spectral_2_MSP.c
    @example Click_Spectral_2_PIC.c
    @example Click_Spectral_2_PIC32.c
    @example Click_Spectral_2_DSPIC.c
    @example Click_Spectral_2_AVR.c
    @example Click_Spectral_2_FT90x.c
    @example Click_Spectral_2_STM.mbas
    @example Click_Spectral_2_TIVA.mbas
    @example Click_Spectral_2_CEC.mbas
    @example Click_Spectral_2_KINETIS.mbas
    @example Click_Spectral_2_MSP.mbas
    @example Click_Spectral_2_PIC.mbas
    @example Click_Spectral_2_PIC32.mbas
    @example Click_Spectral_2_DSPIC.mbas
    @example Click_Spectral_2_AVR.mbas
    @example Click_Spectral_2_FT90x.mbas
    @example Click_Spectral_2_STM.mpas
    @example Click_Spectral_2_TIVA.mpas
    @example Click_Spectral_2_CEC.mpas
    @example Click_Spectral_2_KINETIS.mpas
    @example Click_Spectral_2_MSP.mpas
    @example Click_Spectral_2_PIC.mpas
    @example Click_Spectral_2_PIC32.mpas
    @example Click_Spectral_2_DSPIC.mpas
    @example Click_Spectral_2_AVR.mpas
    @example Click_Spectral_2_FT90x.mpas
*/                                                                     /** @} */
/* -------------------------------------------------------------------------- */
/*
  __spectral2_driver.h

  Copyright (c) 2017, MikroElektonika - http://www.mikroe.com

  All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

3. All advertising materials mentioning features or use of this software
   must display the following acknowledgement:
   This product includes software developed by the MikroElektonika.

4. Neither the name of the MikroElektonika nor the
   names of its contributors may be used to endorse or promote products
   derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY MIKROELEKTRONIKA ''AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL MIKROELEKTRONIKA BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------- */