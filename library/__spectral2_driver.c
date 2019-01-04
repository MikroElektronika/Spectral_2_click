/*
    __spectral2_driver.c

-----------------------------------------------------------------------------

  This file is part of mikroSDK.

  Copyright (c) 2017, MikroElektonika - http://www.mikroe.com

  All rights reserved.

----------------------------------------------------------------------------- */

#include "__spectral2_driver.h"
#include "__spectral2_hal.c"

/* ------------------------------------------------------------------- MACROS */

// Setup Configuration
const uint8_t _SPECTRAL2_SOFT_RESET       = 0x01 << 6;
const uint8_t _SPECTRAL2_NORMAL_OPERATION = 0x00 << 6;
const uint8_t _SPECTRAL2_INT_ENABLE       = 0x01 << 5;
const uint8_t _SPECTRAL2_INT_DISABLE      = 0x00 << 5;
const uint8_t _SPECTRAL2_GAIN_1X          = 0x00 << 4;
const uint8_t _SPECTRAL2_GAIN_3_7X        = 0x01 << 4;
const uint8_t _SPECTRAL2_GAIN_16X         = 0x02 << 4;
const uint8_t _SPECTRAL2_GAIN_64X         = 0x03 << 4;
const uint8_t _SPECTRAL2_MODE_0           = 0x00 << 2;
const uint8_t _SPECTRAL2_MODE_1           = 0x01 << 2;
const uint8_t _SPECTRAL2_MODE_2           = 0x02 << 2;
const uint8_t _SPECTRAL2_MODE_3           = 0x03 << 2;


// Led Control
const uint8_t _SPECTRAL2_LED_DRV_CURRENT_12_5mA = 0x00 << 4;
const uint8_t _SPECTRAL2_LED_DRV_CURRENT_25mA   = 0x01 << 4;
const uint8_t _SPECTRAL2_LED_DRV_CURRENT_50mA   = 0x02 << 4;
const uint8_t _SPECTRAL2_LED_DRV_CURRENT_100mA  = 0x03 << 4;
const uint8_t _SPECTRAL2_LED_DRV_ENABLE         = 0x01 << 2;
const uint8_t _SPECTRAL2_LED_DRV_DISABLE        = 0x00 << 2;
const uint8_t _SPECTRAL2_LED_IND_CURRENT_1mA    = 0x00 << 1;
const uint8_t _SPECTRAL2_LED_IND_CURRENT_2mA    = 0x01 << 1;
const uint8_t _SPECTRAL2_LED_IND_CURRENT_4mA    = 0x02 << 1;
const uint8_t _SPECTRAL2_LED_IND_CURRENT_8mA    = 0x03 << 1;
const uint8_t _SPECTRAL2_LED_IND_ENABLE         = 0x01;
const uint8_t _SPECTRAL2_LED_IND_DISABLE        = 0x00;

// Sensor Calibrated Data
const uint8_t _SPECTRAL2_CALIBRATED_DATA_V      = 0x14;
const uint8_t _SPECTRAL2_CALIBRATED_DATA_B      = 0x18;
const uint8_t _SPECTRAL2_CALIBRATED_DATA_G      = 0x1C;
const uint8_t _SPECTRAL2_CALIBRATED_DATA_Y      = 0x20;
const uint8_t _SPECTRAL2_CALIBRATED_DATA_O      = 0x24;
const uint8_t _SPECTRAL2_CALIBRATED_DATA_R      = 0x28;

// Sensor Calibrated Data
const uint8_t _SPECTRAL2_DATA_V      = 0x08;
const uint8_t _SPECTRAL2_DATA_B      = 0x0A;
const uint8_t _SPECTRAL2_DATA_G      = 0x0C;
const uint8_t _SPECTRAL2_DATA_Y      = 0x0E;
const uint8_t _SPECTRAL2_DATA_O      = 0x10;
const uint8_t _SPECTRAL2_DATA_R      = 0x12;




/* ---------------------------------------------------------------- VARIABLES */

#ifdef   __SPECTRAL2_DRV_I2C__
static uint8_t _slaveAddress;
#endif

/* -------------------------------------------- PRIVATE FUNCTION DECLARATIONS */

uint8_t _status(uint8_t status);
void *_memcpy(void *dest, void *src, uint8_t n);

/* --------------------------------------------- PRIVATE FUNCTION DEFINITIONS */

uint8_t _status(uint8_t status)
{
     uint8_t readReg[1];
     uint8_t writeReg[1];
     writeReg[0] = 0x00;

     hal_i2cStart();
     hal_i2cWrite(_slaveAddress, writeReg, 1, END_MODE_RESTART);
     hal_i2cRead(_slaveAddress, readReg, 1, END_MODE_STOP);
     if ((readReg[0] & status) != 0)
        return 1;
     else
        return 0;
}

void *_memcpy(void *dest, void *src, uint8_t n)
{
    char *dp = dest;
    char *sp = src;
    
    while (n--)
        *dp++ = *sp++;
    return dest;
}

/* --------------------------------------------------------- PUBLIC FUNCTIONS */

#ifdef   __SPECTRAL2_DRV_SPI__

void spectral2_spiDriverInit(T_SPECTRAL2_P gpioObj, T_SPECTRAL2_P spiObj)
{
    hal_spiMap( (T_HAL_P)spiObj );
    hal_gpioMap( (T_HAL_P)gpioObj );

    // ... power ON
    // ... configure CHIP
}

#endif
#ifdef   __SPECTRAL2_DRV_I2C__

void spectral2_i2cDriverInit(T_SPECTRAL2_P gpioObj, T_SPECTRAL2_P i2cObj, uint8_t slave)
{
    _slaveAddress = slave;
    hal_i2cMap( (T_HAL_P)i2cObj );
    hal_gpioMap( (T_HAL_P)gpioObj );

    // ... power ON
    // ... configure CHIP
}

#endif
#ifdef   __SPECTRAL2_DRV_UART__

void spectral2_uartDriverInit(T_SPECTRAL2_P gpioObj, T_SPECTRAL2_P uartObj)
{
    hal_uartMap( (T_HAL_P)uartObj );
    hal_gpioMap( (T_HAL_P)gpioObj );

    // ... power ON
    // ... configure CHIP
}

#endif

/* ----------------------------------------------------------- IMPLEMENTATION */


uint8_t spectral2_readByte(uint8_t virtualReg)
{
    uint8_t writeReg[2];
    uint8_t readReg[1];
    
    while(_status(0x02)){}
    
    writeReg[0] = 0x01;
    writereg[1] = virtualReg;
    hal_i2cStart();
    hal_i2cWrite(_slaveAddress, writeReg, 2, END_MODE_STOP);
    
    while(_status(0x01)){}
    
    writeReg[0] = 0x02;
    hal_i2cStart();
    hal_i2cWrite(_slaveAddress,writeReg, 1, END_MODE_RESTART);
    Delay_100ms();
    hal_i2cRead(_slaveAddress, readReg, 1, END_MODE_STOP);
    Delay_100ms();
    return readReg[0];
}

void spectral2_reset()
{
   hal_gpio_rstSet(1);
   Delay_100ms();
   hal_gpio_rstSet(0);
   Delay_100ms();
   hal_gpio_rstSet(1);
   Delay_1sec();
   Delay_1sec();
}

void spectral2_Configuration(uint8_t _data)
{
     uint8_t writeReg[2];
     uint8_t virtualReg = 0x04;

     while(_status(0x02)){}

     writeReg[0] = 0x01;
     writeReg[1] = (virtualReg | 0x80);
     hal_i2cStart();
     hal_i2cWrite(_slaveAddress, writeReg, 2, END_MODE_STOP);
    
     Delay_100ms();
     while(_status(0x02)){}
     
     writeReg[0] = 0x01;
     writeReg[1] = _data;
     hal_i2cStart();
     hal_i2cWrite(_slaveAddress, writeReg, 2, END_MODE_STOP);
     
     Delay_100ms();
     Delay_100ms();
}

void spectral2_setIntegrationTime(uint8_t _time)
{
     uint8_t writeReg[2];
     uint8_t virtualReg = 0x05;

     while(_status(0x02)){}

     writeReg[0] = 0x01;
     writeReg[1] = (virtualReg | 0x80);
     hal_i2cStart();
     hal_i2cWrite(_slaveAddress, writeReg, 2, END_MODE_STOP);

     Delay_100ms();
     while(_status(0x02)){}

     writeReg[0] = 0x01;
     writeReg[1] = _time;
     hal_i2cStart();
     hal_i2cWrite(_slaveAddress, writeReg, 2, END_MODE_STOP);
     
     Delay_100ms();
     Delay_100ms();
}

uint8_t spectral2_getTemperature()
{
    uint8_t writeReg[2];
    uint8_t readReg[1];

    while(_status(0x02)){}

    writeReg[0] = 0x01;
    writereg[1] = 0x06;
    hal_i2cStart();
    hal_i2cWrite(_slaveAddress, writeReg, 2, END_MODE_STOP);

    while(_status(0x01)){}

    writeReg[0] = 0x02;
    hal_i2cStart();
    hal_i2cWrite(_slaveAddress,writeReg, 1, END_MODE_RESTART);
    hal_i2cRead(_slaveAddress, readReg, 1, END_MODE_STOP);

    return readReg[0];
}

void spectral2_ledControl(uint8_t _data)
{
     uint8_t writeReg[2];
     uint8_t virtualReg = 0x07;

     while(_status(0x02)){}

     writeReg[0] = 0x01;
     writeReg[1] = (virtualReg | 0x80);
     hal_i2cStart();
     hal_i2cWrite(_slaveAddress, writeReg, 2, END_MODE_STOP);

     Delay_100ms();
     while(_status(0x02)){}

     writeReg[0] = 0x01;
     writeReg[1] = _data;
     hal_i2cStart();
     hal_i2cWrite(_slaveAddress, writeReg, 2, END_MODE_STOP);

     Delay_100ms();
     Delay_100ms();
}

uint16_t spectral2_getData(uint8_t dataReg)
{
    uint16_t value;
    
    value = spectral2_readByte(dataReg);
    value = value << 8;
    value = value | spectral2_readByte(dataReg + 1);

    Delay_100ms();
    Delay_100ms();
    
    return value;
}

float spectral2_getCalibratedData(uint8_t dataReg)
{
    uint32_t Value;
    float floatData;
    
    Value = spectral2_readByte(dataReg);
    Value = Value << 8;
    Value = Value | spectral2_readByte(dataReg + 1);
    Value = Value << 8;
    Value = Value | spectral2_readByte(dataReg + 2);
    Value = Value << 8;
    Value = Value | spectral2_readByte(dataReg + 3);

    _memcpy(&floatData,&Value,4);
    
    return floatData;
}



/* -------------------------------------------------------------------------- */
/*
  __spectral2_driver.c

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