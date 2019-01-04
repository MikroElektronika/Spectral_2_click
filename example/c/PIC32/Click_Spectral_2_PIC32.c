/*
Example for Spectral_2 Click

    Date          : Jan 2018.
    Author        : MikroE Team

Test configuration PIC32 :
    
    MCU                : P32MX795F512L
    Dev. Board         : EasyPIC Fusion v7
    PIC32 Compiler ver : v4.0.0.0

---

Description :

The application is composed of three sections :

- System Initialization - Initializes I2C module, RST pin as OUTPUT and INT pin as INPUT
- Application Initialization - Driver initialize, reset module and configuration measurement
- Application Task - (code snippet) - Reads the brightness value with R, G, B, I, O and V filter,
                                      every 1 second, and logs on to USBUART.
*/

#include "Click_Spectral_2_types.h"
#include "Click_Spectral_2_config.h"

char fText[50];
float fData;
uint8_t temp;

void systemInit()
{
    mikrobus_gpioInit( _MIKROBUS1, _MIKROBUS_INT_PIN, _GPIO_INPUT );
    mikrobus_gpioInit( _MIKROBUS1, _MIKROBUS_RST_PIN, _GPIO_OUTPUT );
    mikrobus_i2cInit( _MIKROBUS1, &_SPECTRAL2_I2C_CFG[0] );
    mikrobus_logInit( _LOG_USBUART_A, 9600 );
    Delay_ms( 100 );
}

void applicationInit()
{
    spectral2_i2cDriverInit( (T_SPECTRAL2_P)&_MIKROBUS1_GPIO, (T_SPECTRAL2_P)&_MIKROBUS1_I2C, 0x49 );
    spectral2_reset();
    Delay_100ms();
    mikrobus_logWrite("--- System init ---", _LOG_LINE);


    spectral2_Configuration(_SPECTRAL2_NORMAL_OPERATION |
                            _SPECTRAL2_INT_DISABLE |
                            _SPECTRAL2_GAIN_16X |
                            _SPECTRAL2_MODE_2);
    Delay_1sec();
}

void applicationTask()
{
    mikrobus_logWrite("-------------------",_LOG_LINE);

    fData = spectral2_getCalibratedData(_SPECTRAL2_CALIBRATED_DATA_R);
    FloatToStr(fData,fText);
    mikrobus_logWrite("-- R ( Red data ) :",_LOG_TEXT);
    mikrobus_logWrite(fText,_LOG_LINE);
    
    fData = spectral2_getCalibratedData(_SPECTRAL2_CALIBRATED_DATA_G);
    FloatToStr(fData,fText);
    mikrobus_logWrite("-- G ( Green data ) :",_LOG_TEXT);
    mikrobus_logWrite(fText,_LOG_LINE);
    
    fData = spectral2_getCalibratedData(_SPECTRAL2_CALIBRATED_DATA_B);
    FloatToStr(fData,fText);
    mikrobus_logWrite("-- B ( Blue data ) :",_LOG_TEXT);
    mikrobus_logWrite(fText,_LOG_LINE);
    
    fData = spectral2_getCalibratedData(_SPECTRAL2_CALIBRATED_DATA_Y);
    FloatToStr(fData,fText);
    mikrobus_logWrite("-- Y ( Yellow data ) :",_LOG_TEXT);
    mikrobus_logWrite(fText,_LOG_LINE);
    
    fData = spectral2_getCalibratedData(_SPECTRAL2_CALIBRATED_DATA_O);
    FloatToStr(fData,fText);
    mikrobus_logWrite("-- O ( Orange data ) :",_LOG_TEXT);
    mikrobus_logWrite(fText,_LOG_LINE);
    
    fData = spectral2_getCalibratedData(_SPECTRAL2_CALIBRATED_DATA_V);
    FloatToStr(fData,fText);
    mikrobus_logWrite("-- V ( Violet data ) :",_LOG_TEXT);
    mikrobus_logWrite(fText,_LOG_LINE);
    
    Delay_1sec();
}

void main()
{
    systemInit();
    applicationInit();

    while (1)
    {
            applicationTask();
    }
}