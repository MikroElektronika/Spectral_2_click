![MikroE](http://www.mikroe.com/img/designs/beta/logo_small.png)

---

# Spectral_2 Click

- **CIC Prefix**  : SPECTRAL2
- **Author**      : Katarina Perendic
- **Verison**     : 1.0.0
- **Date**        : Jan 2018.

---

### Software Support

We provide a library for the Spectral_2 Click on our [LibStock](https://libstock.mikroe.com/projects/view/2349/spectral-2-click) 
page, as well as a demo application (example), developed using MikroElektronika 
[compilers](http://shop.mikroe.com/compilers). The demo can run on all the main 
MikroElektronika [development boards](http://shop.mikroe.com/development-boards).

**Library Description**

This library will allow you to read the RGB values from the Spectral 2 click board sensor.

Key functions :
- ```void spectral2_reset(); ``` - Function for doing a software reset on the sensor.
- ```void spectral2_Configuration(uint8_t _data); ``` - This function is used for configuring diferent operation modes of the sensor.
- ```float spectral2_getCalibratedData(uint8_t dataReg);``` - Function for reading the RGB values depending on the parameter passed.

**Examples Description**

The application is composed of three sections :

- System Initialization - Initializes I2C module, RST pin as OUTPUT and INT pin as INPUT
- Application Initialization - Driver initialize, reset module and configuration measurement
- Application Task - (code snippet) - Reads the brightness value with R, G, B, I, O and V filter,
                                      every 1 second, and logs on to USBUART.

```
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
```

The full application code, and ready to use projects can be found on our 
[LibStock](https://libstock.mikroe.com/projects/view/2349/spectral-2-click) page.

Other mikroE Libraries used in the example:

- UART Library
- Conversions Library
- C_String Library
- I2C Library

**Additional notes and informations**

Depending on the development board you are using, you may need 
[USB UART click](http://shop.mikroe.com/usb-uart-click), 
[USB UART 2 Click](http://shop.mikroe.com/usb-uart-2-click) or 
[RS232 Click](http://shop.mikroe.com/rs232-click) to connect to your PC, for 
development systems with no UART to USB interface available on the board. The 
terminal available in all Mikroelektronika 
[compilers](http://shop.mikroe.com/compilers), or any other terminal application 
of your choice, can be used to read the message.

---
---
