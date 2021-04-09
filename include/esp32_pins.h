#pragma once

#ifndef _ESP32_PINS_H_
#define _ESP32_PINS_H_
// ON-BOARD
#define LED_BUILTIN 2

#define ESP32_PINS_JTAG 1
//#define ESP32_PINS_SCOPE 1
//#define ESP32_PINS_NORMAL 1

#if defined(ESP32_PINS_JTAG)
    // ESP DEVKIT V1 w/ 30pin version
    // JTAG CONFIG
    // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/jtag-debugging/configure-other-jtag.html

    // LEFT SIDE PINS TOP-BOTTOM
    //PIN1  nBOOTLOADER/ENABLE
    #define LTC_INPUT_PIN         36  // USED ADC1_CH0
    #define INPUT_PIN39           39  //      ADC1_CH3
    #define INPUT_PIN34           34  //      ADC1_CH6
    #define INPUT_PIN35           35  //      ADC1_CH7
    #define SCOPE_INT_32KHZ       32  // USED ADC1_CH4
    #define SCOPE_INT_1HZ         33  // USED ADC1_CH5
    #define LTC_OUTPUT_PIN        25  // USED ADC2_CH8 DAC1
    #define ADC2_CH9              26  //      ADC2_CH9 DAC2
    #define SCOPE_TIMECODE_FRAME  27  // USED ADC2_CH7
    #define JTAG_TMS              14  // USED ADC2_CH6 / JTAG TMS
    #define JTAG_TDI              12  // USED ADC2_CH5 / JTAG TDI
    #define JTAG_TCK              13  // USED ADC2_CH4 / JTAG TCK
    //PIN14 Gnd
    //PIN15 Vin

    // RIGHT SIDE PINS TOP-BOTTOM
    #define SCL1                  23  // USED VSPI_MISO
    #define SCL                   22  // USED
    #define USB_UART0TX           1   // USED
    #define USB_UART0RX           3   // USED
    #define SDA                   21  // USED
    #define SDA1                  19  // USED VSPI_MISO
    #define INT_32KHZ_PIN         18  // USED VSPI_CLK
    #define INT_1HZ_PIN           5   // USED VSPI_CS0
    #define SCOPE_TIMECODE_TASK   17  // USED UART2TX
    #define SCOPE_OLED            16  // USED UART2RX
    #define SCOPE_DS3231          4   // USED ADC2CH0
    #define LED_BUILTIN           2   // USED ADC2CH2
    #define JTAG_TDO              15  // USED ADC2CH3 / JTAG TDO
    //PIN29 Gnd
    //PIN30 3.3v out

#elif defined(ESP32_PINS_SCOPE)
    // ESP DEVKIT V1 w/ 30pin version
    // LEFT SIDE PINS TOP-BOTTOM
    //PIN1  nBOOTLOADER/ENABLE
    #define LTC_INPUT_PIN         36  // USED ADC1_CH0
    #define INPUT_PIN39           39  //      ADC1_CH3
    #define INPUT_PIN34           34  //      ADC1_CH6
    #define INPUT_PIN35           35  //      ADC1_CH7
    #define SCOPE_INT_32KHZ       32  //      ADC1_CH4
    #define SCOPE_INT_1HZ         33  //      ADC1_CH5
    #define LTC_OUTPUT_PIN        25  //      ADC2_CH8 DAC1
    #define SCOPE_OLED            26  // USED ADC2_CH9 DAC2
    #define SCOPE_DS3231          27  // USED ADC2_CH7
    #define SCOPE_TIMECODE_FRAME  14  // USED ADC2_CH6
    #define SCOPE_TIMECODE_SAMPLE 12  // USED ADC2_CH5
    #define SCOPE_TIMECODE_TASK   13  // USED ADC2_CH4
    //PIN14 Gnd
    //PIN15 Vin

    // RIGHT SIDE PINS TOP-BOTTOM
    #define SCL1                  23  // USED VSPI_MISO
    #define SCL                   22  // USED
    #define USB_UART0TX           1   // USED
    #define USB_UART0RX           3   // USED
    #define SDA                   21  // USED
    #define SDA1                  19  // USED VSPI_MISO
    #define INT_32KHZ_PIN         18  // USED VSPI_CLK
    #define INT_1HZ_PIN           5   // USED VSPI_CS0
    #define UART2TX               17
    #define UART2RX               16
    #define ADC2CH0               4
    #define ADC2CH2               2   // Also LED_BUILTIN
    #define ADC2CH3               15
    //PIN29 Gnd
    //PIN30 3.3v out

#elif defined(ESP32_PINS_NORMAL)
    // ESP DEVKIT V1 w/ 36pin version
    // LEFT SIDE PINS TOP-BOTTOM
    //PIN1  nBOOTLOADER/ENABLE
    #define LTC_INPUT_PIN         36  // USED ADC1_CH0
    #define INPUT_PIN39           39  //      ADC1_CH3
    #define INPUT_PIN34           34  //      ADC1_CH6
    #define INPUT_PIN35           35  //      ADC1_CH7
    #define SCOPE_INT_32KHZ       32  //      ADC1_CH4
    #define SCOPE_INT_1HZ         33  //      ADC1_CH5
    #define LTC_OUTPUT_PIN        25  //      ADC2_CH8 DAC1
    #define SCOPE_OLED            26  // USED ADC2_CH9 DAC2
    #define SCOPE_DS3231          27  // USED ADC2_CH7
    #define SCOPE_TIMECODE_FRAME  14  // USED ADC2_CH6
    #define SCOPE_TIMECODE_SAMPLE 12  // USED ADC2_CH5
    #define SCOPE_TIMECODE_TASK    13  // USED ADC2_CH4
    #define ONBOARD_FLASH_SD2     9   // USED
    #define ONBOARD_FLASH_SD3     10  // USED
    #define ONBOARD_FLASH_CMD     11  // USED
    //PIN17 Gnd
    //PIN18 Vin

    // RIGHT SIDE PINS TOP-BOTTOM
    #define SCL1                  23  // USED VSPI_MISO
    #define SCL                   22  // USED
    #define USB_UART0TX           1   // USED
    #define USB_UART0RX           3   // USED
    #define SDA                   21  // USED
    #define SDA1                  19  // USED VSPI_MISO
    #define INT_32KHZ_PIN         18  // USED VSPI_CLK
    #define INT_1HZ_PIN           5   // USED VSPI_CS0
    #define UART2TX               1
    #define UART2RX               3
    #define ADC2CH0               4
    #define ADC2CH2               2   // Also LED_BUILTIN
    #define ADC2CH3               15
    #define ADC2CH1               0
    #define ONBOARD_FLASH_SD1     8
    #define ONBOARD_FLASH_SD0     7
    #define ONBOARD_FLASH_CLK     6
    //PIN36 3.3v out
#else
#error ESP32-PINS - define which pin mapping you want to use
#endif

#endif // #ifndef _ESP32_PINS_H_
