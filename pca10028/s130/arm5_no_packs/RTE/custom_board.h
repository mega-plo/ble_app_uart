#ifndef CUSTOMBSP_H 
#define CUSTOMBSP_H 
 
#define RTS_PIN_NUMBER   0 
#define TX_PIN_NUMBER     1 
#define CTS_PIN_NUMBER   2 
#define RX_PIN_NUMBER     3 

#define LEDS_NUMBER    2

#define LED_1			05
#define LED_2			06

#define LEDS_LIST { LED_1, LED_2}
#define BSP_LED_0      LED_1
#define BSP_LED_1      LED_2

#define BSP_LED_0_MASK (1<<BSP_LED_0)
#define BSP_LED_1_MASK (1<<BSP_LED_1)
#define LEDS_MASK      (BSP_LED_0_MASK | BSP_LED_1_MASK)
/* all LEDs are lit when GPIO is low */
#define LEDS_INV_MASK  LEDS_MASK

#define BUTTONS_NUMBER 1

#define BUTTON_1		25 		

#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP
#define BUTTONS_LIST { BUTTON_1}
#define BSP_BUTTON_0   BUTTON_1
#define BSP_BUTTON_0_MASK (1<<BSP_BUTTON_0)
#define BUTTONS_MASK   0x00010000

#define AD7798_DOUT		04
#define AD7798_DIN		03
#define AD7798_CS		01		
#define AD7798_SCLK		02

#define SPIS_MISO_PIN   28  // SPI MISO signal.
#define SPIS_CSN_PIN    12  // SPI CSN signal.
#define SPIS_MOSI_PIN   25  // SPI MOSI signal.
#define SPIS_SCK_PIN    29  // SPI SCK signal.

#define SPIM0_SCK_PIN   29  // SPI clock GPIO pin number.
#define SPIM0_MOSI_PIN  25  // SPI Master Out Slave In GPIO pin number.
#define SPIM0_MISO_PIN  28  // SPI Master In Slave Out GPIO pin number.
#define SPIM0_SS_PIN    12  // SPI Slave Select GPIO pin number.

 
// Low frequency clock source to be used by the SoftDevice
#ifdef S210
#define NRF_CLOCK_LFCLKSRC      NRF_CLOCK_LFCLKSRC_XTAL_20_PPM
#else
#define NRF_CLOCK_LFCLKSRC      {.source        = NRF_CLOCK_LF_SRC_XTAL,            \
                                 .rc_ctiv       = 0,                                \
                                 .rc_temp_ctiv  = 0,                                \
                                 .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM}
#endif

#endif 
