#ifndef CUSTOMBSP_H 
#define CUSTOMBSP_H 
 
#define RTS_PIN_NUMBER   0 
#define TX_PIN_NUMBER     1 
#define CTS_PIN_NUMBER   2 
#define RX_PIN_NUMBER     3 

#define Push_Button		25 		
#define LED_1			05
#define LED_2			06
#define AD7798_DOUT		04
#define AD7798_DIN		03
#define AD7798_CS		01		
#define AD7798_SCLK		02
 
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
