//*----------------------------------------------------------------------------
//* ADX-ddsPIO.h
//*----------------------------------------------------------------------------
//* Constants and prototypes for the ADX-ddsPIO firmware
//* 
//* This is a working module of an integration effort to build an FT8 
//* transceiver using Raspberry Pi Pico as the main controller and signal
//* generator.
//* 
//* Copyright (c) 2025 by Pedro Colla (LU7DZ)
//*----------------------------------------------------------------------------

/*-------------------------------------------------
   IDENTIFICATION DIVISION.
   (just a programmer's joke)
---------------------------------------------------*/
#define PROGNAME "ADX-ddsPIO"
#define AUTHOR "Dr. Pedro E. Colla (LU7DZ)"
#define VERSION  "1.0"
#define BUILD     "00"


//*==============================================================================================*
//*                                  Environment  & build definitions                            *
//*==============================================================================================*
#define DEBUG    1
#define TRACE    1
//#define FT8TEST  1

#define BOOL2CHAR(x)  (x==true ? "True" : "False")
#undef  _NOP
#define _NOP (byte)0


//*==============================================================================================*
//*                                  Macros and Structures                                       *
//*==============================================================================================*

//*----------------------------------------------------------------------------------------------*
//* Expands a decorated printf-like pseudo-instruction to add debug messages and control them
//*----------------------------------------------------------------------------------------------*
char hi[128];

#ifdef DEBUG
#define cdc_printf(fmt, ...)                           \
    do {                                                \
        int _cdc_len = snprintf(hi,               \
                                sizeof(hi),       \
                                (fmt), ##__VA_ARGS__);  \
        if (_cdc_len > 0) {                             \
            if (_cdc_len > (int)sizeof(hi))       \
                _cdc_len = sizeof(hi);            \
            cdc_write(hi, (uint16_t)_cdc_len);    \
            tud_cdc_write_flush();                \
        }                                               \
    } while (0)
#else //!DEBUG
#define cdc_printf(fmt, ...) (void)0
#endif //cdc_printf macro definition as NOP when not in debug mode, will consume one byte of nothingness

#ifdef DEBUG
#define _printf(fmt, ...) \
    printf("@[%s] " fmt, __func__, ##__VA_ARGS__)
#else  //!DEBUG  
    #define _printf(fmt, ...) (void)0
#endif //DEBUG




/*---------------------------------------------------
 * Define band support
*/
#define NBANDS                7
#define NMODES                4
#define SLOT                  3

#ifdef REWORK
#define N_FREQ                2      // number of using RF frequencies with push switch　(<= 7)
#define FREQ_0          7041000      // RF frequency in Hz
#define FREQ_1          7074000     // in Hz
#endif //REWORK

#define GEN_FRQ_HZ     14074000L    /*Generator Frequency (in Hz)*/
#define FT8_BASE_HZ        1000L    /* FT8 base frequency (in Hz) */
#define AUDIOSAMPLING     48000     // USB Audio sampling frequency (fixed)
#define FSK_ERROR             3     //Minimum frequency shift to change DCO
#define FSKMIN              300     //Minimum FSK frequency computed
#define FSKMAX             3000     //Maximum FSK frequency computed


/*----
   RF and signal pin
*/
#define RFOUT          18    //RF Output Enable
#define RXA            26U   //RX Input for ADC (A0)
#define FSKpin         27    //Frequency counter algorithm, signal input PIN (warning changes must also be done in freqPIO)
#define pin_SW          3U   //pin for freq change switch (D10,input)

/*----
   Output control lines
*/
#define RXSW            2  //RXSW Switch (RX/TX control)

/*---
   LED
*/
#define TX              3  //TX LED
#define FT8             4  //FT8 LED
#define FT4             5  //FT4 LED
#define JS8             6  //JS8 LED
#define WSPR            7  //WSPR LED

/*---
   Calibration signal
*/
#define CAL             9  //Calibration   

/*---
   Switches
*/
#define TXSW            8  //RX-TX Switch (Enable TX Led and RFOUT to RF Driver)

#define UP             10  //UP Switch
#define DOWN           11  //DOWN Switch
#define BEACON         12  //BEACON Jumper
#define SYNC           13  //Time SYNC Switch


//*==============================================================================================*
//*                                  Constants and Hardware defs                                 *
//*==============================================================================================*






//*==============================================================================================*
//*                                  Global Memory Areas                                         *
//*==============================================================================================*
#ifdef REWORK
uint64_t Freq_table[N_FREQ]={FREQ_0,FREQ_1}; // Freq_table[N_FREQ]={FREQ_0,FREQ_1, ...}
#endif //REWORK

