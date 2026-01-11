/*
 * Copyright (C) 2026- Dr. Pedro E. Colla (LU7DZ) <pedro.colla@gmail.com>
 * Version 0.x (family of initial development and integration)
 * 
 * ADX class digital transceiver for low signal modes based on the rp2040 Raspberry Pi Pico
 * processor
 * 
 * =======================================================================================
 * This is mainly an integration effort with some new code developed for this project
 *  
 * The integration effort is being built on top of previous work from many parties,
 * including myself as follows:
 * 
 *----------------------------------------------------------------------------
 * 
 * pico-hf-oscillator rp2040 DDS firmware by  Roman Piksaykin (R2BDY):
 * Digital Controlled Oscillator for Raspberry Pi Pico
 * https://www.qrz.com/db/r2bdy
 *
 *----------------------------------------------------------------------------
 * 
 * ADX transceiver original hardware and firmware design 
 * ADX_UNO_V1.1 - Version release date: 08/05/2022
 * by Barb(Barbaros ASUROGLU) - WB2CBA - 2022
 * https://github.com/WB2CBA/ADX
 * 
 *----------------------------------------------------------------------------
 * 
 * QP-7C_rp2040 digital mode transceiver with CAT control 
 * Copyright (C) 2023- Hitoshi Kawaji <je1rav@gmail.com>
 * https://github.com/je1rav/QP-7C_RP2040_CAT
 *
 *----------------------------------------------------------------------------
 * 
 * ft8_lib FT8 library
 * Copyright (C) 2018 by Karliss Goba (YL3JG)
 * https://github.com/kgoba/ft8_lib
 * 
 * This library is not part of the ADX-ddsPIO firmware as no actual FT8
 * coding is generated on the board, however it's a fundamental tool for
 * the development and integration process, without it the project would have
 * been considerably more difficult
 *
 *----------------------------------------------------------------------------
 * 
 * ADX-rp2040 digital transceiver using the rp2040 (firmware and hardware)
 * Copyright (C) 2024- Dr. Pedro E. Colla (LU7DZ) <pedro.colla@gmail.com>
 * 
 * Original porting of the ADX Arduino based firmware to the rp2040 architecture
 *
 *----------------------------------------------------------------------------
 *
 * And, of course....
 * 
 * The WSJT-X authors, who developed a very interesting and novel communications protocol
 * The details of FT4 and FT8 procotols and decoding/encoding are described here:
 * https://physics.princeton.edu/pulsar/k1jt/FT4_FT8_QEX.pdf
 *
 * This program (firmware) do not generate nor decode FT8 signals, therefore
 * no part of the WSJT-X code has been involved. But the board is intended to
 * be used by a program such as WSJT-X, therefore the acknowledgement is in 
 * order
 *----------------------------------------------------------------------------
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

//*==============================================================================================*
//*                                  Includes and Source Libraries                               *
//*==============================================================================================*
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include "bsp/rp2040/board.h"
#include "tusb.h"
#include "usb_descriptors.h"
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include <hardware/watchdog.h>    //watchdog
#include "usb_audio.h"
#include <inttypes.h>

//*============================================================================*/
//*                           includes and libraries                           */
//*============================================================================*/
#include <stdint.h>
#include "defines.h"
#include "piodco/piodco.h"
#include "./build/dco2.pio.h"
#include "hardware/vreg.h"
#include "pico/multicore.h"
#include "pico/stdio/driver.h"
#include "hardware/rtc.h"
#include "pico/util/datetime.h"
#include "./lib/assert.h"
#include "./debug/logutils.h"
#include "hwdefs.h"
#include "gpstime/GPStime.h"
#include "hfconsole/hfconsole.h"

#ifdef REWORK 
#include "./ft8_lib/common/wave.h"
#include "./ft8_lib/ft8/pack.h"
#include "./ft8_lib/ft8/encode.h"
#include "./ft8_lib/ft8/constants.h"
#endif //REWORK

#include "protos.h"
#include "ADX-ddsPIO.h"

//*==============================================================================================*
//*                                  Global Memory Areas                                         *
//*==============================================================================================*


//*--- for ADC offset at receiving
int32_t adc_offset = 0;   

//*--- for tranceiver
//*fix* uint64_t RF_freq;   // RF frequency (Hz)
//*fix* uint64_t audio_freq_prev=0.0;

//*--- Control the TX function and the frequency decoding from the incoming USB (digital) audio

//*fix* int C_freq = 0;    // FREQ_x: Index of RF frequency. In this case, FREQ_0 is selected as the initial frequency.
//*fix* int Tx_Status = 0; // 0=RX, 1=TX
//*fix* int Tx_Start = 0;  // 0=RX, 1=TX
//*fix* int not_TX_first = 0;

uint32_t Tx_last_mod_time;
uint32_t Tx_last_time;
uint32_t push_last_time;  // to detect the long puch for frequency change by push switch

//*--- for determination of Audio signal frequency 

int16_t mono_prev=0;  
int16_t mono_preprev=0;  
float delta_prev=0;
int16_t sampling=0;
int16_t cycle=0;
uint32_t cycle_frequency[136];

//*--- definitions and areas for USB Audio (IN/OUT)

int16_t monodata[CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ / 4];
int16_t adc_data[CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ / 2];
int16_t pcCounter;
int audio_read_number=0;

//*--- definitions for USB serial (CDC) interface

char cdc_read_buf[64];
char cdc_write_buf[64];

//*============================================================================*/
//*                          Memory area definitions                           */
//*============================================================================*/
PioDco DCO; /* External in order to access in both cores. */

//*--- Operating mode flags

bool upButton, downButton, txButton, syncButton;
bool prevUpButton = true, prevDownButton = true, prevTxButton = true, prevSyncButton = true;


//*--- Global Transceiver status

bool  bTX = false;

//*--- Frequency variables 

uint32_t frqFT8  = GEN_FRQ_HZ;
uint32_t prevfrq = 0;
uint32_t baseFT8 = FT8_BASE_HZ;

//*==============================================================================================*
//*                                         BAND SELECT                                          *
//*==============================================================================================*
// ADX can support up to 4 bands on board. Those 4 bands needs to be assigned to Band1 ... Band4
// from supported 8 bands.
// To change bands press UP and DOWN simultaneously. 
// The Band LED will flash 3 times briefly and stay lit for the stored band. also TX LED will be lit to indicate
// that Band select mode is active. Now change band bank by pressing SW1(<---) or SW2(--->). When
// desired band bank is selected press TX button briefly to exit band select mode.
// Now the new selected band bank will flash 3 times and then stored mode LED will be lit.
// TX won't activate when changing bands so don't worry on pressing TX button when changing bands in
// band mode.
// Assign your prefered bands to B1,B2,B3 and B4
// Supported Bands are: 80m, 40m, 30m, 20m,17m, 15m, 10m
//*----------------------------------------------------------------------------------------------*
int slot[4]   = {40,30,20,10};
int Band_slot = SLOT;                  //This is the default band Band1=1,Band2=2,Band3=3,Band4=4
int Band      =   20;                  //This is the default band
int mode      =    4;                  //Default mode is FT8

long unsigned int Bands[NBANDS][NMODES] = {
                                          { 3568600, 3578000, 3575000, 3573000},
                                          { 7038600, 7078000, 7047500, 7074000},
                                          {10138700,10130000,10140000,10136000},
                                          {14095600,14078000,14080000,14074000},
                                          {18104600,18104000,18104000,18100000},
                                          {21094600,21078000,21140000,21074000},
                                          {28124600,28078000,28180000,28074000}};
//*----------------------------------------------------------------------------------------------*
//*============================================================================*/
//*                       Board Management Functions                           */
//*============================================================================*/
void blinkLED(uint8_t _gpio, uint ms)
{
    gpio_put(_gpio, 1);
    sleep_ms(ms);
    gpio_put(_gpio, 0);
    sleep_ms(ms);
}
//*----------------------------------------------------------------------------*/
//* Init the board hardware and set the default led, also setup the CPU clock  */
//*----------------------------------------------------------------------------*/
void initBoard(){
    
    //* Set system clock to maximum (270 MHz)
    set_sys_clock_khz((PLL_SYS_MHZ * 1000000UL) / 1000UL, true);


    //* Initialize stdio and other hardware elements
    stdio_init_all();
    sleep_ms(1000);
 
    //*--- Initialize the default LED (board) pin
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

}
//*----------------------------------------------------------------------------*/
//* Setup I/O for the ADX board controls (LED, switches and jumpers            */
//*----------------------------------------------------------------------------*/
void ADXsetup(){
    
    gpio_init(RXSW);
    gpio_set_dir(RXSW, GPIO_OUT);   
    gpio_put(RXSW, 1); //Set RX mode

    gpio_init(FT8);
    gpio_set_dir(FT8, GPIO_OUT);   
    gpio_put(FT8, 0); //Turn OFF FT8 LED
 
    gpio_init(FT4);
    gpio_set_dir(FT4, GPIO_OUT);   
    gpio_put(FT4, 0); //Turn OFF FT4 LED

    gpio_init(JS8);
    gpio_set_dir(JS8, GPIO_OUT);
    gpio_put(JS8, 0); //Turn OFF JS8 LED

    gpio_init(WSPR);
    gpio_set_dir(WSPR, GPIO_OUT);
    gpio_put(WSPR, 0); //Turn OFF WSPR LED    

    gpio_init(TX);
    gpio_set_dir(TX, GPIO_OUT);
    gpio_put(TX, 0); //Set RX mode (off)

//*--- (Input switches)

    gpio_init(TXSW);
    gpio_set_dir(TXSW, GPIO_IN);

    gpio_init(UP);
    gpio_set_dir(UP, GPIO_IN);

    gpio_init(DOWN);
    gpio_set_dir(DOWN, GPIO_IN);    

    gpio_init(SYNC);
    gpio_set_dir(SYNC, GPIO_IN);
    
    gpio_init(BEACON);
    gpio_set_dir(BEACON, GPIO_IN);

    //*--- End of ADX control board initialization

}
/*----------------------------------------------------------------------------*/
/* Test a single button                                                       */
/*----------------------------------------------------------------------------*/
bool testButton(uint gpio) {
    bool bstate = gpio_get(gpio);
    return bstate;
}
/*----------------------------------------------------------------------------*/
/* Test buttons and light LEDs accordingly                                    */
/*----------------------------------------------------------------------------*/
void testLED() {

upButton = testButton(UP);
downButton = testButton(DOWN);
txButton = testButton(TXSW);
syncButton = testButton(SYNC);

if (upButton == false) {
    if (prevUpButton == true) {
        cdc_printf("UP button pressed\n");
        gpio_put(FT8, 1);
        prevUpButton = false;
    } 
} else {
    if (prevUpButton == false) {
        cdc_printf("UP button released\n");
        gpio_put(FT8, 0);
        prevUpButton = true;    
    }            
}

if (downButton == false) {
    if (prevDownButton == true) {
        cdc_printf("DOWN button pressed\n");
        gpio_put(FT4, 1);
        prevDownButton = false;
   } 
} else {
    if (prevDownButton == false) {
        cdc_printf("DOWN button released\n");
        gpio_put(FT4, 0);
        prevDownButton = true;    
    }            
}


if (txButton == false) {
    if (prevTxButton == true) {
        cdc_printf("TX button pressed\n");
        gpio_put(TX, 1);
        prevTxButton = false;
    } 
} else {
    if (prevTxButton == false) {
        cdc_printf("TX button released\n");
        gpio_put(TX, 0);
        prevTxButton = true;    
    }            
}

if (syncButton == false) {
    if (prevSyncButton == true) {
        cdc_printf("SYNC button pressed\n");
        gpio_put(WSPR, 1);
        prevSyncButton = false;
    } 
} else {
    if (prevSyncButton == false) {
        cdc_printf("SYNC button released\n");
        gpio_put(WSPR, 0);
        prevSyncButton = true;    
    }            
}
       
}
/*----------------------------------------------------------------------------*/
/* Convert slot to band                                                       */
/*----------------------------------------------------------------------------*/
int slot2Band(int s) {
  if (s < 1 || s > 4) {
    s = 4;
  }
  int b=slot[s-1];
  cdc_printf("Slot(%d) --> Band(%d)\n", s, b);
  return b;
}
/*----------------------------------------------------------------------------*/
/* Convert band number to index                                               */
/*----------------------------------------------------------------------------*/
int band2idx(int b){

int i=0;

switch(b) {
  case 80: i=0; break;
  case 40: i=1; break;
  case 30: i=2; break;
  case 20: i=3; break;
  case 17: i=4; break;
  case 15: i=5; break;
  case 10: i=6; break;

  default:
    i=6; break;
}
cdc_printf("band(%d) idx(%d)\n", b, i);
return i;

}
/*----------------------------------------------------------------------------*/
/* Clear all LEDS                                                             */
/*----------------------------------------------------------------------------*/
void clearLED() {
  
  gpio_put(FT8, 0);
  gpio_put(FT4, 0);
  gpio_put(JS8, 0);
  gpio_put(WSPR, 0);
  
}
/*----------------------------------------------------------------------------*/
/* Assign band                                                                */
/*----------------------------------------------------------------------------*/
void Mode_assign() {

  cdc_printf("Assigning mode(%d) for Band(%d)\n", mode, Band);
  int b=band2idx(Band);
  frqFT8=Bands[b][mode-1];
  clearLED();
  switch(mode) {
     case 1: gpio_put(WSPR,true);break;
     case 2: gpio_put(JS8,true);break;
     case 3: gpio_put(FT4,true);break;
     case 4: gpio_put(FT8,true);break;
  }
  cdc_printf("transceiver mode mode(%d) Band(%d) index(%d) freq(%ld)\n", mode, Band, b, frqFT8);

}

/*----------------------------------------------------------------------------*/
/* Assign Band                                                                */
/*----------------------------------------------------------------------------*/
void Band_assign() {

  clearLED();
  Band=slot2Band(Band_slot);
  switch(Band_slot) {
     case 0: blinkLED(FT8,100); break;
     case 1: blinkLED(FT4,100); break;
     case 2: blinkLED(JS8,100); break;
     case 3: blinkLED(WSPR,100); break;
  }
  Mode_assign();

  cdc_printf("band_slot=%d mode=%d band=%d\n",Band_slot, mode, Band);
}

/*----------------------------------------------------------------------------*/
/* Multiply the tone shift accounting for the Hz and milliHz                  */                                         
/*----------------------------------------------------------------------------*/
void computeFSK(uint8_t index,
                uint32_t *Hz,
                uint32_t *mHz)
{
    /* This is the FT8 FSK whole index {0..7} * 6*/
    *Hz = index * 6;

    /* Fractional part: index * 0.25 = (index % 4) * 250 milliseconds */
    //*fix* *mHz = (index * 250) % 1000;
    *mHz = (uint32_t)(((uint32_t)index * 250u) % 1000u);


    /* Adjust if the fraction exceeds 1 unit */
    if (index >= 4) {
        *Hz += index / 4;
    }
}
/*----------------------------------------------------------------------------*/
/* Place transmitter in TX manually if the button TX is pressed               */                                         
/*----------------------------------------------------------------------------*/
void ManualTX() {

  bool TXSW_State = true;
  PioDCOStart(&DCO);
  PioDCOSetFreq(&DCO, GEN_FRQ_HZ, 0UL);

  setTX(true);
  cdc_printf("Manual TX activated\n");

TXON:
  TXSW_State = testButton(TXSW);
  if (TXSW_State == true) {
    goto EXIT_TX;

  }
  goto TXON;

EXIT_TX:
  setTX(false);
  cdc_printf("Manual TX deactivated\n");

}
/*----------------------------------------------------------------------------*/
/* Change the band                                                            */                                         
/*----------------------------------------------------------------------------*/
void Band_Select() {


  gpio_put(TX,true);
  clearLED();
  for (int i=0;i<3;i++) {
    blinkLED(FT8,1000);
  }

Band_cont:
 
  clearLED();
  switch(Band_slot) {
    case 1: gpio_put(FT8,true); break;
    case 2: gpio_put(FT4,true); break;
    case 3: gpio_put(JS8,true); break;
    case 4: gpio_put(WSPR,true); break;
  }
  
  bool UP_State = testButton(UP);
  bool DOWN_State = testButton(DOWN);

  if ((UP_State == false) && (DOWN_State == true)) {
    sleep_ms(100);

    UP_State = testButton(UP);
    if ((UP_State == false) && (DOWN_State == true)) {
      Band_slot = Band_slot - 1;

      if (Band_slot < 1) {
        Band_slot = 4;
      }
      cdc_printf("<UP> Band_slot=%d\n", Band_slot);

    }
  }

  if ((UP_State == true) && (DOWN_State == false)) {
    sleep_ms(100);

    DOWN_State = testButton(DOWN);
    if ((UP_State == true) && (DOWN_State == false)) {
      Band_slot = Band_slot + 1;

      if (Band_slot > 4) {
        Band_slot = 1;
      }
      cdc_printf("<DOWN> Band_slot=%d\n", Band_slot);

    }
  }


  bool TX_State = testButton(TXSW);
  if (TX_State == false) {
    sleep_ms(100);

    TX_State = testButton(TXSW);
    if (TX_State == false) {
      gpio_put(TX,false);
      goto Band_exit;

    }
  }

  goto Band_cont;

Band_exit:

  Band_assign();
  cdc_printf("completed set Band_slot=%d\n", Band_slot);

}

/*----------------------------------------------------------------------------*/
/* Check buttons                                                              */                                         
/*----------------------------------------------------------------------------*/
void checkButtons() {
  /*------------------------------------------------
     Explore and handle interactions with the user
     thru the UP/DOWN or TX buttons
  */
  bool UP_State = testButton(UP);
  bool DOWN_State = testButton(DOWN);

  /*----
     UP(Pressed) && DOWN(Pressed) && !Transmitting
     Start band selection mode
  */

  if ((UP_State == false) && (DOWN_State == false) && (bTX==false)) {
    sleep_ms(100);
    UP_State = testButton(UP);
    DOWN_State = testButton(DOWN);
    if ((UP_State == false) && (DOWN_State == false) && (bTX == false)) {
       Band_Select();
    }
  }

  /*----
     UP(Pressed) && DOWN(!Pressed) and !Transmitting
     Increase mode in direct sequence
  */

  if ((UP_State == false) && (DOWN_State == true) && (bTX == false)) {
    sleep_ms(100);
    UP_State = testButton(UP);
    if ((UP_State == false) && (DOWN_State == true) && (bTX == false)) {
      mode = mode - 1;
      if (mode < 1) {
        mode = 4;
      }
      Mode_assign();
    }
  }

  /*----
     UP(!Pressed) && DOWN(Pressed) && !Transmitting
     Change mode in the opposite sequence

  */
  DOWN_State = testButton(DOWN);
  if ((UP_State == true) && (DOWN_State == false) && (bTX == false)) {
    sleep_ms(50);

    DOWN_State = testButton(DOWN);
    if ((UP_State == true) && (DOWN_State == false) && (bTX == false)) {
      mode = mode + 1;

      if (mode > 4) {
        mode = 1;
      }
      Mode_assign();
    }
  }

  /*----
     If the TX button is pressed then activate the transmitter until the button is released
  */
  bool TXSW_State = testButton(TXSW);

  if ((bTX == false) && (bTX == false)) {
    sleep_ms(50);

    TXSW_State = testButton(TXSW);
    if ((TXSW_State == false) && (bTX == false)) {
      Mode_assign();
      ManualTX();
    }
  }
}

//*==============================================================================================*
//*                                    Transceiver management                                    *
//*==============================================================================================*

/*----------------------------------------------------------------------------*/
/* Control transmitter TX/RX status, TX LED and RX enable signals             */
/*----------------------------------------------------------------------------*/
void setTX(bool state) {

    frqFT8=GEN_FRQ_HZ;
    PioDCOStart(&DCO);
    PioDCOSetFreq(&DCO, frqFT8, 0UL);

    if (state) {
        prevfrq=0;
        gpio_put(RXSW, 0); //Set TX mode
        gpio_put(TX, 1);
        bTX = state;
        cdc_printf("Transmitter ON / Receiver OFF\n");
    } else {

        cycle = 0;
        sampling = 0;
        mono_preprev = 0;
        mono_prev = 0; 
        gpio_put(RXSW, 1); //Set RX mode
        gpio_put(TX, 0);
        bTX = state;
        cdc_printf("Transmitter OFF / Receiver ON\n");
    }
}   

//*==============================================================================================*
//*                                            MAIN BODY                                         *
//*==============================================================================================*
int main(void)
{

  //*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
  //*                          S E T U P                                      *
  //*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
  
  //*--- Hardware initialization

  stdio_init_all();

  #ifndef PICO_DEFAULT_LED_PIN
  #define PICO_DEFAULT_LED_PIN 25
  #endif

  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
  gpio_put(PICO_DEFAULT_LED_PIN, 1);

  //*---  Initial Processor board initialization & setup 
  
  initBoard();

  //*--- USB sub-system initialization

  tud_init(BOARD_TUD_RHPORT);

  
  //*--- ADC hardware setup
  
  gpio_init(RXA);                        //GPIO26 used as analog input
  gpio_set_dir(RXA, GPIO_IN);            //ADC input pin

  //*--- ADC sub-system configuration and initialization
  adc_init();
  adc_select_input(0);                        // ADC input pin A0
  adc_run(true);                              // start ADC free running
  adc_set_clkdiv(249.0);                      // 192kHz sampling  (48000 / (249.0 +1) = 192)
  adc_fifo_setup(true,false,0,false,false);   // fifo

  //*--- Initialize DCO on core 1 ***/
  cdc_printf("Core 1 started. DCO worker initializing...\n");

  //*--- Initialize ADX control board */
  ADXsetup();
  cdc_printf("Initializing ADX control board...\n");


  //*--- Sync time and define mode 
  Band_assign();

  adc_fifo_drain ();
  adc_offset = adc();
  push_last_time = to_ms_since_boot(get_absolute_time());

  //*---  USB Audio initialization (initialization of monodata[])

  for (int i = 0; i < (CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ / 4); i++) {
    monodata[i] = 0;
  }

  //*--- read the DC offset value (ADC input) to calibrate the mid level ----- 
  
  sleep_ms(100);
  adc_fifo_drain ();
  adc_offset = adc();

  //*--- Initialize timers
  Tx_last_mod_time=to_ms_since_boot(get_absolute_time());

//*--- Set bTX=false (RX mode)
  setTX(false);

//*-------------------------------------------------------
  cdc_printf("launching DDS Generator...\n");

//*-------------------------------------------------------
    
  cdc_printf("launching DCO worker on core 1...\n");
  multicore_launch_core1(core1_entry);
   
  while (true)
  {
    //*--- Call periodically the TinyUSB pre-emptive queue manager to service tasks

    tud_task(); // TinyUSB device task

    //*--- Invoke CAT processor (not implemented yet)
    cat();
    
    //*--- Check for changes in the band or mode      
    checkButtons();

    //*--- Manage the transmission and reception cycle

    if (bTX) {
       transmitting();
    } else {
       receiving();
    }
 
  }
}
/*============================================================================*/
/*                              CORE 1 PROCESSOR                              */
/* This is the code of dedicated core.                                        */
/* We deal with extremely precise real-time task.                             */
/*============================================================================*/
void core1_entry()
{
    _printf("Core 1: DCO worker started.\n");

    /* Run DCO. */
    PioDCOStart(&DCO);

    /* Run the main DCO algorithm. It spins forever. */
    PioDCOWorker2(&DCO);
}

//*==============================================================================================*
//*  transmitting                                                                                *
//*  This procedure manages the transmission part of the transceiver, it senses the USB audio    *
//*  input and meassures the frequency of the signal, then apply that frequency to the RF DDS    *
//*  The housekeeping of the board signaling for transmission is also managed here               *
//*==============================================================================================*
void transmitting(){
  
  uint64_t audio_freq;

  if (audio_read_number > 0) {
    
    for (int i=0;i<audio_read_number;i++){
      
      int16_t mono = monodata[i];
      
      if ((mono_prev < 0) && (mono >= 0)) {

        int16_t difference = mono - mono_prev;
              
        float delta = (float)mono_prev / (float)difference;
        float period = ((float)1.0 + delta_prev) + (float)sampling - delta;
        audio_freq = (uint64_t)(AUDIOSAMPLING/(double)period); // in Hz    
        
        if ((audio_freq > 200) && (audio_freq < 3000)){
          cycle_frequency[cycle]=(uint32_t)audio_freq;
          cycle++;
        }
       
        delta_prev = delta;
        sampling = 0;
        mono_preprev = mono_prev;
        mono_prev = mono;     

      } else {
        
        sampling++;
        mono_preprev = mono_prev;
        mono_prev = mono;
      }
    }
    
    //*--- Here the frequency is averaged every 10 mSecs and processed accordingly

    if ((cycle > 0) && ((to_ms_since_boot(get_absolute_time()) - Tx_last_mod_time) > 10)){      //inhibit the frequency change faster than 20mS
       audio_freq = 0;
       for (int i = 0;i < cycle;i++){
          audio_freq += cycle_frequency[i];
       }
       audio_freq = audio_freq / (uint64_t)cycle;
       cdc_printf("Freq(%" PRIu64 ") Hz\n ",audio_freq);
       transmit(audio_freq);         //* Manipulate the frequency     
       cycle = 0;
       Tx_last_mod_time = to_ms_since_boot(get_absolute_time()); ;
    }
    //*fix* not_TX_first = 1;
    Tx_last_time = to_ms_since_boot(get_absolute_time());      //*--- Senses EoT and switch to RX

  } else {
      
    if ((to_ms_since_boot(get_absolute_time()) - Tx_last_time) > 100) {     // If USBaudio data is not received for more than 50 ms during transmission, the system moves to receiving.
       setTX(false);
       //*fix* Tx_Start = 0;   
       return;
    }
  }

  audio_read_number = USB_Audio_read(monodata);
}

/*----------------------------------------------------------------------------*/
/* Manages changes in frequency                                               */                                         
/*----------------------------------------------------------------------------*/
void transmit(uint64_t freq){                                //freq in Hz

  uint32_t fx=(uint32_t)freq;
  if (bTX) {
     if (fx < FSKMIN || fx > FSKMAX) {
        cdc_printf("Tone(%lu) Hz OOR\n",fx);
        return;
     }

     if (fx > prevfrq) {
        if (fx - prevfrq > FSK_ERROR) {
           cdc_printf("Tone(%lu) Hz above, not taken\n",fx);
           return;
          }
     } else {
        if (prevfrq - fx > FSK_ERROR) {
           cdc_printf("Tone(%lu) Hz below, not taken\n",fx);
           return;
      }
    }

     uint32_t f = frqFT8 + fx;
     PioDCOStart(&DCO);
     PioDCOSetFreq(&DCO, f, 0UL);
     cdc_printf("FSK(%lu) Hz f[%lu Hz]\n",fx, f); 
     prevfrq=fx;
  }

}

//*==============================================================================================*
//*  receiving                                                                                   *
//*  This procedure manages the receiver part of the transceiver, it direct the audio from the   *
//*  board receiver to the ADC and periodically send samples of the digital audio to the PC S    *
//*==============================================================================================*
void receiving() {

  audio_read_number = USB_Audio_read(monodata); // read in the USB Audio buffer to check the transmitting
  if (audio_read_number != 0) 
  {
    setTX(true);
    //*fix* Tx_Start = 1;
    //*fix* not_TX_first = 0;
    return;
  }
  
  int16_t rx_adc = (int16_t)(adc() - adc_offset); //read ADC data (8kHz sampling)

  // write the same 6 stereo data to PC for 48kHz sampling (up-sampling: 8kHz x 6 = 48 kHz)
  for (int i=0;i<6;i++){
    audio_data_write(rx_adc, rx_adc);
  }
}

/*----------------------------------------------------------------------------*/
/* Transfer received audio tones thru the USB audio interface                 */                                         
/*----------------------------------------------------------------------------*/
void receive(){

  //*fix* Tx_Status=0;
  setTX(false);

  //*--- initialization of monodata[]
  for (int i = 0; i < (CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ / 4); i++) {
    monodata[i] = 0;
  } 
  
  //*--- initialization of ADC and the data write counter
  pcCounter=0;
  adc_fifo_drain ();                     //initialization of adc fifo
  adc_run(true);                         //start ADC free running
}
//*==============================================================================================*
//*                              USB Queue Management                                            *
//*==============================================================================================*
void audio_data_write(int16_t left, int16_t right) {
  if (pcCounter >= (48)) {                           //48: audio data number in 1ms
    USB_Audio_write(adc_data, pcCounter);
    pcCounter = 0;  
  }
  adc_data[pcCounter] = (int16_t)((left + right) / 2);
  pcCounter++;
}

//*--- Write to Serial USB port (CDC)
void cdc_write(char *buf, uint16_t length)
{
  tud_cdc_write(buf, length);
  tud_cdc_write_flush();
}

//*--- Read from Serial USB port (CDC) --for future development of a CAT sub-system--
uint32_t cdc_read(void)
{
  if ( tud_cdc_available() )
  {
    // read data
    uint32_t count = tud_cdc_read(cdc_read_buf, sizeof(cdc_read_buf));
    (void) count;
      return count;
  }
  else{
    return 0;
  }
}


//*==============================================================================================*
//*                              ADC Management                                                  *
//*==============================================================================================*
int32_t adc() {
  int32_t adc = 0;
  for (int i=0;i<24;i++){             // 192kHz/24 = 8kHz
    adc += adc_fifo_get_blocking();   // read from ADC fifo
  }  
  return adc;
}

//*==============================================================================================*
//*                              CAT Management (Not Implemented)                                *
//* remote contol (simulatingKenwood TS-2000)
//* original: "ft8qrp_cat11.ico" from https://www.elektronik-labor.de/HF/FT8QRP.html
//* with some modifications mainly to be adapted to C language
//*==============================================================================================*
void cat(void) 
{  

//*##################################################################################################
#ifdef REFACTOR

  char receivedPart1[40];
  char receivedPart2[40];    
  char command[3];
  char command2[3];  
  char parameter[38];
  char parameter2[38]; 
  char sent[42];
  char sent2[42];
  
  uint8_t received_length = (uint8_t)cdc_read(); 
  if (received_length == 0) return;
  for (int i = 0;i<received_length;i++){                   //to Upper case
    if('a' <= cdc_read_buf[i] && cdc_read_buf[i] <= 'z'){
        cdc_read_buf[i] = cdc_read_buf[i] - ('a' - 'A');
    } 
    if (cdc_read_buf[i] == '\n') {
      received_length--;              //replace(from "\n" to "")
    }
  }

  char data[64];
  int bufferIndex = 0;
  uint16_t part1_length = 0;
  uint16_t part2_length = 0; 
  
  for (int i = 0; i < received_length; ++i)
  {
    if (cdc_read_buf[i] != ';')
    {
      data[i] = cdc_read_buf[i];
    }
    else
    {
      if (bufferIndex == 0)
      { 
        for(int ii = 0; ii < i; ++ii) {
          receivedPart1[ii] = data[ii];
        }
        part1_length = (uint16_t)i;
        bufferIndex++;
      }
      else
      {  
        for(int ii = part1_length + 1; ii < i; ++ii) {
          receivedPart2[ii-part1_length-1] = data[ii];
        }
        part2_length = (uint16_t)(i - 1)- part1_length; 
        bufferIndex++;
      }
    }
  }

  strncpy(command, receivedPart1, 2); 
  command[2] = '\0'; 
  if (part1_length > 2){
    strncpy(parameter, receivedPart1+2, part1_length - 2); 
    parameter[part1_length - 2] = '\0';
  }
  if (bufferIndex == 2){

//*- fix    
    //strncpy(command2, receivedPart2, 2);
    snprintf(command2, sizeof command2, "%.2s", receivedPart2);
//*- end of fix 

    command[2] = '\0'; 
    strncpy(parameter2, receivedPart2+2, part2_length -2);
    parameter2[part2_length - 2] = '\0';
  }

  if (strcmp(command,"FA")==0){          
    if (sizeof(part1_length) <= 2)
    {          
      long int freqset = strtol(parameter, NULL, 10);
      if (freqset >= 1000000 && freqset <= 54000000){
        RF_freq = (uint64_t)freqset;
        si5351_set_freq((RF_freq-(uint64_t)BFO_freq), SI5351_PLL_FIXED, SI5351_CLK1);
        si5351_set_freq((RF_freq), SI5351_PLL_FIXED, SI5351_CLK0);
        //NEOPIXEL LED change
        put_pixel(urgb_u32(pixel_color[7].data[0], pixel_color[7].data[1], pixel_color[7].data[2]));
        adc_fifo_drain ();
        adc_offset = adc();
      }
    }
    strcpy(sent, "FA"); // Return 11 digit frequency in Hz.
    snprintf(parameter, 12, "%011d", (int)RF_freq);
    strcat(sent,parameter); 
    strcat(sent, ";");
  }
  else if (strcmp(command,"FB")==0) {                   
    strcpy(sent, "FB"); // Return 11 digit frequency in Hz.
    snprintf(parameter, 12, "%011d", (int)RF_freq);
    strcat(sent,parameter); 
    strcat(sent, ";");
  }
  else if (strcmp(command,"IF")==0) {          
    strcpy(sent, "IF"); // Return 11 digit frequency in Hz.  
    snprintf(parameter, 12, "%011d", (int)RF_freq);
    strcat(sent, parameter);
    strcat(sent, "0001+0000000000"); 
    snprintf(parameter, 2, "%d", Tx_Status);
    strcat(sent, parameter);  
    strcat(sent, "20000000;");          //USB   
  }
  else if (strcmp(command,"MD")==0) {
    strcpy(sent, "MD2");                //USB
  }
  else  if (strcmp(command,"ID")==0)  {  
    strcpy(sent,"ID019;");
  }
  else  if (strcmp(command,"PS")==0)  {  
    strcpy(sent, "PS1;");
  }
  else  if (strcmp(command,"AI")==0)  {  
    strcpy(sent, "AI0;");
  }
  else  if (strcmp(command,"RX")==0)  {  
    strcpy(sent, "RX0;");
  }
  else  if (strcmp(command,"TX")==0)  {  
    strcpy(sent, "TX0;");
  }
  else  if (strcmp(command,"AG")==0)  {  
    strcpy(sent, "AG0000;");
  }
  else  if (strcmp(command,"XT")==0) {  
    strcpy(sent, "XT0;");
  }
  else  if (strcmp(command,"RT")==0)  {  
    strcpy(sent, "RT0;");
  }
  else  if (strcmp(command,"RC")==0)  {  
    strcpy(sent, ";");
  }
  else  if (strcmp(command,"RS")==0)  {  
    strcpy(sent, "RS0;");
  }
  else  if (strcmp(command,"VX")==0)  {  
    strcpy(sent, "VX0;"); 
  }
  else  {
    strcpy(sent, "?;"); 
  }
//----------------------------------------------------
  if (strcmp(command2,"ID")==0)   {  
    strcpy(sent2, "ID019;");
  }
  else  {
    strcpy(sent2, "?;"); 
  }               
  
  if (bufferIndex == 2)  {
    cdc_write(sent2, (uint16_t)strlen(sent2));
  }        
  else  {
    cdc_write(sent, (uint16_t)strlen(sent));
  }  
#endif //REFACTOR
//*##################################################################################################

}

