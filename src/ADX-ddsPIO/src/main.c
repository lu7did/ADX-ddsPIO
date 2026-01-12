/*
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
#include <hardware/watchdog.h>    //watchdog (not used yet)
#include "usb_audio.h"
#include <inttypes.h>
#include "hardware/vreg.h"
#include "pico/multicore.h"
#include "pico/stdio/driver.h"
#include "piodco.h"
#include "../build/dco2.pio.h"

//*==============================================================================================*
//*                             Macros and Structures                                            *
//*==============================================================================================*
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

//*==============================================================================================*
//*                             Constants and parameters                                         *
//*==============================================================================================*
#define AUDIOSAMPLING    48000            // USB Audio sampling frequency (fixed)
#define PLL_SYS_MHZ        270            // RP2040 System Clock (MHz)  
#define GEN_FRQ_HZ    14074000L           // Generator Frequency (in Hz)
#define FT8_BASE_HZ       1000L           // FT8 base frequency (in Hz) <Not used>


#ifdef REMOVE
#define N_FREQ 2 // number of using RF frequencies with push switch　(<= 7)
#define FREQ_0 7041000 // RF frequency in Hz
#define FREQ_1 7074000 // in Hz
uint64_t Freq_table[N_FREQ]={FREQ_0,FREQ_1}; // Freq_table[N_FREQ]={FREQ_0,FREQ_1, ...}
#endif //REMOVE


//*==============================================================================================*
//*                                  Hardware configuration                                      *
//*==============================================================================================*

#define PICO_DEFAULT_LED_PIN 25
#define pin_A0               26U          //pin for ADC (A0)
#define pin_SW                3U          //pin for freq change switch (D10,input)
#define RFOUT                18           //RF out pin
#define FSKpin               27           //Frequency counter algorithm, signal input PIN (not used yet)

/*----
   Output control lines
*/
#define RXSW                  2  //RXSW Switch (RX/TX control)

/*---
   LED
*/

#define TX                    3  //TX LED
#define FT8                   4  //FT8 LED
#define FT4                   5  //FT4 LED
#define JS8                   6  //JS8 LED
#define WSPR                  7  //WSPR LED

/*---
   Calibration signal
*/

#define CAL                   9  //Calibration   
/*---
   Switches
*/
#define TXSW                  8  //RX-TX Switch
#define UP                   10  //UP Switch
#define DOWN                 11  //DOWN Switch
#define BEACON               12  //BEACON Jumper
#define SYNC                 13  //Time SYNC Switch


//*==============================================================================================*
//*                                  Global Memory Areas                                         *
//*==============================================================================================*
uint32_t frqFT8  = GEN_FRQ_HZ;
char hi[80];

//*--- Control block of PIO running the DCO
PioDco DCO; /* External in order to access in both cores. */

//*--- for ADC offset at trecieving
int32_t adc_offset = 0;   

//*--- Transceiver
#ifdef REMOVE
uint64_t RF_freq;   // RF frequency (Hz)
#endif //REMOVE

uint64_t audio_freq_prev=0.0;

#ifdef REMOVE
int C_freq = 0;    // FREQ_x: Index of RF frequency. In this case, FREQ_0 is selected as the initial frequency.
#endif //REMOVE

int Tx_Status = 0; // 0=RX, 1=TX
int Tx_Start = 0;  // 0=RX, 1=TX
int not_TX_first = 0;
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

//*--- for USB Audio
int16_t monodata[CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ / 4];
int16_t adc_data[CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ / 2];
int16_t pcCounter;
int audio_read_number=0;

//*--- for CDC buffering
char cdc_read_buf[64];
char cdc_write_buf[64];

#ifdef REMOVE
bool blinkTx=false;
static absolute_time_t t1;
#endif //REMOVE


//*==============================================================================================*
//*                                  Prototypes                                                  *
//*==============================================================================================*
void core1_entry(void);
void cdc_write(char *, uint16_t);
uint32_t cdc_read(void);
int32_t adc(); 
void transmitting(void);
void receiving(void);
void audio_data_write(int16_t,int16_t);
void cat(void);

#ifdef REMOVE
void transmit(uint64_t);
void receive(void);
void freqChange(void);
int freqcheck(uint64_t);
#endif //REMOVE

//*===============================================================================================*/
//*                                                 CORE 1 PROCESSOR                              */
//* This is the code dedicated in CORE1 to work out the DCO, deal with a precise real-time task   */                                                           */
//*===============================================================================================*/
void core1_entry()
{
    cdc_printf("Core 1: DCO worker started.\n");

    //*--- Set the DCO initial (default) frequency

    uint32_t f = frqFT8 + 0U;
    PioDCOStart(&DCO);
    PioDCOSetFreq(&DCO, f, 0U);

    //*--- Run the main DCO algorithm. It spins forever. */

    PioDCOWorker2(&DCO);
}
//*==============================================================================================*
//*                                  Board management                                            *
//*==============================================================================================*
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
    cdc_printf("ADX I/O control board initialized\n");

}

//*===============================================================================================*/
//*                                                 CORE 0 PROCESSOR                              */
//* This is the code dedicated to manage USB, board LED, switches and the transceiver FSM         */                                                           */
//*===============================================================================================*/


//*----------------------------------------------------------------------------*/
//*                         This is the main                                   */
//* First all relevant hardware and control structures are initialized, then   */
//* an infinite loop is entered which actually manages the transceiver FSM     */
//*----------------------------------------------------------------------------*/
int main(void)
{

  stdio_init_all();

  //*--- Overclock the board a little

  const uint32_t clkhz = PLL_SYS_MHZ * 1000000L;
  set_sys_clock_khz(clkhz / 1000L, true);

  //*--- Initialize stdio and other hardware elements

  stdio_init_all();
  sleep_ms(500);
 
  //*--- define the DEFAULT (board) LED

  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
  gpio_put(PICO_DEFAULT_LED_PIN, 1);


  //*--- Start the DCO

  const uint32_t PIOclkhz = PLL_SYS_MHZ * 1000000L;
  cdc_printf("Core 1 started. DCO worker initializing...\n");
  PioDCOInit(&DCO, RFOUT, PIOclkhz);

  //*--- Start the USB service loop

  tud_init(BOARD_TUD_RHPORT);
  
  //*--- Initialize the ADX board

  cdc_printf("Core 1 started. DCO worker initializing...\n");
  ADXsetup();

  //*--- GPIO setting for the ADC control (receiver) 
  gpio_init(pin_A0);
  gpio_set_dir(pin_A0, GPIO_IN); //ADC input pin

  //*--- Turn off the DEFAULT pin and launch the Core1 process
  
  gpio_put(PICO_DEFAULT_LED_PIN, 0);
  cdc_printf("launching DCO worker on core 1...\n");
  multicore_launch_core1(core1_entry);
  sleep_ms(500);
  
  //*--- ADC (receiver) initialization
  adc_init();
  adc_select_input(0);                        // ADC input pin A0
  adc_run(true);                              // start ADC free running
  adc_set_clkdiv(249.0);                      // 192kHz sampling  (48000 / (249.0 +1) = 192)
  adc_fifo_setup(true,false,0,false,false);   // fifo
  cdc_printf("ADC receiver system initialized\n");

  #ifdef REMOVE
  RF_freq = Freq_table[C_freq];
  freqChange();
  #endif //REMOVE


//*--- USB Audio initialization (initialization of monodata[])
  for (int i = 0; i < (CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ / 4); i++) {
    monodata[i] = 0;
  }

//*--- Calibrate the ADC offset, read the DC offset value (ADC input)
  
  sleep_ms(100);
  adc_fifo_drain ();
  adc_offset = adc();

  #ifdef REMOVE
  //watchdog_enable(delay_ms,pause_on_debug);
  #endif //REMOVE


  //cdc_printf("about to enter infinite loop \n");
  //static absolute_time_t t0;
  //bool blink=false;
  //t0 = get_absolute_time();
  #ifdef REMOVE
  t1 = get_absolute_time();
  #endif //REMOVE

  //*--- Get time for future synchronization

  Tx_last_mod_time=to_ms_since_boot(get_absolute_time());
  
  //*--- Enter the infinite loop

  //*----------------------------------------------------------------------------*/
  //*                    This is the main service loop                           */
  //*----------------------------------------------------------------------------*/
  while (true)
  {
  
    //*--- TUD (TinyUSB Dispatcher call)
    tud_task();        // TinyUSB device task

    //*--- CAT (using CDC, not implemented yet
    cat(); // remote control (simulating Kenwood TS-2000) 


    if (Tx_Start==0) {                     //Tx_Start=0 (RX) || Tx_Start=1 (TX)
        receiving();
    } else {
        transmitting();
    }
    
  }
}
/*===================================[ End of Main]=================================================*/
//*----------------------------------------------------------------------------*/
//*  This procedure controls the main transmission cycle of the transceiver    */
//*  The actual FSK data is created by an external program (i.e. WSJT-X) and   */
//*  sent to this firmware over USB audio
//*----------------------------------------------------------------------------*/
void transmitting(){
  
  uint64_t audio_freq;


  //*--- Check if there are digital audio samples to read, if so start the transmission cycle

  if (audio_read_number > 0) {

    //*--- Process the samples

    for (int i=0;i<audio_read_number;i++){
      
      int16_t mono = monodata[i];
 
      //*--- Detect an upward moving zero crossing

      if ((mono_prev < 0) && (mono >= 0)) {
         int16_t difference = mono - mono_prev;
         float delta = (float)mono_prev / (float)difference;
         float period = ((float)1.0 + delta_prev) + (float)sampling - delta;
         audio_freq = (uint64_t)(AUDIOSAMPLING/(double)period); // in Hz    

         //*--- Compute the period and FSK frequency, discard if above or below limits
         if ((audio_freq > 200) && (audio_freq < 3000)){
            cycle_frequency[cycle]=(uint32_t)audio_freq;
            cycle++;
         }
       
         delta_prev = delta;
         sampling = 0;
         mono_preprev = mono_prev;
         mono_prev = mono;     

      } else {

        //*--- This is not a zero crossing, ignore samples

        sampling++;
        mono_preprev = mono_prev;
        mono_prev = mono;
      }
    }

    //*--- When enough data has been collected (10 mSec) an average is computed to compensate for errorr

    if ((cycle > 0) && ((to_ms_since_boot(get_absolute_time()) - Tx_last_mod_time) > 10)){      //inhibit the frequency change faster than 20mS
       audio_freq = 0;
       for (int i = 0;i < cycle;i++){
          audio_freq += cycle_frequency[i];
       }
       audio_freq = audio_freq / (uint64_t)cycle;
       uint32_t f = frqFT8 + (uint32_t)audio_freq;

       //*--- as the FSK frequency has been detected change the DCO accordingly

       PioDCOSetFreq(&DCO, f, 0U);
       
       cdc_printf("FSK(%" PRIu64 ") Hz\n ",audio_freq);

       #ifdef REMOVE
       sprintf(hi,"FSK(%" PRIu64 ") Hz\n ",audio_freq);
       cdc_write(hi, (uint16_t)strlen(hi));
       #endif //REMOVE

       //*--- and initialize next averaging cycle

       cycle = 0;
       Tx_last_mod_time = to_ms_since_boot(get_absolute_time()); ;
    }

    //*--- More cycles needs to be averaged, continue

    not_TX_first = 1;
    Tx_last_time = to_ms_since_boot(get_absolute_time());
  
  } else { 

    //*--- No USB audio has been detected for a while, wait 100 mSecs and declare the frame to be terminated

    if ((to_ms_since_boot(get_absolute_time()) - Tx_last_time) >= 100 && Tx_Start==1)  {     // If USBaudio data is not received for more than 50 ms during transmission, the system moves to receiving. 
      cdc_printf("End of FT8 transmission\n");
      Tx_Start = 0;
      gpio_put(PICO_DEFAULT_LED_PIN, 0);

      //*--- Prepare for next cycle

      cycle = 0;
      sampling = 0;
      mono_preprev = 0;
      mono_prev = 0;     

      //*--- Return the DCO frequency to the base in order to operate as a receiver

      uint32_t fbfo = frqFT8 + 0U;
      PioDCOSetFreq(&DCO, fbfo, 0U);
      return;
    }
  } 
  audio_read_number = USB_Audio_read(monodata);
}
//*----------------------------------------------------------------------------*/
//*                    This is receiving functions                             */
//* While no data is being sent over USB Audio the RX signals are digitized and*/
//* sent over USB to the receiver program (external, likely WSJT-X)            */
//*                  THIS FUNCTION IS ONLY PARTIALLY IMPLEMENTED               */
//*----------------------------------------------------------------------------*/
void receiving() {

  audio_read_number = USB_Audio_read(monodata); // read in the USB Audio buffer to check the transmitting
  if (audio_read_number != 0) 
  {
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
    cdc_printf("Start of FT8 transmission\n");
    Tx_last_time=to_ms_since_boot(get_absolute_time());
    Tx_Start=1;
    return;
  }
  
  #ifdef PENDING
  int16_t rx_adc = (int16_t)(adc() - adc_offset); //read ADC data (8kHz sampling)
  // write the same 6 stereo data to PC for 48kHz sampling (up-sampling: 8kHz x 6 = 48 kHz)
  for (int i=0;i<6;i++){
    audio_data_write(rx_adc, rx_adc);
  }
  #endif //PENDING

  return;

}
/*----------------------------------------------------------------------------*/
//*  Audio data is sent over USB (Receiver)                                   */
/*----------------------------------------------------------------------------*/
void audio_data_write(int16_t left, int16_t right) {
  if (pcCounter >= (48)) {                           //48: audio data number in 1ms
    USB_Audio_write(adc_data, pcCounter);
    pcCounter = 0;  
  }
  adc_data[pcCounter] = (int16_t)((left + right) / 2);
  pcCounter++;
}



//*##################################################################################################
#ifdef REMOVE
void transmit(uint64_t freq){                                //freq in Hz

  uint64_t fx=freq;
  RF_freq = RF_freq + fx - fx;     //*Phony construct to avoid compilation errors
  //*##################################################################################################
#ifdef REFACTOR
    gpio_put(pin_RX, 0);   //RX off
    gpio_put(pin_TX, 1);   //TX on
    si5351_clock_enable(SI5351_CLK1, 0);   //RX osc. off
  #ifdef Superheterodyne
    si5351_clock_enable(SI5351_CLK2, 0);   //BFO osc. off
  #endif
    si5351_clock_enable(SI5351_CLK0, 1);   //TX osc. on

#endif //REFACTOR
//*##################################################################################################
    Tx_Status=1;
    //gpio_put(PICO_DEFAULT_LED_PIN, 1);

//*##################################################################################################
#ifdef REFACTOR
    gpio_put(pin_RED, ONBOARD_LED_ON);
    gpio_put(pin_GREEN, ONBOARD_LED_OFF);
    adc_run(false);                         //stop ADC free running
#endif //REFACTOR
//*##################################################################################################

//*##################################################################################################
#ifdef REFACTOR 
  si5351_set_freq((RF_freq + (freq)), SI5351_PLL_FIXED, SI5351_CLK0);
  //RF_freq = RF_freq + freq - freq;
#endif //REFACTOR
//*##################################################################################################
}
#endif //REMOVE
//*##################################################################################################


//*##################################################################################################
#ifdef REMOVE

//*-----------------------   Integrate AD/C functions before removal --------------------------------
void receive(){

//*##################################################################################################
#ifdef REFACTOR
  gpio_put(pin_TX,0);  //TX off
  gpio_put(pin_RX,1);  //RX on
  si5351_clock_enable(SI5351_CLK0, 0);   //TX osc. off
  si5351_clock_enable(SI5351_CLK1, 1);   //RX osc. on
#ifdef Superheterodyne
  si5351_clock_enable(SI5351_CLK2, 1);   //BFO osc. on
#endif
#endif //REFACTOR
//*##################################################################################################

#ifdef BUGHUNT
  frqFT8=GEN_FRQ_HZ;
  PioDCOStart(&DCO);
  PioDCOSetFreq(&DCO, frqFT8, 0UL);
  //gpio_put(PICO_DEFAULT_LED_PIN, 0);

#endif //BUGHUNT

  Tx_Status=0;


//*##################################################################################################
#ifdef REFACTOR
  gpio_put(pin_RED, ONBOARD_LED_OFF);
  gpio_put(pin_GREEN, ONBOARD_LED_ON);
#endif //REFACTOR
//*##################################################################################################
#ifdef CHASEBUG
  // initialization of monodata[]
  for (int i = 0; i < (CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ / 4); i++) {
    monodata[i] = 0;
  } 
  
  // initialization of ADC and the data write counter
  pcCounter=0;
  adc_fifo_drain ();                     //initialization of adc fifo
  adc_run(true);                         //start ADC free running
#endif //CHASEBUG

}
#endif //REMOVE



#ifdef REMOVE

//*-----------------------   Integrate AD/C functions before removal --------------------------------

void freqChange(){
//*##################################################################################################
#ifdef REFACTOR
  if (gpio_get(pin_SW)==0 && push_last_time==0){
    push_last_time = to_ms_since_boot(get_absolute_time());
  }
#endif //REFACTOR
//*##################################################################################################



  if (gpio_get(pin_SW)==0 && (to_ms_since_boot(get_absolute_time()) - push_last_time) > 700){     //wait for 700ms long push


    C_freq++;
    if  (C_freq >= N_FREQ){
      C_freq = 0;
    }
    RF_freq = Freq_table[C_freq];

//*##################################################################################################
#ifdef REFACTOR
    si5351_set_freq((RF_freq-(uint64_t)BFO_freq), SI5351_PLL_FIXED, SI5351_CLK1);
    put_pixel(urgb_u32(pixel_color[C_freq].data[0], pixel_color[C_freq].data[1], pixel_color[C_freq].data[2]));
#endif //REFACTOR
//*##################################################################################################



    adc_fifo_drain ();
    adc_offset = adc();
    push_last_time = 0;
  }

//*##################################################################################################
#ifdef REFACTOR
  if (gpio_get(pin_SW)!=0){
    push_last_time = 0;
  }
#endif //REFACTOR
//*##################################################################################################

}

int32_t adc() {
  int32_t adc = 0;
  for (int i=0;i<24;i++){             // 192kHz/24 = 8kHz
    adc += adc_fifo_get_blocking();   // read from ADC fifo
  }  
  return adc;
}
#endif //REMOVE
//*##################################################################################################


//*---------------------------------------------------------------------------------*/
//*                        ADC Sub-System                                           */
//* The ADC sub-system samples the mixer output of the receiver and sent it over USB*/
//*---------------------------------------------------------------------------------*/
int32_t adc() {
  int32_t adc = 0;
  for (int i=0;i<24;i++){             // 192kHz/24 = 8kHz
    adc += adc_fifo_get_blocking();   // read from ADC fifo
  }  
  return adc;
}



//*---------------------------------------------------------------------------------*/
//*                        CAT Sub-System                                           */
//* Receives CAT commands over USB Serial emulation (CDC) and changes the           */
//* operation of the transceiver accordingly                                        */
//* The CAT protocol being used is for a Kenwood TS-2000                            */
//* original: "ft8qrp_cat11.ico" from https://www.elektronik-labor.de/HF/FT8QRP.html*/
//* with some modifications mainly to be adapted to C language made by Hitoshi      */
//*---------------------------------------------------------------------------------*/
void cat(void) 
{  

#ifdef CAT

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
#endif //CAT
}

//*---------------------------------------------------------------------------------*/
//* Functions to manage the CDC serial emulation (for debug and CAT)                */
//*---------------------------------------------------------------------------------*/
void cdc_write(char *buf, uint16_t length)
{
  tud_cdc_write(buf, length);
  tud_cdc_write_flush();
}

#ifdef CAT
void cdc_write_int(int64_t integer) 
{
  char buf[64];
  int length = sprintf(buf, "%lld", integer);
  tud_cdc_write(buf, (uint32_t)length);
  tud_cdc_write_flush();
}
#endif //CAT


#ifdef CAT
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
#endif //CAT

#ifdef REMOVAL
int freqcheck(uint64_t frequency)  // retern 1=out-of-band, 0=in-band
{
  if (frequency < 135700) {
    return 1;
  }
  else if (frequency > 135800 && frequency < 472000) {
    return 1;
  }
  else if (frequency > 479000 && frequency < 1800000) {
    return 1;
  }
  else if (frequency > 1875000 && frequency < 1907500) {
    return 1;
  }
  else if (frequency > 1912500 && frequency < 3500000) {
    return 1;
  }
  else if (frequency > 3580000 && frequency < 3662000) {
    return 1;
  }
  else if (frequency > 3687000 && frequency < 3716000) {
    return 1;
  }
  else if (frequency > 3770000 && frequency < 3791000) {
    return 1;
  }
  else if (frequency > 3805000 && frequency < 7000000) {
    return 1;
  }
  else if (frequency > 7200000 && frequency < 10100000) {
    return 1;
  }
  else if (frequency > 10150000 && frequency < 14000000) {
    return 1;
  }
  else if (frequency > 14350000 && frequency < 18068000) {
    return 1;
  }
  else if (frequency > 18168000 && frequency < 21000000) {
    return 1;
  }
  else if (frequency > 21450000 && frequency < 24890000) {
    return 1;
  }
  else if (frequency > 24990000 && frequency < 28000000) {
    return 1;
  }
  else if (frequency > 29700000 && frequency < 50000000) {
    return 1;
  }
  else if (frequency > 54000000) {
    return 1;
  }
  else return 0;
}
#endif //REMOVAL