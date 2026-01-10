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

//*==============================================================================================*
//*                                  Macros and Structures                                       *
//*==============================================================================================*

//*----------------------------------------------------------------------------------------------*
//* Expands a decorated printf-like pseudo-instruction to add debug messages and control them
//*----------------------------------------------------------------------------------------------*
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
//*                                  Constants and Hardware defs                                 *
//*==============================================================================================*

#define AUDIOSAMPLING 48000          // USB Audio sampling frequency (fixed)
#define pin_A0 26U                   //pin for ADC (A0)
#define pin_SW 3U                    //pin for freq change switch (D10,input)

#define N_FREQ 2 // number of using RF frequencies with push switch　(<= 7)
#define FREQ_0 7041000 // RF frequency in Hz
#define FREQ_1 7074000 // in Hz


//*==============================================================================================*
//*                                  Global Memory Areas                                         *
//*==============================================================================================*

uint64_t Freq_table[N_FREQ]={FREQ_0,FREQ_1}; // Freq_table[N_FREQ]={FREQ_0,FREQ_1, ...}


//*--- Message control area

char hi[80];
uint16_t n=0;

//*--- for ADC offset at receiving
int32_t adc_offset = 0;   

//*--- for tranceiver
uint64_t RF_freq;   // RF frequency (Hz)
uint64_t audio_freq_prev=0.0;

//*--- Control the TX function and the frequency decoding from the incoming USB (digital) audio

int C_freq = 0;    // FREQ_x: Index of RF frequency. In this case, FREQ_0 is selected as the initial frequency.
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

//*--- definitions and areas for USB Audio (IN/OUT)
int16_t monodata[CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ / 4];
int16_t adc_data[CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ / 2];
int16_t pcCounter;
int audio_read_number=0;

//*--- definitions for USB serial (CDC) interface
char cdc_read_buf[64];
char cdc_write_buf[64];

//*==============================================================================================*
//*                                  Prototypes                                                  *
//*==============================================================================================*
void cdc_write(char *, uint16_t);
uint32_t cdc_read(void);
int32_t adc(); 
void freqChange(void);
void receive(void);
void transmit(uint64_t);
void transmitting(void);
void receiving(void);
void audio_data_write(int16_t,int16_t);
void cat(void);
int freqcheck(uint64_t);


//*==============================================================================================*
//*                                            MAIN BODY                                         *
//*==============================================================================================*
int main(void)
{
  //*fix* board_init();

  //*--- Hardware initialization

  stdio_init_all();

  #ifndef PICO_DEFAULT_LED_PIN
  #define PICO_DEFAULT_LED_PIN 25
  #endif

  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
  gpio_put(PICO_DEFAULT_LED_PIN, 1);

  //*--- USB sub-system initialization

  tud_init(BOARD_TUD_RHPORT);

  
  //*--- ADC hardware setup
  
  gpio_init(pin_A0);                        //GPIO26 used as analog input
  gpio_set_dir(pin_A0, GPIO_IN);            //ADC input pin

  //*--- ADC sub-system configuration and initialization
  adc_init();
  adc_select_input(0);                        // ADC input pin A0
  adc_run(true);                              // start ADC free running
  adc_set_clkdiv(249.0);                      // 192kHz sampling  (48000 / (249.0 +1) = 192)
  adc_fifo_setup(true,false,0,false,false);   // fifo

  //*--- Setup initial frequency of the transceiver

  RF_freq = Freq_table[C_freq];
  freqChange();


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

  while (true)
  {
    //*--- Call periodically the TinyUSB pre-emptive queue manager to service tasks

    tud_task(); // TinyUSB device task

    //*--- Manage the transmission and reception cycle

    transmitting();

  }
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

      sprintf(hi,"Freq(%" PRIu64 ") Hz\n ",audio_freq);
      cdc_write(hi, (uint16_t)strlen(hi));

      #ifdef REFACTOR
         transmit(audio_freq);         //* Manipulate the frequency
      #endif //REFACTOR
      
      cycle = 0;
      Tx_last_mod_time = to_ms_since_boot(get_absolute_time()); ;
    }
    not_TX_first = 1;
    

    #ifdef REFACTOR
    Tx_last_time = to_ms_since_boot(get_absolute_time());      //*--- Senses EoT and switch to RX
    #endif //REFACTOR

  }

  audio_read_number = USB_Audio_read(monodata);
}

//*--- Manages the actual change of frequency

void transmit(uint64_t freq){                                //freq in Hz
  if (Tx_Status == 0 && freqcheck(RF_freq+3000)==0)
  {
     uint64_t fx=freq;
     RF_freq = RF_freq + fx - fx;     //*Phony construct to avoid compilation errors
     Tx_Status=1;
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
    Tx_Start = 1;
    not_TX_first = 0;
    return;
  }

  freqChange();
  
  int16_t rx_adc = (int16_t)(adc() - adc_offset); //read ADC data (8kHz sampling)

  // write the same 6 stereo data to PC for 48kHz sampling (up-sampling: 8kHz x 6 = 48 kHz)
  for (int i=0;i<6;i++){
    audio_data_write(rx_adc, rx_adc);
  }
}

//*--- Handle the actual manipulation of received signals at the ADC port

void receive(){

  Tx_Status=0;

  //*--- initialization of monodata[]
  for (int i = 0; i < (CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ / 4); i++) {
    monodata[i] = 0;
  } 
  
  //*--- initialization of ADC and the data write counter
  pcCounter=0;
  adc_fifo_drain ();                     //initialization of adc fifo
  adc_run(true);                         //start ADC free running
}

//*--- Change frequency

void freqChange(){

  if (gpio_get(pin_SW)==0 && (to_ms_since_boot(get_absolute_time()) - push_last_time) > 700){     //wait for 700ms long push

    C_freq++;
    if  (C_freq >= N_FREQ){
      C_freq = 0;
    }
    RF_freq = Freq_table[C_freq];

    adc_fifo_drain ();
    adc_offset = adc();
    push_last_time = 0;
  }

}
//*--- Control required by the Japan's licensing conditions
//*--- It is controlled by other means and will be removed 

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

