#ifndef PROTOS_H_
#define PROTOS_H_

#include "defines.h"

/* main.c */

void RAM (SpinnerMFSKTest)(void);
void RAM (SpinnerSweepTest)(void);
void RAM (SpinnerRTTYTest)(void);
void RAM (SpinnerMilliHertzTest)(void);
void RAM (SpinnerWide4FSKTest)(void);
void RAM (SpinnerGPSreferenceTest)(void);

/* conswrapper.c */

void ConsoleCommandsWrapper(char *cmd, int narg, char *params);
void PushErrorMessage(int id);
void PushStatusMessage(void);

//*==============================================================================================*
//*                                  Prototypes                                                  *
//*==============================================================================================*

void DDSGenerator(void);
void core1_entry(void);


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
void setTX(bool state);



#endif
