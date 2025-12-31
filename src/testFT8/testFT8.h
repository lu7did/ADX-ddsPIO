//*----------------------------------------------------------------------------
//* testFT8.c
//*----------------------------------------------------------------------------
//* Testbed for the ADX DDS PIO-based frequency synthesizer FT8 Transceiver
//* using Raspberry Pi Pico.
//* 
//* This is a working module of an integration effort to build an FT8 
//* transceiver using Raspberry Pi Pico as the main controller and signal
//* generator.
//* 
//* Copyright (c) 2025 by Pedro Colla (LU7DZ)
//*----------------------------------------------------------------------------
//* Largely based on the work of Roman Piksaykin (R2BDY):
//* Digital Controlled Oscillator for Raspberry Pi Pico
//* https://www.qrz.com/db/r2bdy        
//*----------------------------------------------------------------------------

#include "macros.h"
/*-------------------------------------------------
   IDENTIFICATION DIVISION.
   (just a programmer's joke)
---------------------------------------------------*/
#define PROGNAME "testFT8"
#define AUTHOR "Dr. Pedro E. Colla (LU7DZ)"
#define VERSION  "0.1"
#define BUILD     "00"

#define DEBUG    1
#define GEN_FRQ_HZ 14074000L               /*Generator Frequency (in Hz)*/

char hi[128];

void DDSGenerator(void);
void core1_entry(void);


/*---------------------------------------------------
 * Define band support
*/
#define NBANDS        7
#define NMODES        4
#define SLOT          4

//***********************************************************************************************
//* The following defines the ports used to connect the hard switches (UP/DOWN/TX) and the LED
//* to the rp2040 processor in replacement of the originally used for the ADX Arduino Nano or UNO
//* (see documentation for the hardware schematic and pinout
//***********************************************************************************************
/*----
   Output control lines
*/
#define RXSW            2  //RXSW Switch

/*---
   LED
*/
#define WSPR            8  //WSPR LED
#define JS8             7  //JS8 LED
#define FT4             5  //FT4 LED
#define FT8             4  //FT8 LED


#define TX              3  //TX LED
#define CAL             9  //Calibration   
/*---
   Switches
*/
#define UP             11  //UP Switch
#define DOWN           12  //DOWN Switch
#define TXSW           10  //RX-TX Switch
#define SYNC           13  //Time SYNC Switch
#define BEACON         14  //BEACON Jumper

/*---
   Signal input/output pin
*/

#define RFOUT           6  //DDS Signal output PIN
#define FSKpin         27  //Frequency counter algorithm, signal input PIN (warning changes must also be done in freqPIO)




