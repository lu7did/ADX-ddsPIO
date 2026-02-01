/*
 * =======================================================================================
 * EEPROM
 * (c) Dr. Pedro E. Colla (LU7DZ) <pedro.colla@gmail.com>
 * 
 * emulate EEPROM in flash memory 
 * =======================================================================================*/

 //*---------------------------------------------------------------------------------------*
 //*                  Headers, libraries and re-entrancy management                        *
 //*---------------------------------------------------------------------------------------*
 #ifndef EEPROM_H
#define EEPROM_H

#include <stdint.h>
#include <stddef.h>

//*---------------------------------------------------------------------------------------*
//* Define a 256 bytes storage area for the run time configuration 
//*---------------------------------------------------------------------------------------*
//#define EEPROM_SIZE         4096
//#define FLASH_TARGET_OFFSET (PICO_FLASH_SIZE_BYTES - EEPROM_SIZE)
#define FLASH_TARGET_OFFSET (1024 * 1024) 
#define FLASH_SECTOR_SIZE    4096 

//*---------------------------------------------------------------------------------------*
//* Define a 256 bytes storage area for the run time configuration 
//*---------------------------------------------------------------------------------------*
typedef struct {
    uint8_t  ID;                 //* EEPROM signature (0x00 initialized/!0x00 filled)
    uint8_t  mode;               //* Mode defined
    uint8_t  Band_slot;          //* Slot defining band as index into the Bands[] array
    uint32_t audiosampling;      //* USB Audio sampling frequency (fixed)
    uint32_t pll_sys_mhz;        //* RP2040 System Clock (MHz)
    uint32_t frqFT8;        //* RF (CLK0/CLK1) base frequency (in Hz)
    uint32_t frqbfo;           //* BFO (CLK2) Frequency
    char     si4732region[8];   //* SI4732 Region
    char     si4732mode[8];      //* Receiver mode (AM,SSB,FM)       
    char     si4732band[8];      //* Band
    uint8_t  si4732vol;          //* Volume level {0..63}
    uint8_t  si4732mute;         //* Mute status {0..1}
    uint8_t reserved[71];
} EEPROMdata_t;
//*---------------------------------------------------------------------------------------*
//*                              Prototypes 
//*---------------------------------------------------------------------------------------*
void EEPROM_read(EEPROMdata_t *dest);
void EEPROM_write(const EEPROMdata_t *src);
void EEPROM_reset();

#endif
