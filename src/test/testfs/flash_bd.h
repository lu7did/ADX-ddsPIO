// flash_bd.h
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifndef ADX_FLASH_EEPROM_OFFSET
  #define ADX_FLASH_EEPROM_OFFSET   (1024u * 1024u)   // tu EEPROM emulada empieza en 1MB
#endif

#ifndef ADX_FLASH_EEPROM_SIZE
  #define ADX_FLASH_EEPROM_SIZE     (4096u)           // 4KB
#endif

// Sector lógico del MSC/FATFS
#ifndef ADX_MSC_SECTOR_SIZE
  #define ADX_MSC_SECTOR_SIZE       512u
#endif

// Tamaño TOTAL del disco MSC (recomendación: >= 1MB)
// Si lo dejás muy chico, MacOS se pone más hincha.
#ifndef ADX_FLASH_FS_SIZE
  #define ADX_FLASH_FS_SIZE         (2u * 1024u * 1024u)  // 2MB como ejemplo estable
#endif

// Offset del FS: inmediatamente después de EEPROM (alineá a sector flash 4096)
#ifndef FLASH_SECTOR_SIZE
  #define FLASH_SECTOR_SIZE         4096u
#endif

#define ADX_ALIGN_UP(x, a)          (((x) + ((a)-1u)) & ~((a)-1u))

#define ADX_FLASH_FS_OFFSET         ADX_ALIGN_UP((ADX_FLASH_EEPROM_OFFSET + ADX_FLASH_EEPROM_SIZE), FLASH_SECTOR_SIZE)

#define ADX_MSC_SECTOR_COUNT        (ADX_FLASH_FS_SIZE / ADX_MSC_SECTOR_SIZE)

// API block device
bool     flash_bd_init(void);
uint32_t flash_bd_block_count(void);
uint32_t flash_bd_block_size(void);

bool flash_bd_read_blocks(uint32_t lba, void* buffer, uint32_t block_count);
bool flash_bd_write_blocks(uint32_t lba, const void* buffer, uint32_t block_count);

// opcional para formateo limpio
void flash_bd_erase_all(void);
