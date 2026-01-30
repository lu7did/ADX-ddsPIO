#include "flash_bd.h"

#include <string.h>
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"

#ifndef XIP_BASE
#define XIP_BASE 0x10000000u
#endif

static inline uint32_t lba_to_flash_addr(uint32_t lba) {
  return ADX_FLASH_FS_OFFSET + lba * ADX_MSC_SECTOR_SIZE;
}

bool flash_bd_init(void) {
  // Si necesitás validaciones de rango, ponelas acá.
  // Por ejemplo: ADX_FLASH_FS_OFFSET alineado a 4096, tamaño no excede flash total, etc.
  return true;
}
bool flash_bd_read_blocks(uint32_t lba, void* dst, uint32_t count) {
  if (!dst || count == 0) return false;
  if ((uint64_t)lba + (uint64_t)count > (uint64_t)ADX_MSC_SECTOR_COUNT) return false;

  uint32_t flash_addr = lba_to_flash_addr(lba);
  const uint8_t* src = (const uint8_t*)(XIP_BASE + flash_addr);

  memcpy(dst, src, (size_t)count * ADX_MSC_SECTOR_SIZE);
  return true;
}

static bool program_rmw_4k(uint32_t flash_addr, const uint8_t* data512, uint32_t offset_in_4k) {
  // flash_addr: dirección exacta donde iría el bloque de 512 (dentro del FS)
  // offset_in_4k: offset dentro del sector 4KB (0..3584), múltiplo de 512
  // data512: buffer 512 a escribir

  // 1) alinear al sector 4KB
  uint32_t base4k = flash_addr & ~(FLASH_SECTOR_SIZE - 1u);

  // 2) leer el sector 4KB completo desde XIP
  uint8_t tmp[FLASH_SECTOR_SIZE];
  const uint8_t* cur = (const uint8_t*)(XIP_BASE + base4k);
  memcpy(tmp, cur, FLASH_SECTOR_SIZE);

  // 3) modificar solo el pedazo 512 dentro del tmp
  memcpy(tmp + offset_in_4k, data512, ADX_MSC_SECTOR_SIZE);

  // 4) erase + program del 4KB completo (atómico respecto a interrupciones)
  uint32_t ints = save_and_disable_interrupts();
  flash_range_erase(base4k, FLASH_SECTOR_SIZE);
  flash_range_program(base4k, tmp, FLASH_SECTOR_SIZE);
  restore_interrupts(ints);

  return true;
}
bool flash_bd_write_blocks(uint32_t lba, const void* src, uint32_t count) {
  if (!src || count == 0) return false;
  if ((uint64_t)lba + (uint64_t)count > (uint64_t)ADX_MSC_SECTOR_COUNT) return false;

  // Escribimos de a 512, pero cada write hace RMW del sector 4KB correspondiente.
  for (uint32_t i = 0; i < count; i++) {
    uint32_t cur_lba = lba + i;
    uint32_t flash_addr = lba_to_flash_addr(cur_lba);

    // offset dentro de 4KB según posición del LBA
    // (cada LBA = 512, en 4KB entran 8 LBAs)
    uint32_t offset_in_4k = (flash_addr & (FLASH_SECTOR_SIZE - 1u));

    // Asegurar que estamos alineados a 512, si no hay bug en tu geometría.
    if ((offset_in_4k % ADX_MSC_SECTOR_SIZE) != 0) return false;

    if (!program_rmw_4k(flash_addr, src + i * ADX_MSC_SECTOR_SIZE, offset_in_4k)) {
      return false;
    }
  }

  return true;
}

void flash_bd_erase_all(void) {
  // Borra TODA el área del FS (en múltiplos de 4KB)
  uint32_t bytes = ADX_MSC_SECTOR_COUNT * ADX_MSC_SECTOR_SIZE;
  uint32_t erase_len = (bytes + FLASH_SECTOR_SIZE - 1u) & ~(FLASH_SECTOR_SIZE - 1u);

  uint32_t ints = save_and_disable_interrupts();
  flash_range_erase(ADX_FLASH_FS_OFFSET, erase_len);
  restore_interrupts(ints);
}
