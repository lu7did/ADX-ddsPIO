// usb_msc_ramdisk.c  (versión depurada: RAM cache + commit a flash solo al EJECT)

#include "tusb.h"
#include <string.h>
#include <stdbool.h>

#include "hardware/flash.h"
#include "hardware/sync.h"

#include "fat12_64k.h"   // unsigned char fat12_64k_img[]; unsigned int fat12_64k_img_len;

#define FLASH_SECTOR_SZ       4096u

#define MSC_BLOCK_SIZE        512u
#define MSC_BLOCK_COUNT       128u
#define MSC_DISK_BYTES        (MSC_BLOCK_SIZE * MSC_BLOCK_COUNT)   // 65536

// MSC en los últimos 64KB de flash, alineado a 4KB
#define MSC_FLASH_OFFSET_RAW  (PICO_FLASH_SIZE_BYTES - MSC_DISK_BYTES)
#define MSC_FLASH_OFFSET      (MSC_FLASH_OFFSET_RAW & ~(FLASH_SECTOR_SZ - 1u))

// --- EEPROM existente (solo para check opcional; no se usa para ubicar MSC)
#define EEPROM_FLASH_OFFSET   (1024u * 1024u)
#define EEPROM_FLASH_BYTES    (4096u)

_Static_assert(MSC_DISK_BYTES == 65536u, "MSC disk must be 64KB");
_Static_assert(sizeof(fat12_64k_img) == MSC_DISK_BYTES, "fat12_64k_img size must match MSC disk size");

// Check opcional de solapamiento (habilitar si querés seguridad extra)
#if 0
#if (MSC_FLASH_OFFSET < (EEPROM_FLASH_OFFSET + EEPROM_FLASH_BYTES)) && ((MSC_FLASH_OFFSET + MSC_DISK_BYTES) > EEPROM_FLASH_OFFSET)
#error "MSC flash region overlaps EEPROM emulation region"
#endif
#endif

// ---- Prototipos internos
static void mark_dirty_range(uint32_t addr, uint32_t len);
static void msc_init_once(void);
static void __not_in_flash_func(msc_commit_dirty_to_flash)(void);

// ---- RAM backing + dirty mask (64KB / 4KB = 16 sectores => 16 bits)
static uint8_t  msc_disk[MSC_DISK_BYTES];
static uint16_t dirty_mask = 0;

// ---- Helpers
static inline const uint8_t* msc_flash_ptr(void)
{
  // XIP_BASE viene del SDK (no redefinir)
  return (const uint8_t*)(XIP_BASE + MSC_FLASH_OFFSET);
}

static inline uint8_t sector_index_from_addr(uint32_t addr)
{
  return (uint8_t)(addr / FLASH_SECTOR_SZ); // 0..15
}

static bool flash_has_valid_fat12(void)
{
  const uint8_t* p = msc_flash_ptr();
  // Firma del boot sector 0x55AA
  return (p[510] == 0x55 && p[511] == 0xAA);
}

static void ram_init_from_default(void)
{
  // Carga FAT12 base (NO persiste hasta EJECT)
  memcpy(msc_disk, fat12_64k_img, fat12_64k_img_len);
  dirty_mask = 0;
}

static void ram_init_from_flash(void)
{
  memcpy(msc_disk, msc_flash_ptr(), MSC_DISK_BYTES);
  dirty_mask = 0;
}

static void msc_init_once(void)
{
  static bool inited = false;
  if (inited) return;

  if (flash_has_valid_fat12()) {
    ram_init_from_flash();
  } else {
    ram_init_from_default();
  }

  inited = true;
}

static void __not_in_flash_func(msc_commit_dirty_to_flash)(void)
{
  if (!dirty_mask) return;

  uint32_t ints = save_and_disable_interrupts();

  for (uint8_t s = 0; s < 16; s++) {
    if (dirty_mask & (uint16_t)(1u << s)) {
      uint32_t off = (uint32_t)s * FLASH_SECTOR_SZ;
      flash_range_erase(MSC_FLASH_OFFSET + off, FLASH_SECTOR_SZ);
      flash_range_program(MSC_FLASH_OFFSET + off, &msc_disk[off], FLASH_SECTOR_SZ);
    }
  }

  restore_interrupts(ints);
  dirty_mask = 0;
}

static void mark_dirty_range(uint32_t addr, uint32_t len)
{
  if (!len) return;

  uint32_t start = addr;
  uint32_t end   = addr + len - 1u;

  uint8_t s0 = sector_index_from_addr(start);
  uint8_t s1 = sector_index_from_addr(end);

  for (uint8_t s = s0; s <= s1; s++) {
    dirty_mask |= (uint16_t)(1u << s);
  }
}

// --------------------------------------------------------------------
// TinyUSB MSC callbacks (NO duplicados)
// --------------------------------------------------------------------

void tud_msc_inquiry_cb(uint8_t lun, uint8_t vendor_id[8], uint8_t product_id[16], uint8_t product_rev[4])
{
  (void) lun;

  const char vid[] = "LU7DZ   ";
  const char pid[] = "ADX-ddsPIO MSC   ";
  const char rev[] = "1.0 ";

  memcpy(vendor_id,  vid, 8);
  memcpy(product_id, pid, 16);
  memcpy(product_rev, rev, 4);
}

void tud_msc_capacity_cb(uint8_t lun, uint32_t* block_count, uint16_t* block_size)
{
  (void) lun;
  *block_count = MSC_BLOCK_COUNT;
  *block_size  = MSC_BLOCK_SIZE;
}

bool tud_msc_is_writable_cb(uint8_t lun)
{
  (void) lun;
  return true;
}

bool tud_msc_test_unit_ready_cb(uint8_t lun)
{
  (void) lun;
  msc_init_once();
  return true;
}

int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset, void* buffer, uint32_t bufsize)
{
  (void) lun;
  msc_init_once();

  uint32_t addr = lba * MSC_BLOCK_SIZE + offset;
  if (addr + bufsize > MSC_DISK_BYTES) return -1;

  memcpy(buffer, &msc_disk[addr], bufsize);
  return (int32_t) bufsize;
}

int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset, uint8_t* buffer, uint32_t bufsize)
{
  (void) lun;
  msc_init_once();

  uint32_t addr = lba * MSC_BLOCK_SIZE + offset;
  if (addr + bufsize > MSC_DISK_BYTES) return -1;

  memcpy(&msc_disk[addr], buffer, bufsize);
  mark_dirty_range(addr, bufsize);
  return (int32_t) bufsize;
}

// SCSI callback requerido por tu TinyUSB (si no está, falla el link)
int32_t tud_msc_scsi_cb(uint8_t lun, uint8_t const scsi_cmd[16], void* buffer, uint16_t bufsize)
{
  (void) buffer;
  (void) bufsize;

  // Si no implementamos comandos SCSI específicos, informamos "illegal request"
  // para que el host no se quede colgado.
  uint8_t const cmd = scsi_cmd[0];
  (void) cmd;

  tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x20, 0x00);
  return -1;
}

// Commit SOLO al expulsar (load_eject==true). Cambios se leen en el próximo boot.
bool tud_msc_start_stop_cb(uint8_t lun, uint8_t power_condition, bool start, bool load_eject)
{
  (void) lun;
  (void) power_condition;
  (void) start;

  if (load_eject) {
    msc_init_once();
    msc_commit_dirty_to_flash();
  }
  return true;
}

bool tud_msc_prevent_allow_medium_removal_cb(uint8_t lun, uint8_t prohibit_removal, uint8_t control)
{
  (void) lun;
  (void) prohibit_removal;
  (void) control;
  return true;
}
