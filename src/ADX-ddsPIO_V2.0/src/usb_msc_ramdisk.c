#include "tusb.h"
#include <string.h>
#include <stdbool.h>
#include "fat12_64k.h"


#ifndef MSC_BLOCK_SIZE
#define MSC_BLOCK_SIZE   512u
#endif

#ifndef MSC_BLOCK_COUNT
#define MSC_BLOCK_COUNT  128u   // 64 KiB

#endif

static uint8_t msc_disk[MSC_BLOCK_SIZE * MSC_BLOCK_COUNT];

static void msc_disk_init(void)
{
  // El header generado por xxd suele llamarlo: unsigned char fat12_64k_img[];
  // y: unsigned int fat12_64k_img_len;
  memcpy(msc_disk, fat12_64k_img, fat12_64k_img_len);
}

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

bool tud_msc_test_unit_ready_cb(uint8_t lun)
{
  static bool inited = false;
  (void) lun;

  if (!inited) {
    msc_disk_init();
    inited = true;
  }

  return true;
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

int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset, void* buffer, uint32_t bufsize)
{
  (void) lun;

  const uint32_t addr = lba * MSC_BLOCK_SIZE + offset;
  const uint32_t max  = MSC_BLOCK_SIZE * MSC_BLOCK_COUNT;

  if (addr + bufsize > max) return -1;

  memcpy(buffer, &msc_disk[addr], bufsize);
  return (int32_t) bufsize;
}

// En tu versión: buffer NO es const
int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset, uint8_t* buffer, uint32_t bufsize)
{
  (void) lun;

  const uint32_t addr = lba * MSC_BLOCK_SIZE + offset;
  const uint32_t max  = MSC_BLOCK_SIZE * MSC_BLOCK_COUNT;

  if (addr + bufsize > max) return -1;

  memcpy(&msc_disk[addr], buffer, bufsize);
  return (int32_t) bufsize;
}

int32_t tud_msc_scsi_cb(uint8_t lun, uint8_t const scsi_cmd[16], void* buffer, uint16_t bufsize)
{
  (void) scsi_cmd;
  (void) buffer;
  (void) bufsize;

  tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x20, 0x00);
  return -1;
}

// CLAVE: en tu SDK devuelve bool
bool tud_msc_start_stop_cb(uint8_t lun, uint8_t power_condition, bool start, bool load_eject)
{
  (void) lun;
  (void) power_condition;
  (void) start;
  (void) load_eject;
  return true;
}

bool tud_msc_prevent_allow_medium_removal_cb(uint8_t lun, uint8_t prohibit_removal, uint8_t control)
{
  (void) lun;
  (void) prohibit_removal;
  (void) control;
  return true;
}

