#include "tud_audio_headset_stereo_desc.h"
#include "usb_descriptors.h"
#include "tusb.h"

#define USB_VID   0xCafe
#define USB_PID   0x4011
#define USB_BCD   0x0100

// Endpoints
#define EPNUM_CDC_NOTIF   0x81
#define EPNUM_CDC_OUT     0x02
#define EPNUM_CDC_IN      0x82

#define EPNUM_MSC_OUT     0x03
#define EPNUM_MSC_IN      0x83

#define EPNUM_AUDIO_OUT   0x04
#define EPNUM_AUDIO_IN    0x84

// Device descriptor
const tusb_desc_device_t desc_device =
{
  .bLength            = sizeof(tusb_desc_device_t),
  .bDescriptorType    = TUSB_DESC_DEVICE,
  .bcdUSB             = 0x0200,

  .bDeviceClass       = TUSB_CLASS_MISC,
  .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
  .bDeviceProtocol    = MISC_PROTOCOL_IAD,

  .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

  .idVendor           = USB_VID,
  .idProduct          = USB_PID,
  .bcdDevice          = USB_BCD,

  .iManufacturer      = 0x01,
  .iProduct           = 0x02,
  .iSerialNumber      = 0x03,

  .bNumConfigurations = 0x01
};

uint8_t const * tud_descriptor_device_cb(void)
{
  return (uint8_t const*) &desc_device;
}

// Total length: config + CDC + MSC + AUDIO
//#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + TUD_MSC_DESC_LEN + CFG_TUD_AUDIO_FUNC_1_DESC_LEN)
//#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + TUD_MSC_DESC_LEN + TUD_AUDIO_HEADSET_STEREO_DESC_LEN)

#include "adx_uac2_len.h"

#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + TUD_MSC_DESC_LEN + ADX_UAC2_FUNC_DESC_LEN)


uint8_t const desc_configuration[] =
{
  // Config: 1, interface count, string index, total length, attributes, power mA/2
  TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),

  // CDC: itfnum, string index, notif ep, notif size, out ep, in ep, ep size
  TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 0x04,
                     EPNUM_CDC_NOTIF, 8,
                     EPNUM_CDC_OUT, EPNUM_CDC_IN, 64),

  // MSC: itfnum, string index, out ep, in ep, ep size
  TUD_MSC_DESCRIPTOR(ITF_NUM_MSC, 0x05, EPNUM_MSC_OUT, EPNUM_MSC_IN, 64),

  // AUDIO (tu macro UAC2)
  TUD_AUDIO_HEADSET_STEREO_DESCRIPTOR(0x06, EPNUM_AUDIO_OUT, EPNUM_AUDIO_IN)

};

uint8_t const * tud_descriptor_configuration_cb(uint8_t index)
{
  (void) index;
  return desc_configuration;
}

// Strings
static const char* string_desc_arr[] =
{
  (const char[]){ 0x09, 0x04 },  // 0: English (US)
  "LU7DZ",                        // 1
  "ADX-ddsPIO Composite",         // 2
  "0001",                         // 3
  "ADX CDC",                      // 4
  "ADX MSC",                      // 5
  "ADX Audio",                    // 6
};

static uint16_t _desc_str[32];

uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
  (void) langid;
  uint8_t chr_count;

  if (index == 0)
  {
    memcpy(&_desc_str[1], string_desc_arr[0], 2);
    chr_count = 1;
  }
  else
  {
    if (index >= sizeof(string_desc_arr)/sizeof(string_desc_arr[0])) return NULL;

    const char* str = string_desc_arr[index];
    chr_count = 0;

    while (str[chr_count] && chr_count < 31)
    {
      _desc_str[1+chr_count] = (uint16_t) str[chr_count];
      chr_count++;
    }
  }

  _desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2*chr_count + 2));
  return _desc_str;
}
