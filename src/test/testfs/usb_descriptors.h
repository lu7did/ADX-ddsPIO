#pragma once
#include <stdint.h>

// Interface numbers
enum {
  ITF_NUM_CDC = 0,
  ITF_NUM_CDC_DATA,
  ITF_NUM_MSC,

  ITF_NUM_AUDIO_CONTROL,
  // Para MIC_ONE_CH, tu macro puede necesitar o no streaming itf adicional;
  // si tu TinyUSB lo requiere, agregamos ITF_NUM_AUDIO_STREAMING aquí.

  ITF_NUM_TOTAL
};

// Endpoint numbers (ejemplo)
#define EPNUM_CDC_NOTIF   0x81
#define EPNUM_CDC_OUT     0x02
#define EPNUM_CDC_IN      0x82

#define EPNUM_MSC_OUT     0x03
#define EPNUM_MSC_IN      0x83

#define EPNUM_AUDIO_IN    0x84  // mic device->host
