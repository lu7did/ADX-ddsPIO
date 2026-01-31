#ifndef _USB_DESCRIPTORS_H_
#define _USB_DESCRIPTORS_H_

//#include "tusb.h"

// Unit numbers are arbitrary selected
#define UAC2_ENTITY_CLOCK               0x04
// Speaker path
#define UAC2_ENTITY_SPK_INPUT_TERMINAL  0x01
#define UAC2_ENTITY_SPK_FEATURE_UNIT    0x02
#define UAC2_ENTITY_SPK_OUTPUT_TERMINAL 0x03
// Microphone path
#define UAC2_ENTITY_MIC_INPUT_TERMINAL  0x11
#define UAC2_ENTITY_MIC_OUTPUT_TERMINAL 0x13

enum
{
  ITF_NUM_CDC = 0,
  ITF_NUM_CDC_DATA,

  ITF_NUM_MSC,                 // <-- NUEVO

  ITF_NUM_AUDIO_CONTROL,
  ITF_NUM_AUDIO_STREAMING_SPK,
  ITF_NUM_AUDIO_STREAMING_MIC,

  ITF_NUM_TOTAL
};

// ------------------------------------------------------------
// Audio descriptor length (tu macro original)
// ------------------------------------------------------------

#include "adx_uac2_len.h"

#endif

