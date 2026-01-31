/*
 * The MIT License (MIT)
 * 
 * Copyright (c) 2023 Hitoshi Kawaji <je1rav@gmail.com>
 *    Modified slightly from "uac2_headset" to use as a UAC2-CDC composite device
 * Copyright (c) 2020 Ha Thach (tinyusb.org)
 * Copyright (c) 2020 Jerzy Kasenberg
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

// Nota: no es estrictamente necesario incluir usb_descriptors.h acá.
// Si en tu árbol usb_descriptors.h incluye tusb.h, puede generar includes cruzados.
// Pero si ya te funciona así, lo dejamos como estaba.
//#include "usb_descriptors.h"



#include "adx_uac2_len.h"

#ifndef CFG_TUD_AUDIO_FUNC_1_DESC_LEN
#define CFG_TUD_AUDIO_FUNC_1_DESC_LEN   ADX_UAC2_FUNC_DESC_LEN
#endif


//--------------------------------------------------------------------+
// Board Specific Configuration
//--------------------------------------------------------------------+

#ifndef BOARD_TUD_RHPORT
#define BOARD_TUD_RHPORT      0
#endif

#ifndef BOARD_TUD_MAX_SPEED
#define BOARD_TUD_MAX_SPEED   OPT_MODE_DEFAULT_SPEED
#endif

//--------------------------------------------------------------------
// Common Configuration
//--------------------------------------------------------------------

#ifndef CFG_TUSB_MCU
#define CFG_TUSB_MCU           OPT_MCU_RP2040
#endif

// IMPORTANTE en Pico SDK: usar OPT_OS_PICO (aunque no uses RTOS)
#ifndef CFG_TUSB_OS
#define CFG_TUSB_OS            OPT_OS_PICO
#endif

#ifndef CFG_TUSB_DEBUG
#define CFG_TUSB_DEBUG         0
#endif

// Enable Device stack
#define CFG_TUD_ENABLED        1

// Max speed (FS en RP2040, pero dejamos el define estándar)
#define CFG_TUD_MAX_SPEED      BOARD_TUD_MAX_SPEED

// RHPort mode: device + speed
#ifndef CFG_TUSB_RHPORT0_MODE
#define CFG_TUSB_RHPORT0_MODE  (OPT_MODE_DEVICE | BOARD_TUD_MAX_SPEED)
#endif

#ifndef CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_SECTION
#endif

#ifndef CFG_TUSB_MEM_ALIGN
#define CFG_TUSB_MEM_ALIGN     __attribute__ ((aligned(4)))
#endif

//--------------------------------------------------------------------
// DEVICE CONFIGURATION
//--------------------------------------------------------------------

#ifndef CFG_TUD_ENDPOINT0_SIZE
#define CFG_TUD_ENDPOINT0_SIZE  64
#endif

//------------- CLASS -------------//
// Debe coincidir con tu usb_descriptors.c (CDC + MSC + AUDIO)
#define CFG_TUD_CDC             1
#define CFG_TUD_MSC             1     // <--- ACTIVADO (si querés desactivar, poné 0)
#define CFG_TUD_HID             0
#define CFG_TUD_MIDI            0
#define CFG_TUD_AUDIO           1
#define CFG_TUD_VENDOR          0

#define CFG_TUD_MSC_EP_BUFSIZE  64   // o 512 si querés más


//--------------------------------------------------------------------
// CDC CLASS CONFIGURATION
//--------------------------------------------------------------------

// FIFO size of TX and RX
#define CFG_TUD_CDC_RX_BUFSIZE   (TUD_OPT_HIGH_SPEED ? 1024 : 512)
#define CFG_TUD_CDC_TX_BUFSIZE   (TUD_OPT_HIGH_SPEED ? 1024 : 512)

// Endpoint transfer buffer size
#define CFG_TUD_CDC_EP_BUFSIZE   (TUD_OPT_HIGH_SPEED ? 1024 : 512)

//--------------------------------------------------------------------
// MSC CLASS CONFIGURATION
//--------------------------------------------------------------------

// OBLIGATORIO si CFG_TUD_MSC = 1 (si no, TinyUSB tira #error)
#ifndef CFG_TUD_MSC_EP_BUFSIZE
#define CFG_TUD_MSC_EP_BUFSIZE   512   // 512 = tamaño de bloque típico

#endif

//--------------------------------------------------------------------
// AUDIO CLASS DRIVER CONFIGURATION
//--------------------------------------------------------------------

// How many formats are used, need to adjust USB descriptor if changed
#define CFG_TUD_AUDIO_FUNC_1_N_FORMATS                               2

// Audio format type I specifications
#if defined(__RX__)
#define CFG_TUD_AUDIO_FUNC_1_MAX_SAMPLE_RATE                         48000
#else
#define CFG_TUD_AUDIO_FUNC_1_MAX_SAMPLE_RATE                         96000
#endif

#define CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX                           1
#define CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX                           2

// 16bit in 16bit slots
#define CFG_TUD_AUDIO_FUNC_1_FORMAT_1_N_BYTES_PER_SAMPLE_TX          2
#define CFG_TUD_AUDIO_FUNC_1_FORMAT_1_RESOLUTION_TX                  16
#define CFG_TUD_AUDIO_FUNC_1_FORMAT_1_N_BYTES_PER_SAMPLE_RX          2
#define CFG_TUD_AUDIO_FUNC_1_FORMAT_1_RESOLUTION_RX                  16

#if defined(__RX__)
// 8bit in 8bit slots
#define CFG_TUD_AUDIO_FUNC_1_FORMAT_2_N_BYTES_PER_SAMPLE_TX          1
#define CFG_TUD_AUDIO_FUNC_1_FORMAT_2_RESOLUTION_TX                  8
#define CFG_TUD_AUDIO_FUNC_1_FORMAT_2_N_BYTES_PER_SAMPLE_RX          1
#define CFG_TUD_AUDIO_FUNC_1_FORMAT_2_RESOLUTION_RX                  8
#else
// 24bit in 32bit slots
#define CFG_TUD_AUDIO_FUNC_1_FORMAT_2_N_BYTES_PER_SAMPLE_TX          4
#define CFG_TUD_AUDIO_FUNC_1_FORMAT_2_RESOLUTION_TX                  24
#define CFG_TUD_AUDIO_FUNC_1_FORMAT_2_N_BYTES_PER_SAMPLE_RX          4
#define CFG_TUD_AUDIO_FUNC_1_FORMAT_2_RESOLUTION_RX                  24
#endif

// EP IN
#define CFG_TUD_AUDIO_ENABLE_EP_IN                1

#define CFG_TUD_AUDIO_FUNC_1_FORMAT_1_EP_SZ_IN    TUD_AUDIO_EP_SIZE(CFG_TUD_AUDIO_FUNC_1_MAX_SAMPLE_RATE, CFG_TUD_AUDIO_FUNC_1_FORMAT_1_N_BYTES_PER_SAMPLE_TX, CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX)
#define CFG_TUD_AUDIO_FUNC_1_FORMAT_2_EP_SZ_IN    TUD_AUDIO_EP_SIZE(CFG_TUD_AUDIO_FUNC_1_MAX_SAMPLE_RATE, CFG_TUD_AUDIO_FUNC_1_FORMAT_2_N_BYTES_PER_SAMPLE_TX, CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX)

#define CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ      TU_MAX(CFG_TUD_AUDIO_FUNC_1_FORMAT_1_EP_SZ_IN, CFG_TUD_AUDIO_FUNC_1_FORMAT_2_EP_SZ_IN)*2
#define CFG_TUD_AUDIO_FUNC_1_EP_IN_SZ_MAX         TU_MAX(CFG_TUD_AUDIO_FUNC_1_FORMAT_1_EP_SZ_IN, CFG_TUD_AUDIO_FUNC_1_FORMAT_2_EP_SZ_IN)

// EP OUT
#define CFG_TUD_AUDIO_ENABLE_EP_OUT               1

#define CFG_TUD_AUDIO_UNC_1_FORMAT_1_EP_SZ_OUT    TUD_AUDIO_EP_SIZE(CFG_TUD_AUDIO_FUNC_1_MAX_SAMPLE_RATE, CFG_TUD_AUDIO_FUNC_1_FORMAT_1_N_BYTES_PER_SAMPLE_RX, CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX)
#define CFG_TUD_AUDIO_UNC_1_FORMAT_2_EP_SZ_OUT    TUD_AUDIO_EP_SIZE(CFG_TUD_AUDIO_FUNC_1_MAX_SAMPLE_RATE, CFG_TUD_AUDIO_FUNC_1_FORMAT_2_N_BYTES_PER_SAMPLE_RX, CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX)

#define CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ     TU_MAX(CFG_TUD_AUDIO_UNC_1_FORMAT_1_EP_SZ_OUT, CFG_TUD_AUDIO_UNC_1_FORMAT_2_EP_SZ_OUT)*2
#define CFG_TUD_AUDIO_FUNC_1_EP_OUT_SZ_MAX        TU_MAX(CFG_TUD_AUDIO_UNC_1_FORMAT_1_EP_SZ_OUT, CFG_TUD_AUDIO_UNC_1_FORMAT_2_EP_SZ_OUT)

// Número de AS interfaces en la función (OUT + IN) = 2
#define CFG_TUD_AUDIO_FUNC_1_N_AS_INT             2

// Control request buffer
#define CFG_TUD_AUDIO_FUNC_1_CTRL_BUF_SZ          64

#ifdef __cplusplus
}
#endif

#endif /* _TUSB_CONFIG_H_ */
