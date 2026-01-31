#pragma once

// Forzar OSAL de TinyUSB para Pico (evita tu_fifo_t sin mutex_*)
#ifdef CFG_TUSB_OS
  #undef CFG_TUSB_OS
#endif
#define CFG_TUSB_OS OPT_OS_PICO

// Opcional: evitar que alguien defina esto con otra cosa antes
#ifdef tud_int_handler
  #undef tud_int_handler
#endif


#ifndef __ASSEMBLER__
  #include <assert.h>
#endif

//*--- Avoid collisions
#ifdef assert
#undef assert
#endif

#include <assert.h>

// --- static_assert compat (C11 / C++11) ---
#if !defined(__cplusplus)
  #if !defined(static_assert)
    #if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
      #define static_assert _Static_assert
    #else
      // fallback: si no hay C11, degradamos a typedef trick (evita romper headers)
      #define static_assert(cond, msg) typedef char static_assertion_##__LINE__[(cond)?1:-1]
    #endif
  #endif
#endif

