
/*
 * =======================================================================================
 * project_config.h
 * (c) Dr. Pedro E. Colla (LU7DZ) <pedro.colla@gmail.com>
 * 
 * Implementation of a rp2040 based controller  of a physical file system 
 * Definition of various configuration items
 * =======================================================================================
 * This is mainly an integration effort, the code in this library has been developed 
 * from scratch for this project.
 * However the work received an huge benefit from previous work from many parties,
 * including myself as follows:
 *----------------------------------------------------------------------------
 * Version 1.0
 * - Initial release
 */

#pragma once

//*--- Force O/S for TinyUSB to be PICO (avoid tu_fifo_t not having mutex_*)

#ifdef CFG_TUSB_OS
#undef CFG_TUSB_OS
#endif
#define CFG_TUSB_OS OPT_OS_PICO

//*--- Optional: avoid circular definitions
#ifdef tud_int_handler
  #undef tud_int_handler
#endif

//*--- Avoid parts of the package to be processed by the ASM 

#ifndef __ASSEMBLER__
  #include <assert.h>
#endif

//*--- Yet another collision avoidance
#ifdef assert
#undef assert
#endif

#include <assert.h>

//*--- definition of static_assert compat (C11 / C++11) ---
#if !defined(__cplusplus)
  #if !defined(static_assert)
    #if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
      #define static_assert _Static_assert
    #else
      //*--- fallback: if no C 11 it's degraded using a typedef trick to avoid breaking headers
      #define static_assert(cond, msg) typedef char static_assertion_##__LINE__[(cond)?1:-1]
    #endif
  #endif
#endif

