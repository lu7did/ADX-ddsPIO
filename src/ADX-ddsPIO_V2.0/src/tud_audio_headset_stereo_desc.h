#pragma once

// Evita redefinición si en algún lado ya lo definieron
#ifndef TUD_AUDIO_HEADSET_STEREO_DESCRIPTOR
  // Pegá aquí EXACTAMENTE la macro que tomaste del example que corresponda
  // (cdc_uac2 o uac2_headset) y sólo UNA versión.
  #define TUD_AUDIO_HEADSET_STEREO_DESCRIPTOR(_stridx, _epout, _epin) \
    /* ... macro completa ... */
#endif

#ifndef TUD_AUDIO_HEADSET_STEREO_DESC_LEN
  #define TUD_AUDIO_HEADSET_STEREO_DESC_LEN  /* ... valor o macro ... */
#endif
