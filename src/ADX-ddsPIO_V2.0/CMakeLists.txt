cmake_minimum_required(VERSION 3.13)

# ------------------------------------------------------------------------------------
# Pico SDK import
# ------------------------------------------------------------------------------------
# Default PICO_SDK_PATH si no se pasó por -D ni por env
if(NOT DEFINED PICO_SDK_PATH)
  if(DEFINED ENV{PICO_SDK_PATH})
    set(PICO_SDK_PATH "$ENV{PICO_SDK_PATH}" CACHE PATH "Path to Pico SDK")
  else()
    set(PICO_SDK_PATH "/Users/PCOLLA/Documents/GitHub/pico/pico-sdk" CACHE PATH "Path to Pico SDK")
  endif()
endif()

include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(ADX-ddsPIO C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

set(TARGET ADX-ddsPIO)

add_executable(${TARGET}
  src/main.c
  src/usb_descriptors.c
  src/usb_audio.c
  src/usb_msc.c
  src/flash_bd.c
  src/fs.c
  src/fatfs/ff.c
  src/fatfs/diskio.c
  src/fatfs/fattime.c
  src/piodco.c
  src/si4732.c
  src/EEPROM.c
  src/quad.c
  src/si4732_patch_ssb.c
  src/lib/adx_assert.c
)

# IMPORTANTE:
# - Esto garantiza que TinyUSB encuentre tu tusb_config.h (en src/)
target_include_directories(${TARGET} PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/src
  ${CMAKE_CURRENT_LIST_DIR}/src/lib
  ${CMAKE_CURRENT_LIST_DIR}/src/fatfs
  ${CMAKE_CURRENT_LIST_DIR}/src/lib/jsmn
  ${CMAKE_CURRENT_LIST_DIR}/src/lib/thirdparty
)

# Flags: evitá romper terceros
target_compile_options(${TARGET} PRIVATE
  -Wall -Wextra
  -Werror
  -Wsign-conversion
  -Wconversion
)

# No tratar warnings de lwIP como errores (si está presente)
if (TARGET pico_lwip)
  target_compile_options(pico_lwip INTERFACE -Wno-error -Wno-sign-conversion -Wno-conversion)
endif()
if (TARGET pico_lwip_nosys)
  target_compile_options(pico_lwip_nosys INTERFACE -Wno-error -Wno-sign-conversion -Wno-conversion)
endif()

# Si querés meter project_config.h forzado
target_compile_options(${TARGET} PRIVATE
  $<$<COMPILE_LANGUAGE:C>:-include ${CMAKE_CURRENT_LIST_DIR}/src/project_config.h>
  $<$<COMPILE_LANGUAGE:CXX>:-include ${CMAKE_CURRENT_LIST_DIR}/src/project_config.h>
)

# Evitar que stdio use USB (para no interferir con la device stack / CDC)
pico_enable_stdio_usb(${TARGET} 0)
pico_enable_stdio_uart(${TARGET} 1)

target_link_libraries(${TARGET}
  pico_stdlib
  tinyusb_device
  tinyusb_board
  pico_multicore
  hardware_pio
  hardware_adc
  hardware_rtc
  hardware_flash
  hardware_sync
  hardware_i2c
  hardware_gpio
  hardware_watchdog
  hardware_clocks
  hardware_pll
)

pico_add_extra_outputs(${TARGET})

pico_generate_pio_header(${TARGET} ${CMAKE_CURRENT_LIST_DIR}/src/BFO.pio)
pico_generate_pio_header(${TARGET} ${CMAKE_CURRENT_LIST_DIR}/src/dco2.pio)
pico_generate_pio_header(${TARGET} ${CMAKE_CURRENT_LIST_DIR}/src/quad.pio)
