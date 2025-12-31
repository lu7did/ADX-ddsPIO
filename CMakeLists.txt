set(PROJECT ADX-ddsPIO)
cmake_minimum_required(VERSION 3.12)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
project(${PROJECT} C CXX ASM)
pico_sdk_init()
add_executable(${PROJECT} testDDS.c)

target_sources(ADX-ddsPIO PUBLIC
	    ${CMAKE_CURRENT_LIST_DIR}/lib/assert.c
        ${CMAKE_CURRENT_LIST_DIR}/piodco/piodco.c
        ${CMAKE_CURRENT_LIST_DIR}/gpstime/GPStime.c
        ${CMAKE_CURRENT_LIST_DIR}/debug/logutils.c
        ${CMAKE_CURRENT_LIST_DIR}/testDDS.c 
        ${CMAKE_CURRENT_LIST_DIR}/conswrapper.c
        ${CMAKE_CURRENT_LIST_DIR}/hfconsole/hfconsole.c
        )


#target_link_libraries(${PROJECT} pico_stdlib)

add_compile_options(-Wall)

# Add any user requested libraries
target_link_libraries(
        ADX-ddsPIO
        pico_stdlib
        pico_sync
        pico_multicore
        hardware_timer
        hardware_clocks
        hardware_pio
        hardware_uart
        )

pico_add_extra_outputs(ADX-ddsPIO)
pico_add_extra_outputs(${PROJECT})
pico_enable_stdio_usb(${PROJECT} 1)
pico_enable_stdio_uart(${PROJECT} 0)

target_include_directories(ADX-ddsPIO PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/gpstime
  ${CMAKE_CURRENT_LIST_DIR}/hfconsole
  ${CMAKE_CURRENT_LIST_DIR}/..
)
pico_add_extra_outputs(ADX-ddsPIO)