clear
cd /Users/PCOLLA/Documents/GitHub/ADX-ddsPIO/src/ADX-ddsPIO
unset CMAKE_ARGS
#*  CMake para Raspberry Pi Pico (convencional)
#*  cmake -S . -B build -DFAMILY=rp2040 -DPICO_SDK_PATH=/Users/PCOLLA/Documents/GitHub/pico/pico-sdk

#* CMake para RaspBerry Pi Pico W (Wireless)
#*  cmake -S . -B build -DPICO_BOARD=pico_w -DFAMILY=rp2040 -DPICO_SDK_PATH=/Users/PCOLLA/Documents/GitHub/pico/pico-sdk

make -C build -j

