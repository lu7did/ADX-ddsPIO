/**
 * Band Scanner Example for ADX-rp2040-DDS
 * 
 * This example demonstrates scanning through amateur radio bands
 * and monitoring FT8 activity
 * 
 * Usage:
 * 1. Replace main() in src/main.c with this code
 * 2. Rebuild the project
 * 3. Flash to the Pico
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "config.h"
#include "dds.h"
#include "usb_audio.h"
#include "adx_transceiver.h"
#include "ft8_protocol.h"

// Band names for display
static const char* band_names[] = {
    "80m", "40m", "30m", "20m", "17m", "15m", "10m"
};

int main() {
    // Initialize stdio
    stdio_init_all();
    
    // Initialize status LED
    gpio_init(PIN_STATUS_LED);
    gpio_set_dir(PIN_STATUS_LED, GPIO_OUT);
    
    printf("Band Scanner Example\n");
    printf("====================\n\n");
    
    // Initialize all subsystems
    dds_init();
    usb_audio_init();
    adx_transceiver_init();
    ft8_protocol_init();
    
    // Start in RX mode
    adx_set_mode(ADX_MODE_RX);
    
    // Wait for USB audio
    printf("Waiting for USB audio...\n");
    while (!usb_audio_is_ready()) {
        usb_audio_task();
        sleep_ms(100);
    }
    printf("USB audio ready!\n\n");
    
    // Scanning parameters
    adx_band_t current_band = ADX_BAND_40M;
    uint32_t time_on_band = 0;
    const uint32_t scan_interval = 30000;  // 30 seconds per band
    
    printf("Starting band scan...\n");
    printf("Scan interval: %lu seconds per band\n\n", scan_interval / 1000);
    
    while (true) {
        // LED blink pattern
        gpio_put(PIN_STATUS_LED, 1);
        sleep_ms(50);
        gpio_put(PIN_STATUS_LED, 0);
        
        // Process tasks
        adx_transceiver_task();
        usb_audio_task();
        ft8_protocol_task();
        
        // Check if it's time to change bands
        if (time_on_band >= scan_interval) {
            // Move to next band
            current_band = (current_band + 1) % ADX_BAND_COUNT;
            adx_set_band(current_band);
            
            printf("Scanning: %s (%lu Hz)\n", 
                   band_names[current_band],
                   dds_get_frequency());
            
            time_on_band = 0;
        }
        
        time_on_band += 100;
        sleep_ms(100);
    }
    
    return 0;
}
