/**
 * Simple Transmit Example for ADX-rp2040-DDS
 * 
 * This example demonstrates basic FT8 transmission
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

int main() {
    // Initialize stdio
    stdio_init_all();
    
    // Initialize status LED
    gpio_init(PIN_STATUS_LED);
    gpio_set_dir(PIN_STATUS_LED, GPIO_OUT);
    
    printf("Simple Transmit Example\n");
    
    // Initialize all subsystems
    dds_init();
    usb_audio_init();
    adx_transceiver_init();
    ft8_protocol_init();
    
    // Configure for 40m band (7.074 MHz)
    adx_set_band(ADX_BAND_40M);
    
    // Set transmit power to 50%
    adx_set_power(50);
    
    printf("Configuration:\n");
    printf("  Band: 40m (7.074 MHz)\n");
    printf("  Power: 50%%\n");
    printf("  Mode: FT8\n");
    
    // Wait for USB audio to be ready
    printf("Waiting for USB audio...\n");
    while (!usb_audio_is_ready()) {
        usb_audio_task();
        sleep_ms(100);
    }
    printf("USB audio ready!\n");
    
    // Transmit sequence
    uint32_t sequence = 0;
    
    while (true) {
        // Blink LED
        gpio_put(PIN_STATUS_LED, 1);
        sleep_ms(100);
        gpio_put(PIN_STATUS_LED, 0);
        
        // Process tasks
        adx_transceiver_task();
        usb_audio_task();
        ft8_protocol_task();
        
        // Transmit every 60 seconds (4 FT8 time slots)
        if (!ft8_is_transmitting() && (sequence % 600 == 0)) {
            printf("\n--- Starting FT8 transmission #%lu ---\n", sequence / 600 + 1);
            
            // Start transmission with your callsign and grid
            // Replace these with your actual callsign and grid
            ft8_start_transmission("LU7DID", "GF05", -10);
        }
        
        sequence++;
        sleep_ms(100);
    }
    
    return 0;
}
