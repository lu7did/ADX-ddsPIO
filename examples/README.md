# ADX-rp2040-DDS Examples

This directory contains example programs demonstrating various features of the ADX-rp2040-DDS system.

## Available Examples

### 1. Simple Transmit (`simple_transmit.c`)

A basic example that demonstrates FT8 transmission.

**Features:**
- Initialize all subsystems
- Configure for 40m band
- Wait for USB audio
- Transmit FT8 messages periodically

**Usage:**
```bash
# Replace main.c with the example
cp examples/simple_transmit.c src/main.c

# Rebuild
cd build
make

# Flash to Pico
cp adx_rp2040_dds.uf2 /media/RPI-RP2/
```

### 2. Band Scanner (`band_scanner.c`)

Scans through amateur radio bands monitoring for activity.

**Features:**
- Cycle through all supported bands
- 30-second dwell time per band
- Display current band and frequency
- Receive mode only

**Usage:**
```bash
# Replace main.c with the example
cp examples/band_scanner.c src/main.c

# Rebuild
cd build
make

# Flash to Pico
cp adx_rp2040_dds.uf2 /media/RPI-RP2/
```

## Modifying Examples

### Changing Your Callsign

In `simple_transmit.c`, replace:
```c
ft8_start_transmission("LU7DID", "GF05", -10);
```

With your callsign and grid:
```c
ft8_start_transmission("YOUR_CALL", "YOUR_GRID", -10);
```

### Adjusting Scan Interval

In `band_scanner.c`, modify:
```c
const uint32_t scan_interval = 30000;  // milliseconds
```

### Changing Default Band

Both examples can be modified to use different default bands:
```c
adx_set_band(ADX_BAND_20M);  // Use 20m instead of 40m
```

Available bands:
- `ADX_BAND_80M` - 80 meters (3.573 MHz)
- `ADX_BAND_40M` - 40 meters (7.074 MHz)
- `ADX_BAND_30M` - 30 meters (10.136 MHz)
- `ADX_BAND_20M` - 20 meters (14.074 MHz)
- `ADX_BAND_17M` - 17 meters (18.100 MHz)
- `ADX_BAND_15M` - 15 meters (21.074 MHz)
- `ADX_BAND_10M` - 10 meters (28.074 MHz)

## Creating Your Own Examples

1. Start with one of the existing examples
2. Include necessary headers:
   ```c
   #include "config.h"
   #include "dds.h"
   #include "usb_audio.h"
   #include "adx_transceiver.h"
   #include "ft8_protocol.h"
   ```
3. Initialize all subsystems in your `main()`
4. Implement your custom logic
5. Process tasks in the main loop

## Testing Examples

### Required Hardware
- Raspberry Pi Pico or RP2040 board
- USB connection to computer
- (Optional) Oscilloscope for RF verification

### Monitoring Output
Connect to the USB serial port:
```bash
# Linux/macOS
screen /dev/ttyACM0 115200

# Windows - Use PuTTY or similar
```

### Verifying Operation
- Status LED should blink
- USB audio device should appear
- Serial output shows initialization messages
- GPIO 0 should show RF output (measure with scope)

## Notes

- These examples are for educational and testing purposes
- Ensure compliance with amateur radio regulations
- Use appropriate RF filtering and shielding
- Valid amateur radio license may be required
- Start with low power for initial testing

## Contributing

Have an interesting example? Submit a pull request! See [CONTRIBUTING.md](../CONTRIBUTING.md) for guidelines.

## Support

For questions or issues with examples:
1. Check the [main README](../README.md)
2. Review the [API documentation](../API.md)
3. Open an issue on GitHub

## License

All examples are provided under the same MIT License as the main project.
