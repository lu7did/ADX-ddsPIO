# ADX-rp2040-DDS Quick Reference

## Quick Start

### 1. Build
```bash
export PICO_SDK_PATH=/path/to/pico-sdk
mkdir build && cd build
cmake .. && make -j4
```

### 2. Flash
1. Hold BOOTSEL + Connect USB
2. Copy `adx_rp2040_dds.uf2` to RPI-RP2 drive

### 3. Connect
```bash
screen /dev/ttyACM0 115200  # Serial console
```

## GPIO Pin Map

| Pin | Function | Description |
|-----|----------|-------------|
| 0 | DDS_OUT | RF Output |
| 16 | TX_CTL | Transmit Control |
| 17 | RX_CTL | Receive Control |
| 25 | LED | Status LED |
| USB | Audio/Debug | USB Connection |

## Frequency Bands

| Band | Frequency | Wavelength |
|------|-----------|------------|
| 80m | 3.573 MHz | ~80 meters |
| 40m | 7.074 MHz | ~40 meters |
| 30m | 10.136 MHz | ~30 meters |
| 20m | 14.074 MHz | ~20 meters |
| 17m | 18.100 MHz | ~17 meters |
| 15m | 21.074 MHz | ~15 meters |
| 10m | 28.074 MHz | ~10 meters |

## Basic API

### Initialize
```c
dds_init();
usb_audio_init();
adx_transceiver_init();
ft8_protocol_init();
```

### Set Frequency
```c
adx_set_band(ADX_BAND_40M);  // 40 meters
```

### Control TX/RX
```c
adx_set_mode(ADX_MODE_TX);   // Transmit
adx_set_mode(ADX_MODE_RX);   // Receive
```

### Set Power
```c
adx_set_power(50);  // 50% power
```

### FT8 Transmit
```c
ft8_start_transmission("CALLSIGN", "GRID", -10);
```

## Configuration Files

| File | Purpose |
|------|---------|
| `src/config.h` | Hardware/Software config |
| `src/tusb_config.h` | USB configuration |
| `CMakeLists.txt` | Build settings |

## Common Commands

### Build
```bash
cd build && make -j4
```

### Clean
```bash
cd build && make clean
```

### Rebuild
```bash
cd build && cmake .. && make -j4
```

### Check Size
```bash
arm-none-eabi-size adx_rp2040_dds.elf
```

## WSJT-X Setup

1. **Audio Settings**
   - Input: ADX-rp2040-DDS
   - Output: ADX-rp2040-DDS
   - Sample Rate: 48000 Hz

2. **Radio Settings**
   - Rig: None
   - Mode: USB

3. **Frequencies**
   - 40m FT8: 7.074 MHz
   - 20m FT8: 14.074 MHz

## Troubleshooting

| Problem | Solution |
|---------|----------|
| No USB serial | Check cable, try different port |
| No audio device | Wait 5-10s, check USB |
| LED not blinking | Re-flash firmware |
| No RF output | Check GPIO 0 with scope |

## Status LED Patterns

| Pattern | Meaning |
|---------|---------|
| 1s on, 9s off | Normal operation |
| Rapid blink | Scanning mode |
| Solid on | Transmitting |
| Off | Not initialized |

## Safety Checklist

- [ ] Amateur radio license (if required)
- [ ] Proper RF filtering
- [ ] Antenna connected
- [ ] Low power for testing
- [ ] Monitor temperature
- [ ] Check for interference

## File Structure

```
ADX-rp2040-DDS/
├── src/
│   ├── main.c              # Main application
│   ├── dds.c/h             # DDS module
│   ├── usb_audio.c/h       # USB audio
│   ├── adx_transceiver.c/h # Transceiver
│   ├── ft8_protocol.c/h    # FT8 protocol
│   └── config.h            # Configuration
├── examples/               # Code examples
├── CMakeLists.txt         # Build config
├── README.md              # Documentation
└── BUILD.md               # Build guide
```

## Useful Links

- [Pico SDK Docs](https://raspberrypi.github.io/pico-sdk-doxygen/)
- [WSJT-X Download](https://physics.princeton.edu/pulsar/k1jt/wsjtx.html)
- [FT8 Operating Tips](https://physics.princeton.edu/pulsar/k1jt/FT8_Operating_Tips.pdf)
- [GitHub Repo](https://github.com/lu7did/ADX-rp2040-DDS)

## Version Info

Current Version: **0.1.0** (Experimental)

Check [CHANGELOG.md](CHANGELOG.md) for version history.

## Support

- Issues: https://github.com/lu7did/ADX-rp2040-DDS/issues
- Docs: See README.md, API.md, TUTORIAL.md
- Contributing: See CONTRIBUTING.md

---

**73 de LU7DID** 📡
