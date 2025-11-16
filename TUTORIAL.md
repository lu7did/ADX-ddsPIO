# ADX-rp2040-DDS Tutorial

## Getting Started with ADX-rp2040-DDS

This tutorial will guide you through setting up and using your ADX-rp2040-DDS digital transceiver.

## Prerequisites

### Hardware
- Raspberry Pi Pico or RP2040-based board
- USB cable
- Computer with USB port
- (Optional) External RF amplifier and filters

### Software
- Pico SDK installed
- Build tools (CMake, ARM GCC)
- FT8 software (e.g., WSJT-X)
- Serial terminal (e.g., minicom, PuTTY)

## Step 1: Hardware Setup

### Basic Connections

```
┌─────────────────┐
│  Raspberry Pi   │
│     Pico        │
│                 │
│  USB ───────────┼──── Computer
│                 │
│  GPIO 0 ────────┼──── DDS Output (to RF circuit)
│  GPIO 16 ───────┼──── TX Control
│  GPIO 17 ───────┼──── RX Control
│  GPIO 25 ───────┼──── Status LED
│                 │
└─────────────────┘
```

### Power
Connect the Pico to your computer via USB. This provides both power and data connection.

## Step 2: Building the Firmware

### Install Pico SDK

```bash
# Clone Pico SDK
git clone https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk
git submodule update --init
export PICO_SDK_PATH=$(pwd)
```

### Build ADX-rp2040-DDS

```bash
# Clone this repository
git clone https://github.com/lu7did/ADX-rp2040-DDS.git
cd ADX-rp2040-DDS

# Create build directory
mkdir build
cd build

# Configure and build
cmake ..
make -j4
```

### Output Files

After successful build:
- `adx_rp2040_dds.uf2` - Main firmware file

## Step 3: Flashing the Firmware

1. **Enter bootloader mode:**
   - Disconnect Pico from USB
   - Hold the BOOTSEL button
   - Connect to USB while holding BOOTSEL
   - Release BOOTSEL

2. **Flash firmware:**
   - The Pico appears as a USB drive (RPI-RP2)
   - Copy `adx_rp2040_dds.uf2` to this drive
   - The Pico will automatically reboot

3. **Verify:**
   - Status LED should blink once per second
   - USB serial port appears

## Step 4: Connecting to the Device

### Serial Terminal

Connect to the USB serial port:

```bash
# Linux/macOS
screen /dev/ttyACM0 115200

# Windows - use PuTTY
# Port: COMx, Baud: 115200
```

You should see:
```
ADX-rp2040-DDS Digital Transceiver Starting...
System Clock: 125000000 Hz
DDS initialized successfully
USB audio initialized successfully
ADX transceiver initialized successfully
FT8 protocol initialized successfully
ADX-rp2040-DDS is now running (experimental)
```

### USB Audio Device

The device appears as "ADX-rp2040-DDS" in your system's audio devices.

**Linux:**
```bash
aplay -l  # List playback devices
arecord -l  # List recording devices
```

**Windows:**
- Open Sound Settings
- Look for "ADX-rp2040-DDS"

**macOS:**
- Open System Preferences > Sound
- Look for "ADX-rp2040-DDS"

## Step 5: Using with WSJT-X

### Configure WSJT-X

1. **Open WSJT-X**

2. **File > Settings > Audio**
   - Input: Select "ADX-rp2040-DDS"
   - Output: Select "ADX-rp2040-DDS"
   - Sample Rate: 48000 Hz

3. **File > Settings > Radio**
   - Rig: None (USB audio only)
   - Mode: USB

4. **File > Settings > Frequencies**
   - Working frequency: 7.074 MHz (40m FT8)

### Operating

1. **Set your callsign and grid:**
   - File > Settings > General
   - Enter your callsign
   - Enter your grid locator

2. **Start monitoring:**
   - Click "Monitor" button
   - Watch for decoded signals

3. **Transmitting:**
   - Currently experimental
   - TX/RX controlled by software

## Step 6: Basic Testing

### Test DDS Output

You can verify the DDS output on GPIO 0 with an oscilloscope or frequency counter.

Expected output:
- Frequency: 7.074 MHz (40m band default)
- Waveform: PWM/Square wave

### Test USB Audio

Play a test tone through the device:

```bash
# Linux - generate test tone
speaker-test -D hw:ADXrp2040DDS -c 2 -r 48000
```

## Troubleshooting

### No USB Serial Port

- Check USB cable (must support data, not just power)
- Try different USB port
- Reflash firmware

### No Audio Device

- Wait 5-10 seconds after connecting
- Check USB connection
- Verify in device manager/system profiler

### LED Not Blinking

- Check power supply
- Verify firmware flashed correctly
- Try re-flashing

### No RF Output

- Check GPIO 0 with oscilloscope
- Verify band selection
- Check external RF circuit

## Advanced Configuration

### Change Default Band

Edit `src/adx_transceiver.c`:

```c
// Change this line:
adx_state.band = ADX_BAND_40M;  // Default to 40m

// To:
adx_state.band = ADX_BAND_20M;  // Default to 20m
```

Rebuild and reflash.

### Adjust TX Power

Edit `src/config.h`:

```c
#define TX_POWER_DEFAULT    50  // Change to desired %
```

### Custom Frequencies

Edit band frequency table in `src/adx_transceiver.c`:

```c
static const uint32_t band_frequencies[ADX_BAND_COUNT] = {
    3573000,   // 80m - customize these
    7074000,   // 40m
    // ...
};
```

## Safety and Legal

⚠️ **Important:**
- This generates RF signals
- Ensure proper filtering and shielding
- Use appropriate antenna
- Comply with local amateur radio regulations
- Valid amateur radio license may be required
- Start with low power for testing

## Next Steps

1. Test with different bands
2. Experiment with FT8 communications
3. Add external RF amplifier
4. Design and build proper RF filters
5. Create custom PCB

## Support

- GitHub Issues: Report bugs and request features
- Documentation: See API.md for programming reference
- Contributing: See CONTRIBUTING.md to help develop

## Resources

- [WSJT-X Software](https://physics.princeton.edu/pulsar/k1jt/wsjtx.html)
- [FT8 Operating Guide](https://physics.princeton.edu/pulsar/k1jt/FT8_Operating_Tips.pdf)
- [RP2040 Datasheet](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf)
- [Pico SDK Documentation](https://raspberrypi.github.io/pico-sdk-doxygen/)

## Enjoy!

Have fun experimenting with your ADX-rp2040-DDS digital transceiver! 73 de LU7DID
