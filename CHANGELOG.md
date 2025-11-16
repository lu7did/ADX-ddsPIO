# Changelog

All notable changes to the ADX-rp2040-DDS project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Planned
- Complete FT8 encoding/decoding implementation
- Full USB audio device functionality
- Hardware validation and testing
- Performance optimization
- Additional digital modes (PSK31, RTTY)
- Web-based configuration interface

## [0.1.0] - 2025-11-16

### Added
- Initial project structure and build system
- CMake configuration for RP2040/Pico SDK
- DDS (Direct Digital Synthesis) module
  - PIO/PWM-based signal generation
  - Frequency control (1-30 MHz)
  - No external Si5351 required
- USB Audio Device module
  - TinyUSB integration
  - 48 kHz stereo audio
  - Digital sound card interface
- ADX Transceiver module
  - TX/RX mode switching
  - Multi-band support (80m, 40m, 30m, 20m, 17m, 15m, 10m)
  - Power control (0-100%)
- FT8 Protocol module
  - Basic FT8 framework
  - Message structure
  - Transmission control
- Configuration system
  - Centralized config.h
  - Hardware pin assignments
  - Audio and DDS parameters
- Documentation
  - Comprehensive README.md
  - BUILD.md with build instructions
  - HARDWARE.md with hardware design
  - API.md with complete API reference
  - CONTRIBUTING.md for contributors
  - TUTORIAL.md for getting started
- USB descriptors for device identification
- TinyUSB configuration
- .gitignore for build artifacts

### Technical Details
- Target: Raspberry Pi RP2040 (Cortex-M0+ @ 125MHz)
- Language: C11
- Build System: CMake 3.13+
- USB Stack: TinyUSB
- Audio: 48kHz, 16-bit, stereo
- DDS: Direct synthesis via PIO/PWM

### Notes
- This is an experimental release
- Hardware testing pending
- FT8 encoding/decoding are placeholders
- Full USB audio not yet implemented

## Version History

### Version Numbering
- Major: Significant feature additions or API changes
- Minor: New features, backward compatible
- Patch: Bug fixes and minor improvements

### Release Tags
- v0.1.0 - Initial experimental release

## Future Roadmap

### v0.2.0 (Planned)
- Complete FT8 implementation
- Working USB audio device
- Hardware validation

### v0.3.0 (Planned)
- Additional digital modes
- Improved DDS performance
- Configuration interface

### v1.0.0 (Future)
- Production-ready release
- Full feature set
- Tested and stable

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for how to contribute to this project.

## Links

- [GitHub Repository](https://github.com/lu7did/ADX-rp2040-DDS)
- [Issue Tracker](https://github.com/lu7did/ADX-rp2040-DDS/issues)
- [Releases](https://github.com/lu7did/ADX-rp2040-DDS/releases)
