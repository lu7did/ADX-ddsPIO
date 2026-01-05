# ADX-ddsPIO

This is a project aimed to achieve several (experimental) goals:

* Port the ADX firmware to the rp2040 (already done with ADX-rp2040).
* Implement a USB digital audio interface.
* Implement an all Raspberry Pico based DDS generation.

Once implemented an ADX class FT8 digital transceiver can be created using
different hardware strategies.

Basic specs for this transceiver would be:

* Operate in any of the HF bands (160m to 10m), attempts to marginal use on 6m will be made.
* 2-3W output (WSPR,FT8;JS8,FT4).
* Different alternatives for receiver: DC (CD2003GP), superhet (CD20023GP) and digital (Si4732)
* SWR protection.
* Beacon mode (WSPR,FT8).
* ATU reset output.
* PA SWR protection (zener diode).

This project relies *heavily* on the superb work made by Roman (R2BDT) with his
pico-WSPR-tx project (link) which in turns relies on his pico-hf-oscillator [link](https://github.com/RPiks/pico-hf-oscillator)
project.

Most of the USB digital interface work couldn't have been done without the 
insights provided by Hitoshi-san (JE1RAV) with his QP-7C_RP2040 project [link](https://github.com/je1rav/QP-7C_RP2040).

The ADX architecture has been originally developed and promoted by Barb
(WB2CBA) and had been spinned off by dozens of experimental implementations, most
of the transceiver hardware is rooted on his signature original design.

The ADX-rp2040 firmware [link](https://github.com/lu7did/ADX-rp2040) was originally developed by me as an adaptation of the
ADX Arduino firmware ported to the rp2040 architecture, most of the actual code was refactored in the process.

With enough luck and cooperation this project might turn into a system which
includes the different components in order to be reproduced, lots of work needs
to occur for this to happen.

The project it is in the very preliminary stages, no firm designs nor firmware
is yet available.

73 de Pedro (LU7DZ/LT7D)


# Circuit Schematics

## Circuit Design (version 0.x)

This circuit is largely based on the original ADX transceiver [link](https://github.com/WB2CBA/ADX) by Barb ([WB2CBA](https://www.qrz.com/db/WB2CBA)).
![Alt Text](doc/ADX-ddsPIO-Schematic.png?raw=true "Transceiver Circuit (Version 0.x)")

The main differences with the original ADX circuit are:

* Si5351 module no longer present.
* Audio signal from SPKR processed to generate a on/off signal.
* Additional SYNC button (for future time sync).
* Additional BEACON jumper (for future automatic beacon).


The pinout assignment for this version is shown in the following table:


![Alt Text](doc/ADX-ddsPIO_pinout.png?raw=true "Raspberry Pi Pico pinout assignment")

## Test resources

The following test resources has been built

### ADX-ddsPIO mockup

In order to develop the first stages of the firmware a reduced mockup has been used
![Alt Text](doc/ADX-ddsPIO-mockup.jpeg?raw=true "ADX-ddsPIO-mockup")
This reduced scope board contains just the processor and the push buttons. A small
RF probe is feed into a wire for local "on the air" tests.

### testDDS utility

This utility is a reduced firmware implementing the DDS using a fixed frequency given
by the GEN_FRQ_HZ parameter in the ddsTest.h file. It's main purpose is to explore
the concept and the initial measurements using the Raspberry Pi Pico as an RF generator.
Also to integrate the code originally on the *pico-hf-oscillator* package on this
project.

```
This code is a crude porting of the test.c program from the pico-hf-oscillator package
```

### testFT8 utility

This utility has been created as a reduced scope test bed for the board and some of the
DDS functions applied to the generation of FT8 signals.

* When boot if the SYNC button is **NOT** pressed the firmware enters the LED test mode
  where when each of the buttons are pressed a different LED is lit.
  Meanwhile the DDS generates RF at the frequency indicated by the *GEN_FRQ_HZ* parameter.
* When boot if the SYNC button **IS KEPT** pressed the firmware waits and upon release enters the FT8 test mode, it's 
  important the SYNC button is released as close to the top of the minute (sec=0) of
  any minute. In this way the internal clock is set at a random date time but the 
  second is synchronized. At the start of each minute a fixed message is sent using
  FT8. The message to be sent can be changed by modifying the *message* memory area, the 
  base frequency (and band) can be set changing the *GEN_FRQ_HZ* (in Hz) parameter and the
  shift within the FT8 sub-band changing the *FT8_BASE_HZ* (in HZ) parameter.
  While activated in this mode the TX can be turned on by pressing the **TX** button.

The following figure shows how WSJT-X receives the local message
![Alt Text](doc/testFT8_test.png?raw=true "testFT8 Test")


```
When using a wire out of the GPIO 18 pin to transmit "on the air" signals the spectrum
is *extremely* dirty. Do not put this signal into any meaningful antenna and use it
only by short evaluation tests.
```

## testADX utility

This utility continues the integration of the firmware system incorporating the 
features of the ADX board such as UP/DOWN buttons, TX button and the four mode LED
(FT8,FT4,JS8 and WSPR) as well as the TX LED.
The board continuously generates an RF signal, when in RX mode (RXSW high) the output
is connected with the receiver (RXOSC) whilst when in TX mode (RXSW low, TX high) the
output is connected to the driver section.
If properly synchronized (see testFT8) upon start the firmware sends a message at the
top of every minute using the defined frequency and mode.

```
When using a wire out of the GPIO 18 pin to transmit "on the air" signals the spectrum
is *extremely* dirty. Do not put this signal into any meaningful antenna and use it
only by short evaluation tests.
```

# DCO for Raspberry Pi Pico 

(Excerpts from the original **package pico-hf-oscillator** by Roman Piksaykin (R2BDY)
https://www.qrz.com/db/R2BDY

The *pico-hf-oscillator* library for Raspberry Pi Pico includes the headers and source code and all 
necessary build files to build a custom application which turns pico into precise PLL digital
frequency oscillator of the whole of HF radio spectrum (1 Hz to 32.333 MHz) with millihertz resolution.

## Precise frequency resolution

The library provides about 23 milli-Hz frequency resolution. This resolution is limited by 24-bit register which is used in algorithm.
A workingWSPR beacon which has been built on the base of this project proves that the quality of generated signal is sufficient to
such precise (~1.46 Hz step) frequency manipulation digital modes.

The upper freq. limit is ~32.333 MHz and it is achieved only using Pico overclocking to 270MHz.

![mfsk-spectra](https://github.com/RPiks/pico-hf-oscillator/assets/47501785/a8309813-8e77-407e-abfc-58cbd262c35c)

Here is an example of narrowband FSK (9.4 MHz carrier, 5 Hz step, 20 Hz range in total).

## Phased locked loop in C

The DCO uses phase locked loop principle programmed in C.

## *NO* additional hardware

The DCO provides the output signal on the GPIO pin. However if you want to
transmit the signal, you should calculate and provide a lowpass filter of
appropriate frequency. Please also figure out whether you possess rights
to emit radio frequency energy on desired frequency.

## GPS reference frequency correction (optional) since v.0.9

GPS reference frequency correction option provides an absolute frequency error within about ~1Hz in long term.
![pico-hf-oscillator](https://github.com/RPiks/pico-hf-oscillator/assets/47501785/06700e39-6b5f-4a6a-828a-d1cfdd9418ce)

## Dual-core

The DCO uses extensively the secodary core of the pico. The first one is for
your ideas how to modulate the DCO to obtain a desired signal.
The DCO does *NOT* use any floating point operations - all time-critical 
instructions run in 1 CPU cycle.

## Radio transmitters

Owing to the meager frequency step, it is possible to use 3, 5, or 7th harmonics 
of generated frequency. The practical resolution will be quite the same - far
below 1 Hz. Such solution completely cover all HF and VHF band up to ~233 MHz.


## Feedback to Roman

Roman will appreciate any thoughts or comments on that matter.

Roman Piksaykin, amateur callsign R2BDY
https://www.qrz.com/db/R2BDY

