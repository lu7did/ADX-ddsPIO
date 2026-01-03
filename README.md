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

