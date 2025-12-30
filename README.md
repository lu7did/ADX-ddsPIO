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
![Alt Text](doc/ADX-ddsPIO.png?raw=true "Transceiver Circuit (Version 0.x)")

The main differences with the original ADX circuit are:

* Si5351 module no longer present.
* Audio signal from SPKR processed to generate a on/off signal.
* Additional SYNC button (for future time sync).
* Additional BEACON jumper (for future automatic beacon).


The pinout assignment for this version is shown in the following table:


![Alt Text](doc/ADX-ddsPIO_pinout.png?raw=true "Raspberry Pi Pico pinout assignment")
