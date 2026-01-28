# ADX-ddsPIO


A brief story of the project start with the excelent ADX Transceiver from Barb (WB2CBA) which can be found at

* Github site [link](http://www.github.com/WB2CBA/ADX).
* ADX transceiver blog [link](https://antrak.org.tr/blog/adx-arduino-digital-transceiver)

The ADX transceiver is powered by an Arduino Nano (ADX) or Arduino Uno (ADX_UNO) boards using both the 
ATMEL ATMEGA382p processor.

In order to leverage the capabilities of the transceiver with a powerful processor such as the Raspberry Pi Pico
which uses the rp2040 architecture a project called **ADX-rp2040** was started.

Then a map between the Arduino board I/O and the rp2040 I/O was made showing some differences needs to be addressed
which requires additional circuitry.

As an evolution of that project the goal to reduce the footprint of the transceiver, trying to remove the 
signal generator function (Si5351 based), this effort leads to this project.

The project aims to achiveve Several (experimental) goals:

* Port the ADX firmware to the rp2040 (already done with the [ADX-rp2040](https://www.github.com/lu7did/ADX-rp2040)
* Design and develop a hardware board to support the project
* Implement a USB digital audio interface, the board can be seen as a virtual sound card.
* Implement an all Raspberry Pico based DDS generation (no Si5351 board).

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

This project relies *heavily* on the superb work made by several authors:

* Roman (R2BDT) with his pico-WSPR-tx project (link) which in turns relies on his pico-hf-oscillator project
  [link](https://github.com/RPiks/pico-hf-oscillator)

* Hitoshi-san (JE1RAV) with his QP-7C_RP2040 project did most of the heavy lifting to use a USB for both
  digital audio and serial interface (CDC).
  [link](https://github.com/je1rav/QP-7C_RP2040).

* As said the ADX architecture has been originally developed and promoted by Barb 
  (WB2CBA) and had been spinned off by dozens of experimental implementations, most
  of the transceiver hardware is rooted on his signature original design.

* The ADX-rp2040 firmware was originally developed by me as an adaptation of the ADX Arduino firmware,
  differences with the original code were quite large so the actual final code was refactored in the process.
  [link](https://github.com/lu7did/ADX-rp2040)

* Hans Summers (G0UPL) developed the original QCX transceiver, which creates a whole
  family of high performance, yet affordable,  homebrew transceivers.

* Guido (PE1NNZ) who developed the original uSDX microcode for the Arduino which inspired
  manyfold designs, specially the uSDX class transceivers, and opened the gate for a generation
  of low cost at reasonable performance ham radio equipment.
 
With enough luck and cooperation this project might turn into a system which
includes the different components in order to be reproduced, lots of work needs
to occur for this to happen.

73 de Pedro (LU7DZ/LT7D)

# Support and issues

This is an experimental, work-in-progress, non-profit project performed as closest to the ham spirit as possible. Only spare, hobby,
time is available to move the project forward or to provide support on usage or issues.

Until further notice is provided this project is **work in progress** and not able to be reproduced directly.

If anybody has questions or issues please:
 
* Be sure you read the documentation first.
* Check on the issues list of the GitHub site, the issue might have been described there or even a workaround might exists for it.
* State it as an  English request. English isn’t even my fourth language, still I do my best to adhere to it. 
* For casual question you can use the groups.io uSDX forum and for a longer ones please open and issue at the GitHub portal of the project. 
* Express very clearly which version and level the firmware has. In most cases using the latest would solve the issue.
* Ensure the issue happens with a freshly downloaded last version of the firmware, don’t expect me to debug any modification you did.
* Describe in your own words the problem and what you did to expose it and what workarounds you attempted.
* Add the content of the monitoring terminal session with DEBUG enabled to help me understand what is going on.
* Attach any other documentation you think might help debugging the issue.
 

Please report ONE (1) issue per entry, and proceed as clean as possible with the debug instructions given to you to further understand or to fix the problem. 
I’ll address your issue as soon as my available time allows, not necessarily in a FIFO way.

# ADX-ddsPIO Firmware

## Architecture

The overall program architecture can be seen in the following activity graph
![Alt Text](doc/ADX-ddsPIO_activity.png?raw=true "ADX-ddsPIO Architecture")


The main Transmit-Receive cycle loop can be seen in the following activity graph
![Alt Text](doc/ADX-ddsPIO_TRX_activity.png?raw=true "ADX-ddsPIO TRX cycle")


## ADX-ddsPIO Firmware (Version 1.0) 

This is the first version of the firmware, the main functions implemented
are:

* Operate in any of the HF bands (160m to 10m), attempts to marginal use on 6m will be made.
* 1W RF power, DC receiver (CD2003GP based) with SWR protection.
* Management of the ADX-ddsPIO board.
* Receive and transmit digital audio data over a USB connection.
* Is able to operate with weak mode signals.

The firmware is completed at this stage and able to sustain QSO as shown in the following picture
where a contact is been made between the board (dubbed as LU2EIC) and my 20m digital FT8 station
running as LU7DZ.
![Alt Text](doc/ADX-ddsPIO_MaidenQSO.png?raw=true "ADX-ddsPIO Maiden QSO")
Although the RF setup is close to trivial, just few meters distance, the activity has been held
"over the air" and shows the firmware is able to properly encode and decode signals both at the
audio and RF levels. My 20m station is composed of a QDX transceiver being run by a Raspberry Pi
controller running a modified version of WSJT-Z whilst the testbed for the ADX-ddsPIO is my
Mac running the MacOS version of WSJTX. For all practical purposes this is considered the 
"maiden QSO" of the board and all goals for this level can be called as completed.


## ADX-ddsPIO Firmware (Version 1.1)


Features

* Permanent settings (EEPROM emulation).
* CAT control (TS2000 emulation).


## ADX-ddsPIO Firmware (Version 1.2)

* Superhet ADX-S support.
* Implementation of a BFO (465 KHz) local oscillator.

## ADX-ddsPIO Firmware (Version 1.3)

* Quadrature based receiver (SDR)

## ADX-ddsPIO Firmware (Version 2.0)

* Si4732 based receiver

```
Work in progress
```


* Quadrature clock support.


## Build environment

The development environment used is Visual Studio code using the standard Raspberry Pi Pico C/C++ SDK.

Please follow the official documentation of both to install and configure the functionality.


## Flashing the firmware

The firmware could be flashed directly to the Raspberry Pi Pico (rp2040) processor on the
board following the conventional BOOTSEL method.

The file to flash is *./src/build/ADX-ddsPIO.uf2* .


# Hardware


I started the porting of the firmware assuming an ADX transceiver board, no more but no less features, but being powered
by a Raspberry Pico board; the porting of the software was a great deal of a learning curve not only for the rp2040
architecture being different from the ATMEGA382p and being more powerful, but also a substantially different build chain
which in some cases is implementing partially some features. For migration purposed the Raspberry Pi Pico board was used.

## Board support

At his point the Raspberry Pi Pico (standard) and the RP2040 Zero boards are *supported*.

The Raspberry Pi Pico W (wireless) is **not supported**.

## Circuit Design Si4732 based Receiver (version 2.x)

This circuit is largely based on the former RDX_rp2040 design
![Alt Text](doc/ADX-ddsPIO_V2.x_si4732-Schematic.png?raw=true "Transceiver Circuit (Version 2.x)")


The main differences with the verion 1.x:

* The receiver configuration has been changed to use the Si4732 receiver chipset.

### Modifications to adapt RDX board to ADX-ddsPIO

Modifications could be made to the RDX board to operate with firmware level 2.x or up
main advantage at this point is the availability of a PCB design which allows the usage
of SMD components easily.

The required mods for the RDX board to work with the ADX-ddsPIO firmware are:

* Cut the PCB wire PICO ADC0 (GPIO26) to R10/R11/C10.
* Run a wire  from R10/R11/C10 to PICO ADC2 (GPIO28).
* Run 2 pullup 10K resistors from PICO GPIO16 to +3.3V and GPIO17 to +3.3V.
* Do not use a Si5351 breakout board
* Run a wire from GPIO13 to the CLK0 pin of the Si5351 place.
* Run a 10K pullup resistor from Si4732 pin 3 (INTB) to +3.3V.
* If running with an external +12V cc supply remove jumper J7.
  
```
Work in progress
```

## Circuit Design Superhet Receiver (version 1.x)

This circuit is largely based on the ADX transceiver modification made by Hitoshi Kawaji (JE1RAV) and
marketed as a kit form by Adam Rong (BD6CR) at CRKits.com ([link](http://crkits.com/) ).
![Alt Text](doc/ADX-ddsPIO_V1.x_superhet-Schematic.png?raw=true "Transceiver Circuit (Version 1.x)")

The main differences with the verion 0.x:

* The receiver configuration has been changed to a superhet.

## Circuit Design Direct Conversion Receiver (version 1.x)

This circuit is largely based on the original ADX transceiver [link](https://github.com/WB2CBA/ADX) by Barb ([WB2CBA](https://www.qrz.com/db/WB2CBA)).
![Alt Text](doc/ADX-ddsPIO_V1.x_DC-Schematic.png?raw=true "Transceiver Circuit (Version 1.x) Direct Conversion")

The main differences with the original ADX circuit are:

* Si5351 module no longer present.
* Analog audio circuit has been removed as the audio information is received digitally thru USB.

## Pinout assignment

The pinout assignment for this version is shown in the following table:

![Alt Text](doc/ADX-ddsPIO_pinout.png?raw=true "Raspberry Pi Pico pinout assignment")

## Clock Architecture

The board supports and the firmware implements several clock models which can be configured
for different board setups.

* Single clock (DC Receiver).
* Dual clock   (DC Receiver).
* Dual clock + IF (Superheterodyne)
* Quadrature (SDR)

### Single clock

In this setup a single clock serves the entire board, output is present at pin GPIO13 and
deliver a single frequency depending on the band and the mode, typically is the lower
frequency of the mode sub-band at each band (ie. FT8 at 10m 28074 KHz) when the board is
in receiver mode and turns into the actual FSK modulation when in transmit mode.
This clock has a resolution of $x = \pm 1 Hz$ and it's generated using the VCO
technology derived from the work of Roman (R2BDT) described later in detail.

```
This option is the default when no clock option is activated at build time
```

### Dual clock 

In this setup the clock is produced simultaneously from GPIO13 and GPIO14, both signals
are identical, both in receiving or transmitting mode. It's intended to make the circuit
simpler. Actual operation of the board will require other signals such as RXSW and TXA
to properly operate to be controlled by the firmware. This clock also uses a derivative
of the VCO done by Roman (R2BDT) modified to replicate the output of the PIO on pins
GPIO13 and GPIO14 simultaneously.

```
This option is activated with the compilation directive #define DUALCLK 1
This option is deactivated with the compilation directive #define QUAD 1
```
### Dual clock + IF

Then the board operates with a superheterodyne receptor configuration the receiver process
requires two clocks, the main one to operate the down-converter mixer from the HF frequency
to an intermediate frequency and other as the intermediate frequency oscillator frequency.

The down-converter local oscillator (RFLO) can be either one of the clocks despicted above.

The intermediate frequency clock (RFIF) is a special clock, it's lower in frequency and 
therefore it's easier to implement on a processor with limited resources. Also it's a 
fixed frequency as it doesn't vary with the operation.

Unfortunately attempts to extend the VCO concept to simultaneously create a different
clock at a different frequency prooved to be beyond the limits of the processor; the VCO
uses for itself a whole core (*core1*) in **blocking mode** so no other task can be interleaved
without causing jitter, spurious outputs or frequency drift or all three together. At the same
time the other core (*core0*) it's actually devoted to provide all the other tasks of the
board such as USB Audio, USB CDC (serial), board management, signaling, timers, etc.

A different approach is then intended. the rp2040 processor has several specialized processors
called PIO (Programmable Input/Output) which are a limited memory, limited instruction set (RISC)
processors but completely independent from the main processor. Each processor executes their 
program as part of a *state machine* (SM) which dictates which instruction is executed
at each clock cycle.  They are programmed in a special 
Assembler language (*PioASM*) and are extremely useful to perform I/O without tying up the
processor or forcing the handling of interrupts on the main cores.  

To generate a clock for the intermediate frquency a PIO is assigned with that purpose
the state machine (SM) of the selected PIO runs at the frequency $f_{sm}$.
If the PIO program makes a toggle of the signal every cycle then the output frequency ($f_{out}$) would be:

$f_{out}=\frac{f_{sm}}{2}$

For an estimated 465 KHz frequency for the IF of the receiver then

$f_{sm}= 2*465 KHz = 930 KHz$

The clock used by the PIO is derived from the system clock ($clk_{sys}$) thru a fractional divisor,
as the board is being clocked at 270 MHz then

$clk_{div} = \frac{clk_{sys}}{f_{sm}} = 270e6 / 930e3 \approx{290.32266}$

However, the actual divisor used by the rp2040 has a resolution of 1/256, so in this case:


$f_{out} = \frac{f_{clk}}{2 \cdot div}$


where:


$div = INT + \frac{FRAC}{256}$

The ideal divisor would be 

$div_{ideal} = \frac{f_{clk}}{2 \cdot f_{obj}}$

$div_{ideal} = \frac{270\,000\,000}{2 \cdot 465\,000} = 290.3236$

As the hardware allows only for steps of 

$\Delta div = \frac{1}{256} \approx 0.00390625$

A split is made:

$INT = 290$

$FRAC = round(0.3236 \cdot 256) = 82$

A split needs to be made

$div_{real} = 290 + \frac{82}{256} = 290.3203$

And the real frequency obtained would be

$f_{real} = \frac{270\,000\,000}{2 \cdot 290.3203}$

$f_{real} \approx 465003.6529 \ \text{Hz}$

The quantization introduces then an error of 

$\Delta f = f_{real} - f_{obj}$

$\Delta f \approx -3.6529 \ \text{Hz}$

Being the relative error 

$\varepsilon = \frac{\Delta f}{f_{obj}}$
$\approx 7.85 \times 10^{-6}$

$\varepsilon \approx 8 \ \text{ppm}$

This error factor is accounting **only** for the fractional divisor error of the PIO. Other sources
of error needs to be considered such as
* Board crystal tolerances.
* PLL error.
* Thermal drift.
* Fractional divider internal jitter.

However, the level of error obtained is a good compromise between performance, cost and simplicity.

The waveform obtained at GPIO15 is as follows for a nominal $f_{BFO}=446400 \text{Hz}$

![Alt Text](doc/ADX-ddsPIO_BFO.png?raw=true "ADX-ddsPIO BFO")  

The outputs at GPIO13 and GPIO14 would be a clock at the operating frequency whilst the output
at GPIO15 (when option **#define SUPERHET 1** is set) would be a fixed clock of the established
frequency (normally in the 450 KHz range) as shown in the following picture.

![Alt Text](doc/ADX-ddsPIO_BFO2.png?raw=true "ADX-ddsPIO BFO")  

```
This option is activated with the compilation directive #define SUPERHET 1
This option is deactivated with the compilation directive #define QUAD 1
```

### Quadrature clock

If a Software Defined Radio (SDR) receiver is needed the board has to provide two specific signals
called by convention clock *I* and clock *Q* which are of the same frequency but whose phases are
90 $\textdegree$ appart or $\frac{\pi}{2}$ 

Again, trying to obtain such signal using the VCO technology it's quite difficult with the 
resources available, an approach to create them using again a PIO processor with that purpose
is made.

The result is a *digital quadrature frequency synth (I/Q)* and it's architecture and logic
follows.

 
#### Architecture

The architecture combines:

* Dynamic reconfiguration of the board clock (PLL_SYS).
* Generate quadrature signals using a 4-state PIO logic.
* Manipulate the fractional divisor (Q8.8) of the PIO state machine.
* Perform a frequency optimization of the error.

The system achieves typical errors in the 0-50 Hz range over the HF spectrum, being the receiver clock 
and being stable on that error this is quite compatible with digital modes such as FT8, the only effect
would be than the *"nominal"* frequency of the incoming signal with be shifted up or down by the error
(few Hz) in a stable way.


Main components are

* PLL_SYS (RP2040).
* Main boar clock (clk_sys).
* Configurable divider:
	* refdiv
	* fbdiv
	* postdiv1
	* postdiv2
* PIO (Programmable I/O)

The PIO executes a cyclic firmware with 4 states which generates the quadrature
signal over 2 output pins

	00 → 01 → 11 → 10 → repeat

Each instruction is executed in one clock cycle of the state machine, therefore
manipulating the clock of that PIO state machine using a fractional divider the
output frequency can be manipulated:

$D=d_{int}+\frac{d_{frac}}{256}$

Selecting two consecutive pins the signals *I* and *Q* can be extracted from each.

The synthesis math model follows

Each instruction of the PIO is executed in:
$T_{inst}=\frac{D}{f_{clk_sys}}$ 

In order to complete a full period all 4 instructions needs to be executed

$T_{out} = 4 \times T_{inst}  = \frac{4D}{f_{clk_sys}}$ 

Therefore the output frequency ($f_{out}$) will be:
$f_{out}=\frac{f_{clk_sys}}{4D}$

Using
$N=256D$

Results

$f_{out}=\frac{f_{clk_sys} \times 64}{N}$

Which is the master equation of the system operation

When a target frequency ($f_{req}$) is needed as a goal
and the system is running with a clock ($f_{clk}$) the
problem is to define a divider


$N^*=\frac {f_{clk} \times 64}{f_{req}}$ 

But the hardware is limited to 
$N \in Z,1 \le 65535$

Therefore not all values of $N$ are feasible but
$N=Round({N^*})$

And the true frequency resulting would be

$f_{out} (N)=\frac{f_clk \times 64}{N}$

and the error 
$ε=f_{out} (N)-f_{req}$

When computed over the HF range the error could become quite large, from
several Hz in the lower bands to close to 30 KHz in the higher bands, this
is not acceptable.

To minimize the error two factors needs to be defined, the divisor but also the system clock
in a way that an exploration of the  discrete space of solutions 

* $refdiv\in[1,16]$
* $fbdiv\in[16,320]$
* $postdiv1\in[1,7]$
* $postdiv2\in[1,postdiv]$

With:
* $f_{ref}=\frac{f_{xosc}}{refdiv}$
* $f_{vco}=\frac{f_ref}{fbdiv}$
* $f_{clk}=\frac{f_{vco}}{postdiv1 \times postdiv2}$

Subject to the following constraints:

* $400 \text{MHz} \le f_{vco} \le 1600 \text{MHz}$
* $f_{clk} \le f_{max_sys}$

 
A minimization optimization problem can then be solved with the 
following border conditions

For each valid PLL configuration a computation is made
$N^*=\frac {f_{clk} \times 64}{f_{req}}$ 

All candidates are evaluated
$[N-1,N,N+1]$

For each 
$f_{out} (N)=\frac {f_{clk} \times 64}{N}$

Searching for the minimum of 
$\left \lvert \epsilon \right \rvert = \left \lvert {f_{out}-f_{req}} \right \rvert$

Selecting as a result the value with the minimum absolute 
error 
$[f_{clk},N]$

This is a discrete search with a double optimization:
* PLL quantization.
* Divider (Q8.8) quantization

The theoretical error limit is  
$\left \lvert \Delta N \right \rvert \le 0.5$

taking derivatives 
$\frac {df}{dN}=- \frac{f_{clk} \times 64}{N^2}$

Being the approximate máximum error value:
$\left \lvert \epsilon_{max} \right \rvert \approx \frac{f_{clk} \times 64}{2 \times N^2}$

Since
$N \approx \frac{f_{clk} \times 64}{f_{req}}$ 

results
$\left \lvert {\epsilon}_{max} \right \rvert \approx \frac{f_{req}^2}{2 \times f_{clk} \times 64}$

As a consequence 

* Error is reduced when $clk_{sys}$ increases.
* Error grow quadratically with the $f_{req}$
* The divisor cuantization controls the residual error.

A typical expected result would be
|Band|Freq|clk_sys|Error|
| --- | --- | --- | --- |
| 80 m |  3 573 000 | 167 428 571 |   0 Hz |
| 40 m |  7 074 000 | 233 000 000 |  ±3 Hz |
| 20 m | 14 074 000 | 268 285 602 |  ±2 Hz |
| 15 m | 21 074 000 | 178 800 000 | ±30 Hz |
| 10 m | 28 074 000 | 261 000 000 | ±50 Hz |


The output of this clock is implemented to be obtained at pins GPIO14 (I) and GPIO15 (Q).

The resulting waveform is similar to the following example (for 3.5 MHz)
![Alt Text](doc/ADX-ddsPIO_quad.png?raw=true "ADX-ddsPIO Quadrature frequency synthetizer")  



```
This clock is activated by the directive #define QUAD 1
```

## Test resources

The following test resources has been built


### ADX-ddsPIO mockup

In order to develop the first stages of the firmware a reduced mockup has been used
![Alt Text](doc/ADX-ddsPIO-mockup.jpeg?raw=true "ADX-ddsPIO-mockup")
This reduced scope board contains just the processor and the push buttons. A small
RF probe is feed into a wire for local "on the air" tests.

### ADX-ddsPIO prototype

To continue developing the board a first cut prototype board is built as shown in the
figure
![Alt Text](doc/RP2040Z_proto.jpeg?raw=true "ADX-ddsPIO Prototype")
The building technique is to make connection with hand made wiring, the Raspberry Pi Pico
board has been replaced with a rp2040 Zero board which is essentially the same but with
a much smaller footprint. The main disadvantage of this board is that not all the GPIO
pins are expossed, but still as the ADX-ddsPIO board uses just few GPIO lines it's quite
adequate.
```
Just the transmitting part sans finals has been made as yet as the measurements associated
with the firmware development requires large periods with the board in TX mode and there
is no point on running finals for that.
``` 
### ADX-ddsPIO extended prototype

To continue the development of the firmware adding the reception sub-systems an extended
prototype including a receiver has been built.
![Alt Text](doc/ADX-ddsPIO_prototype_rp2040Z.jpeg?raw=true "ADX-ddsPIO-prototype")
This prototype contains the processor, the RF driver and the simple *CD2003GP* based
DC receiver. The processor board used is the breakboard rp2040Z which provides the
same funcionality than the Raspberry Pi Pico with a reduced availability of GPIO and
other pins but with a much smaller board footprint.
A small RF probe is feed into a wire for local "on the air tests".


### Modified RDX board

The RDX board can receive modifications which allows it to be used as a debug
platform for the Si4732 receiver implementation.

![Alt Text](doc/ADX-ddsPIO_RDX.jpg?raw=true "ADX-ddsPIO RDX")

Mods follows:

```
Work in progress
```


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

## testUSB2 utility

This utility is a strip down version of QP-7C_rp2040_cat with both the USB audio and
serial data enabled.
The USB audio stream is read and the frequency computed averaging the results every
10 mSecs, the result is displayed thru the serial monitor (while the USB audio 
continue working) showing both the frequency counting algorithm and the operation 
of the USB serial monitor.

```
The test configuration needs the followigh steps:

* Compile and load the firmware.
* Open the serial monitor pointing to the */dev/tty.usbmodem00000xx - TinyUSB* device
* Start the supervision of the serial monitor channel.
* Start WSJTX and configure in the AUDIO tab
	* Input: ADX-ddsPIO
	* Output: ADX-ddsPIO
* Start the TX Tone, the serial monitor should mark the tone frequency as set in WSJTX.
* Start the FT8 TX, the serial monitor will show the different tones sent.


The counting algorithm has +/- 1  Hz accuracy, this can be improved later choosing
the averaging time, but should be enough to properly decode the frequency and
change it accordingly.
```

## testUSB3

This is a hack to integrate the Audio USB version (testUSB2) with the DCO version (testADX)
into a single firmware, crude at best, but mainly intended to explore and resolver the
(really many) hardware conflicts and library incompatibilities.

For most part it's a functional transmitter as shown in the following picture where a local
copy of WJST-X running on a Mac (test station) is evaluated in my main WSJT-X station, it can
be seen the transmission and the decoding of it.

![Alt Text](doc/testUSB3.png?raw=true "testUSB3 Test")

At the bottom of the image the WSJT-X configuration dialog can be seen where the input and
output devices is actually the ADX-ddsPIO board recognized as such.

## Quad

This is the prototype of the digital quadrature frequency synth (I/Q) clock

It's just used to evaluate the overall performance and yield unmodulated clock signals thru
ports GPIO14 (I) and GPIO15 (Q)

## si4732_rp2040

This is the prototype and test bench for a rp2040 based si4732 library

## Others

There are other implementations which aren't functional by itself but only part of 
prototype and experimentation efforts on specific aspects.

* testUSB Algorithm for frequency measurement over the USB audio channcel.
* QP-7C_rp2040_cat Full implementation of USB Serial and Audio.

```
Unless otherwise specified do not expect to flash these projects on a raspberry pi pico 
and obtain any functionality
```

## DCO for Raspberry Pi Pico 

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

