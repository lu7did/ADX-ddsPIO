#define AUDIOSAMPLING    48000            // USB Audio sampling frequency (fixed)
#define PLL_SYS_MHZ        270            // RP2040 System Clock (MHz)  
#define PLL_SYS_MHZ_PLUS   290            // RP2040 System Clock (MHz) --OVERCLOCK--
#define GEN_FRQ_HZ    14074000L           // Generator Frequency (in Hz)
#define FT8_BASE_HZ       1000L           // FT8 base frequency (in Hz) <Not used>
#define FREQ_BFO        446400L           // BFO Frequency 

#define SLOT                  3
#define NBANDS                7
#define NMODES                4

#ifdef RDX
#define pin_ADC              26U          //pin for ADC (A0)
#else
#define pin_ADC              28U          //pin for ADC (A2)
#endif //RDX

#define pin_SW                3U          //pin for freq change switch (D10,input)

#define  CLK0                13           //RF output (transmitter)
#define  CLK1                12           //RF output (receiver)

#ifdef SUPERHET
#define CLK2                 14           //RF out Receiver IF (465 KHz)

#endif //SUPERHET

#ifdef QUAD
#define RFI                 14
#define RFQ                 15
#endif //QUAD

//#define I2C_PORT           i2c0
//#define SDA                  26           //I2C SDA (Data) bus
//#define SCL                  27           //I2C SCL (Clock) bus

#define RXSW                  2  //RXSW Switch (RX/TX control)
#define TXA                   9  //OE signal to driver

/*---
   LED
*/

#define TX                    3  //TX LED
#define FT8                   4  //FT8 LED
#define FT4                   5  //FT4 LED
#define JS8                   6  //JS8 LED
#define WSPR                  7  //WSPR LED

/*---
   Switches
*/    
#define TXSW                  8  //RX-TX Switch
#define UP                   10  //UP Switch
#define DOWN                 11  //DOWN Switch
#define BEACON               12  //BEACON Jumper


/*----
   Definitions for the Si4732 support
*/
#ifdef SI4732

#define I2C_PORT i2c0
#define SDA_PIN              16  //There must be an alternative as the rp2040Z does not exposse this
#define SCL_PIN              17  //nor this but alternatives in clear view are 26 & 27 which are also used by
#define RST_PIN               1  //pin 9 is available in rp2040Z and free in RDX but 1 is in conflict


#define SI4732_DEFAULT_REGION  "ar"
#define SI4732_DEFAULT_MODE   "ssb"
#define SI4732_DEFAULT_BAND   "20m"
#define SI4732_DEFAULT_VOLUME    50
#define SI4732_DEFAULT_MUTE       0


#define SI4732_LOAD_PATCH         1
int slot[4]   = {40,30,20,10};
int Band_slot = SLOT;                  //This is the default band Band1=1,Band2=2,Band3=3,Band4=4
int Band      =   20;                  //This is the default band
int mode      =    4;                  //Default mode is FT8

long unsigned int Bands[NBANDS][NMODES] = {
                                          { 3568600, 3578000, 3575000, 3573000},
                                          { 7038600, 7078000, 7047500, 7074000},
                                          {10138700,10130000,10140000,10136000},
                                          {14095600,14078000,14080000,14074000},
                                          {18104600,18104000,18104000,18100000},
                                          {21094600,21078000,21140000,21074000},
                                          {28124600,28078000,28180000,28074000}};


blinkslow
blinkmed
blinkfast

boardled
