#INCLUDE <16f1826.h> 

#FUSES INTRC_IO      // Internal RC clock (OSC1 and OSC2 pins are normal I/O)
#FUSES NOWDT         // Watch Dog Timer disabled
#FUSES PUT           // Power Up Timer enabled
#FUSES NOMCLR        // Master Clear pin is used for I/O
#FUSES PROTECT       // Code protected from reads
#FUSES CPD           // Data EEPROM code protected
#FUSES BROWNOUT      // Brownout Reset enabled
#FUSES BORV25        // Brownout Reset at 2.5V
#FUSES NOCLKOUT      // Disable clock output on OSC2
#FUSES NOIESO        // Internal External Switch Over Mode disabled
#FUSES NOFCMEN       // Fail-safe clock monitor disabled
#FUSES WRT           // Program memory write protected                                              
#FUSES NOLVP         // Low Voltage Programming disabled
#FUSES NODEBUG       // No debug mode for ICD

#USE   DELAY(internal = 32MHz)
//#USE   RS232(stream=RS232, baud=38400, XMIT=PIN_B5, RCV=PIN_B2, parity=N, bits=8, stop=1)

#DEFINE DRV_STEP     PIN_A0                  // Step output pin to driver (Rising edge causes the indexer to move one step)
#DEFINE DRV_RESET    PIN_A1                  // Reset output pin to driver (Active-high reset input initializes all internal logic and disables the Hbridge outputs. Internal pulldown.)
#DEFINE USER_FAULT   PIN_A2                  // Fault output pin to user
#DEFINE USER_DIR     PIN_A3                  // Direction input pin from user
#DEFINE DRV_DIR      PIN_A4                  // Direction output pin to driver (Logic level, sets the direction of stepping)

#DEFINE SPI_SDO      PIN_A6                  // SPI data output pin
#DEFINE SPI_CS       PIN_A7                  // SPI chip select pin

#DEFINE USER_STEP    PIN_B0                  // Step input pin from user
#DEFINE SPI_SDI      PIN_B1                  // SPI data input pin

#DEFINE USER_ENABLE  PIN_B3                  // Enable input pin from user
#DEFINE SPI_SCK      PIN_B4                  // SPI clock output pin

#DEFINE DRV_STALL    PIN_B6                  // Stall input pin from driver (Internal stall detect mode: logic low when motor stall detected. Pull up mevcut)
#DEFINE DRV_FAULT    PIN_B7                  // Fault input pin from driver (Logic low when in fault condition. Pull up mevcut)

// SPI Registers
#WORD SPI_SSP1CON1   = 0x215                 // Synchronous serial port control register
#WORD SPI_SSP1STAT   = 0x214                 // Synchronous serial port status register
#WORD SPI_SSP1BUF    = 0x211                 // Synchronous serial port buffer register
#WORD SPI_PIR1       = 0x011                 // Peripheral interrupt request register
// Pin function control registers
#WORD PIN_APFCON0    = 0x11D                 // Alternate pin function control register 0
#WORD PIN_APFCON1    = 0x11E                 // Alternate pin function control register 1

// Bits of SSP1CON1 register
#BIT SPI_WRITE_FLAG     = SPI_SSP1CON1.7     // Synchronous serial port write collision detect bit
#BIT SPI_ENABLE         = SPI_SSP1CON1.5     // Synchronous serial port enable bit
#BIT SPI_CLOCK_POLARITY = SPI_SSP1CON1.4     // Synchronous serial port clock polarity select bit
#BIT SPI_MODE_3         = SPI_SSP1CON1.3     // Synchronous serial port mode select bits
#BIT SPI_MODE_2         = SPI_SSP1CON1.2   
#BIT SPI_MODE_1         = SPI_SSP1CON1.1   
#BIT SPI_MODE_0         = SPI_SSP1CON1.0  

// Bits of SPI_SSP1STAT register
#BIT SPI_INPUT_SAMPLE   = SPI_SSP1STAT.7     // Synchronous serial port data input sample bit
#BIT SPI_CLOCK_EDGE     = SPI_SSP1STAT.6     // Synchronous serial port clock edge select bit
#BIT SPI_BUFFER_STATUS  = SPI_SSP1STAT.0     // Synchronous serial port buffer full status bit

// Bits of SPI_PIR1 register
#BIT SPI_FLAG           = SPI_PIR1.3         // Synchronous serial port interrupt flag bit

// Bits of PIN_APFCON0 register
#BIT PIN_RX_SELECT      = PIN_APFCON0.7      // RX pin selection bit
#BIT PIN_SDO_SELECT     = PIN_APFCON0.6      // SDO pin selection bit 
#BIT PIN_SS_SELECT      = PIN_APFCON0.5      // SS pin selection bit 

// Bits of PIN_APFCON1 register
#BIT PIN_TX_SELECT      = PIN_APFCON1.0      // TX pin selection bit

int   const acc_lim          = 116;                                                 // Number of steps before we hit max speed. acc=10000 dec=10000 
unsigned int const periods[acc_lim]={2449,1015,779,656,578,523,481,447,420,397,378,361,346,333,322,311,302,293,285,277,271,264,258,253,247,243,238,234,229,226,222,218,215,212,209,206,203,200,197,195,192,190,188,186,184,182,180,
178,176,174,172,171,169,167,166,164,163,162,160,159,157,156,155,154,152,151,150,149,148,147,146,145,144,143,142,141,140,139,138,137,136,135,134,133,132,131,130,129,128,127,126,125,124,123,122,121,120,119,118,117,116,115,
114,113,112,111,110,109,108,107,106,105,104,103,102,101};

int16          i                    =  0;
int16          delay                =  50;
int16          distance_step        =  20000;
   

// Sets variables to default values 
void set_variables()
{
   output_low(SPI_CS);     // Chip select is active high so keep it low to prevent out-of-sync transaction
   output_low(USER_FAULT); // Clear fault status 
   output_low(DRV_RESET);  // Reset is active high so keep reset pin low to activate driver 
   output_low(DRV_DIR);    // Set default direction
   output_low(DRV_STEP);   // Keep step output low until a step command is received from the user
}

void pulser(){
   
         for(i=0; i<acc_lim; i++)
         {
            
            delay_us(periods[i]/2);  //Rampa kalkýþ
            delay_us(delay);
            delay_us(periods[i]/2);
            delay_us(delay);
         }

         for(i=0; i<distance_step; i++)
         {
            output_high(USER_STEP);
            delay_us(delay);             //Stepler
            output_low(USER_STEP);
            delay_us(delay);
         }
         
         for(i=acc_lim; i>0; i--)
         {
            
            output_high(USER_STEP);    //Rampa Duruþ
            delay_us(periods[i-1]/2);;
            output_low(USER_STEP);
            delay_us(periods[i-1]/2);
         }
   
}

void main()
{
   // Set I/O states of the ports
   //           76543210                 
   set_tris_a(0b00100100);       
   set_tris_b(0b11000110);
   
   while(true){      
   
      output_high(USER_ENABLE);
      delay_ms(100);
      output_high(USER_DIR);
      delay_ms(10);
      
      pulser();
      
      output_low(USER_ENABLE);
      delay_ms(1000);
      output_low(USER_DIR);
      delay_ms(10);
   
      pulser();
      delay_ms(1000);
   
   }

}
