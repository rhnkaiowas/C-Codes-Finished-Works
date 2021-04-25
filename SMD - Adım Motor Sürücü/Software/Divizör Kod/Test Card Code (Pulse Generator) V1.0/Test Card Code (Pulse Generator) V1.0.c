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

#DEFINE DRV_RESET    PIN_A0                  // Reset output pin to driver (Active-high reset input initializes all internal logic and disables the Hbridge outputs. Internal pulldown.)
#DEFINE DRV_SLEEP    PIN_A1                  // Sleep output pin to driver (Logic high to enable device)
#DEFINE USER_FAULT   PIN_A2                  // Fault output pin to user
#DEFINE USER_DIR     PIN_A3                  // Direction input pin from user
#DEFINE DRV_DIR      PIN_A4                  // Direction output pin to driver (Logic level, sets the direction of stepping)
#DEFINE DRV_STEP     PIN_A7                  // Step output pin to driver (Rising edge causes the indexer to move one step)
#DEFINE USER_STEP    PIN_B0                  // Step input pin from user
#DEFINE USER_ENABLE  PIN_B3                  // Enable output pin from user
#DEFINE DRV_STALL    PIN_B6                  // Stall input pin from driver (Internal stall detect mode: logic low when motor stall detected. Pull up mevcut)
#DEFINE DRV_FAULT    PIN_B7                  // Fault input pin from driver (Logic low when in fault condition. Pull up mevcut)

#DEFINE SPI_SDI      PIN_B1                  // SPI data input pin
#DEFINE SPI_SDO      PIN_A6                  // SPI data output pin
#DEFINE SPI_SCK      PIN_B4                  // SPI clock output pin
#DEFINE SPI_CS       PIN_B2                  // SPI chip select pin

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

// Sets alternative pin functions
void set_pins()
{
   // Set RB2 as RX pin
   //PIN_RX_SELECT = 1;
   // Set RA6 as SDO pin
   PIN_SDO_SELECT = 1;
   // Set RA5 as SS pin
   PIN_SS_SELECT = 1;
   // Set RB5 as TX pin
   //PIN_TX_SELECT = 1;
}
// Sets SPI parameters
void set_SPI()
{
   // Disable SPI to set registers
   SPI_ENABLE = 0;
   // Set idle state of the clock to low 
   SPI_CLOCK_POLARITY = 0;
   // Set SPI mode to SPI 
   SPI_MODE_3 = 0; 
   SPI_MODE_2 = 0;
   SPI_MODE_1 = 1;
   SPI_MODE_0 = 0;
   // Input data sampled at the middle of data output time
   SPI_INPUT_SAMPLE = 0;
   // Transmit occurs on transition from active to idle clock state
   SPI_CLOCK_EDGE = 1;

   // Enable SPI
   SPI_ENABLE = 1;
}
// Sets variables to default values 
void set_variables()
{
   output_low(SPI_CS);     // Chip select is active high so keep it low to prevent out-of-sync transaction
   output_low(USER_FAULT); // Clear fault status 
   output_low(DRV_SLEEP);  // Put driver into sleep mode until user enables the motor
   output_low(DRV_RESET);  // Reset is active high so keep reset pin low to activate driver 
   output_low(DRV_DIR);    // Set default direction
   output_low(DRV_STEP);   // Keep step output low until a step command is received from the user
}
// Sets driver parameters to default values
void set_driver()
{
   // Driver control register sets these parameters
   // data operation type                       (bit 15)    1=read, 
   //                                                       0=write
   // register adress                           (bit 14-12) 000=CTRL reg
   // DTIME (dead time between MOSFET switching)(bit 11-10) 00=400ns, 
   //                                                       01=450ns, 
   //                                                       10=650ns, 
   //                                                       11=850ns
   // ISGain (current amplifier gain)           (bit 9-8)   00=Gain of 5, 
   //                                                       01=Gain of 10, 
   //                                                       10=Gain of 20, 
   //                                                       11=Gain of 40
   // EXSTALL (source of stall detection)       (bit 7)     0=Internal
   //                                                       1=External
   // MODE (microstepping mode)                 (bit 6-3)   0000=Full-step, 71% current
   //                                                       0001=Half step
   //                                                       0010=1/4 step
   //                                                       0011=1/8 step
   //                                                       0100=1/16 step
   //                                                       0101=1/32 step
   //                                                       0110=1/64 step
   //                                                       0111=1/128 step
   //                                                       1000=1/256 step
   // RSTEP (single step output)                (bit 2)     0=No action
   //                                                       1=Indexer will advance one step; automatically cleared after write
   // direction                                 (bit 1)     0=Direction set by DIR pin
   //                                                       1=Direction set by inverse of DIR pin
   // enable                                    (bit 0)     0=Disable motor
   //                                                       1=Enable motor
   //  
   //                 76543210
   
   output_high(SPI_CS);
   int Ctrl_1     = 0b00000011;
   int Ctrl_0     = 0b01000001;
   SPI_SSP1BUF = Ctrl_1;
   while (!SPI_FLAG);
   SPI_FLAG = 0;
   SPI_SSP1BUF = Ctrl_0;
   while (!SPI_FLAG);
   SPI_FLAG = 0;
   output_low(SPI_CS);
   delay_ms(10);

   output_high(SPI_CS);
   int Torque_1   = 0b00010000;
   int Torque_0   = 0b10000100;
   SPI_SSP1BUF = Torque_1;
   while (!SPI_FLAG);
   SPI_FLAG = 0;
   SPI_SSP1BUF = Torque_0;
   while (!SPI_FLAG);
   SPI_FLAG = 0;
   output_low(SPI_CS);
   delay_ms(10);
   
   output_high(SPI_CS);
   int Off_1      = 0b00100000;
   int Off_0      = 0b01111000;
   SPI_SSP1BUF = Off_1;
   while (!SPI_FLAG);
   SPI_FLAG = 0;
   SPI_SSP1BUF = Off_0;
   while (!SPI_FLAG);
   SPI_FLAG = 0;
   output_low(SPI_CS);
   delay_ms(10);
   
   output_high(SPI_CS);
   int Blank_1    = 0b00110001;
   int Blank_0    = 0b11111111;
   SPI_SSP1BUF = Blank_1;
   while (!SPI_FLAG);
   SPI_FLAG = 0;
   SPI_SSP1BUF = Blank_0;
   while (!SPI_FLAG);
   SPI_FLAG = 0;
   output_low(SPI_CS);
   delay_ms(10);
   
   output_high(SPI_CS);
   int Decay_1    = 0b01000101;
   int Decay_0    = 0b01001010;
   SPI_SSP1BUF = Decay_1;
   while (!SPI_FLAG);
   SPI_FLAG = 0;
   SPI_SSP1BUF = Decay_0;
   while (!SPI_FLAG);
   SPI_FLAG = 0;
   output_low(SPI_CS);
   delay_ms(10);
   
   output_high(SPI_CS);
   int Stall_1    = 0b01011001;
   int Stall_0    = 0b00010100;
   SPI_SSP1BUF = Stall_1;
   while (!SPI_FLAG);
   SPI_FLAG = 0;
   SPI_SSP1BUF = Stall_0;
   while (!SPI_FLAG);
   SPI_FLAG = 0;
   output_low(SPI_CS);
   delay_ms(10);
   
   output_high(SPI_CS);
   int Drive_1    = 0b01101111;
   int Drive_0    = 0b10100000;
   SPI_SSP1BUF = Drive_1;
   while (!SPI_FLAG);
   SPI_FLAG = 0;
   SPI_SSP1BUF = Drive_0;
   while (!SPI_FLAG);
   SPI_FLAG = 0;
   output_low(SPI_CS);
   delay_ms(10);
   
   output_high(SPI_CS);
   int Status_1   = 0b01110000;
   int Status_0   = 0b00000000;
   SPI_SSP1BUF = Status_1;
   while (!SPI_FLAG);
   SPI_FLAG = 0;
   SPI_SSP1BUF = Status_0;
   while (!SPI_FLAG);
   SPI_FLAG = 0;
   output_low(SPI_CS);
   delay_ms(10);
   
   output_high(DRV_SLEEP);
   delay_ms(100);
   
   output_high(SPI_CS);
   Ctrl_1     = 0b00000001;
   Ctrl_0     = 0b01000001;
   SPI_SSP1BUF = Ctrl_1;
   while (!SPI_FLAG);
   SPI_FLAG = 0;
   SPI_SSP1BUF = Ctrl_0;
   while (!SPI_FLAG);
   SPI_FLAG = 0;
   output_low(SPI_CS);
   delay_ms(10);
}

void motion_cycle()
{
   while(true)
   {
      /*
      output_high(DRV_STEP);
      delay_us(120);
      output_low(DRV_STEP);
      delay_us(120);
      
      if(USER_STEP)
         output_high(DRV_STEP);
      else
         output_low(DRV_STEP);
         
      if(USER_DIR)
         output_high(DRV_DIR);
      else
         output_low(DRV_DIR);
         */
   }
}

/*
void wait()
{
output_low(DRV_SLEEP);
   while(true){
      check_states();
      output_high(USER_FAULT);  //LED YAK
      if(USER_ENABLE==1&&fault==0&&stall==0){
            motion_cycle();
      }
   }
}

void check_states()
{
   
   check_stall(){
   //adresi oku
   }
   
   check_fault(){
   //adresi oku
   }
}
*/

void main()
{
   // Set I/O states of the ports
   //           76543210                 
   set_tris_a(0b00000000);       
   set_tris_b(0b11000010);
   
   // Set alternative pin functions
   set_pins();
   // Set SPI parameters
   set_SPI();
   
   // Give user a hint of system start by turning on fault led
   output_high(USER_FAULT);
   delay_ms(500);
   
   unsigned int16 i;
   
   while(true)
   {
      output_low(USER_DIR);
      output_low(USER_ENABLE);
      
      delay_ms(1000);
      for(i=0; i<51200; ++i)
      {
         output_high(USER_STEP);   //yüksek tork bölümü (low enable)
         delay_us(100);
         output_low(USER_STEP);
         delay_us(100);
      }
      output_high(USER_ENABLE);
      delay_ms(1000);
      
      output_high(USER_DIR);
      output_high(USER_ENABLE);
      delay_ms(1000);
      for(i=0; i<51200; ++i)
      {
         output_high(USER_STEP);    //düsük tork bölümü (high enable)
         delay_us(100);
         output_low(USER_STEP);
         delay_us(100);
      }
      output_high(USER_ENABLE);
      delay_ms(1000);
   }
}
