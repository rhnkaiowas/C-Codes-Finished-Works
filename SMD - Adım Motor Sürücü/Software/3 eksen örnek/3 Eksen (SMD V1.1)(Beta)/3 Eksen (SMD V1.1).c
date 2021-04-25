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

#USE   DELAY(internal = 32MHz)
#USE   RS232(baud=38400,XMIT=PIN_B5,RCV=PIN_B2,parity=N,bits=8,stop=1)

#DEFINE DRV_STEP     PIN_A0                  // Reset output pin to driver (Active-high reset input initializes all internal logic and disables the Hbridge outputs. Internal pulldown.)
#DEFINE DRV_RESET    PIN_A1                  // Sleep output pin to driver (Logic high to enable device)
#DEFINE LIMIT_SWITCH PIN_A2                  // Limit Switches input pin(normal 0V, while trigged 5V)
#DEFINE USER_DIR     PIN_A3                  // Direction input pin from user
#DEFINE DRV_DIR      PIN_A4                  // Direction output pin to driver (Logic level, sets the direction of stepping)

#DEFINE SPI_SDO      PIN_A6                  // SPI data output pin
#DEFINE SPI_CS       PIN_A7                  // Step output pin to driver (Rising edge causes the indexer to move one step)

#DEFINE USER_STEP    PIN_B0                  // Step input pin from user
#DEFINE SPI_SDI      PIN_B1                  // SPI data input pin

#DEFINE USER_ENABLE  PIN_B3                  // Enable input pin from user
#DEFINE SPI_SCK      PIN_B4                  // SPI clock output pin

#DEFINE DRV_STALL    PIN_B6                  // Stall input pin from driver (Internal stall detect mode: logic low when motor stall detected. Pull up mevcut)
#DEFINE DRV_FAULT    PIN_B7                  // Fault input pin from driver (Logic low when in fault condition. Pull up mevcut)



// Interrupt On Change Registers
#WORD IOC_INTCON     =0x00B
#WORD IOC_IOCBP      =0x394
#WORD IOC_IOCBN      =0x395
#WORD IOC_IOCBF      =0x396
#WORD ANSELB         =0x18d

//Bits of IOC_INTCON register
#BIT IOC_GIE           = IOC_INTCON.7        //General interrupt enable bit
#BIT IOC_PEIE          = IOC_INTCON.6        //Peripheral interrupt enable bit
#BIT IOC_IOCIE         = IOC_INTCON.3        //Interrupt on change enable bit

//Bits of IOC_IOCBP register
#BIT IOC_PINB3_RISING            = IOC_IOCBP.3        //Pin B3 interrupt on change enable bit(Rising Edge)

//Bits of IOC_IOCBN register
#BIT IOC_PINB3_FALLING           = IOC_IOCBN.3        //Pin B3 interrupt on change enable bit(Falling Edge)

//Bits of IOC_IOCBF register
#BIT IOC_PINB3_FLAG              = IOC_IOCBF.3        //Pin B3 interrupt on change interrupt flag bit(Both Rising and Falling Edges)
//Bits of ANSELB register

#BIT ANSELB_PINB3                = ANSELB.3           //Selection of Pin B3 as a Digial I/O (0) or an Analog input (1) selection bit

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

int1  dir         = 0;
int1  step        = 0;
int1 limit_old    = 0;
int8 up_lim_add   = 0;
int8 down_lim_add = 1;

// Sets alternative pin functions
void set_pins()
{
   // Set RB2 as RX pin
   PIN_RX_SELECT = 1;
   // Set RA6 as SDO pin
   PIN_SDO_SELECT = 1;
   // Set RA5 as SS pin
   PIN_SS_SELECT = 1;
   // Set RB5 as TX pin
   PIN_TX_SELECT = 1;
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
                    //FEDCBA98
   int Ctrl_1     = 0b00000001;
                    //76543210
   int Ctrl_0     = 0b00011001;
   SPI_SSP1BUF = Ctrl_1;
   while (!SPI_FLAG);
   SPI_FLAG = 0;
   SPI_SSP1BUF = Ctrl_0;
   while (!SPI_FLAG);
   SPI_FLAG = 0;
   output_low(SPI_CS);
   delay_ms(10);

   output_high(SPI_CS);
                    //FEDCBA98
   int Torque_1   = 0b00010000;
                    //76543210
   int Torque_0   = 0b01111000;
   SPI_SSP1BUF = Torque_1;
   while (!SPI_FLAG);
   SPI_FLAG = 0;
   SPI_SSP1BUF = Torque_0;
   while (!SPI_FLAG);
   SPI_FLAG = 0;
   output_low(SPI_CS);
   delay_ms(10);
   
   output_high(SPI_CS);
                    //FEDCBA98
   int Off_1      = 0b00100000;
                    //76543210
   int Off_0      = 0b01111001;
   SPI_SSP1BUF = Off_1;
   while (!SPI_FLAG);
   SPI_FLAG = 0;
   SPI_SSP1BUF = Off_0;
   while (!SPI_FLAG);
   SPI_FLAG = 0;
   output_low(SPI_CS);
   delay_ms(10);
   
   output_high(SPI_CS);
                    //FEDCBA98
   int Blank_1    = 0b00110001;
                    //76543210
   int Blank_0    = 0b10010110;
   SPI_SSP1BUF = Blank_1;
   while (!SPI_FLAG);
   SPI_FLAG = 0;
   SPI_SSP1BUF = Blank_0;
   while (!SPI_FLAG);
   SPI_FLAG = 0;
   output_low(SPI_CS);
   delay_ms(10);
   
   output_high(SPI_CS);
                    //FEDCBA98
   int Decay_1    = 0b01000101;
                    //76543210
   int Decay_0    = 0b00011100;
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
                    //FEDCBA98
   int Drive_1    = 0b01101010;
                    //76543210
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

}

void check_limit_state()
{
   int1 limit_new = input(LIMIT_SWITCH);
   
   if(limit_old == 0 && limit_new == 1)
   {
      if(input(USER_DIR) == true)
      {
         write_eeprom(up_lim_add, 1);
         write_eeprom(down_lim_add, 0);
      }
      if(input(USER_DIR) == false)
      {
         write_eeprom(up_lim_add, 0);
         write_eeprom(down_lim_add, 1);
      }
   }
   else if(limit_old == 1 && limit_new == 0)
   {
      write_eeprom(up_lim_add, 0);
      write_eeprom(down_lim_add, 0);
   }
   
   limit_old = limit_new;
}
void set_enable(int1 state)
{
   int Ctrl_1 = 0b00000001;
   int Ctrl_0;
   
   if(state == true)
      Ctrl_0     = 0b00011001;
   else
      Ctrl_0     = 0b00011000;
   
   output_high(SPI_CS);
   SPI_SSP1BUF = Ctrl_1;
   while (!SPI_FLAG);
   SPI_FLAG = 0;
   SPI_SSP1BUF = Ctrl_0;
   while (!SPI_FLAG);
   SPI_FLAG = 0;
   output_low(SPI_CS);
   delay_ms(10);
}
void set_direction()
{
   if(dir != input(USER_DIR))
   {
      output_bit(DRV_DIR, input(USER_DIR));
      dir = input(USER_DIR);
   }
}
void motion_cycle()
{
   step  = input(USER_STEP);
   dir   = input(USER_DIR);
   while(true)
   { 
      if(input(USER_ENABLE) == true)
      {
         set_enable(true);
         
         while(input(USER_ENABLE))
         {
            check_limit_state();
            set_direction ();
            
            if(read_eeprom(up_lim_add) == 0 && dir == 1)
            {
               if(step != input(USER_STEP))
               {
                  output_bit(DRV_STEP, input(USER_STEP));
                  step = input(USER_STEP);
               }
            }
            else if(read_eeprom (down_lim_add) == 0 && dir == 0)
            {
               if(step != input(USER_STEP))
               {
                  output_bit(DRV_STEP, input(USER_STEP));
                  step = input(USER_STEP);
               }
            
            }
         }
      }
      else if (!input(USER_ENABLE) || input(LIMIT_SWITCH) )
      {
         set_enable(false);
      }
   }
}

void main()
{
// Set I/O states of the ports
   //           76543210                 
   set_tris_a(0b00101100);       
   set_tris_b(0b11001111);

   // Set alternative pin functions
   set_pins();
   // Set SPI parameters
   set_SPI();
   //set_IOC();
   
   // Give user a hint of system start by turning on fault led
   
   // Set variables to default values
   set_variables();
   // Set driver variables to default values
   set_driver();
   //setup_uart(UART_AUTODETECT);
   
   //int1 up_limit = read_eeprom (up_lim_add);
   //int1 down_limit = read_eeprom (down_lim_add);
   //if(up_limit == 1 || down_limit == 1)
      //limit_old = 1;
   putc('c');
   //fprintf("______________________________________________________________\n\r");
   //fprintf("\n\n\rMODESIS LASER POSITIONING SYSTEM\n\n\r");
   //fprintf("\n\n\rCard Version V1.1 Software Version v1.0\n\n\r");
   //motion_cycle();
}
