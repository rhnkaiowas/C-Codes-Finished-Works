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
#USE   RS232(stream=RS232, baud=38400, xmit=PIN_B5, rcv=PIN_B2, parity=N, bits=8, stop=1)

#DEFINE DRV_STEP           PIN_A0                  // Step output pin to driver (Rising edge causes the indexer to move one step)
#DEFINE DRV_RESET          PIN_A1                  // Reset output pin to driver (Active-high reset input initializes all internal logic and disables the Hbridge outputs. Internal pulldown.)
#DEFINE DRV_DIR            PIN_A4                  // Direction output pin to driver (Logic level, sets the direction of stepping)
#DEFINE SPI_SDO            PIN_A6                  // SPI data output pin
#DEFINE SPI_CS             PIN_A7                  // SPI chip select pin
#DEFINE SPI_SDI            PIN_B1                  // SPI data input pin
#DEFINE SPI_SCK            PIN_B4                  // SPI clock output pin
#DEFINE DRV_STALL          PIN_B6                  // Stall input pin from driver (Internal stall detect mode: logic low when motor stall detected. Pull up mevcut)
#DEFINE DRV_FAULT          PIN_B7                  // Fault input pin from driver (Logic low when in fault condition. Pull up mevcut)

#DEFINE LIMIT_SWITCH       PIN_A2                  // Fault output pin to user
#DEFINE USER_DIR           PIN_A3                  // Direction input pin from user
#DEFINE USER_STEP          PIN_B0                  // Step input pin from user
#DEFINE HOME_SWITCH        PIN_B3                  // Enable input pin from user

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
#BIT IOC_PINB0_RISING            = IOC_IOCBP.0        //Pin B3 interrupt on change enable bit(Rising Edge)
#BIT IOC_PINB3_RISING            = IOC_IOCBP.3        //Pin B3 interrupt on change enable bit(Rising Edge)

//Bits of IOC_IOCBN register
#BIT IOC_PINB0_FALLING           = IOC_IOCBN.0        //Pin B3 interrupt on change enable bit(Falling Edge)
#BIT IOC_PINB3_FALLING           = IOC_IOCBN.3        //Pin B3 interrupt on change enable bit(Rising Edge)
/*
//Bits of IOC_IOCBF register
#BIT IOC_PINB3_FLAG              = IOC_IOCBF.3        //Pin B3 interrupt on change interrupt flag bit(Both Rising and Falling Edges)
#BIT IOC_PINB3_FLAG              = IOC_IOCBP.3        //Pin B3 interrupt on change enable bit(Rising Edge)

//Bits of ANSELB register
#BIT ANSELB_PINB3                = ANSELB.3           //Selection of Pin B3 as a Digial I/O (0) or an Analog input (1) selection bit
*/
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
//#BIT PIN_RX_SELECT      = PIN_APFCON0.7      // RX pin selection bit
#BIT PIN_SDO_SELECT     = PIN_APFCON0.6      // SDO pin selection bit 
#BIT PIN_SS_SELECT      = PIN_APFCON0.5      // SS pin selection bit 

// Bits of PIN_APFCON1 register
//#BIT PIN_TX_SELECT      = PIN_APFCON1.0      // TX pin selection bit

// CTRL register of the driver
unsigned int16 reg_ctrl = 0;
// TORQUE register of the driver
unsigned int16 reg_torque = 0;
// OFF register of the driver
unsigned int16 reg_off = 0;
// BLANK register of the driver
unsigned int16 reg_blank = 0;
// DECAY register of the driver
unsigned int16 reg_decay = 0;
// STALL register of the driver
unsigned int16 reg_stall = 0;
// DRIVE register of the driver
unsigned int16 reg_drive = 0;
// STATUS register of the driver
unsigned int16 reg_status = 0;
// Driver debug mode
int1 drv_debug = 1;

char reg_rs232_message = 0;

int delay = 20;

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

// Writes the given register byte to the driver
void write_register_byte(unsigned int8 reg_byte)
{
   // write the byte to spi buffer
   SPI_SSP1BUF = reg_byte;
   // Wait until the end of the write operation
   while (!SPI_FLAG);
   // Clear the write-completed-flag of the spi module
   SPI_FLAG = 0;
}
// Writes the given register to the driver
void write_register(unsigned int16 reg)
{
   if(drv_debug)
      fprintf(RS232,"%Lu\n\r", reg);
   
   // start spi write operation by setting the chip select port to high
   output_high(SPI_CS);
   // Get and write the MSB of the register
   write_register_byte(make8(reg, 1));
   // Get and write the LSB of the register
   write_register_byte(make8(reg, 0));
   // stop spi write operation by setting the chip select port to low
   output_low(SPI_CS);
   delay_ms(10);
}
// Sets the given number of bits of the register from starting address to the given value
unsigned int16 set_register_bits(unsigned int16 reg, unsigned int8 bit_address, unsigned int8 bit_size, unsigned int16 value)
{
   // bit address must be smaller then register size (16 bit)
   if(bit_address > 15) return 0;
   // bit size can't be larger then the number of bits between the given address and the last bit of the register
   if(bit_size > (16 - bit_address)) return 0;
   
   //fprintf(RS232,"reg: %Lu adr: %u size: %u val: %lu\n\r", reg, bit_address, bit_size, value);
   
   // Get the bits of the "value" and set it to the corresponding bit in the given address
   for(int i=0; i<bit_size; i++)
   {
      if(bit_test(value, i) == 1)
         bit_set(reg, bit_address + i);
      else
         bit_clear(reg, bit_address + i);
   }
   
   //fprintf(RS232,"reg: %Lu\n\r", reg);
   
   return reg;
}
/*
// Driver common register variables
// data operation type                             (bit 15)    0: write, 
//                                                             1: read
// register address                                (bit 14-12) 000: CTRL register
//                                                             001: TORQUE register
//                                                             010: OFF register
//                                                             011: BLANK register
//                                                             100: DECAY register
//                                                             101: STALL register
//                                                             110: DRIVE register
//                                                             111: STATUS register
*/
enum reg_type           {type_ctrl = 0, type_torque = 1, type_off = 2, type_blank = 3, type_decay = 4, type_stall = 5, type_drive = 6, type_status = 7};
enum op_type            {op_write = 0, op_read = 1};
enum reg_common_mask    {reg_type_addr = 12, op_type_addr = 15};
enum reg_common_size    {reg_type_size = 3, op_type_size = 1};
/*
// CTRL register bits
// data operation type                             (bit 15)    0: write, 
//                                                             1: read
// register address                                (bit 14-12) 000: CTRL reg
// DTIME (dead time btw MOSFET switching)    (R/W) (bit 11-10) 00: 400 ns, 
//                                                             01: 450 ns, 
//                                                             10: 650 ns, 
//                                                             11: 850 ns
// ISGain (current amplifier gain)           (R/W) (bit 9-8)   00: Gain of 5, 
//                                                             01: Gain of 10, 
//                                                             10: Gain of 20, 
//                                                             11: Gain of 40
// EXSTALL (source of stall detection)       (R/W) (bit 7)     0: Internal
//                                                             1: External
// MODE (microstepping mode)                 (R/W) (bit 6-3)   0000: Full-step, 71% current
//                                                             0001: Half step
//                                                             0010: 1/4 step
//                                                             0011: 1/8 step
//                                                             0100: 1/16 step
//                                                             0101: 1/32 step
//                                                             0110: 1/64 step
//                                                             0111: 1/128 step
//                                                             1000: 1/256 step
// RSTEP (single step output)                (W)   (bit 2)     0: No action
//                                                             1: Indexer will advance one step; automatically cleared after write
// direction                                 (R/W) (bit 1)     0: Direction set by DIR pin
//                                                             1: Direction set by inverse of DIR pin
// motor state                               (R/W) (bit 0)     0: Disable motor
//                                                             1: Enable motor
*/
enum dead_time          {dtime_400ns = 0, dtime_450ns = 1, dtime_650ns = 2, dtime_850ns = 3};
enum curent_amp_gain    {gain_5 = 0, gain_10 = 1, gain_20 = 2, gain_40 = 3};
enum src_stall_detect   {stall_int = 0, stall_ext = 1};
enum microstepping      {full_step = 0, half_step = 1, _4x = 2, _8x = 3, _16x = 4, _32x = 5, _64x = 6, _128x = 7, _256x = 8};
enum single_step_out    {no_action = 0, one_step = 1};
enum dir_control        {dir_pin = 0, inv_dir_pin = 1};
enum motor_state        {motor_disable = 0, motor_enable = 1};
enum reg_ctrl_mask      {motor_state_addr = 0, dir_control_addr = 1, single_step_out_addr = 2, microstepping_addr = 3, src_stall_detect_addr = 7, curent_amp_gain_addr = 8, dead_time_addr = 10};
enum reg_ctrl_size      {motor_state_size = 1, dir_control_size = 1, single_step_out_size = 1, microstepping_size = 4, src_stall_detect_size = 1, curent_amp_gain_size = 2, dead_time_size = 2};
// Sets CTRL register
void set_ctrl_reg(dead_time dtime, curent_amp_gain gain, src_stall_detect stall, microstepping mode, single_step_out rstep, dir_control dir, motor_state state)
{
   if(drv_debug)
      fprintf(RS232,"CTRL Register\t: ");
   
   // clear the register
   reg_ctrl = 0b0000000000000000;
   // set write operation
   reg_ctrl = set_register_bits(reg_ctrl, op_type_addr, op_type_size, op_write); 
   // set register address
   reg_ctrl = set_register_bits(reg_ctrl, reg_type_addr, reg_type_size, type_ctrl);
   // set dead time between MOSFET switching
   reg_ctrl = set_register_bits(reg_ctrl, dead_time_addr, dead_time_size, dtime); 
   // set current amplifier gain
   reg_ctrl = set_register_bits(reg_ctrl, curent_amp_gain_addr, curent_amp_gain_size, gain);
   // set source of stall detection
   reg_ctrl = set_register_bits(reg_ctrl, src_stall_detect_addr, src_stall_detect_size, stall);
   // set microstepping mode
   reg_ctrl = set_register_bits(reg_ctrl, microstepping_addr, microstepping_size, mode);
   // set single step output mode
   reg_ctrl = set_register_bits(reg_ctrl, single_step_out_addr, single_step_out_size, rstep);
   // set direction
   reg_ctrl = set_register_bits(reg_ctrl, dir_control_addr, dir_control_size, dir);
   // set motor state
   reg_ctrl = set_register_bits(reg_ctrl, motor_state_addr, motor_state_size, state);

   // write register to SPI
   write_register(reg_ctrl);
}
// Sets CTRL register
void set_dead_time(dead_time dtime)
{
   if(drv_debug)
      fprintf(RS232,"CTRL Register\t: ");
      
   // set write operation
   reg_ctrl = set_register_bits(reg_ctrl, op_type_addr, op_type_size, op_write); 
   // set register address
   reg_ctrl = set_register_bits(reg_ctrl, reg_type_addr, reg_type_size, type_ctrl);
   // set dead time between MOSFET switching
   reg_ctrl = set_register_bits(reg_ctrl, dead_time_addr, dead_time_size, dtime); 
   // write register to SPI
   write_register(reg_ctrl);
}
// Sets CTRL register
void set_amp_gain(curent_amp_gain gain)
{
   if(drv_debug)
      fprintf(RS232,"CTRL Register\t: ");
      
   // set write operation
   reg_ctrl = set_register_bits(reg_ctrl, op_type_addr, op_type_size, op_write); 
   // set register address
   reg_ctrl = set_register_bits(reg_ctrl, reg_type_addr, reg_type_size, type_ctrl);
   // set current amplifier gain
   reg_ctrl = set_register_bits(reg_ctrl, curent_amp_gain_addr, curent_amp_gain_size, gain);
   // write register to SPI
   write_register(reg_ctrl);
}
// Sets CTRL register
void set_stall_detection(src_stall_detect stall)
{
   if(drv_debug)
      fprintf(RS232,"CTRL Register\t: ");
      
   // set write operation
   reg_ctrl = set_register_bits(reg_ctrl, op_type_addr, op_type_size, op_write); 
   // set register address
   reg_ctrl = set_register_bits(reg_ctrl, reg_type_addr, reg_type_size, type_ctrl);
   // set source of stall detection
   reg_ctrl = set_register_bits(reg_ctrl, src_stall_detect_addr, src_stall_detect_size, stall);
   // write register to SPI
   write_register(reg_ctrl);
}
// Sets CTRL register
void set_microstepping_mode(microstepping mode)
{
   if(drv_debug)
      fprintf(RS232,"CTRL Register\t: ");
      
   // set write operation
   reg_ctrl = set_register_bits(reg_ctrl, op_type_addr, op_type_size, op_write); 
   // set register address
   reg_ctrl = set_register_bits(reg_ctrl, reg_type_addr, reg_type_size, type_ctrl);
   // set microstepping mode
   reg_ctrl = set_register_bits(reg_ctrl, microstepping_addr, microstepping_size, mode);
   // write register to SPI
   write_register(reg_ctrl);
}
// Sets CTRL register
void set_rstep(single_step_out rstep)
{
   if(drv_debug)
      fprintf(RS232,"CTRL Register\t: ");
        
   // set write operation
   reg_ctrl = set_register_bits(reg_ctrl, op_type_addr, op_type_size, op_write); 
   // set register address
   reg_ctrl = set_register_bits(reg_ctrl, reg_type_addr, reg_type_size, type_ctrl);
   // set single step output mode
   reg_ctrl = set_register_bits(reg_ctrl, single_step_out_addr, single_step_out_size, rstep);
   // write register to SPI
   write_register(reg_ctrl);
}
// Sets CTRL register
void set_dir_mode(dir_control dir)
{
   if(drv_debug)
      fprintf(RS232,"CTRL Register\t: ");
      
   // set write operation
   reg_ctrl = set_register_bits(reg_ctrl, op_type_addr, op_type_size, op_write); 
   // set register address
   reg_ctrl = set_register_bits(reg_ctrl, reg_type_addr, reg_type_size, type_ctrl);
   // set direction mode
   reg_ctrl = set_register_bits(reg_ctrl, dir_control_addr, dir_control_size, dir);
   // write register to SPI
   write_register(reg_ctrl);
}
// Sets CTRL register
void set_motor_state(motor_state state)
{
   if(drv_debug)
      fprintf(RS232,"CTRL Register\t: ");
      
   // set write operation
   reg_ctrl = set_register_bits(reg_ctrl, op_type_addr, op_type_size, op_write); 
   // set register address
   reg_ctrl = set_register_bits(reg_ctrl, reg_type_addr, reg_type_size, type_ctrl);
   // set motor state
   reg_ctrl = set_register_bits(reg_ctrl, motor_state_addr, motor_state_size, state);
   // write register to SPI
   write_register(reg_ctrl);
}
/*
// TORQUE register bits
// data operation type                             (bit 15)    0: write, 
//                                                             1: read
// register address                                (bit 14-12) 001: Torque reg
// Reserved                                        (bit 11)
// SMPLTH (Back EMF sample threshold)        (R/W) (bit 10-8)  000: 50 us, 
//                                                             001: 100 us, 
//                                                             010: 200 us, 
//                                                             011: 300 us,
//                                                             100: 400 us,
//                                                             101: 600 us,
//                                                             110: 800 us, 
//                                                             111: 1000 us,
// Torque (Output current for H-bridges)     (R/W) (bit 7-0)   0xFFh
*/
enum emf_samp_thr       {emf_50us = 0, emf_100us = 1, emf_200us = 2, emf_300us = 3, emf_400us = 4, emf_600us = 5, emf_800us = 6, emf_1000us = 7};
enum reg_torque_mask    {torque_addr = 0, emf_samp_thr_addr = 8};
enum reg_torque_size    {torque_size = 8, emf_samp_thr_size = 3};
// Sets TORQUE register
void set_torque_reg(emf_samp_thr emf, unsigned int8 torque)
{
   if(drv_debug)
      fprintf(RS232,"TORQUE Register\t: ");
      
   // clear the register
   reg_torque = 0b0000000000000000;
   // set write operation
   reg_torque = set_register_bits(reg_torque, op_type_addr, op_type_size, op_write); 
   // set register address
   reg_torque = set_register_bits(reg_torque, reg_type_addr, reg_type_size, type_torque);
   // set back EMF sample threshold
   reg_torque = set_register_bits(reg_torque, emf_samp_thr_addr, emf_samp_thr_size, emf);
   // set output current for H-bridges
   reg_torque = set_register_bits(reg_torque, torque_addr, torque_size, torque);
   
   // write register to SPI
   write_register(reg_torque);
}
// Sets back EMF sample threshold
void set_emf_samp_thr(emf_samp_thr emf)
{
   if(drv_debug)
      fprintf(RS232,"TORQUE Register\t: ");
      
   // set write operation
   reg_torque = set_register_bits(reg_torque, op_type_addr, op_type_size, op_write); 
   // set register address
   reg_torque = set_register_bits(reg_torque, reg_type_addr, reg_type_size, type_torque);
   // set back EMF sample threshold
   reg_torque = set_register_bits(reg_torque, emf_samp_thr_addr, emf_samp_thr_size, emf);
   // write register to SPI
   write_register(reg_torque);
}
// Sets full-scale output current for both H-bridges
void set_torque(unsigned int8 torque)
{
   if(drv_debug)
      fprintf(RS232,"TORQUE Register\t: ");
      
   // set write operation
   reg_torque = set_register_bits(reg_torque, op_type_addr, op_type_size, op_write); 
   // set register address
   reg_torque = set_register_bits(reg_torque, reg_type_addr, reg_type_size, type_torque);
   // set output current for H-bridges
   reg_torque = set_register_bits(reg_torque, torque_addr, torque_size, torque);
   // write register to SPI
   write_register(reg_torque);
}
/*
// OFF register bits
// data operation type                             (bit 15)    0: write, 
//                                                             1: read
// register address                                (bit 14-12) 010: OFF reg
// Reserved                                        (bit 11-9)
// PWMMODE (indexer mode)                    (R/W) (bit 8)     0: Use internal indexer, 
//                                                             1: Bypass indexer, use xINx inputs to control outputs 
// TOFF (Sets fixed off time, 500ns steps)   (R/W) (bit 7-0)   0x00h: 500 ns
//                                                             0xFFh: 128 us
*/
enum pwm_mode           {int_indexer = 0, ext_indexer = 1};
enum reg_off_mask       {toff_addr = 0, pwm_mode_addr = 8};
enum reg_off_size       {toff_size = 8, pwm_mode_size = 1};
// Sets OFF register
void set_off_reg(pwm_mode indexer, unsigned int8 toff)
{
   if(drv_debug)
      fprintf(RS232,"TOFF Register\t: ");
      
   // clear the register
   reg_off = 0b0000000000000000;
   // set write operation
   reg_off = set_register_bits(reg_off, op_type_addr, op_type_size, op_write); 
   // set register address
   reg_off = set_register_bits(reg_off, reg_type_addr, reg_type_size, type_off);
   // set pwm indexer mode
   reg_off = set_register_bits(reg_off, pwm_mode_addr, pwm_mode_size, indexer);
   // set fixed off time
   reg_off = set_register_bits(reg_off, toff_addr, toff_size, toff);

   // write register to SPI
   write_register(reg_off);
}
// Sets pwm indexer mode
void set_pwm_mode(pwm_mode indexer)
{
   if(drv_debug)
      fprintf(RS232,"TOFF Register\t: ");
      
   // set write operation
   reg_off = set_register_bits(reg_off, op_type_addr, op_type_size, op_write); 
   // set register address
   reg_off = set_register_bits(reg_off, reg_type_addr, reg_type_size, type_off);
   // set pwm indexer mode
   reg_off = set_register_bits(reg_off, pwm_mode_addr, pwm_mode_size, indexer);
   // write register to SPI
   write_register(reg_off);
}
// Sets fixed off time
void set_off_time(unsigned int8 toff)
{
   if(drv_debug)
      fprintf(RS232,"TOFF Register\t: ");
      
   // set write operation
   reg_off = set_register_bits(reg_off, op_type_addr, op_type_size, op_write); 
   // set register address
   reg_off = set_register_bits(reg_off, reg_type_addr, reg_type_size, type_off);
   // set fixed off time
   reg_off = set_register_bits(reg_off, toff_addr, toff_size, toff);
   // write register to SPI
   write_register(reg_off);
}
/*
// BLANK register bits
// data operation type                             (bit 15)    0: write, 
//                                                             1: read
// register address                                (bit 14-12) 011: BLANK reg
// Reserved                                        (bit 11-9)
// ABT (adaptive blanking time state)        (R/W) (bit 8)     0: Disable adaptive blanking time, 
//                                                             1: Enable adaptive blanking time
// TBLANK (Sets blanking time, 20ns steps)   (R/W) (bit 7-0)   0x00h: 1.00 us
//                                                             ...
//                                                             0x32h: 1.00 us
//                                                             0x33h: 1.02 us
//                                                             ...
//                                                             0xFEh: 5.10 us
//                                                             0xFFh: 5.12 us
*/
enum adap_blank_state   {disable_abt = 0, enable_abt = 1};
enum reg_blank_mask     {tblank_addr = 0, adap_blank_time_addr = 8};
enum reg_blank_size     {tblank_size = 8, adap_blank_time_size = 1};
// Sets BLANK register
void set_blank_reg(adap_blank_state abt, unsigned int8 tblank)
{
   if(drv_debug)
      fprintf(RS232,"BLANK Register\t: ");
      
   // clear the register
   reg_blank = 0b0000000000000000;
   // set write operation
   reg_blank = set_register_bits(reg_blank, op_type_addr, op_type_size, op_write); 
   // set register address
   reg_blank = set_register_bits(reg_blank, reg_type_addr, reg_type_size, type_blank);
   // set adaptive blanking time state
   reg_blank = set_register_bits(reg_blank, adap_blank_time_addr, adap_blank_time_size, abt);
   // set blanking time
   reg_blank = set_register_bits(reg_blank, tblank_addr, tblank_size, tblank);
   
   // write register to SPI
   write_register(reg_blank);
}
// Sets adaptive blank time state
void set_adap_blank_state(adap_blank_state abt)
{
   if(drv_debug)
      fprintf(RS232,"BLANK Register\t: ");
      
   // set write operation
   reg_blank = set_register_bits(reg_blank, op_type_addr, op_type_size, op_write); 
   // set register address
   reg_blank = set_register_bits(reg_blank, reg_type_addr, reg_type_size, type_blank);
   // set adaptive blanking time state
   reg_blank = set_register_bits(reg_blank, adap_blank_time_addr, adap_blank_time_size, abt);
   // write register to SPI
   write_register(reg_blank);
}
// Sets blanking time
void set_blank_time(unsigned int8 tblank)
{
   if(drv_debug)
      fprintf(RS232,"BLANK Register\t: ");
      
   // set write operation
   reg_blank = set_register_bits(reg_blank, op_type_addr, op_type_size, op_write); 
   // set register address
   reg_blank = set_register_bits(reg_blank, reg_type_addr, reg_type_size, type_blank);
   // set blanking time
   reg_blank = set_register_bits(reg_blank, tblank_addr, tblank_size, tblank);
   // write register to SPI
   write_register(reg_blank);
}
/*
// DECAY register bits
// data operation type                             (bit 15)    0: write, 
//                                                             1: read
// register address                                (bit 14-12) 100: DECAY reg
// Reserved                                        (bit 11)
// DECMODE (decay modes)                     (R/W) (bit 10-8)  000: Force slow decay at all times
//                                                             001: Slow decay for increasing current, mixed decay for decreasing current (indexer mode only)
//                                                             010: Force fast decay at all times
//                                                             011: Use mixed decay at all times
//                                                             100: Slow decay for increasing current, auto mixed decay for decreasing current (indexer mode only)
//                                                             101: Use auto mixed decay at all times
//                                                             110-111: Reserved
// TDECAY (Sets transition time, 500ns steps)(R/W) (bit 7-0)   0x00h: 500 ns
//                                                             0xFFh: 128 us
*/
enum decay_mode         {force_slow_decay = 0, slow_mixed_decay = 1, force_fast_decay = 2, mixed_decay = 3, slow_auto_mixed_decay = 4, auto_mixed_decay = 5};
enum reg_decay_mask     {tdecay_addr = 0, decay_mode_addr = 8};
enum reg_decay_size     {tdecay_size = 8, decay_mode_size = 3};
// Sets DECAY register
void set_decay_reg(decay_mode dmode, unsigned int8 tdecay)
{
   if(drv_debug)
      fprintf(RS232,"DECAY Register\t: ");
      
   // clear the register
   reg_decay = 0b0000000000000000;
   // set write operation
   reg_decay = set_register_bits(reg_decay, op_type_addr, op_type_size, op_write); 
   // set register address
   reg_decay = set_register_bits(reg_decay, reg_type_addr, reg_type_size, type_decay);
   // set decay mode
   reg_decay = set_register_bits(reg_decay, decay_mode_addr, decay_mode_size, dmode);
   // set decay time
   reg_decay = set_register_bits(reg_decay, tdecay_addr, tdecay_size, tdecay);

   // write register to SPI
   write_register(reg_decay);
}
// Sets decay (transition) mode
void set_decay_mode(decay_mode dmode)
{
   if(drv_debug)
      fprintf(RS232,"DECAY Register\t: ");
      
   // set write operation
   reg_decay = set_register_bits(reg_decay, op_type_addr, op_type_size, op_write); 
   // set register address
   reg_decay = set_register_bits(reg_decay, reg_type_addr, reg_type_size, type_decay);
   // set decay mode
   reg_decay = set_register_bits(reg_decay, decay_mode_addr, decay_mode_size, dmode);
   // write register to SPI
   write_register(reg_decay);
}
// Sets decay (transition) time
void set_decay_time(unsigned int8 tdecay)
{
   if(drv_debug)
      fprintf(RS232,"DECAY Register\t: ");
      
   // set write operation
   reg_decay = set_register_bits(reg_decay, op_type_addr, op_type_size, op_write); 
   // set register address
   reg_decay = set_register_bits(reg_decay, reg_type_addr, reg_type_size, type_decay);
   // set decay time
   reg_decay = set_register_bits(reg_decay, tdecay_addr, tdecay_size, tdecay);
   // write register to SPI
   write_register(reg_decay);
}
/*
// STALL register bits
// data operation type                             (bit 15)    0: write,
//                                                             1: read
// register address                                (bit 14-12) 101: STALL reg
// VDIV (Back EMF divider)                   (R/W) (bit 11-10) 00: Back EMF is divided by 32
//                                                             01: Back EMF is divided by 16
//                                                             10: Back EMF is divided by 8
//                                                             11: Back EMF is divided by 4
// SDCNT (Back EMF sample step mode)         (R/W) (bit 9-8)   00: STALLn asserted on first step with back EMF below SDTHR
//                                                             01: STALLn asserted after 2 steps
//                                                             10: STALLn asserted after 4 steps
//                                                             11: STALLn asserted after 8 steps
// SDTHR (Sets stall detect threshold)       (R/W) (bit 7-0)   0x00h: 0
//                                                             0xFFh: 255
*/
enum emf_vol_div        {divide_by_32 = 0, divide_by_16 = 1, divide_by_8 = 2, divide_by_4 = 3};
enum emf_samp_mode      {after_1_step = 0, after_2_steps = 1, after_4_steps = 2, after_8_steps = 3};
enum reg_stall_mask     {stall_thr_addr = 0, emf_samp_mode_addr = 8, emf_vol_div_addr = 10};
enum reg_stall_size     {stall_thr_size = 8, emf_samp_mode_size = 2, emf_vol_div_size = 2};  
// Sets STALL register
void set_stall_reg(emf_vol_div divider, emf_samp_mode smode, unsigned int8 stall_thr)
{
   if(drv_debug)
      fprintf(RS232,"STALL Register\t: ");
      
   // clear the register
   reg_stall = 0b0000000000000000;
   // set write operation
   reg_stall = set_register_bits(reg_stall, op_type_addr, op_type_size, op_write); 
   // set register address
   reg_stall = set_register_bits(reg_stall, reg_type_addr, reg_type_size, type_stall);
   // set back EMF voltage divider
   reg_stall = set_register_bits(reg_stall, emf_vol_div_addr, emf_vol_div_size, divider);
   // set back EMF sample step mode
   reg_stall = set_register_bits(reg_stall, emf_samp_mode_addr, emf_samp_mode_size, smode);
   // set stall detect threshold
   reg_stall = set_register_bits(reg_stall, stall_thr_addr, stall_thr_size, stall_thr);
   
   // write register to SPI
   write_register(reg_stall);
}
// Sets back EMF voltage divider
void set_emf_vol_div(emf_vol_div divider)
{
   if(drv_debug)
      fprintf(RS232,"STALL Register\t: ");
      
   // set write operation
   reg_stall = set_register_bits(reg_stall, op_type_addr, op_type_size, op_write); 
   // set register address
   reg_stall = set_register_bits(reg_stall, reg_type_addr, reg_type_size, type_stall);
   // set back EMF voltage divider
   reg_stall = set_register_bits(reg_stall, emf_vol_div_addr, emf_vol_div_size, divider);
   // write register to SPI
   write_register(reg_stall);
}
// Sets back EMF sample step mode
void set_emf_samp_mode(emf_samp_mode smode)
{
   if(drv_debug)
      fprintf(RS232,"STALL Register\t: ");
      
   // set write operation
   reg_stall = set_register_bits(reg_stall, op_type_addr, op_type_size, op_write); 
   // set register address
   reg_stall = set_register_bits(reg_stall, reg_type_addr, reg_type_size, type_stall);
   // set back EMF sample step mode
   reg_stall = set_register_bits(reg_stall, emf_samp_mode_addr, emf_samp_mode_size, smode);
   // write register to SPI
   write_register(reg_stall);
}
// Sets stall detect threshold
void set_stall_thr(unsigned int8 stall_thr)
{
   if(drv_debug)
      fprintf(RS232,"STALL Register\t: ");
      
   // set write operation
   reg_stall = set_register_bits(reg_stall, op_type_addr, op_type_size, op_write); 
   // set register address
   reg_stall = set_register_bits(reg_stall, reg_type_addr, reg_type_size, type_stall);
   // set stall detect threshold
   reg_stall = set_register_bits(reg_stall, stall_thr_addr, stall_thr_size, stall_thr);
   // write register to SPI
   write_register(reg_stall);
}
/*
// DRIVE register bits
// data operation type                       (R/W) (bit 15)    0: write, 
//                                                             1: read
// register address                          (R/W) (bit 14-12) 110: STALL reg
// IDRIVEP (High-side gate peak current)     (R/W) (bit 11-10) 00: 50 mA peak (source)
//                                                             01: 100 mA peak (source)
//                                                             10: 150 mA peak (source)
//                                                             11: 200 mA peak (source)
// IDRIVEN (Low-side gate peak current)      (R/W) (bit 9-8)   00: 100 mA peak (sink)
//                                                             01: 200 mA peak (sink)
//                                                             10: 300 mA peak (sink)
//                                                             11: 400 mA peak (sink)
// TDRIVEP (High-side gate drive time)       (R/W) (bit 7-6)   00: 250 ns
//                                                             01: 500 ns
//                                                             10: 1 us
//                                                             11: 2 us
// TDRIVEN (Low-side gate drive time)        (R/W) (bit 5-4)   00: 250 ns
//                                                             01: 500 ns
//                                                             10: 1 us
//                                                             11: 2 us
// OCPDEG (OCP deglitch time)                (R/W) (bit 3-2)   00: 1 us
//                                                             01: 2 us
//                                                             10: 4 us
//                                                             11: 8 us
// OCPTH (OCP threshold)                     (R/W) (bit 1-0)   00: 250 mV
//                                                             01: 500 mV
//                                                             10: 750 mV
//                                                             11: 1000 mV
*/
enum hs_peak_cur        {hs_50mA = 0, hs_100mA = 1, hs_150mA = 2, hs_200mA = 3};
enum ls_peak_cur        {ls_100mA = 0, ls_200mA = 1, ls_300mA = 2, ls_400mA = 3};
enum hs_drive_time      {hs_250ns = 0, hs_500ns = 1, hs_1us = 2, hs_2us = 3};
enum ls_drive_time      {ls_250ns = 0, ls_500ns = 1, ls_1us = 2, ls_2us = 3};
enum ocp_deg_time       {ocp_1us = 0, ocp_2us = 1, ocp_4us = 2, ocp_8us = 3};
enum ocp_thr            {ocp_250mV = 0, ocp_500mV = 1, ocp_750mV = 2, ocp_1000mV = 3};
enum reg_drive_mask     {ocp_thr_addr = 0, ocp_deg_time_addr = 2, ls_drive_time_addr = 4, hs_drive_time_addr = 6, ls_peak_cur_addr = 8, hs_peak_cur_addr = 10};
enum reg_drive_size     {ocp_thr_size = 2, ocp_deg_time_size = 2, ls_drive_time_size = 2, hs_drive_time_size = 2, ls_peak_cur_size = 2, hs_peak_cur_size = 2}; 
// Sets DRIVE register
void set_drive_reg(hs_peak_cur idrivep, ls_peak_cur idriven, hs_drive_time tdrivep, ls_drive_time tdriven, ocp_deg_time ocpdeg, ocp_thr ocpth)
{
   if(drv_debug)
      fprintf(RS232,"DRIVE Register\t: ");
      
   // clear the register
   reg_drive = 0b0000000000000000;
   // set write operation
   reg_drive = set_register_bits(reg_drive, op_type_addr, op_type_size, op_write); 
   // set register address
   reg_drive = set_register_bits(reg_drive, reg_type_addr, reg_type_size, type_drive);
   // set high-side gate peak current
   reg_drive = set_register_bits(reg_drive, hs_peak_cur_addr, hs_peak_cur_size, idrivep);
   // set low-side gate peak current
   reg_drive = set_register_bits(reg_drive, ls_peak_cur_addr, ls_peak_cur_size, idriven);
   // set high-side gate drive time
   reg_drive = set_register_bits(reg_drive, hs_drive_time_addr, hs_drive_time_size, tdrivep);
   // set low-side gate drive time
   reg_drive = set_register_bits(reg_drive, ls_drive_time_addr, ls_drive_time_size, tdriven);
   // set OCP deglitch time
   reg_drive = set_register_bits(reg_drive, ocp_deg_time_addr, ocp_deg_time_size, ocpdeg);
   // set OCP threshold
   reg_drive = set_register_bits(reg_drive, ocp_thr_addr, ocp_thr_size, ocpth);

   // write register to SPI
   write_register(reg_drive);
}
/*
// Sets high-side gate peak current
void set_hs_peak_cur(hs_peak_cur idrivep)
{
   if(drv_debug)
      fprintf(RS232,"DRIVE Register\t: ");
      
   // set write operation
   reg_drive = set_register_bits(reg_drive, op_type_addr, op_type_size, op_write); 
   // set register address
   reg_drive = set_register_bits(reg_drive, reg_type_addr, reg_type_size, type_drive);
   // set high-side gate peak current
   reg_drive = set_register_bits(reg_drive, hs_peak_cur_addr, hs_peak_cur_size, idrivep);
   // write register to SPI
   write_register(reg_drive);
}
// Sets low-side gate peak current
void set_ls_peak_cur(ls_peak_cur idriven)
{
   if(drv_debug)
      fprintf(RS232,"DRIVE Register\t: ");
      
   // set write operation
   reg_drive = set_register_bits(reg_drive, op_type_addr, op_type_size, op_write); 
   // set register address
   reg_drive = set_register_bits(reg_drive, reg_type_addr, reg_type_size, type_drive);
   // set low-side gate peak current
   reg_drive = set_register_bits(reg_drive, ls_peak_cur_addr, ls_peak_cur_size, idriven);
   // write register to SPI
   write_register(reg_drive);
}
// Sets high-side gate drive time
void set_hs_drive_time(hs_drive_time tdrivep)
{
   if(drv_debug)
      fprintf(RS232,"DRIVE Register\t: ");
      
   // set write operation
   reg_drive = set_register_bits(reg_drive, op_type_addr, op_type_size, op_write); 
   // set register address
   reg_drive = set_register_bits(reg_drive, reg_type_addr, reg_type_size, type_drive);
   // set high-side gate drive time
   reg_drive = set_register_bits(reg_drive, hs_drive_time_addr, hs_drive_time_size, tdrivep);
   // write register to SPI
   write_register(reg_drive);
}
// Sets low-side gate drive time
void set_ls_drive_time(ls_drive_time tdriven)
{
   if(drv_debug)
      fprintf(RS232,"DRIVE Register\t: ");
      
   // set write operation
   reg_drive = set_register_bits(reg_drive, op_type_addr, op_type_size, op_write); 
   // set register address
   reg_drive = set_register_bits(reg_drive, reg_type_addr, reg_type_size, type_drive);
   // set low-side gate drive time
   reg_drive = set_register_bits(reg_drive, ls_drive_time_addr, ls_drive_time_size, tdriven);
   // write register to SPI
   write_register(reg_drive);
}
// Sets OCP deglitch time
void set_ocp_deg_time(ocp_deg_time ocpdeg)
{
   if(drv_debug)
      fprintf(RS232,"DRIVE Register\t: ");
      
   // set write operation
   reg_drive = set_register_bits(reg_drive, op_type_addr, op_type_size, op_write); 
   // set register address
   reg_drive = set_register_bits(reg_drive, reg_type_addr, reg_type_size, type_drive);
   // set OCP deglitch time
   reg_drive = set_register_bits(reg_drive, ocp_deg_time_addr, ocp_deg_time_size, ocpdeg);
   // write register to SPI
   write_register(reg_drive);
}
// Sets OCP threshold
void set_ocp_thr(ocp_thr ocpth)
{
   if(drv_debug)
      fprintf(RS232,"DRIVE Register\t: ");
      
   // set write operation
   reg_drive = set_register_bits(reg_drive, op_type_addr, op_type_size, op_write); 
   // set register address
   reg_drive = set_register_bits(reg_drive, reg_type_addr, reg_type_size, type_drive);
   // set OCP threshold
   reg_drive = set_register_bits(reg_drive, ocp_thr_addr, ocp_thr_size, ocpth);
   // write register to SPI
   write_register(reg_drive);
}
*/
/*
// STATUS register bits
// data operation type                             (bit 15)    0: write, 
//                                                             1: read
// register address                                (bit 14-12) 111: STATUS reg
// Reserved                                        (bit 11-8)
// STDLAT (Stall clear flag)                 (R/W) (bit 7)     0: Normal operation
//                                                             1: Latched stall detect
//                                                             (Write a '0' to this bit to clear the fault and resume operation)
// STD (Stall detected flag)                 (R)   (bit 6)     0: Normal operation
//                                                             1: Stall detected
// UVLO (Undervoltage lockout flag)          (R)   (bit 5)     0: Normal operation
//                                                             1: Undervoltage lockout
//                                                             (UVLO bit will clear after VM has increased over VUVLO)
// BPDF (Channel B predriver fault flag)     (R/W) (bit 4)     0: Normal operation
//                                                             1: Channel B predriver fault
//                                                             (Write a '0' to this bit to clear the fault and resume operation)
// APDF (Channel A predriver fault flag)     (R/W) (bit 3)     0: Normal operation
//                                                             1: Channel A predriver fault
//                                                             (Write a '0' to this bit to clear the fault and resume operation)
// BOCP (Channel B overcurrent flag)         (R/W) (bit 2)     0: Normal operation
//                                                             1: Channel B overcurrent shutdown
//                                                             (Write a '0' to this bit to clear the fault and resume operation)
// AOCP (Channel A overcurrent flag)         (R/W) (bit 1)     0: Normal operation
//                                                             1: Channel A overcurrent shutdown
//                                                             (Write a '0' to this bit to clear the fault and resume operation)
// OTS (Overtemperature shutdown flag)       (R)   (bit 0)     0: Normal operation
//                                                             1: Device has entered overtemperature shutdown
//                                                             (OTS bit will clear once temperature has fallen to safe levels
)
*/
enum reg_status_mask    {a_over_curr_flag_addr = 1, b_over_curr_flag_addr = 2, a_fault_flag_addr = 3, b_fault_flag_addr = 4, stall_flag_addr = 7};
enum reg_status_size    {a_over_curr_flag_size = 1, b_over_curr_flag_size = 1, a_fault_flag_size = 1, b_fault_flag_size = 1, stall_flag_size = 1}; 
// Clears STATUS register
void clear_status_reg()
{
   if(drv_debug)
      fprintf(RS232,"STATUS Register\t: ");
      
   // clear the register
   reg_status = 0b0000000000000000;
      // set write operation
   reg_status = set_register_bits(reg_status, op_type_addr, op_type_size, op_write); 
   // set register address
   reg_status = set_register_bits(reg_status, reg_type_addr, reg_type_size, type_status);
   
   // write register to SPI
   write_register(reg_status);
}
/*
// Clears STATUS register
void clear_stall_flag()
{
   if(drv_debug)
      fprintf(RS232,"STATUS Register\t: ");
      
   // set write operation
   reg_status = set_register_bits(reg_status, op_type_addr, op_type_size, op_write); 
   // set register address
   reg_status = set_register_bits(reg_status, reg_type_addr, reg_type_size, type_status);
   // clear stall flag
   reg_status = set_register_bits(reg_status, stall_flag_addr, stall_flag_size, 0);
   // write register to SPI
   write_register(reg_status);
}
// Clears STATUS register
void clear_b_fault_flag()
{
   if(drv_debug)
      fprintf(RS232,"STATUS Register\t: ");
      
   // set write operation
   reg_status = set_register_bits(reg_status, op_type_addr, op_type_size, op_write); 
   // set register address
   reg_status = set_register_bits(reg_status, reg_type_addr, reg_type_size, type_status);
   // clear channel b predriver fault flag
   reg_status = set_register_bits(reg_status, b_fault_flag_addr, b_fault_flag_size, 0);
   // write register to SPI
   write_register(reg_status);
}
// Clears STATUS register
void clear_a_fault_flag()
{
   if(drv_debug)
      fprintf(RS232,"STATUS Register\t: ");
      
   // set write operation
   reg_status = set_register_bits(reg_status, op_type_addr, op_type_size, op_write); 
   // set register address
   reg_status = set_register_bits(reg_status, reg_type_addr, reg_type_size, type_status);
   // clear channel a predriver fault flag
   reg_status = set_register_bits(reg_status, a_fault_flag_addr, a_fault_flag_size, 0);
   // write register to SPI
   write_register(reg_status);
}
// Clears STATUS register
void clear_b_over_curr_flag()
{
   if(drv_debug)
      fprintf(RS232,"STATUS Register\t: ");
      
   // set write operation
   reg_status = set_register_bits(reg_status, op_type_addr, op_type_size, op_write); 
   // set register address
   reg_status = set_register_bits(reg_status, reg_type_addr, reg_type_size, type_status);
   // clear channel b overcurrent fault flag
   reg_status = set_register_bits(reg_status, b_over_curr_flag_addr, b_over_curr_flag_size, 0);
   // write register to SPI
   write_register(reg_status);
}
// Clears STATUS register
void clear_a_over_curr_flag()
{
   if(drv_debug)
      fprintf(RS232,"STATUS Register\t: ");
      
   // set write operation
   reg_status = set_register_bits(reg_status, op_type_addr, op_type_size, op_write); 
   // set register address
   reg_status = set_register_bits(reg_status, reg_type_addr, reg_type_size, type_status);
   // clear channel a overcurrent fault flag
   reg_status = set_register_bits(reg_status, a_over_curr_flag_addr, a_over_curr_flag_size, 0);
   // write register to SPI
   write_register(reg_status);
}
*/
// Sets driver parameters
void set_driver()
{
   // dead_time         :  dtime_400ns 
   //                      dtime_450ns 
   //                      dtime_650ns 
   //                      dtime_850ns
   // curent_amp_gain   :  gain_5
   //                      gain_10
   //                      gain_20
   //                      gain_40
   // src_stall_detect  :  stall_int
   //                      stall_ext
   // microstepping     :  full_step
   //                      half_step
   //                      _4x
   //                      _8x
   //                      _16x
   //                      _32x
   //                      _64x
   //                      _128x
   //                      _256x
   // single_step_out   :  no_action
   //                      one_step
   // dir_control       :  dir_pin
   //                      inv_dir_pin
   // motor_state       :  motor_disable
   //                      motor_enable
   set_ctrl_reg(dtime_400ns, gain_10, stall_int, _256x, no_action, dir_pin, motor_enable);
   //set_dead_time(dtime_400ns);
   //set_amp_gain(gain_5);
   //set_stall_detection(stall_int);
   //set_microstepping_mode(_64x);
   //set_rstep(no_action);
   //set_dir_mode(dir_pin);
   //set_motor_state(motor_enable);
   
   // emf_samp_thr      :  emf_50us
   //                      emf_100us
   //                      emf_200us
   //                      emf_300us
   //                      emf_400us
   //                      emf_600us
   //                      emf_800us
   //                      emf_1000us
   set_torque_reg(emf_100us, 65);
   //set_emf_samp_thr(emf_50us);
   //set_torque(200);
   
   // pwm_mode          :  int_indexer
   //                      ext_indexer
   set_off_reg(int_indexer, 18);
   //set_pwm_mode(int_indexer);
   //set_off_time(70);
   
   // adap_blank_state  :  disable_abt
   //                      enable_abt
   set_blank_reg(enable_abt, 125);
   //set_adap_blank_state(enable_abt);
   //set_blank_time(125);
   
   //decay_mode         :  force_slow_decay
   //                      slow_mixed_decay
   //                      force_fast_decay
   //                      mixed_decay
   //                      slow_auto_mixed_decay
   //                      auto_mixed_decay
   set_decay_reg(auto_mixed_decay, 6);
   //set_decay_mode(auto_mixed_decay);
   //set_decay_time(41);
   
   // emf_vol_div       :  divide_by_32
   //                      divide_by_16
   //                      divide_by_8
   //                      divide_by_4
   // emf_samp_mode     :  after_1_step
   //                      after_2_steps
   //                      after_4_steps
   //                      after_8_steps
   set_stall_reg(divide_by_8, after_1_step, 60);
   //set_emf_vol_div(divide_by_8);
   //set_emf_samp_mode(after_2_steps);
   //set_stall_thr(20);
   
   // hs_peak_cur       :  hs_50mA
   //                      hs_100mA
   //                      hs_150mA
   //                      hs_200mA
   // ls_peak_cur       :  ls_100mA
   //                      ls_200mA
   //                      ls_300mA
   //                      ls_400mA
   // hs_drive_time     :  hs_250ns
   //                      hs_500ns
   //                      hs_1us
   //                      hs_2us
   // ls_drive_time     :  ls_250ns
   //                      ls_500ns
   //                      ls_1us
   //                      ls_2us
   // ocp_deg_time      :  ocp_1us
   //                      ocp_2us
   //                      ocp_4us
   //                      ocp_8us
   // ocp_thr           :  ocp_250mV
   //                      ocp_500mV
   //                      ocp_750mV
   //                      ocp_1000mV
   set_drive_reg(hs_50mA, ls_100mA, hs_1us, ls_1us, ocp_1us, ocp_250mV);
   //set_hs_peak_cur(hs_200mA);
   //set_ls_peak_cur(ls_400mA);
   //set_hs_drive_time(hs_1us);
   //set_ls_drive_time(ls_1us);
   //set_ocp_deg_time(ocp_1us);
   //set_ocp_thr(ocp_250mV);
   
   clear_status_reg();
   //clear_stall_flag();
   //clear_b_fault_flag();
   //clear_a_fault_flag();
   //clear_b_over_curr_flag();
   //clear_a_over_curr_flag();
   
   delay_ms(100);
}

// Handles the messages of RS232 connection
void rs232_message()
{
   char command = fgetc(RS232);
   unsigned int i=0;
      
   switch (command)
   {
      case 'S':   unsigned int input_delay[3];
                  for(i=0; i<3; i++)
                  {
                     input_delay[i]=(unsigned)(fgetc(RS232)-48);
                  }
                  
                  delay=100*input_delay[0]+10*input_delay[1]+1*input_delay[2];
                  break;
      case 'D':   drv_debug = !drv_debug;
                  fprintf(RS232, "Driver Debug: %u", drv_debug);
                  break;
      case 'O':   unsigned int input_off[3];
                  for(i=0; i<3; i++)
                  {
                     input_off[i]=(unsigned)(fgetc(RS232)-48);
                  }
                  
                  unsigned int off = 100*input_off[0] + 10*input_off[1] + 1*input_off[2];
                  set_off_time(off);
                  break;
      case 'B':   unsigned int input_blank[3];
                  for(i=0; i<3; i++)
                  {
                     input_blank[i]=(unsigned)(fgetc(RS232)-48);
                  }
                  
                  unsigned int blank = 100*input_blank[0] + 10*input_blank[1] + 1*input_blank[2];
                  set_blank_time(blank);
                  break;
      default :   return; 
   }
}

#INT_RDA
void isr_rs232_message()
{
   clear_interrupt(INT_RDA);
   disable_interrupts(INT_RDA);
   // Receive the RS232 message
   reg_rs232_message = 1;
} 

void main()
{
   delay_ms(10);
   
   // Set I/O states of the ports
   //           76543210                  
   set_tris_a(0b00101100);       
   set_tris_b(0b11001111);

   // Set alternative pin functions
   set_pins();
   // Set SPI parameters
   set_SPI();
   // Set variables to default values
   set_variables();
   // Set driver
   set_driver();

   enable_interrupts(global);
   enable_interrupts(INT_RDA);
   
   while(true)
   {  
      if(reg_rs232_message)
      {   
         disable_interrupts(INT_RDA);
         
         reg_rs232_message=0;
         rs232_message();
         
         enable_interrupts(INT_RDA);
      }
      else
      {
         output_high(DRV_STEP);
         delay_us(10);
         output_low(DRV_STEP);
         delay_us(delay - 10);
      }
   } 
}
