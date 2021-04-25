#INCLUDE <30f6015.h> 
#DEVICE ADC=10                                                    // Configures the read_adc return size as 10 bit
#INCLUDE <math.h>

#FUSES NOWDT                                                      // No Watch Dog Timer 
#FUSES FRC                                                        // Internal Fast RC Oscillator
#FUSES NOCKSFSM                                                   // Clock Switching is disabled, fail Safe clock monitor is disabled 
#FUSES NOBROWNOUT                                                 // Not reset when brownout detected 
#FUSES PROTECT                                                    // Code is protected from reading 
#FUSES NOWRT                                                      // Program memory not write protected 
#FUSES NODEBUG                                                    // No Debug mode for ICD

#USE DELAY(clock=7370000)                                         // delay() func. adjusted for internal osc clock speed.
#USE RS232(stream=RS485,UART1,baud=38400,parity=N,bits=8,stop=1)  // Set UART1 as RS485 stream
#USE RS232(stream=RS232,UART2,baud=38400,parity=N,bits=8,stop=1)  // Set UART2 as RS232 stream

//Register for Reset Info
#WORD RCON                 = 0x740           //Reset control register
#BIT RCON_POR              = RCON.0          //POR
#BIT RCON_BOR              = RCON.1          //BOR
#BIT RCON_IDLE             = RCON.2
#BIT RCON_SLEEP            = RCON.3
#BIT RCON_WDTO             = RCON.4
#BIT RCON_SWDTEN           = RCON.5
#BIT RCON_SWR              = RCON.6
#BIT RCON_EXTR             = RCON.7         //MCLR Reset during normal operation
#BIT RCON_BGST             = RCON.13
#BIT RCON_IOPUWR           = RCON.14
#BIT RCON_TRAPR            = RCON.15

// Registers of quadrature encoder interface module
// Refer to Microchip dsPIC30f Family Reference Manual for detailed information
#WORD QEI_QEICON        = 0x122           // Control and status register QEICON allows control of the QEI operation and status flags indicating the module state.
#WORD QEI_DFLTCON       = 0x124           // Digital filter control register DFLTCON allows control of the digital input filter operation.
#WORD QEI_POSCNT        = 0x126           // Position count register POSCNT allows reading and writing of the 16-bit position counter.
#WORD QEI_MAXCNT        = 0x128           // Maximum count register MAXCNT holds a value that will be compared to the POSCNT counter in some operations.
// Bits of the QEICON register
#BIT QEI_QEICON_CNTERR  = QEI_QEICON.15   // Count error status flag bit
#BIT QEI_QEICON_QEISIDL = QEI_QEICON.13   // Stop in idle mode bit
#BIT QEI_QEICON_INDEX   = QEI_QEICON.12   // Index pin state status bit (read only)
#BIT QEI_QEICON_UPDN    = QEI_QEICON.11   // Position counter direction status bit (read only)
#BIT QEI_QEICON_QEIM2   = QEI_QEICON.10   // Quadrature encoder interface mode select bits
#BIT QEI_QEICON_QEIM1   = QEI_QEICON.9
#BIT QEI_QEICON_QEIM0   = QEI_QEICON.8
#BIT QEI_QEICON_SWPAB   = QEI_QEICON.7    // Phase A and phase B input swap select bit 
#BIT QEI_QEICON_PCDOUT  = QEI_QEICON.6    // Position counter direction state output rnable bit
#BIT QEI_QEICON_TQGATE  = QEI_QEICON.5    // Timer gated time accumulation rnable bit
#BIT QEI_QEICON_TQCKPS1 = QEI_QEICON.4    // Timer input clock prescale select bits
#BIT QEI_QEICON_TQCKPS0 = QEI_QEICON.3
#BIT QEI_QEICON_POSRES  = QEI_QEICON.2    // Position counter reset enable bit
#BIT QEI_QEICON_TQCS    = QEI_QEICON.1    // Timer clock source select bit
#BIT QEI_QEICON_UDSRC   = QEI_QEICON.0    // Position counter direction selection control bit
// Bits of the DFLTCON register
#BIT QEI_DFLTCON_IMV1   = QEI_DFLTCON.10  // Index match value bits
#BIT QEI_DFLTCON_IMV0   = QEI_DFLTCON.9   // These bits allow the user to specify the state of the QEA and QEB input pins during an Index pulse when the POSCNT register is to be reset.
#BIT QEI_DFLTCON_CEID   = QEI_DFLTCON.8   // Count error interrupt disable bit
#BIT QEI_DFLTCON_QEOUT  = QEI_DFLTCON.7   // QEA/QEB/IND digital filter output enable bit 
#BIT QEI_DFLTCON_QECK2  = QEI_DFLTCON.6   // QEA/QEB/IND digital filter clock divide select bits
#BIT QEI_DFLTCON_QECK1  = QEI_DFLTCON.5     
#BIT QEI_DFLTCON_QECK0  = QEI_DFLTCON.4

// Registers of the motor control PWM module
// Refer to Microchip dsPIC30f Family Reference Manual for detailed information
#WORD PWM_PTCON         = 0x1C0           // PWM Time base control register
#WORD PWM_PTPER         = 0x1C4           // PWM Time base period register
#WORD PWM_PWMCON1       = 0x1C8           // PWM Control register #1
#WORD PWM_PWMCON2       = 0x1CA           // PWM Control register #2
#WORD PWM_PDC1          = 0x1D6           // PWM Duty cycle register #1
#WORD PWM_IEC2          = 0x090           // PWM Interrupt Enable Control Register
#WORD PWM_IFS2          = 0x088           // PWM Interrupt Flag Status Register
// Bits of the PTCON register
#BIT PWM_PTCON_PTEN     = PWM_PTCON.15    // PWM Time base timer enable bit
#BIT PWM_PTCON_PTOPS3   = PWM_PTCON.7     // PWM Time base output postscale select bits 
#BIT PWM_PTCON_PTOPS2   = PWM_PTCON.6
#BIT PWM_PTCON_PTOPS1   = PWM_PTCON.5
#BIT PWM_PTCON_PTOPS0   = PWM_PTCON.4
#BIT PWM_PTCON_PTCKPS1  = PWM_PTCON.3     // PWM Time base input clock prescale select bits
#BIT PWM_PTCON_PTCKPS0  = PWM_PTCON.2    
#BIT PWM_PTCON_PTMOD1   = PWM_PTCON.1     // PWM Time base mode select bits
#BIT PWM_PTCON_PTMOD0   = PWM_PTCON.0
// Bits of the PWMCON1 register
#BIT PWM_PWMCON1_PMOD1  = PWM_PWMCON1.8   // PWM1 I/O pin pair mode bit
#BIT PWM_PWMCON1_PEN1H  = PWM_PWMCON1.4   // PWM1H I/O pin enable bit
#BIT PWM_PWMCON1_PEN1L  = PWM_PWMCON1.0   // PWM1L I/O pin enable bit
// Bits of the PWMCON2 register  
#BIT PWM_PWMCON2_IUE    = PWM_PWMCON2.2   // Immediate update enable bit
#BIT PWM_PWMCON2_OSYNC  = PWM_PWMCON2.1   // Output override synchronization bit
#BIT PWM_PWMCON2_UDIS   = PWM_PWMCON2.0   // PWM update disable bit
// Bits of the IEC2 register
#BIT PWM_IEC2_PWMIE     = PWM_IEC2.7      // PWM Interrupt enable bit 
// Bits of the IFS2 register
#BIT PWM_IFS2_PWMIF     = PWM_IFS2.7      // PWM Interrupt flag status bit    

// Registers of the UART1 module
#WORD UART_IFS0          = 0x088           // UART Interrupt Flag Status Register
// Bits of the IFS0 register
#BIT UART_IFS0_U1RXIF    = UART_IFS0.9     // UART Receiver Interrupt flag status bit  
#BIT UART_IFS0_U1TXIF    = UART_IFS0.10    // UART Transmiter Interrupt flag status bit  

// Led pins
#DEFINE LED          PIN_B8               // Led used in debugging

// Laser pins
#DEFINE LAS_1        PIN_B0               // Control pin of the first laser
#DEFINE LAS_2        PIN_B1               // Control pin of the second laser
#DEFINE LAS_3        PIN_D6               // Control pin of the third laser

//Parallel Port Pin Assignments
#DEFINE PP_D0        PIN_D8               // Parallel port data pin first bit
#DEFINE PP_D1        PIN_D9               // Parallel port data pin second bit
#DEFINE PP_D2        PIN_D0               // Parallel port data pin third bit
#DEFINE PP_D3        PIN_C13              // Parallel port data pin fourth bit
#DEFINE PP_D4        PIN_C14              // Parallel port data pin fifth bit
#DEFINE PP_D5        PIN_D1               // Parallel port data pin sixth bit
#DEFINE PP_D6        PIN_D2               // Parallel port data pin seventh bit
#DEFINE PP_D7        PIN_D3               // Parallel port data pin eigth bit
#DEFINE PP_STR       PIN_D10              // Parallel port strobe pin
#DEFINE PP_ACK       PIN_D4               // Parallel port acknowledge pin used to 
#DEFINE PP_RDY       PIN_D5               // Parallel port ready pin

// Pins used to control digital potentiometer
#DEFINE DP_CS        PIN_G9               // Digital potentiometer chip select pin assignment (active low)
#DEFINE DP_UD        PIN_G8               // Digital potentiometer up/down control pin assignment

// Pins used to control motor driver
#DEFINE MD_MS1       PIN_F0               // Step resolution select pin 1 assignment
#DEFINE MD_MS2       PIN_D7               // Step resolution select pin 2 assignment
#DEFINE MD_SR        PIN_F1               // Active mode (synchronous rectification) input pin assignment
#DEFINE MD_RESET     PIN_E0               // Reset input pin assignment (active low)
#DEFINE MD_STEP      PIN_E1               // Step input pin assignment
#DEFINE MD_ENABLE    PIN_E2               // Enable input pin assignment (active low)
#DEFINE MD_SLEEP     PIN_E3               // Sleep input pin assignment (active low)
#DEFINE MD_DIR       PIN_E4               // Direction input pin assignment
#DEFINE MD_SW        PIN_D11              // Homing switch input pin assignment

// Pins used to control FRAM
#DEFINE FR_CS        PIN_G7               // FRAM chip select pin assignment (active low)
#DEFINE FR_WP        PIN_G6               // FRAM write protection pin assignment (active low)
#DEFINE FR_SCK       PIN_E5               // FRAM serial clock pin assignment
#DEFINE FR_SI        PIN_E6               // FRAM serial input pin assignment
#DEFINE FR_SO        PIN_E7               // FRAM serial output pin assignment

// Speed ramp states
#DEFINE HOME         0
#DEFINE ACCEL        1
#DEFINE DECEL        2
#DEFINE RUN          3
#DEFINE POS          4

// PWM module operating modes
#DEFINE FREE         0
#DEFINE SINGLE       1

int1           debug_mode_dp     = 0;              // Debug state of digital potentiometer
int1           debug_mode_pp     = 0;              // Debug state of parallel port communication
int1           debug_mode_fr     = 0;              // Debug state of FRAM
int1           debug_mode_md     = 0;              // Debug state of motor driver
int1           debug_mode_pwm    = 0;              // Debug state of motor control PWM module
int1           debug_mode_qei    = 0;              // Debug state of quadrature encoder

unsigned int   dp_tap_limit      = 32;             // Digital potentiometer maximum tap level
unsigned int   dp_voltage_limit  = 1023;           // Digital potentiometer maximum voltage level

unsigned int   mt_voltage_limit  = 803;            // Motor torque control voltage is limited to 4V (which is defined in driver specs)
unsigned int   mt_percent_trip   = 70;             // Motor torque percent while system is moving
unsigned int   mt_percent_rest   = 28;             // Motor torque percent while system is stationary

unsigned int   md_conv_const     = 0;              // Constant used to convert encoder count to motor step
unsigned int   md_move_range     = 400;            // Movement range of the device
unsigned int   md_home_pos       = 45;             // Sets the number of encoder counts between the home position and homing switch position
unsigned int   md_end_pos        = 1000;           // Sets the number of encoder counts between the home position and the end of movement range
unsigned int   md_end_offset     = 50;             // Offset of the maximum count number from the end position
unsigned int   md_home_offset    = 100;            // Offset of the maximum count number from the home position
unsigned int   md_min_distance   = 70;             // Minimum distance between symmetricaly moving lasers
unsigned int   md_target_count   = 0;              // Target position count

unsigned int   md_accel          = 3000;           // Acceleration of the motor (0.01 mm/s2)
unsigned int   md_decel          = 3000;           // Deceleration of the motor (0.01 mm/s2)

unsigned int   md_min_delay      = 100;            // Minimum time delay (max speed)
unsigned char  md_run_state      = 0;              // What part of the speed ramp we are in.
unsigned int   md_decel_lim      = 0;              // What step_pos to start decelaration
unsigned int   md_accel_lim      = 0;              // What step_pos to end accelaration
unsigned int   md_decel_count    = 0;              // Counter used when decelerateing to calculate step_delay.
unsigned int   md_pos_iter       = 0;              // Number of iterations performed during positioning
int            md_error          = 0;              // Positioning error
unsigned int   md_backlash       = 0;              // Backlash of the positioning system

unsigned int   md_cc_step        = 0;              // Motor count used in conversion constant conversion
unsigned int   md_cc_count       = 0;              // Encoder count used in conversion constant conversion

unsigned int   fr_serial_no      = 0;              // External adress of product serial no
unsigned int   fr_move_range     = 13;             // External adress of movement range
unsigned int   fr_home_pos       = 15;             // External adress of home position
unsigned int   fr_end_pos        = 17;             // External addres of end position
unsigned int   fr_conv_const     = 19;             // External adress of conversion constant
unsigned int   fr_backlash       = 21;             // External adress of backlash compensation
unsigned int   fr_comm_type      = 23;             // External adress of communication type
unsigned int   fr_last_pos       = 24;             // External adress of last position
unsigned int   fr_pos_table      = 26;             // External adress of position table

unsigned int   pp_str_check      = 1000;           // Parallel port strobe signal filter iteration number
unsigned int   pp_str_delay      = 1;              // Parallel port strobe signal filter pause time (ms)
unsigned int   pp_ack_delay      = 10;            // Parallel port acknowledge signal pause time (ms)
unsigned int   pp_rdy_delay      = 10;            // Parallel port ready signal pause time (ms)

int1           reg_comm_type     = 0;              // Communication type register
int1           reg_pp_stop       = 0;              // Parallel port emergency stop signal register
int1           reg_md_home       = 0;              // Homing action register
int1           reg_md_home_return= 0;              // Home correction return register
int1           reg_rs232_message = 0;              // RS232 message flag
int1           reg_md_cc_sample  = 0;              // Special case register for conversion constant calculation

#DEFINE ACC_15

#IFDEF ACC_10
unsigned int   md_max_acc_lim    = 116;             // Number of steps before we hit max speed. acc=10000 dec=10000 
unsigned int const delays[116]={2449,1015,779,656,578,523,481,447,420,397,378,361,346,333,322,311,302,293,285,277,271,264,258,253,247,243,238,234,229,226,222,218,215,212,209,206,203,200,197,195,192,190,188,186,184,182,180,
178,176,174,172,171,169,167,166,164,163,162,160,159,157,156,155,154,152,151,150,149,148,147,146,145,144,143,142,141,140,139,138,137,136,135,134,133,132,131,130,129,128,127,126,125,124,123,122,121,120,119,118,117,116,115,
114,113,112,111,110,109,108,107,106,105,104,103,102,101};
#ENDIF

#IFDEF ACC_15
unsigned int   md_max_acc_lim    = 88;             // Number of steps before we hit max speed. acc=15000 dec=15000
unsigned int const delays[88]={2000,828,636,536,472,427,393,365,343,325,309,295,283,272,263,254,246,239,233,226,221,216,211,206,202,198,194,191,187,184,181,178,175,173,170,168,166,163,161,159,157,155,153,152,150,148,147,
145,144,142,141,139,138,137,135,134,133,132,131,130,129,128,126,125,124,123,122,121,120,119,118,117,116,115,114,113,112,111,110,109,108,107,106,105,104,103,102,101};
#ENDIF

#IFDEF ACC_20
unsigned int   md_max_acc_lim    = 71;             // Number of steps before we hit max speed. acc=20000 dec=20000 
unsigned int const delays[71]={1732,717,551,464,409,370,340,316,297,281,267,255,245,236,227,220,213,207,201,196,191,187,183,179,175,172,168,165,162,159,157,154,152,150,147,145,143,141,140,138,136,134,133,131,130,128,127,
126,124,123,122,121,120,118,117,116,115,114,113,112,111,110,109,108,107,106,105,104,103,102,101};
#ENDIF

#IFDEF ACC_25
unsigned int   md_max_acc_lim    = 59;             // Number of steps before we hit max speed. acc=25000 dec=25000 
unsigned int const delays[59]={1549,642,492,415,366,331,304,283,266,251,239,228,219,211,203,197,191,185,180,175,171,167,163,160,157,153,150,148,145,143,140,138,136,134,132,130,128,126,125,123,122,120,119,117,116,115,114,
112,111,110,109,108,107,106,105,104,103,102,101};
#ENDIF

#IFDEF ACC_30
unsigned int   md_max_acc_lim    = 50;             // Number of steps before we hit max speed. acc=30000 dec=30000 
unsigned int const delays[50]={1414,586,449,379,334,302,278,258,243,229,218,209,200,192,186,180,174,169,164,160,156,153,149,146,143,140,137,135,132,130,128,126,124,122,120,119,117,115,114,113,111,110,108,107,106,105,104,
103,102,101};
#ENDIF

void move_pos(unsigned int16 position);
unsigned int16 qei_get_count();

// Initializes digital potentiometer
void dp_init() 
{
   output_high(DP_CS);                       // Chip select is active low so keep it disabled
   output_low(DP_UD);                        // Up/down control pin can be in any state
}
// Increases digital potentiometer by given tap
void dp_up(unsigned int tap)
{
   if(tap>dp_tap_limit)       // Tap cannot be more than tap limit   
      tap=dp_tap_limit;
      
   output_high(DP_UD);        // Set U/D to high before activating digital potentiometer
   delay_us(1);
   output_low(DP_CS);         // Activate digital potentiometer
   delay_us(1);
   
   unsigned int i;
   for(i=0;i<tap;i++)
   {
      output_low(DP_UD);      // Low part of pulse
      delay_us(1);
      output_high(DP_UD);     // High part of pulse
      delay_us(1);
      output_low(DP_UD);      // End one square pulse
      delay_us(1); 
   }
   
   output_high(DP_CS);        // Deactivate digital potentiometer  
}
// Decreases digital potentiometer by given tap
void dp_down(unsigned int tap)
{
   if(tap>dp_tap_limit)       // Tap cannot be more than tap limit   
      tap=dp_tap_limit;
      
   output_low(DP_UD);         // Set U/D to low before activating digital potentiometer
   delay_us(1);
   output_low(DP_CS);         // Activate digital potentiometer
   delay_us(1);            
   
   unsigned int i;
   for(i=0;i<tap;i++)
   {
      output_high(DP_UD);     // High part of pulse
      delay_us(1);
      output_low(DP_UD);      // End one square pulse
      delay_us(1);  
   }
   
   output_high(DP_CS);        // Deactivate digital potentiometer
}
// Sets digital potentiometer to given voltage
void dp_set(unsigned int voltage)
{
   unsigned int iteration_limit=3;                                   // Maximum number of iterations to be performed to set voltage
   unsigned int iteration=0;                                         // Number of iterations performed to set voltage
   unsigned int region=15;                                           // We want the voltage to be inside this region (+/-) 
   unsigned int tap_desired=voltage*dp_tap_limit/dp_voltage_limit;   // Find the desired potentiometer tap
   unsigned int voltage_current=read_adc();                          // Read the current voltage
   unsigned int voltage_diff=abs(voltage-voltage_current);           // Find the difference between the current and desired voltage 
   unsigned int tap_current;                                         // Current tap value
   
   if(debug_mode_dp)
   {
      fprintf(RS232,"\n\r\tVoltage Desired: %u",voltage);
      fprintf(RS232,"\n\r\tTap Desired: %u",dp_tap_limit*voltage/dp_voltage_limit);
      fprintf(RS232,"\n\r\tVoltage Current: %u",voltage_current);
      fprintf(RS232,"\n\r\tTap Current: %u",dp_tap_limit*voltage_current/dp_voltage_limit);
      fprintf(RS232,"\n\r\tVoltage Difference: %u",voltage_diff);
   }
   
   while(voltage_diff>region)                                        // Try until voltage is found
   {                                                                 // or iteration limit is reached 
      if(iteration <iteration_limit)                     
      {
         iteration++;                                                // Set iteration number
         tap_current=dp_tap_limit*voltage_current/dp_voltage_limit;  // Convert new voltage to tap

         if(tap_desired>tap_current)                                 // Set the digital potentiometer to desired tap
         {
            dp_up(tap_desired-tap_current);
         }
         else if(tap_desired<tap_current)
         {
            dp_down(tap_current-tap_desired);
         }
         else                                                        // If tap level is same as the previous one and it still outside the region
         {                                                           // then go one step up or down according to desired voltage
            if(voltage>voltage_current)
               dp_up(1);
            else
               dp_down(1);
         }
         
         delay_ms(2);
            
         voltage_current=read_adc();                                 // Read the new voltage
         
         if(voltage>voltage_current)                                 // Find the difference between the new and desired voltages for next iteration check
            voltage_diff=voltage - voltage_current;
         else
            voltage_diff=voltage_current - voltage;
         
         if(debug_mode_dp)
         {
            fprintf(RS232,"\n\rITERATION: %u",iteration);
            fprintf(RS232,"\n\r\tTap Current: %u",dp_tap_limit*voltage_current/dp_voltage_limit);
            fprintf(RS232,"\n\r\tVoltage Current : %u",voltage_current);
            fprintf(RS232,"\n\r\tVoltage Difference: %u",abs(voltage-voltage_current));
         }
      }
      else
      {
         if(debug_mode_dp)
            fprintf(RS232,"\n\rIteration limit reached");
         return;
      }
   }
   
   if(debug_mode_dp)
   {
      fprintf(RS232,"\n\rFOUND!");
      fprintf(RS232,"\n\r\tTap Current : %u",dp_tap_limit*voltage_current/dp_voltage_limit);
      fprintf(RS232,"\n\r\tVoltage Current : %u",read_adc());
      fprintf(RS232,"\n\r\tVoltage Difference: %u",abs(voltage-read_adc()));
   }
}

// Returns current motor torque in percent (%0-100)
unsigned int md_mt_get()
{
   unsigned int analog=read_adc();                                         // Read the voltage level
   unsigned int percent=100*(unsigned int32)analog/mt_voltage_limit;       // Convert voltage to percent
   
   if(debug_mode_dp)
      fprintf(RS232,"\n\r\tPercent Current: %u\n\r",percent);
      
   return percent;  
}
// Sets motor torque approximate to desired percent and returns the actual percent (%0-100)
unsigned int md_mt_set(unsigned int percent)
{
   // percent must be between 0 and 100
   if(percent>100)
      percent=100;
      
   if(debug_mode_dp)
   {
      fprintf(RS232,"\n\r\tPercent Desired:");
      fprintf(RS232,"%u",percent);
   }
   
   unsigned int voltage=(unsigned int32)percent*mt_voltage_limit/100;      // convert percent to voltage
   dp_set(voltage);                                      // set digital potentiometer
   
   return md_mt_get();
}
// Initializes motor driver
void md_init()
{
   md_mt_set(mt_percent_rest);               // Set motor torque before initialization of motor driver

   output_high(MD_RESET);                    // Reset is active low so keep it disabled
   output_high(MD_SLEEP);                    // Sleep is active low so keep it disabled
   output_low(MD_SR);                        // Activate synchronous rectification
   output_high(MD_DIR);                      // Direction control pin can be in any state
   output_high(MD_MS1);                      // MS1 and MS2 high enables 8x microstepping mode
   output_high(MD_MS2);
   output_low(MD_ENABLE);                    // Enable is active so keep it high
   output_low(MD_STEP);                      // A low-to-high transition advances the motor one increment so keep step input low
}

// Initializes external EEPROM (FRAM)
void fr_init() 
{
   output_high(FR_CS);     // Chip select is active low when it is high the device enters low-power standby mode so initialy keep it high
   output_low(FR_WP);      // Write protection pin is active low and prevents write operations to the status register so initially keep it low
   output_low(FR_SI);      // Serial input pin is driven to high logic state during communication so initially keep it low
   output_low(FR_SCK);     // Inputs are latched on the rising edge and outputs occur on the falling edge so initially serial clock can be in any state 
}
// Writes to external EEPROM (FRAM) and takes 16 bit address and 8 bit data as parameters
void fr_write_byte(unsigned int16 address, unsigned int8 data) 
{
   unsigned int8 cmd[4];
   unsigned int8 i;
   unsigned int8 wren;
   unsigned int8 write;
   unsigned int8 wrdi;
   
   //     76543210    
   wren=0b00000110;
   //      76543210 
   write=0b00000010;
   //     76543210
   wrdi=0b00000100;
   
   cmd[0]=data;
   cmd[1]=address;
   cmd[2]=address/256;
   cmd[3]=write;
   
   //Sample on Rising Edge of EEPROM_CLK
   //Clock Period=4usec(2usec high, 2usec low)
   output_low(FR_CS);
   delay_us(1);
   for(i=0; i<8; ++i)
   {
      output_bit(FR_SI, shift_left(&wren,1,0));
      delay_us(1);
      output_high(FR_SCK);
      delay_us(1);
      output_low(FR_SI);
      delay_us(1);
      output_low(FR_SCK);
      delay_us(1);
   }
   delay_us(1);
   output_high(FR_CS);
   delay_us(1);
   
   output_low(FR_CS);
   delay_us(1);
   for(i=0; i<32; ++i)
   {
      output_bit(FR_SI, shift_left(cmd,4,0));
      delay_us(1);
      output_high(FR_SCK);
      delay_us(1);
      output_low(FR_SI);
      delay_us(1);
      output_low(FR_SCK);
      delay_us(1);
   }
   delay_us(1);
   output_high(FR_CS);
   delay_us(1);
   
   output_low(FR_CS);
   delay_us(1);
   for(i=0; i<8; ++i)
   {
      output_bit(FR_SI, shift_left(&wrdi,1,0));
      delay_us(1);
      output_high(FR_SCK);
      delay_us(1);
      output_low(FR_SI);
      delay_us(1);
      output_low(FR_SCK);
      delay_us(1);
   }
   delay_us(1);
   output_high(FR_CS);
   delay_us(1);
}
// Reads from external EEPROM (FRAM) and takes 16 bit address and 8 bit data as parameters
unsigned int8 fr_read_byte(unsigned int16 address) 
{
   unsigned int8 cmd[3];
   unsigned int8 i,data;
   unsigned int8 read;
   
   //     76543210 
   read=0b00000011;
   
   //Sample on Rising Edge of EEPROM_CLK
   //Read on Rising Edge of EEPROM_CLK
   //Clock Period=4usec(2usec high, 2usec low)
   cmd[0]=address;
   cmd[1]=address/256;
   cmd[2]=read;

   output_low(FR_CS);
   delay_us(1);
   for(i=0; i<24; ++i)
   {
      output_bit(FR_SI, shift_left(cmd,3,0));
      delay_us(1);
      output_high(FR_SCK);
      delay_us(1);
      output_low(FR_SI);
      delay_us(1);
      output_low(FR_SCK);
      delay_us(1);
   }
   for(i=0; i<8; ++i)
   {
      delay_us(1);
      shift_left(&data,1,input(FR_SO));
      output_high(FR_SCK);
      delay_us(1);
      delay_us(1);
      output_low(FR_SCK);
      delay_us(1);
   }
   output_high(FR_CS);
   delay_us(1);
   
   return(data);
}
// Writes to external EEPROM (FRAM) and takes 16 bit address and 16 bit data as parameters
void fr_write(unsigned int16 address,unsigned int16 data)
{
   unsigned int8 data_high;
   unsigned int8 data_low;
   data_low=(unsigned int8)data;
   data_high=(unsigned int8)(data>>8);
   
   fr_write_byte(address,data_low);
   fr_write_byte(address+1,data_high);
   
   return;
}
// Reads from external EEPROM (FRAM) and takes 16 bit address and 16 bit data as parameters
unsigned int16 fr_read(unsigned int16 address)
{
   unsigned int16 data_buffer;
   unsigned int16 data_low;
   unsigned int16 data_high;
   
   data_low=fr_read_byte(address);
   data_high=fr_read_byte(address+1);
   
   data_buffer=(data_high<<8)+data_low;
   
   return data_buffer;
}

// Returns serial number of the device via RS232
void data_get_serial_no()
{
   if(!debug_mode_fr) 
      return;
      
   unsigned int8 i=0;
   unsigned int8 input[13];
   
   fprintf(RS232, "Serial No:");
   for(i=0; i<13; i++)
   {
      input[i]=fr_read_byte(fr_serial_no+i);
      fputc(input[i]+48,RS232);
   }
   fprintf(RS232, "\n\r");
}
// Returns movement range of the device
unsigned int16 data_get_move_range()
{
   unsigned int16 range = fr_read(fr_move_range);
   
   if(debug_mode_fr) 
      fprintf(RS232, "Movement Range: %u mm\n\r", range);
   
   return range;
}
// Returns home position
unsigned int16 data_get_home_pos()
{
   unsigned int16 home_pos = fr_read(fr_home_pos);
   
   if(debug_mode_fr) 
      fprintf(RS232, "Home Distance: %u counts\n\r", home_pos);
      
   return home_pos;
}
// Returns end position
unsigned int16 data_get_end_pos()
{
   unsigned int16 end = fr_read(fr_end_pos);
   
   if(debug_mode_fr)
      fprintf(RS232, "End Distance: %u counts\n\r", end);
   
   return end;
}
// Returns positioning velocity
unsigned int16 data_get_conv_const()
{
   unsigned int16 conv_const = fr_read(fr_conv_const);
   
   if((conv_const < 12000)||(conv_const > 13000))
      conv_const = 12500;
   
   if(debug_mode_fr) 
      fprintf(RS232, "Conversion Constant: %u \n\r", conv_const);
   
   return conv_const;
}
// Returns the backlash of the system
unsigned int16 data_get_backlash()
{
   unsigned int16 backlash = fr_read(fr_backlash);

   if(debug_mode_fr) 
      fprintf(RS232, "Backlash: %u counts\n\r", backlash);
   
   return backlash;
}
// Returns communication type
unsigned int8 data_get_comm_type()
{
   unsigned int8 comm_type=fr_read_byte(fr_comm_type);
   
   if(debug_mode_fr)
      switch (comm_type)
      {
         case 0   :  fprintf(RS232,"Communication Type: Parallel Port\n\r");
                     break;
         case 1   :  fprintf(RS232,"Communication Type: Serial Port\n\r");
                     break;
         default  :  fprintf(RS232,"Communication Type: Undefined\n\r");
                     break;
      }
      
   return comm_type;
}
// Returns last position
unsigned int16 data_get_last_pos()
{
   unsigned int16 position=fr_read(fr_last_pos);
   
   if(debug_mode_fr) 
      fprintf(RS232, "Last Position: %u mm\n\r", position);
   
   return position;
}
// Returns the position in the given index
unsigned int16 data_get_pos(unsigned int8 index)
{
   unsigned int16 position=fr_read(fr_pos_table+index*2);
   
   if(debug_mode_fr)
      fprintf(RS232,"Index: %u, Position: %u mm\n\r", index, position);
   
   return position;
}
// Returns position table via RS232
void data_get_pos_table()
{
   debug_mode_fr=1;
   
   unsigned int input;
   unsigned int i;
   unsigned int index;
   
   input=(unsigned)(fgetc(RS232)-48);
   fputc(input+48,RS232);

   if(input>9)
      input=9;

   fprintf(RS232,"\n\r");
   
   for(i=0; i<10; i++)
   {
      index=input*10+i;
      data_get_pos(index);
   }
   
   debug_mode_fr=0;
}
// Returns all system data via RS232
void data_get_sys_data()
{
   fprintf(RS232,"MODESIS LASER POSITIONING SYSTEM\n\n\r");
   debug_mode_fr=1;
   data_get_serial_no();
   data_get_move_range();
   data_get_home_pos();
   data_get_end_pos();
   data_get_conv_const();
   data_get_backlash();
   data_get_comm_type();
   data_get_last_pos();
   debug_mode_fr=0;
   fprintf(RS232,"\n\r");
}
// Returns the reset register state of he microcontroller
void data_get_reset_state()
{
   fprintf(RS232,"RCON REGISTER VALUE(10'luk Tabanda):  ");
   fprintf(RS232,"%u",RCON);
   fprintf(RS232,"\n\r");
   fprintf(RS232,"RCON REGISTER BITLERI:");
   fprintf(RS232,"\n\r");
   
   fprintf(RS232,"POR:    ");
   fprintf(RS232,"%u",RCON_POR);
   fprintf(RS232,"\n\r");
   
   fprintf(RS232,"BOR:    ");
   fprintf(RS232,"%u",RCON_BOR);
   fprintf(RS232,"\n\r");
   
   fprintf(RS232,"IDLE:   ");
   fprintf(RS232,"%u",RCON_IDLE);
   fprintf(RS232,"\n\r");
   
   fprintf(RS232,"SLEEP:  ");
   fprintf(RS232,"%u",RCON_SLEEP);
   fprintf(RS232,"\n\r");
   
   fprintf(RS232,"WDTO:   ");
   fprintf(RS232,"%u",RCON_WDTO);
   fprintf(RS232,"\n\r");
   
   fprintf(RS232,"SWDTEN: ");
   fprintf(RS232,"%u",RCON_SWDTEN);
   fprintf(RS232,"\n\r");
   
   fprintf(RS232,"SWR:    ");
   fprintf(RS232,"%u",RCON_SWR);
   fprintf(RS232,"\n\r");
   
   fprintf(RS232,"EXTR:   ");
   fprintf(RS232,"%u",RCON_EXTR);
   fprintf(RS232,"\n\r");
   
   fprintf(RS232,"BGST:   ");
   fprintf(RS232,"%u",RCON_BGST);
   fprintf(RS232,"\n\r");
   
   fprintf(RS232,"IOPUWR: ");
   fprintf(RS232,"%u",RCON_IOPUWR);
   fprintf(RS232,"\n\r");
   
   fprintf(RS232,"TRAPR:  ");
   fprintf(RS232,"%u",RCON_TRAPR);
   fprintf(RS232,"\n\r");
   
   fprintf(RS232,"Register Ekrana Yazdirilip Sifirlandi");
   fprintf(RS232,"\n\r");
   RCON = 0x0000;
}

// Sets serial nuber of the device
void data_set_serial_no()
{
   unsigned int8 i=0;
   unsigned int8 input[13];
   for(i=0; i<13; i++)
   {
      input[i]=(unsigned)(fgetc(RS232)-48);
      fr_write_byte(fr_serial_no+i, input[i]);
      fputc(input[i]+48,RS232);
   }
}
// Sets movement range of the device
void data_set_move_range()
{
   unsigned int8 i=0;
   unsigned int input[4];
   for(i=0;i<4;i++)
   {
      input[i]=(unsigned)(fgetc(RS232)-48);
      fputc(input[i]+48,RS232);
   }
   unsigned int range=1000*input[0]+100*input[1]+10*input[2]+1*input[3];
   
   if(range<400)
   {
      fprintf(RS232, "\n\rInvalid entry: Movement range cannot be smaller than 400 mm. Movement range is set to 400 mm.");
      range=400;
   }
   else if(range>2000)
   {
      fprintf(RS232, "\n\rInvalid entry: Movement range cannot be larger than 2000 mm. Movement range is set to 2000 mm.");
      range=2000;
   }
   
   if(range==2000)
      md_min_distance=120;
   else
      md_min_distance=70;
      
   fr_write(fr_move_range, range);
   md_move_range=range;
}
// Sets home position
void data_set_home_pos()
{
   unsigned int8 i=0;
   unsigned int input[4];
   for(i=0; i<4; i++)
   {
      input[i]=(unsigned)(fgetc(RS232)-48);
      fputc(input[i]+48,RS232);
   }
   unsigned int home_pos = 1000*input[0]+100*input[1]+10*input[2]+1*input[3];
   
   if(home_pos<10)
   {
      fprintf(RS232, "\n\rInvalid entry: Home position cannot be smaller than 10. Home position is not changed");
      return;
   }
   
   fr_write(fr_home_pos,home_pos);
   md_home_pos=home_pos;
}
// Sets end position
void data_set_end_pos()
{
   unsigned int8 i=0;
   unsigned int input[5];
   for(i=0; i<5; i++)
   {
      input[i]=(unsigned)(fgetc(RS232)-48);
      fputc(input[i]+48,RS232);
   }
   unsigned int end=10000*input[0]+1000*input[1]+100*input[2]+10*input[3]+1*input[4];
   
   if(end<md_home_pos)
   {
      end=data_get_home_pos()+50;
      fprintf(RS232, "\n\rInvalid entry: End position cannot be smaller than home position. End position is set to its default.");
   }
   
   fr_write(fr_end_pos,end);
   md_end_pos=end;
}
// Sets positioning velocity
void data_set_conv_const()
{
   md_conv_const = 12500;
   md_cc_step = 0;
   md_cc_count = 0;
   
   move_pos(md_min_distance);

   unsigned int cc_first_count = qei_get_count();
   
   reg_md_cc_sample =1;
   move_pos(md_move_range);
   
   unsigned int conv_const = ((float)(cc_first_count - md_cc_count)/(float)md_cc_step)*10000;

   fprintf(RS232, "Count: %u counts\n\r", cc_first_count-md_cc_count);
   fprintf(RS232, "Step: %u steps\n\r", md_cc_step);
   fprintf(RS232, "Conversion Constant: %u counts\n\r", conv_const);
  
   fr_write(fr_conv_const,conv_const);
   md_conv_const = conv_const;
}
// Sets backlash
void data_set_backlash()
{
   unsigned int8 i=0;
   unsigned int input[2];
   for(i=0; i<2; i++)
   {
      input[i]=(unsigned)(fgetc(RS232)-48);
      fputc(input[i]+48,RS232);
   }
   unsigned int backlash = 10*input[0]+1*input[1];
   
   fr_write(fr_backlash,backlash);
   md_backlash = backlash;
}
// Sets communication type
void data_set_comm_type()
{
   unsigned int8 input=(unsigned)(fgetc(RS232)-48);
   fputc(input+48,RS232);
   switch (input)
   {
      case 1   :  output_low(PP_ACK);
                  output_low(PP_RDY);
                  
                  fr_write_byte(fr_comm_type, 1);
                  break;
      case 0   :    
      default  :  output_high(PP_ACK);
                  output_high(PP_RDY);
                  
                  fr_write_byte(fr_comm_type, 0);
                  break;
   }
}
// Sets last position
void data_set_last_pos(unsigned int16 position)
{
   fr_write(fr_last_pos, position);
}
// Sets position to given index
void data_set_pos(unsigned int8 index, unsigned int16 position)
{
   fr_write(fr_pos_table+index*2, position);
}

// Initializes the quadrature encoder interface module by using default values
void qei_init()
{
   // Disable QEI module 
   QEI_QEICON_QEIM0=0;
   QEI_QEICON_QEIM1=0;
   QEI_QEICON_QEIM2=0;
   // Clear any count errors
   QEI_QEICON_CNTERR=0;
   // Continue module operation during sleep
   QEI_QEICON_QEISIDL=0;
   // QEA and QEB not swapped
   QEI_QEICON_SWPAB=0;
   // Normal I/O pin operation
   QEI_QEICON_PCDOUT=0;
   // Timer gated time accumulation disabled
   QEI_QEICON_TQGATE=0;
   // 1:1 timer input clock prescale
   QEI_QEICON_TQCKPS0=0;
   QEI_QEICON_TQCKPS1=0;
   // Index pulse does not reset the position counter
   QEI_QEICON_POSRES = 0;
   // Internal clock as timer clock source
   QEI_QEICON_TQCS = 0;
   // QEI_QEICON_UPDN defines timer counter (QEI_POSCNT) direction
   QEI_QEICON_UDSRC = 0;
   
   // Count error interrupts enabled
   QEI_DFLTCON_CEID = 0; 
   // Digital filter outputs enabled
   QEI_DFLTCON_QEOUT = 0; 
   // 1:256 clock divide for digital filter
   QEI_DFLTCON_QECK2 = 1;
   QEI_DFLTCON_QECK1 = 1;
   QEI_DFLTCON_QECK0 = 0;
   
   // Reset position counter
   QEI_POSCNT = 0x0000;
   
   // Set maximum count
   QEI_MAXCNT = 0xFFFF;
   
   // QEI module enabled in x4 mode with position counter reset by QEI_MAXCNT match
   QEI_QEICON_QEIM2 = 1;
   QEI_QEICON_QEIM1 = 1;
   QEI_QEICON_QEIM0 = 1;
   
   if(debug_mode_qei)
      fprintf(RS232,"Quadrature Encoder Initialized\n\n\r");
}
// Sets the position counter to given value
void qei_set_count(unsigned int16 value)
{ 
   QEI_POSCNT = value;
   
   if(debug_mode_qei)
      fprintf(RS232,"Quadrature Encoder Count Set: %u\n\n\r", value);
}
// Sets the maximum count to given value
void qei_set_max_count(unsigned int16 value)
{ 
   QEI_MAXCNT = value;
   
   if(debug_mode_qei)
      fprintf(RS232,"Quadrature Encoder Maximum Count Set: %u\n\n\r", value);
}
// Returns the value of the position counter
unsigned int16 qei_get_count(){ return QEI_POSCNT;}

// Sets the operating mode of the PWM module
void pwm_select_mode(unsigned int mode)
{
   if(mode == FREE)
   {
      // Select free running PWM time base mode
      PWM_PTCON_PTMOD1     = 0;
      PWM_PTCON_PTMOD0     = 0;
   }
   else if(mode == SINGLE)
   {
      // Select single event PWM time base mode
      PWM_PTCON_PTMOD1     = 0;
      PWM_PTCON_PTMOD0     = 1;
   }
}
// Initializes the PWM module by using default values
void pwm_init()
{
   // Disable PWM module
   PWM_PTCON_PTEN       = 0;
   
   // Select 1:1 output postscale
   PWM_PTCON_PTOPS3     = 0;
   PWM_PTCON_PTOPS2     = 0;
   PWM_PTCON_PTOPS1     = 0;
   PWM_PTCON_PTOPS0     = 0;
   // Select 1:1 input prescale
   PWM_PTCON_PTCKPS1    = 0;
   PWM_PTCON_PTCKPS0    = 0;
   
   // Set the operating mode of PWM module
   pwm_select_mode(FREE);

   // Select independent output mode for PWM1 I/O pair
   PWM_PWMCON1_PMOD1    = 1;
   // Set PWM1H pin as PWM output
   PWM_PWMCON1_PEN1H    = 1;
   
   // Synchronize PDC registers to the PWM time base
   PWM_PWMCON2_IUE      = 0;
   // Enable PWM interrupt
   PWM_IEC2_PWMIE       = 0;
   
   // Set time base period value
   PWM_PTPER            = 0;
   // Set time base duty cycle
   PWM_PDC1             = 0;

   if(debug_mode_md)
      fprintf(RS232,"PWM Module Initialized\n\n\r");
}
// Set PWM period time (us)
void pwm_set_period(unsigned int16 period)
{
   // PWM period
   unsigned int16 pwm_period; 
   
   // Set the PWM period
   if(period==0)
      pwm_period = 0;
   else
      pwm_period =  (7 * period / 400) + (3 * period / 40) + ( 7 * period / 4) - 1;
   
   // Updates from duty cycle and period buffer registers are disabled
   PWM_PWMCON2_UDIS = 1;
   // Set time base period value
   PWM_PTPER        = pwm_period;
   // Set time base duty cycle if PWM module is enabled (duty cycle > 0%)
   PWM_PDC1         = pwm_period;
   // Updates from duty cycle and period buffer registers are enabled
   PWM_PWMCON2_UDIS = 0;
}
// Disable PWM module
void pwm_disable()
{
   // Set duty cycle to zero
   PWM_PDC1=0;
   // Wait for the last signal
   delay_us(PWM_PTPER + 1 / 30);
   // Disable PWM module
   PWM_PTCON_PTEN = 0;
   // Set duty cycle to its initial value
   PWM_PDC1=PWM_PTPER;
   
   if(debug_mode_pwm)
      fprintf(RS232,"PWM Disabled\n\r");
}
// Enable PWM module
void pwm_enable()
{
   // Disable RS232 receive byte interrupt
   disable_interrupts(INT_RDA2);
   
   // Enable PWM module
   PWM_PTCON_PTEN = 1;
   // Start pwm cycle
   int1 reg_md_running = 1;
   unsigned int md_step_count = 0;
   
   while(reg_md_running)
   {
      if(PWM_IFS2_PWMIF)
      {
         // Clear the flag register
         PWM_IFS2_PWMIF = 0;
         
         md_step_count++;
      
         switch(md_run_state) 
         {
            case HOME:
               if(reg_md_home_return == 0)
               {
                  if(reg_md_home == 0)
                  {
                     if(input(MD_SW))
                     {
                        if(md_step_count < md_max_acc_lim)
                        {
                           md_decel_count = md_step_count;
                           pwm_set_period(delays[md_step_count]);
                        }
                        else
                        {
                           md_decel_count = md_max_acc_lim;
                           pwm_set_period(md_min_delay);
                        }
                     }
                     else
                     {
                        reg_md_home = 1;
                        md_decel_count--;
                        pwm_set_period(delays[md_decel_count]);
                     }
                  }
                  else
                  {
                     if(md_decel_count == 0)
                     {  
                        reg_md_home_return = 1;
                        output_low(MD_DIR);
                        pwm_set_period(1000);
                     }
                     else
                     {
                        md_decel_count--;
                        pwm_set_period(delays[md_decel_count]);
                     }
                  }
               }
               else
               {
                  if(input(MD_SW))
                  {
                     pwm_disable();
                     reg_md_running = 0;
                     
                     delay_ms(50);
                     qei_set_count(md_home_offset);
                     md_mt_set(mt_percent_rest);
                  }
               }
               break;
               
            case ACCEL:
               // Check if we should start deceleration.
               if(md_step_count >= md_accel_lim) 
               {
                  if(md_step_count == md_decel_lim)
                  {
                     md_decel_count--;
                     pwm_set_period(delays[md_decel_count]);
                     md_run_state = DECEL;
                  }
                  else if(md_decel_lim - md_step_count < 2)
                  {
                     pwm_set_period(delays[md_step_count]);
                  }
                  else
                  {
                     pwm_set_period(md_min_delay);
                     md_run_state = RUN;
                  }
               }
               else
               {
                  pwm_set_period(delays[md_step_count]);
               }
               break;
      
            case RUN:
               // Check if we should start decelration.
               if(md_step_count == md_decel_lim) 
               {
                  md_decel_count--;
                  pwm_set_period(delays[md_decel_count]);
                  md_run_state = DECEL;
               }
               break;
      
            case DECEL:
               // Check if we at last step
               if(md_decel_count == 0)
               {
                  pwm_disable();
                  md_pos_iter = 0;
                  md_run_state = POS;
                  
                  delay_ms(50);

                  md_error = md_target_count - qei_get_count();
                  
                  if(md_error > 1)
                     output_low(MD_DIR);
                  else
                     output_high(MD_DIR);
               }
               else
               {
                  md_decel_count--;
                  pwm_set_period(delays[md_decel_count]);
               }
               break;
               
            case POS:
               md_pos_iter++;
               
               delay_ms(50);
               
               if(reg_md_cc_sample)
               {
                  reg_md_cc_sample = 0;
                  delay_ms(50);
                  md_cc_step = md_step_count;
                  md_cc_count = qei_get_count();
               }

               int difference;
               
               if(md_error < 1)
                  difference = qei_get_count() - md_target_count;
               else
                  difference = md_target_count - qei_get_count();

               if((difference < 1)||(md_pos_iter > 50))
               {
                  pwm_disable();
                  reg_md_running = 0;
                  md_mt_set(mt_percent_rest);
               }
               else
               {
                  if(md_target_count > qei_get_count())
                     output_low(MD_DIR);
                  else
                     output_high(MD_DIR);
                        
                  pwm_set_period(1000);
                  pwm_select_mode(SINGLE);
                  // Enable PWM module
                  PWM_PTCON_PTEN = 1;
               }
               break;
         }
      }
   }

   // Enable RS232 receive byte interrupt
   clear_interrupt(INT_RDA2);
   enable_interrupts(INT_RDA2);
   reg_rs232_message = 0;
}

// Move to given position (encoder count)
void move_to(unsigned int count)
{
   // Calculate the number of steps
   unsigned int displ;            
   // Calculate the current motor step by using current encoder count
   unsigned int current_count = qei_get_count();
   // Calculate the target motor step by using given encoder step
   md_target_count = count; 
   
   // Set direction
   if(md_target_count > current_count)
   {
      output_low(MD_DIR);
      displ = md_target_count - current_count;
   }
   else
   {
      output_high(MD_DIR);
      displ = current_count - md_target_count;
   }
   
   displ = (long)displ * 10000 / md_conv_const;
   
   if(debug_mode_pwm)
      fprintf(RS232,"Displacement: %u\n\r", displ);
   
   // If displacement is zero than no need to move
   if(displ == 0)
      return;
      
   // Find out after how many steps we must start deceleration.
   md_accel_lim = ((long)displ * md_decel) / (md_accel + md_decel);
   // We must accelerate at least 1 step before we can start deceleration.
   if(md_accel_lim == 0)
      md_accel_lim = 1;
   
   // Use the limit we hit first to calc decel.
   if(md_accel_lim >= md_max_acc_lim)
   {
      md_accel_lim = md_max_acc_lim;
      
      // Find step to start decleration.
      md_decel_count = ((long)md_max_acc_lim * md_accel) / md_decel;
      md_decel_lim = displ - md_decel_count; 
   }
   else
   {
      md_decel_lim = displ - md_accel_lim;
      md_decel_count = md_accel_lim;
   }
   
   // We must decelerate at least 1 step to stop.
   if(md_decel_lim == 0)
      md_decel_lim = 1;

   if(debug_mode_pwm)
   {
      fprintf(RS232,"Acceleration Limit: %u\n\r", md_accel_lim);
      fprintf(RS232,"Deceleration Limit: %u\n\r", md_decel_lim);
      fprintf(RS232,"Deceleration Count: %u\n\r", md_decel_count);
   }

   md_run_state = ACCEL;
   md_mt_set(mt_percent_trip);

   pwm_set_period(delays[0]);
   pwm_select_mode(FREE);
   pwm_enable();
}
// Move to given position (mm)
void move_pos(unsigned int16 position)
{
   if(position>md_move_range)
   {
      position=md_move_range;
      if(debug_mode_md)
         fprintf(RS232, "Invalid entry: System cannot move to a position beyond the movement range. System will move to maximum possible distance.");
   }
   else if(position<md_min_distance)
   {
      position=md_min_distance;
      if(debug_mode_md)
         fprintf(RS232, "Invalid entry: System cannot move to a position smaller than %u mm. System will move to minimum possible distance.", md_min_distance);
   }
   
   fprintf(RS232,"\n\rTarget Position: %u mm\n\r", position);

   data_set_last_pos(position);
   
   unsigned int16 count = md_end_pos-((float)(position-md_min_distance)/2.0f)*((float)(md_end_pos-md_home_pos)/((float)(md_move_range-md_min_distance)/2.0f));
   
   if(qei_get_count() < count)
      count-=md_backlash;
   
   move_to(count);
   
   delay_ms(100);
   
   if(debug_mode_md)
   {
      fprintf(RS232,"Initial Error: %d\n\r",md_error);
      fprintf(RS232,"Target Encoder count: %u\n\r",md_target_count);
      fprintf(RS232,"Current Encoder count: %u\n\r",qei_get_count());
      fprintf(RS232,"Number of Iterations: %u\n\r",md_pos_iter);
      fprintf(RS232,"Initial Error: %d\n\r",md_error);
   }
   
   fprintf(RS232,"Error: %d\n\r",(int)qei_get_count()-(int)md_target_count);
}
// Homing Function
void move_home()
{
   fprintf(RS232,"Homing...\n\r");   
   
   md_run_state = HOME;
   md_decel_count = 0;
   reg_md_home = 0;
   reg_md_home_return = 0;
   
   output_high(MD_DIR);
   md_mt_set(mt_percent_trip);

   pwm_set_period(delays[0]);
   pwm_select_mode(FREE);
   pwm_enable();
}
// Initializes the system for movement
void move_init()
{
   md_move_range = data_get_move_range();
   md_move_range = data_get_move_range();
   md_home_pos = data_get_home_pos();
   md_end_pos = data_get_end_pos();
   md_backlash = data_get_backlash();
   qei_set_max_count(md_end_pos+md_end_offset);
   md_conv_const = data_get_conv_const();
   
   if(md_move_range==2000)
      md_min_distance=120;
         
   move_home();                        // Move to home position
   
   delay_ms(500);
      
   if(debug_mode_md)
      fprintf(RS232,"Moving Last Position: %u mm\n\r", data_get_last_pos());

   move_pos(data_get_last_pos());
}

void pp_str_low_check()
{
   int1 check = 1;
   int sum = 0;
   
   while(check)
   {
      sum = 0;
      if(input(PP_STR) == 1)
      {
         unsigned int i;
         for(i=0; i<100; i++)
         {
            delay_us(pp_str_delay);
            if(input(PP_STR) == 1)
               sum += 1;
         }
         if(sum > 70)
         {
            check = 0;
         }
      }
   }
}
void pp_str_high_check()
{
   int1 check = 1;
   int sum = 0;
   
   while(check)
   {
      sum = 0;
      if(input(PP_STR) == 0)
      {
         unsigned int i;
         for(i=0; i<100; i++)
         {
            delay_us(pp_str_delay);
            if(input(PP_STR) == 0)
               sum += 1;
         }
         if(sum > 70)
         {
            check = 0;
         }
      }
   }
}
// Checks strobe signal state change
void pp_str_check()
{
   pp_str_low_check();
   pp_str_high_check();
}
//Gets byte via parallel port
unsigned int pp_get_byte()
{
   unsigned int data=0;
   unsigned int D7=0;
   unsigned int D6=0;
   unsigned int D5=0;
   unsigned int D4=0;
   unsigned int D3=0;
   unsigned int D2=0;
   unsigned int D1=0;
   unsigned int D0=0;
   
   unsigned int i;
   int1 check = 1;
   
   while(check)
   {
      int sum7 = 0;
      int sum6 = 0;
      int sum5 = 0;
      int sum4 = 0;
      int sum3 = 0;
      int sum2 = 0;
      int sum1 = 0;
      int sum0 = 0;
            
      for(i=0; i<100; i++)
      {
         delay_us(pp_str_delay);
         if(input(PP_D7) == 0)
            sum7 += 1;
         if(input(PP_D6) == 0)
            sum6 += 1;
         if(input(PP_D5) == 0)
            sum5 += 1;
         if(input(PP_D4) == 0)
            sum4 += 1;
         if(input(PP_D3) == 0)
            sum3 += 1;
         if(input(PP_D2) == 0)
            sum2 += 1;
         if(input(PP_D1) == 0)
            sum1 += 1;
         if(input(PP_D0) == 0)
            sum0 += 1;
      }
      
      if(sum7 > 70)
         D7 = 1;
      else if (sum7 < 30)
         D7 = 0;
      else
         continue;
         
      if(sum6 > 70)
         D6 = 1;
      else if (sum6 < 30)
         D6 = 0;
      else
         continue;
         
      if(sum5 > 70)
         D5 = 1;
      else if (sum5 < 30)
         D5 = 0;
      else
         continue;
         
      if(sum4 > 70)
         D4 = 1;
      else if (sum4 < 30)
         D4 = 0;
      else
         continue;
         
      if(sum3 > 70)
         D3 = 1;
      else if (sum3 < 30)
         D3 = 0;
      else
         continue;
         
      if(sum2 > 70)
         D2 = 1;
      else if (sum2 < 30)
         D2 = 0;
      else
         continue;
         
      if(sum1 > 70)
         D1 = 1;
      else if (sum1 < 30)
         D1 = 0;
      else
         continue;
         
      if(sum0 > 70)
         D0 = 1;
      else if (sum0 < 30)
         D0 = 0;
      else
         continue;
         
      check = 0;
   }
   
   data=(D7<<7)+(D6<<6)+(D5<<5)+(D4<<4)+(D3<<3)+(D2<<2)+(D1<<1)+D0;

   return data;
}
//Gets BCD byte via parallel port
unsigned int pp_get_BCD_byte()
{
   unsigned int data=0;
   unsigned int D7=0;
   unsigned int D6=0;
   unsigned int D5=0;
   unsigned int D4=0;
   unsigned int D3=0;
   unsigned int D2=0;
   unsigned int D1=0;
   unsigned int D0=0;

   
   unsigned int i;
   int1 check = 1;
   
   while(check)
   {
      int sum7 = 0;
      int sum6 = 0;
      int sum5 = 0;
      int sum4 = 0;
      int sum3 = 0;
      int sum2 = 0;
      int sum1 = 0;
      int sum0 = 0;
            
      for(i=0; i<100; i++)
      {
         delay_us(pp_str_delay);
         if(input(PP_D7) == 0)
            sum7 += 1;
         if(input(PP_D6) == 0)
            sum6 += 1;
         if(input(PP_D5) == 0)
            sum5 += 1;
         if(input(PP_D4) == 0)
            sum4 += 1;
         if(input(PP_D3) == 0)
            sum3 += 1;
         if(input(PP_D2) == 0)
            sum2 += 1;
         if(input(PP_D1) == 0)
            sum1 += 1;
         if(input(PP_D0) == 0)
            sum0 += 1;
      }
      
      if(sum7 > 70)
         D7 = 1;
      else if (sum7 < 30)
         D7 = 0;
      else
         continue;
         
      if(sum6 > 70)
         D6 = 1;
      else if (sum6 < 30)
         D6 = 0;
      else
         continue;
         
      if(sum5 > 70)
         D5 = 1;
      else if (sum5 < 30)
         D5 = 0;
      else
         continue;
         
      if(sum4 > 70)
         D4 = 1;
      else if (sum4 < 30)
         D4 = 0;
      else
         continue;
         
      if(sum3 > 70)
         D3 = 1;
      else if (sum3 < 30)
         D3 = 0;
      else
         continue;
         
      if(sum2 > 70)
         D2 = 1;
      else if (sum2 < 30)
         D2 = 0;
      else
         continue;
         
      if(sum1 > 70)
         D1 = 1;
      else if (sum1 < 30)
         D1 = 0;
      else
         continue;
         
      if(sum0 > 70)
         D0 = 1;
      else if (sum0 < 30)
         D0 = 0;
      else
         continue;
         
      check = 0;
   }
   
   unsigned int byte0=0;
   unsigned int byte1=0;
   
   byte0=8*D7+4*D6+2*D5+D4;
   byte1=8*D3+4*D2+2*D1+D0;
   data=byte0*10+byte1;
   
   return data;
}
// Returns binary coded decimal byte of the given input
unsigned int pp_get_BCD_byte(unsigned int input)
{
   unsigned int byte0=0;
   unsigned int byte1=0;
   
   byte0=8*bit_test(input,7)+4*bit_test(input,6)+2*bit_test(input,5)+bit_test(input,4);
   byte1=8*bit_test(input,3)+4*bit_test(input,2)+2*bit_test(input,1)+bit_test(input,0);

   return byte0*10+byte1;
}
// Sets position given via parallel port
void pp_set_pos()
{
   delay_ms(pp_rdy_delay);
   output_high(PP_RDY);

   pp_str_check();

   output_low(PP_ACK);
   output_low(PP_RDY);
   delay_ms(pp_ack_delay);
   output_high(PP_ACK);
   
   unsigned int index=pp_get_BCD_byte();
      
   delay_ms(pp_rdy_delay);
   output_high(PP_RDY);
   
   pp_str_check();
   
   output_low(PP_ACK);
   output_low(PP_RDY);
   delay_ms(pp_ack_delay);
   output_high(PP_ACK);
   
   unsigned int pos0=pp_get_BCD_byte();
   
   delay_ms(pp_rdy_delay);
   output_high(PP_RDY);
   
   pp_str_check();

   output_low(PP_ACK);
   output_low(PP_RDY);
   delay_ms(pp_ack_delay);
   output_high(PP_ACK);
   
   unsigned int pos1=pp_get_BCD_byte();

   data_set_pos(index, pos0*100+pos1);
   
   fprintf(RS232,"Position set -> index: %u position: %u\n\r", index, pos0*100+pos1);
   
   pp_str_low_check();
}
// Moves to position given via parallel port
void pp_move_pos(unsigned int input)
{
   unsigned int index=pp_get_BCD_byte();;
   unsigned int position=data_get_pos(index);
   
   fprintf(RS232,"Move Position -> index: %u position: %u\n\r", index, position);
      
   move_pos(position);
}
// Gets command via parallel port when system is idle
void pp_get_command()
{
   output_low(PP_ACK);
   output_low(PP_RDY);
   delay_ms(pp_ack_delay);
   output_high(PP_ACK);
   
   unsigned int input=pp_get_byte();

   switch (input)
   {
      case 0x00FE:   fprintf(RS232,"Emergency Stop Command\n\r");
                     
                     reg_pp_stop=1;
                     break;
      case 0x00FD:   fprintf(RS232,"Homing Command\n\r");
      
                     move_home();   
                     break;
      case 0x00FC:   fprintf(RS232,"Laser Off Command\n\r");
                        
                     output_low(LAS_1);
                     output_low(LAS_2);
                     output_low(LAS_3);
                     break;
      case 0x00FB:   fprintf(RS232,"Laser On Command\n\r");
                     
                     output_high(LAS_1);
                     output_high(LAS_2);
                     output_high(LAS_3);
                     break; 
      case 0x00FF:   fprintf(RS232,"Set Position Command\n\r");
                     
                     pp_set_pos();
                     break;
      default:       pp_move_pos(input); 
                     break;
   }
   
   delay_ms(pp_rdy_delay);
   output_high(PP_RDY);
}

// Handles the messages of RS232 connection
void rs232_message()
{
   char input;
   
   input=fgetc(RS232);
   switch (input){
      case 'c':   fprintf(RS232,"\n\rCOMMAND LIST\n\r");
                  fprintf(RS232,"\n\rSet Serial Number\t(s)");
                  fprintf(RS232,"\n\rSet Move Range \t\t(r)");
                  fprintf(RS232,"\n\rSet Homing Distance \t(h)");
                  fprintf(RS232,"\n\rSet End Distance \t(e)");
                  fprintf(RS232,"\n\rSet Conversion Constant\t(v)");
                  fprintf(RS232,"\n\rSet Backlash \t\t(b)");
                  fprintf(RS232,"\n\rSet Communication Type \t(t)");
                  fprintf(RS232,"\n\rGet System Data \t(i)");
                  fprintf(RS232,"\n\rGet Position List \t(m)");
                  fprintf(RS232,"\n\rMove To Position \t(x)");
                  fprintf(RS232,"\n\rSet Table Data \t\t(q)");
                  fprintf(RS232,"\n\rTest Run (low res) \t(o)");
                  fprintf(RS232,"\n\rTest Run (high res) \t(l)");
                  fprintf(RS232,"\n\rSet Debug State \t(d)");
                  fprintf(RS232,"\n\rGet Encoder Count \t(w)");
                  fprintf(RS232,"\n\rStart Memory Test \t(z)");
                  fprintf(RS232,"\n\rClear Memory \t\t(f)");
                  fprintf(RS232,"\n\n\r");
                  break;
      case 's':   fprintf(RS232,"\n\rSet Serial Number (13 Characters): ");
                  data_set_serial_no();
                  fprintf(RS232,"\n\n\r");
                  break;
      case 'r':   fprintf(RS232,"\n\rSet Move Range in mm (xxxx): ");
                  data_set_move_range();
                  fprintf(RS232,"\n\n\r");
                  break;
      case 'h':   fprintf(RS232,"\n\rSet Homing Distance in counts (xxxx): ");
                  data_set_home_pos();
                  fprintf(RS232,"\n\n\r");
                  break;
      case 'e':   fprintf(RS232,"\n\rSet End Distance in counts (xxxxx): ");
                  data_set_end_pos();
                  fprintf(RS232,"\n\n\r");
                  break;
      case 'v':   fprintf(RS232,"\n\rSet Conversion Constant (xxxxx): ");
                  data_set_conv_const();
                  fprintf(RS232,"\n\n\r");
                  break;
      case 'b':   fprintf(RS232,"\n\rSet Backlash (xx): ");
                  data_set_backlash();
                  fprintf(RS232,"\n\n\r");
                  break;
      case 't':   fprintf(RS232,"\n\rSet Communication Type (0: Parallel, 1: RS232");
                  data_set_comm_type();
                  fprintf(RS232,"\n\n\r");
                  break;
      case 'i':   data_get_sys_data();
                  break;
      case 'm':   fprintf(RS232,"\n\rGet Position List(0<=X<=9): ");
                  data_get_pos_table();
                  break;
      case 'x':   fprintf(RS232,"\n\rMove To(0<=X<=9999): ");
                  unsigned int i=0;
                  unsigned int input_step[4];
                  unsigned int step=0;
                  for(i=0;i<4;i++){
                     input_step[i]=(unsigned)(fgetc(RS232)-48);
                     fputc(input_step[i]+48,RS232);
                  }
                  fprintf(RS232,"\n\n\r");
                  
                  step=1000*input_step[0]+100*input_step[1]+10*input_step[2]+1*input_step[3];
                  
                  move_pos(step);
                  break;
      case 'q':   fprintf(RS232,"\n\rEnter Table Index (xx): ");
                  unsigned int k=0;
                  unsigned int input_index[2];
                  unsigned int index=0;
                  for(k=0;k<2;k++){
                     input_index[k]=(unsigned)(fgetc(RS232)-48);
                     fputc(input_index[k]+48,RS232);
                  }
                  fprintf(RS232,"\n\n\r");
                  
                  index=10*input_index[0]+1*input_index[1];
                  
                  fprintf(RS232,"\n\rIndex: %u", index);
                  
                  fprintf(RS232,"\n\rEnter Position (xxxx): ");
                  unsigned int input_pos[2];
                  unsigned int posi=0;
                  for(k=0;k<4;k++){
                     input_pos[k]=(unsigned)(fgetc(RS232)-48);
                     fputc(input_pos[k]+48,RS232);
                  }
                  fprintf(RS232,"\n\n\r");
                  
                  posi=1000*input_pos[0]+100*input_pos[1]+10*input_pos[2]+1*input_pos[3];
                  
                  fprintf(RS232,"\n\rPosition: %u", posi);
                  
                  data_set_pos(index, posi);
                  break;
      case 'o':   fprintf(RS232,"\n\rSpeed (xxxx): ");
                  unsigned int r=0;
                  unsigned int input_speed[4];
                  unsigned int delay=0;
                  for(r=0;r<4;r++){
                     input_speed[r]=(unsigned)(fgetc(RS232)-48);
                     fputc(input_speed[r]+48,RS232);
                  }
                  fprintf(RS232,"\n\r");
                  
                  delay=1000*input_speed[0]+100*input_speed[1]+10*input_speed[2]+1*input_speed[3];
      
                  unsigned int m;
                  unsigned int pos_cal=md_min_distance;
                  unsigned int num=(md_move_range-md_min_distance)/40+1;
                  
                  for(m=0; m<num; m++)
                  {
                     move_pos(pos_cal+m*40);
                     delay_ms(delay);
                  }
                  
                  pos_cal=pos_cal+(num-1)*40;
                  
                  for(m=0; m<num; m++)
                  {
                     move_pos(pos_cal-m*40);
                     delay_ms(delay);
                  }
                  break;
      case 'l':   fprintf(RS232,"\n\rSpeed (xxxx): ");
                  unsigned int p=0;
                  for(p=0;p<4;p++){
                     input_speed[p]=(unsigned)(fgetc(RS232)-48);
                     fputc(input_speed[p]+48,RS232);
                  }
                  fprintf(RS232,"\n\r");
                  
                  delay=1000*input_speed[0]+100*input_speed[1]+10*input_speed[2]+1*input_speed[3];
      
                  unsigned int l;
                  unsigned int pos_cal_2=md_min_distance;
                  unsigned int num_2=(md_move_range-md_min_distance)/10+1;
                  
                  for(l=0; l<num_2; l++)
                  {
                     move_pos(pos_cal_2+l*10);
                     delay_ms(delay);
                  }
                  
                  pos_cal_2=pos_cal_2+(num_2-1)*10;
                  
                  for(l=0; l<num_2; l++)
                  {
                     move_pos(pos_cal_2-l*10);
                     delay_ms(delay);
                  }
                  break;
      case 'd':   fprintf(RS232,"\n\rSet debug state\n\r");
                  fprintf(RS232,"Digital potentiometer (a)(0 or 1): \n\r");
                  fprintf(RS232,"Parallel port         (b)(0 or 1): \n\r");
                  fprintf(RS232,"FRAM                  (c)(0 or 1): \n\r");
                  fprintf(RS232,"Motor driver          (d)(0 or 1): \n\r");
                  fprintf(RS232,"Encoder               (e)(0 or 1): \n\r");
                  fprintf(RS232,"PWM Module            (f)(0 or 1): \n\r");
                  fprintf(RS232,"All Modes             (g)(0 or 1): \n\r");
                  fprintf(RS232,"Selection (xx): ");
                  
                  int1 value=0;
                  unsigned int f=0;
                  char input_ds[2];
                  for(f=0;f<2;f++){
                     input_ds[f]=fgetc(RS232);
                     fputc(input_ds[f],RS232);
                  }
                  fprintf(RS232,"\n\n\r");
                  
                  if(input_ds[1]=='0')
                     value=0;
                  else if(input_ds[1]=='1')
                     value=1;
                  else
                  {
                     fprintf(RS232,"Invalid Entry\n\r");
                     break;
                  }

                  switch (input_ds[0]){
                     case 'a':   debug_mode_dp = value;
                                 break;
                     case 'b':   debug_mode_pp = value;
                                 break;
                     case 'c':   debug_mode_fr = value;
                                 break;
                     case 'd':   debug_mode_md = value;
                                 break;
                     case 'e':   debug_mode_qei = value;
                                 break;
                     case 'f':   debug_mode_pwm = value;
                                 break;          
                     case 'g':   debug_mode_dp = value;
                                 debug_mode_pp = value;
                                 debug_mode_fr = value;
                                 debug_mode_md = value;
                                 debug_mode_qei = value;
                                 debug_mode_pwm = value;
                                 break;
                     default:    fprintf(RS232,"Invalid Entry\n\r");
                                 break;
                  }
                  break;
      case 'w':   fprintf(RS232,"\n\rEncoder count %4u\n\r",qei_get_count());
                  break;
      case 'z':   fprintf(RS232,"\n\rStart Memory Test (Yes -> 1, No -> 0): ");
                  unsigned int input_mem_test=(unsigned)(fgetc(RS232)-48);
                  fputc(input_mem_test+48,RS232);
                 
                  unsigned int8 mem=0;
                  
                  if(input_mem_test == 1)
                  {
                     for(mem=0; mem<100; mem++)
                        data_set_pos(mem, mem+1);
                        
                     fprintf(RS232,"\n\r");
                     
                     debug_mode_fr=1;
                     for(mem=0; mem<100; mem++)
                        data_get_pos(mem);
                     debug_mode_fr=0;
                  }
                  break;
      case 'f':   fprintf(RS232,"\n\rClear Memory (Yes -> 1, No -> 0): ");
                  input_mem_test=(unsigned)(fgetc(RS232)-48);
                  fputc(input_mem_test+48,RS232);
                  
                  if(input_mem_test == 1)
                  {
                     for(mem=0; mem<100; mem++)
                        data_set_pos(mem, 0);
                        
                     fprintf(RS232,"\n\r");
                     
                     debug_mode_fr=1;
                     for(mem=0; mem<100; mem++)
                        data_get_pos(mem);
                     debug_mode_fr=0;
                  }
                  break;
      case 'a':   reset_cpu();
                  break;
   }
   
   return;
}

// RS232 receive byte interrupt
#INT_RDA2
void isr_rs232_message()
{
   // Receive the RS232 message
   reg_rs232_message = 1;  
} 
// Clears RS232 interrupt flags
void rs232_clear()
{
   UART_IFS0_U1RXIF = 0;
   UART_IFS0_U1TXIF = 0;
   reg_rs232_message = 0;
}

// Main method
void main()
{
   // Set I/O states of the ports
   //           FEDCBA9876543210
   set_tris_b(0b1111111011111100);
   set_tris_c(0b1111111111111111);
   set_tris_d(0b1111111100001111);
   set_tris_e(0b1111111110000000);
   set_tris_f(0b1111111111111100);
   set_tris_g(0b1111111100111111);
   
   // Set parallel port pins
   output_low(PP_ACK);
   output_low(PP_RDY);
   
   // Turn on debug led
   output_high(LED);
   
   //Turn on lasers
   output_high(LAS_1);
   output_high(LAS_2);
   output_high(LAS_3);

   // Set A/D converter to read motor torque control voltage
   setup_adc(ADC_CLOCK_INTERNAL);
   setup_adc_ports(sAN2);
   set_adc_channel(2);
   
   delay_ms(500);
   fprintf(RS232,"\n\r");
   fprintf(RS232,"______________________________________________________________\n\r");
   fprintf(RS232,"\n\n\rMODESIS LASER POSITIONING SYSTEM\n\n\r");
   fprintf(RS232,"\n\n\r Code Version V2.0 Without Interrupt\n\n\r");
   fprintf(RS232,"Press 'c' for command list\n\n\r");
   data_get_reset_state();

   // Initialize components
   qei_init();                            // Initialize quadrature encoder 
   pwm_init();                            // Initialize PWM module
   dp_init();                             // Initialize digital potentiometer
   md_init();                             // Initialize motor driver
   fr_init();                             // Initialize FRAM
   move_init();                           // Initializes the system for movement
   
   // Enable RS232 receive byte interrupt
   enable_interrupts(INT_RDA2);
   
   reg_comm_type = data_get_comm_type();  // Set communication type
   
   switch (reg_comm_type)
   {
      case 0   :  output_high(PP_ACK);
                  output_high(PP_RDY);
                  break;
      case 1   :  output_low(PP_ACK);
                  output_low(PP_RDY);
                  break;
      default:    output_high(PP_ACK);
                  output_high(PP_RDY);
                  fr_write_byte(fr_comm_type, 0);
                  break;
   }
   
   while(true)
   {
      if(input(PP_STR) == 0)
      {
         // Disable RS232 receive byte interrupt
         disable_interrupts(INT_RDA2);
         
         unsigned int i;
         int sum = 0;
         
         for(i=0; i<100; i++)
         {
            delay_us(pp_str_delay);
            if(input(PP_STR) == 0)
               sum += 1;
         }
         if(sum > 70){
            pp_get_command();
         }
         
         //fprintf(RS232,"sum: %u\n\r",sum);
         
         // Enable RS232 receive byte interrupt
         clear_interrupt(INT_RDA2);
         enable_interrupts(INT_RDA2);
      }
      if(reg_rs232_message)
      {
         // Disable RS232 receive byte interrupt
         disable_interrupts(INT_RDA2);
         reg_rs232_message = 0;
         rs232_message();
         
         // Enable RS232 receive byte interrupt
         clear_interrupt(INT_RDA2);
         enable_interrupts(INT_RDA2);

      }
   }
}
