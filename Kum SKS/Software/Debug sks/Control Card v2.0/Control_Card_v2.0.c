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

// RS485 TX and RX control pins
#DEFINE TX_Enable           PIN_G3        // If high Transmit enabled.
#DEFINE RX_Disable          PIN_G2        // If low Receive enabled.
// Led pins
#DEFINE LED          PIN_B8               // Led used in debugging

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

// Speed ramp states
#DEFINE HOME         0
#DEFINE ACCEL        1
#DEFINE DECEL        2
#DEFINE RUN          3
#DEFINE POS          4

// PWM module operating modes
#DEFINE FREE         0
#DEFINE SINGLE       1

// Movement direction modes
#DEFINE DOWN         0
#DEFINE UP           1

// Motor states
#DEFINE DISABLE      0
#DEFINE ENABLE       1

int1           debug_mode_dp     = 0;                 // Debug state of digital potentiometer
int1           debug_mode_md     = 0;                 // Debug state of motor driver
int1           debug_mode_pwm    = 0;                 // Debug state of motor control PWM module

unsigned int   dp_tap_limit      = 32;                // Digital potentiometer maximum tap level
unsigned int   dp_voltage_limit  = 1023;              // Digital potentiometer maximum voltage level

unsigned int   mt_voltage_limit  = 803;               // Motor torque control voltage is limited to 4V (which is defined in driver specs)
unsigned int   mt_percent_trip   = 100;               // Motor torque percent while system is moving

unsigned int   md_conv_const     = 1250;              // Constant used to convert pwm steps to actual distance in mm
unsigned int   md_move_range     = 2000;              // Movement range of the device (in 0.1 mm)
unsigned int   md_home_offset    = 100;               // Additional distance to be covered (in addition to move range) when homing the pins (in 0.1 mm)

unsigned int16 md_current_step   = 0;                 // Current step count
int            md_direction      = 1;                 // Direction variable used to increment or decrement the current step. It is 1 for upward, -1 for downward movement
unsigned int16 md_current_pos    = 0;                 // Current position

unsigned int   md_accel          = 3000;              // Acceleration of the motor (0.01 mm/s2)
unsigned int   md_decel          = 3000;              // Deceleration of the motor (0.01 mm/s2)
unsigned char  md_run_state      = 0;                 // What part of the speed ramp we are in.
unsigned int   md_decel_lim      = 0;                 // What step_pos to start decelaration
unsigned int   md_accel_lim      = 0;                 // What step_pos to end accelaration
unsigned int   md_decel_count    = 0;                 // Counter used when decelerateing to calculate step_delay.
unsigned int   md_step_count     = 0;                 // Number of PWM steps

int1           reg_md_running    = 0;                 // PWM run state flag
int1           reg_md_homing     = 0;                 // Moving to home position flag
int1           reg_demo_run      = 0;                 // Demo run flag
int1           reg_rs232_message = 0;                 // RS232 message flag
int1           reg_rs485_busy    = 0;                 // RS485 ready for communication flag

unsigned int16 reg_pin_state     = 0;                 // Register holding the pin states

unsigned int   md_min_delay                  = 1000;  // Minimum time delay (max speed)
unsigned int16 const md_max_acc_lim          = 349;   // Number of steps before we hit max speed. acc=10000 dec=10000 
unsigned int16 const delays[md_max_acc_lim]  = {2500,2500,2500,2500,2499,2498,2498,2497,2496,2494,2493,2492,2490,2489,
2487,2485,2483,2481,2479,2477,2475,2472,2470,2467,2464,2461,2458,2455,2452,2449,2445,2442,2438,2434,2430,2427,2423,2418,
2414,2410,2406,2401,2397,2392,2387,2382,2377,2372,2367,2362,2357,2351,2346,2340,2335,2329,2323,2317,2311,2305,2299,2293,
2287,2280,2274,2267,2261,2254,2247,2240,2234,2227,2220,2212,2205,2198,2191,2183,2176,2168,2161,2153,2145,2138,2130,2122,
2114,2106,2098,2090,2081,2073,2065,2056,2048,2039,2031,2022,2014,2005,1996,1987,1978,1969,1960,1951,1942,1933,1924,1915,
1906,1896,1887,1878,1868,1859,1849,1840,1830,1820,1811,1801,1791,1781,1772,1762,1752,1742,1732,1722,1712,1702,1692,1682,
1672,1661,1651,1641,1631,1620,1610,1600,1590,1579,1569,1558,1548,1537,1527,1517,1506,1496,1485,1474,1464,1453,1443,1432,
1421,1411,1400,1390,1379,1368,1358,1347,1336,1325,1315,1304,1293,1283,1272,1261,1250,1240,1229,1218,1208,1197,1186,1176,
1165,1154,1143,1133,1122,1111,1101,1090,1080,1069,1058,1048,1037,1027,1016,1005,995,984,974,964,953,943,932,922,911,901,
891,880,870,860,850,840,829,819,809,799,789,779,769,759,749,739,729,720,710,700,690,681,671,661,652,642,633,623,614,605,
595,586,577,568,559,550,540,532,523,514,505,496,487,479,470,462,453,445,436,428,420,411,403,395,387,379,371,363,356,348,
340,333,325,318,310,303,296,289,281,274,267,260,254,247,240,234,227,221,214,208,202,196,190,184,178,172,166,161,155,150,
144,139,134,129,124,119,114,109,104,100,95,91,87,83,78,74,70,67,63,59,56,52,49,46,43,40,37,34,31,29,26,24,22,20,18,16,14,
12,11,9,8,7,5,4,3,3,2,1,1,1,1};

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
unsigned int md_torque_get()
{
   unsigned int analog=read_adc();                                         // Read the voltage level
   unsigned int percent=100*(unsigned int32)analog/mt_voltage_limit;       // Convert voltage to percent
   
   if(debug_mode_dp)
      fprintf(RS232,"\n\r\tPercent Current: %u\n\r",percent);
      
   return percent;  
}
// Sets motor torque approximate to desired percent and returns the actual percent (%0-100)
unsigned int md_torque_set(unsigned int percent)
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
   
   return md_torque_get();
}
// Sets the movement direction
void md_set_dir(unsigned int mode)
{
   if(mode == DOWN)
   {
      output_low(MD_DIR);
      md_direction = -1;
   }
   else
   {
      output_high(MD_DIR);
      md_direction = 1;
   }
}
// Sets the motor state
void md_set_state(unsigned int mode)
{
   if(mode == DISABLE)
      output_high(MD_ENABLE);
   else
      output_low(MD_ENABLE);
}
// Initializes motor driver
void md_init()
{
   md_torque_set(mt_percent_trip);           // Set motor torque before initialization of motor driver

   output_high(MD_RESET);                    // Reset is active low so keep it high
   output_high(MD_SLEEP);                    // Sleep is active low so keep it high
   output_low(MD_SR);                        // Activate synchronous rectification
   output_high(MD_DIR);                      // Direction control pin can be in any state
   output_high(MD_MS1);                      // MS1 and MS2 high enables 8x microstepping mode
   output_high(MD_MS2);
   output_high(MD_ENABLE);                   // Enable is active so keep it low at startup
   output_low(MD_STEP);                      // A low-to-high transition advances the motor one increment so keep step input low
}

// Sets the bits of pin state register
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
// Sends given 16 pin states to pin control card
void send_pin_state(unsigned int16 state)
{
   unsigned int8 pin_reg_high = make8(state, 1);
   unsigned int8 pin_reg_low = make8(state, 0);
   
   //fprintf(RS232,"num: %u, high: %u, low: %u\n\r" state, pin_reg_high, pin_reg_low);

   reg_rs485_busy = 1;
   fputc('p', RS485);
   while(reg_rs485_busy);

   reg_rs485_busy = 1;
   fputc(pin_reg_high, RS485);
   while(reg_rs485_busy);
   
   reg_rs485_busy = 1;
   fputc(pin_reg_low, RS485);
   while(reg_rs485_busy);
}
// Connects or disconnects all pins
void set_all_pins(int1 state)
{
   if(state)
      reg_pin_state = 65535;
   else
      reg_pin_state = 0;
   
   send_pin_state(reg_pin_state);
}
// Sets the pin state by using its x and y coordinate
void set_pin(unsigned int x, unsigned int y, unsigned int state)
{
   unsigned int8 pin_ID = x * 4 + y;
   
   reg_pin_state = set_register_bits(reg_pin_state, pin_ID, 1, state);
   send_pin_state(reg_pin_state);
}
// Sets time required to move a pin to its up position
void set_pin_up_time(unsigned int16 time)
{
   unsigned int8 time_high = make8(time, 1);
   unsigned int8 time_low = make8(time, 0);
   
   reg_rs485_busy = 1;
   fputc('u', RS485);
   while(reg_rs485_busy);
   
   reg_rs485_busy = 1;
   fputc(time_high, RS485);
   while(reg_rs485_busy);
   
   reg_rs485_busy = 1;
   fputc(time_low, RS485);
   while(reg_rs485_busy);
}
// Sets time required to move a pin to its down position
void set_pin_down_time(unsigned int16 time)
{
   unsigned int8 time_high = make8(time, 1);
   unsigned int8 time_low = make8(time, 0);
   
   reg_rs485_busy = 1;
   fputc('d', RS485);
   while(reg_rs485_busy);
   
   reg_rs485_busy = 1;
   fputc(time_high, RS485);
   while(reg_rs485_busy);
   
   reg_rs485_busy = 1;
   fputc(time_low, RS485);
   while(reg_rs485_busy);
}
// Updates pin states according to given position (in 0.1mm). This functions is called once in every motor step.
void update_pins(unsigned int16 current_position)
{
   if(reg_demo_run)
   { 
      switch(current_position)
      {
         case 200:   set_pin(1, 2, 0);
                     break;
         case 300:   set_pin(1, 1, 0);
                     break;
         case 400:   set_pin(2, 1, 0);
                     break; 
         case 500:   set_pin(2, 2, 0);
                     break;
         case 600:   set_pin(2, 3, 0);
                     break;
         case 700:   set_pin(1, 3, 0);
                     break;
         case 800:   set_pin(0, 3, 0);
                     break;
         case 900:   set_pin(0, 2, 0);
                     break;
         case 1000:  set_pin(0, 1, 0);
                     break; 
         case 1100:  set_pin(0, 0, 0);
                     break;
         case 1200:  set_pin(1, 0, 0);
                     break;
         case 1300:  set_pin(2, 0, 0);
                     break;     
         case 1400:  set_pin(3, 0, 0);
                     break; 
         case 1500:  set_pin(3, 1, 0);
                     break;
         case 1600:  set_pin(3, 2, 0);
                     break;
         case 1700:  set_pin(3, 3, 0);
                     break;                      
      }
   }
}

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
// Sets PWM period time (us)
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
// Disables PWM module
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
// Controls the PWM cycle
void pwm_control()
{   
   md_step_count++;

   switch(md_run_state) 
   {
      case ACCEL:
         // Check if we should start deceleration.
         if(md_step_count >= md_accel_lim) 
         {
            if(md_step_count == md_decel_lim)
            {
               md_decel_count--;
               pwm_set_period(delays[md_decel_count] + md_min_delay);
               md_run_state = DECEL;
            }
            else if(md_decel_lim - md_step_count < 2)
            {
               pwm_set_period(delays[md_step_count] + md_min_delay);
            }
            else
            {
               pwm_set_period(md_min_delay);
               md_run_state = RUN;
            }
         }
         else
         {
            pwm_set_period(delays[md_step_count] + md_min_delay);
         }
         break;

      case RUN:
         // Check if we should start decelration.
         if(md_step_count == md_decel_lim) 
         {
            md_decel_count--;
            pwm_set_period(delays[md_decel_count] + md_min_delay);
            md_run_state = DECEL;
         }
         break;

      case DECEL:
         // Check if we are at the last step
         if(md_decel_count == 0)
         {
            pwm_disable();
            reg_md_running = 0;
         }
         else
         {
            md_decel_count--;
            pwm_set_period(delays[md_decel_count] + md_min_delay);
         }
         break;
   }
}
// Enables the PWM cycle
void pwm_enable()
{
   // Disable RS232 receive byte interrupt
   disable_interrupts(INT_RDA2);
   
   // Set motion parameters
   md_step_count = 0;
   md_run_state = ACCEL;
   
   // Set motor parameters
   md_set_state(ENABLE);
   delay_ms(100);
   
   // Set pwm parameters
   pwm_set_period(delays[0] + md_min_delay);
   pwm_select_mode(FREE);
   
   // Enable PWM module
   PWM_PTCON_PTEN = 1;
   reg_md_running = 1;
   
   unsigned int16 position = 0;
   unsigned int16 last_pos = 0;

   while(reg_md_running)
   {
      if(PWM_IFS2_PWMIF)
      {
         // Clear the flag register
         PWM_IFS2_PWMIF = 0;
         pwm_control();
         
         if(reg_md_homing == 0)
         {
            md_current_step += md_direction;
            position = (long)md_current_step * md_conv_const / 10000;
            if(position != last_pos)
            {
               update_pins(position);
               last_pos = position;
            }
         }
      }
   }
   
   delay_ms(100);
   md_set_state(DISABLE);

   // Enable RS232 receive byte interrupt
   clear_interrupt(INT_RDA2);
   enable_interrupts(INT_RDA2);
   reg_rs232_message = 0;
}

// Calculates and sets the minimum delay time by using the given linear positioning speed (in mm/s)
void set_speed(unsigned int16 speed)
{
   // Number of teeths of the motor drive pulley: 27
   // Number of teeths of the pin drive pulley  : 10
   // Number of motor steps per revolution      : 1600
   // Pitch of the linear pin                   : 6
   md_min_delay = (((long)27 * 6 * 1000) / 16) / speed;
   if(debug_mode_pwm)
      fprintf(RS232,"Delay: %u\n\n\r", md_min_delay);
}

// Move by the given displacement
void move(unsigned int16 displacement)
{
   unsigned int16 counts = (long)displacement * 10000 / md_conv_const;
   
   if(debug_mode_pwm)
   {
      fprintf(RS232,"Displacement: %u (in mm)\n\r", displacement);
      fprintf(RS232,"Displacement: %u (in steps)\n\r", counts);
   }
      
   // Find out after how many steps we must start deceleration.
   md_accel_lim = ((long)counts * md_decel) / (md_accel + md_decel);
   // We must accelerate at least 1 step before we can start deceleration.
   if(md_accel_lim == 0)
      md_accel_lim = 1;
   
   // Use the limit we hit first to calc decel.
   if(md_accel_lim >= md_max_acc_lim)
   {
      md_accel_lim = md_max_acc_lim;
      
      // Find step to start decleration.
      md_decel_count = ((long)md_max_acc_lim * md_accel) / md_decel;
      md_decel_lim = counts - md_decel_count; 
   }
   else
   {
      md_decel_lim = counts - md_accel_lim;
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

   pwm_enable();
}
// Move to given position (in 0.1 mm)
void move_to_pos(unsigned int16 position)
{
   if(position>md_move_range)
   {
      position=md_move_range;
      if(debug_mode_pwm)
         fprintf(RS232, "Invalid entry: System cannot move to a position beyond the movement range. System will move to maximum possible distance.");
   }
   
   fprintf(RS232,"Moving...\n\r");
   
   // Number of steps to given position
   unsigned int16 displacement;    
   
   // Set direction
   if(position > md_current_pos)
   {
      md_set_dir(UP);
      displacement = position - md_current_pos;
   }
   else
   {
      md_set_dir(DOWN);
      displacement = md_current_pos - position;
   }
  
   // If displacement is zero than no need to move
   if(displacement == 0) return;
   
   // Move to given position
   move(displacement);
   
   // Set the current position
   md_current_pos = position;
   
   fprintf(RS232,"Moving Done!\n\r");
}
// Homing Function
void move_to_home()
{
   fprintf(RS232,"Homing...\n\r");   
   
   set_all_pins(1);
   delay_ms(1500);
   
   reg_md_homing = 1;
   
   // Send all pins to the home position
   md_set_dir(DOWN);
   if(md_current_pos == 0)
      move(md_move_range + md_home_offset);
   else
      move(md_current_pos + md_home_offset);
   
   // Set the current position and step
   md_current_pos = 0;
   md_current_step = 0;
   
   reg_md_homing = 0;
    
   fprintf(RS232,"Homing Done!\n\r");
}

// Demo run of the system
void demo_run()
{
   reg_demo_run = 1;
   
   //move_to_home();
   delay_ms(2000);
   move_to_pos(2000);
   delay_ms(100);
   set_all_pins(0);
   delay_ms(1500);
   
   reg_demo_run = 0;
}

// Handles the messages of RS232 connection
void rs232_message()
{
   char input=fgetc(RS232);
   unsigned int i=0;
   
   switch (input)
   {
      case 'a':   fprintf(RS232,"Set Conversion Constant To(0<=X<=9999): ");
                  unsigned int input_const[4];
                  
                  for(i=0;i<4;i++){
                     input_const[i]=(unsigned)(fgetc(RS232)-48);
                     fputc(input_const[i]+48,RS232);
                  }
                  fprintf(RS232,"\n\r");
                  
                  unsigned int16 conv_const=1000*input_const[0]+100*input_const[1]+10*input_const[2]+1*input_const[3];
                  
                  if(conv_const == 0)
                     md_conv_const = 1;
                  else
                     md_conv_const = conv_const;
                  fprintf(RS232,"Done!\n\r");
                  break;
      case 's':   fprintf(RS232,"Set Speed To(0<=X<=999) (in mm/s): ");
                  unsigned int input_speed[3];
                  
                  for(i=0;i<3;i++){
                     input_speed[i]=(unsigned)(fgetc(RS232)-48);
                     fputc(input_speed[i]+48,RS232);
                  }
                  fprintf(RS232,"\n\r");
                  
                  unsigned int16 speed=100*input_speed[0]+10*input_speed[1]+1*input_speed[2];
                  if(speed == 0)
                     speed = 1;
                  set_speed(speed);
                  fprintf(RS232,"Done!\n\r");
                  break;
      case 'x':   fprintf(RS232,"Move To(0<=X<=9999): ");
                  unsigned int input_pos[4];
                  
                  for(i=0;i<4;i++){
                     input_pos[i]=(unsigned)(fgetc(RS232)-48);
                     fputc(input_pos[i]+48,RS232);
                  }
                  fprintf(RS232,"\n\r");
                  
                  unsigned int16 position=1000*input_pos[0]+100*input_pos[1]+10*input_pos[2]+1*input_pos[3];
                  
                  move_to_pos(position);
                  fprintf(RS232,"Done!\n\r");
                  break;
      case 'h':   move_to_home();
                  break;
      case 'p':   fprintf(RS232,"Pin (x,y,state): ");
                  unsigned int x = (unsigned)(fgetc(RS232)-48);
                  fputc(x+48,RS232);
                  unsigned int y = (unsigned)(fgetc(RS232)-48);
                  fputc(y+48,RS232);
                  unsigned int state = (unsigned)(fgetc(RS232)-48);
                  fputc(state+48,RS232);
                  fprintf(RS232,"\n\r");
                  
                  set_pin(x, y, state);
                  fprintf(RS232,"Done!\n\r");
                  break;
      case 'd':   fprintf(RS232,"Disconnecting pins...\n\r");
                  set_all_pins(0);
                  fprintf(RS232,"Done!\n\r");
                  break;
      case 'c':   fprintf(RS232,"Connecting pins...\n\r");
                  set_all_pins(1);
                  fprintf(RS232,"Done!\n\r");
                  break;
      case 'u':   fprintf(RS232,"Set Up Time To(0<=X<=9999) (in ms): ");
                  unsigned int input_up_time[4];
                  
                  for(i=0;i<4;i++){
                     input_up_time[i]=(unsigned)(fgetc(RS232)-48);
                     fputc(input_up_time[i]+48,RS232);
                  }
                  fprintf(RS232,"\n\r");
                  
                  unsigned int16 up_time=1000*input_up_time[0]+100*input_up_time[1]+10*input_up_time[2]+1*input_up_time[3];
                  
                  set_pin_up_time(up_time);
                  fprintf(RS232,"Done!\n\r");
                  break;
       case 't':   fprintf(RS232,"Set Down Time To(0<=X<=9999) (in ms): ");
                  unsigned int input_down_time[4];
                  
                  for(i=0;i<4;i++){
                     input_down_time[i]=(unsigned)(fgetc(RS232)-48);
                     fputc(input_down_time[i]+48,RS232);
                  }
                  fprintf(RS232,"\n\r");
                  
                  unsigned int16 down_time=1000*input_down_time[0]+100*input_down_time[1]+10*input_down_time[2]+1*input_down_time[3];
                  
                  set_pin_down_time(down_time);
                  fprintf(RS232,"Done!\n\r");
                  break;
      
      case 'b':   fprintf(RS232,"Pin State (0<=X<=65535): ");
                  unsigned int input_state[5];
                  
                  for(i=0;i<5;i++){
                     input_state[i]=(unsigned)(fgetc(RS232)-48);
                     fputc(input_state[i]+48,RS232);
                  }
                  fprintf(RS232,"\n\r");
                  
                  unsigned int16 value = 10000*input_state[0] + 1000*input_state[1]+100*input_state[2]+10*input_state[3]+1*input_state[4];
                  
                  send_pin_state(value);
                  
                  fprintf(RS232,"Done!\n\r");
                  break;
      case 'r':   fprintf(RS232,"Demo Run...");
                  
                  demo_run();
                  
                  fprintf(RS232,"Done!\n\r");
                  break;
   }
}

// RS485 receive byte interrupt
#INT_RDA
void isr_rs485_message()
{
   if(fgetc(RS485) == 'r')
      reg_rs485_busy = 0;
} 
// RS232 receive byte interrupt
#INT_RDA2
void isr_rs232_message()
{
   // Disable RS232 receive byte interrupt
   disable_interrupts(INT_RDA2);
   // Receive the RS232 message
   reg_rs232_message = 1;  
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
   
   delay_ms(500);
   fprintf(RS232,"\n\rSKS Control Card - Code Version V1.1\n\n\r");

   // Turn on debug led
   output_high(LED);

   // Set A/D converter to read motor torque control voltage
   setup_adc(ADC_CLOCK_INTERNAL);
   setup_adc_ports(sAN2);
   set_adc_channel(2);

   // Initialize components
   pwm_init();                            // Initialize PWM module
   dp_init();                             // Initialize digital potentiometer
   md_init();                             // Initialize motor driver

   // Enable RS232 and RS485 receive byte interrupt
   enable_interrupts(INT_RDA);
   enable_interrupts(INT_RDA2);
   // Enable RS485 communication
   output_low(RX_Disable);    
   output_high(TX_Enable);

   while(true)
   {
      if(reg_rs232_message)
      {
         reg_rs232_message = 0;
         rs232_message();
         
         // Enable RS232 receive byte interrupt
         clear_interrupt(INT_RDA2);
         enable_interrupts(INT_RDA2);
      }
   }
}
