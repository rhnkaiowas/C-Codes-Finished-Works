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

#USE   DELAY(internal = 32MHz)                                         // delay() func. adjusted for 20Mhz Primary Osc.

// Pin assignments
#DEFINE DRV_STEP     PIN_A0                  // Step output pin to driver (Rising edge causes the indexer to move one step)
#DEFINE DRV_RESET    PIN_A1                  // Reset output pin to driver (Active-high reset input initializes all internal logic and disables the Hbridge outputs. Internal pulldown.)
#DEFINE DRV_DIR      PIN_A4                  // Direction output pin to driver (Logic level, sets the direction of stepping)
#DEFINE SPI_SDO      PIN_A6                  // SPI data output pin
#DEFINE SPI_CS       PIN_A7                  // SPI chip select pin
#DEFINE SPI_SDI      PIN_B1                  // SPI data input pin
#DEFINE SPI_SCK      PIN_B4                  // SPI clock output pin
#DEFINE DRV_STALL    PIN_B6                  // Stall input pin from driver (Internal stall detect mode: logic low when motor stall detected. Pull up mevcut)
#DEFINE DRV_FAULT    PIN_B7                  // Fault input pin from driver (Logic low when in fault condition. Pull up mevcut)
#DEFINE LM_UP        PIN_A2                  // Up limit switch input pin
#DEFINE LM_DOWN      PIN_A3                  // Down limit switch input pin

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
#BIT PIN_SDO_SELECT     = PIN_APFCON0.6      // SDO pin selection bit 
#BIT PIN_SS_SELECT      = PIN_APFCON0.5      // SS pin selection bit 

enum  State     {OFF = 0, ON   = 1};                                 // Motor states
enum  Direction {UP  = 0, DOWN = 1};                                 // Direction of motion
enum  Motion    {ACC = 0, WALK = 1, RUN  = 2, DEC = 3, STEADY = 4};  // State of the motion

int const off_time      = 10;                                        // Off time of the pwm signal (should be smaller than period)
int8      motion_state  = STEADY;                                    // Current state of the motion
int       upwards       = 0;

unsigned int16   const acc_lim         = 550;                        // Number of steps before we hit max speed. acc=10000 dec=10000 
unsigned int16 const periods[acc_lim]  = {2610,2640,2640,2640,2640,2640,2639,2639,2638,2638,2637,2637,2636,2636,2635,2634,2633,2632,2631,2630,2629,
2628,2627,2626,2625,2624,2622,2621,2620,2618,2617,2615,2614,2612,2610,2609,2607,2605,2603,2601,2599,2597,2595,2593,2591,2589,2587,2585,2582,2580,2578,
2575,2573,2570,2568,2565,2563,2560,2557,2554,2552,2549,2546,2543,2540,2537,2534,2531,2528,2525,2522,2518,2515,2512,2509,2505,2502,2498,2495,2491,2488,
2484,2481,2477,2473,2470,2466,2462,2458,2454,2451,2447,2443,2439,2435,2431,2426,2422,2418,2414,2410,2405,2401,2397,2392,2388,2384,2379,2375,2370,2366,
2361,2356,2352,2347,2342,2338,2333,2328,2323,2318,2313,2309,2304,2299,2294,2289,2284,2279,2273,2268,2263,2258,2253,2247,2242,2237,2232,2226,2221,2215,
2210,2205,2199,2194,2188,2183,2177,2171,2166,2160,2155,2149,2143,2137,2132,2126,2120,2114,2108,2103,2097,2091,2085,2079,2073,2067,2061,2055,2049,2043,
2037,2031,2024,2018,2012,2006,2000,1994,1987,1981,1975,1969,1962,1956,1950,1943,1937,1930,1924,1918,1911,1905,1898,1892,1885,1879,1872,1866,1859,1853,
1846,1839,1833,1826,1820,1813,1806,1800,1793,1786,1779,1773,1766,1759,1752,1746,1739,1732,1725,1718,1712,1705,1698,1691,1684,1677,1671,1664,1657,1650,
1643,1636,1629,1622,1615,1608,1601,1594,1587,1580,1573,1566,1559,1552,1545,1538,1531,1524,1517,1510,1503,1496,1489,1482,1475,1468,1461,1454,1447,1440,
1433,1426,1418,1411,1404,1397,1390,1383,1376,1369,1362,1355,1348,1340,1333,1326,1319,1312,1305,1298,1291,1284,1277,1270,1263,1255,1248,1241,1234,1227,
1220,1213,1206,1199,1192,1185,1178,1171,1164,1157,1150,1143,1136,1129,1122,1115,1108,1101,1094,1087,1080,1073,1066,1059,1052,1045,1038,1031,1024,1017,
1010,1004,997,990,983,976,969,963,956,949,942,935,929,922,915,908,902,895,888,881,875,868,861,855,848,842,835,828,822,815,809,802,796,789,783,776,770,
763,757,751,744,738,731,725,719,712,706,700,694,687,681,675,669,663,657,650,644,638,632,626,620,614,608,602,596,590,584,578,573,567,561,555,549,544,538,
532,526,521,515,510,504,498,493,487,482,476,471,466,460,455,449,444,439,434,428,423,418,413,408,402,397,392,387,382,377,372,368,363,358,353,348,343,339,
334,329,325,320,315,311,306,302,297,293,289,284,280,276,271,267,263,259,255,250,246,242,238,234,230,227,223,219,215,211,208,204,200,197,193,190,186,183,
179,176,172,169,166,163,159,156,153,150,147,144,141,138,135,132,129,127,124,121,118,116,113,111,108,106,103,101,99,96,94,92,90,88,86,84,82,80,78,76,74,72,
71,69,67,66,64,63,61,60,59,57,56,55,54,53,52,51,50,49,48,47,46,45,45,44,44,43,43,42,42,41,41,41,41,41,41,40
};

// Sets alternative pin functions
void set_pins()
{
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
   // start spi write operation by setting the chip select port to high
   output_high(SPI_CS);
   // Get and write the MSB of the register
   write_register_byte(make8(reg, 1));
   // Get and write the MSB of the register
   write_register_byte(make8(reg, 0));
   // stop spi write operation by setting the chip select port to low
   output_low(SPI_CS);
   delay_ms(10);
}
// Sets driver parameters
void set_driver()
{
   write_register(3096);
   write_register(4279);
   write_register(8313);
   write_register(12694);
   write_register(17692);
   write_register(22804);
   write_register(27296);
   write_register(28672);   
   delay_ms(10);
}

// Sets the motor state
void md_set_state(State value)
{
   if (value == on)
      write_register(3097);
   else if (value == off)
      write_register(3096);
}
// Sets the motion direction
void md_set_direction(Direction value)
{
   output_bit(DRV_DIR, !value);     // Set direction pin to given value
}

// Starts motion cycle
void motion_cycle()
{
   // Start cycle
   int16 step_count = 0;
   int16 period     = 0;
   motion_state     = ACC;
   
   md_set_state(ON);
   
   while(true)
   {
      switch(motion_state) 
      { 
         case ACC:
            if(step_count == acc_lim - 1)
            {
               motion_state = RUN;
            }
               
            period = periods[step_count];
            step_count++;
            break;
            
         case RUN:
            if((upwards == 1 && input(LM_UP) == 1) || (upwards == 0 && input(LM_DOWN) == 1))
               motion_state = DEC;
               
            period = periods[step_count - 1];
            break;
   
         case DEC:
            step_count--;
            // Check if we at last step
            if(step_count == 0)
            {
               md_set_state(OFF);
               return;
            }
               
            period = periods[step_count];
            break;
      }

      delay_us(off_time);
      output_low(DRV_STEP);
      delay_us(period - off_time);
      output_high(DRV_STEP);
   }
}

// Main method
void main()
{
   //             76543210
   set_tris_a(0b00101100);       // Set I/O states of the ports
   set_tris_b(0b11001111);
   
   delay_ms(500);

   // Set alternative pin functions
   set_pins();
   // Set SPI parameters
   set_SPI();
   // Set variables to default values
   set_variables();
   // Set driver
   set_driver();
   
   while(true)
   {
      if(input(LM_UP) == 1)
      {
         md_set_direction(DOWN);
         upwards = 0;
      }
      else
      {
         md_set_direction(UP);
         upwards = 1;
      }
      delay_ms(2000);
      // Start motion cycle
      motion_cycle(); 
   }    
}
