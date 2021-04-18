#INCLUDE <16f1826.h> 

#FUSES INTRC_IO                                                      // High Speed Oscilator (>4 Mhz) crystal
#FUSES NOWDT                                                         // Watch Dog Timer disabled
#FUSES PUT                                                           // Power Up Timer enabled
#FUSES MCLR                                                          // Master Clear pin enabled
#FUSES BROWNOUT                                                      // Brownout Reset enabled
#FUSES BORV25                                                        // Brownout Reset at 2.5V
#FUSES NOLVP                                                         // Low Voltage Programming disabled
#FUSES CPD                                                           // Data EEPROM code protected
#FUSES PROTECT                                                       // Code protected from reads
#FUSES NOIESO                                                        // Internal External Switch Over Mode disabled
#FUSES NOFCMEN                                                       // Fail-safe clock monitor disabled

#USE   DELAY(clock=32000000)                                         // delay() func. adjusted for 20Mhz Primary Osc.
// Pin assignments
#DEFINE TRIG         PIN_A2                                          // Trigger input

#DEFINE DIR          PIN_A3                                          // DIR Output
#DEFINE CLOCK        PIN_B0                                          // Clock Output
#DEFINE ENABLE       PIN_B3                                          // Enable Output


int16          frequency            = 5000;
int32          distance_mm          = 400;

int16          delay                = 0;
int32          distance_step        = 0;

// Main method
void main()
{
   //           76543210
   set_tris_a(0b00100100);       
   set_tris_b(0b11000110);
   
   output_low(ENABLE);
   delay_ms(500);
   
   
   delay=1000000/(2*frequency);
   distance_step=800*distance_mm;
   while(true)
   {
      if(input(TRIG))
      {
         int32 i=0;
         
         output_high(ENABLE);
         
         output_low(DIR);
         
                  //+++++++++++++++++++++++++++++++++++++++++++++

         for(i=2000; i>delay; i=i-5)
         {
            output_high(CLOCK);
            delay_us(i);
            output_low(CLOCK);
            delay_us(i);
         }

         //+++++++++++++++++++++++++++++++++++++++++++++
         for(i=0; i<distance_step; i++)
         {
            output_high(CLOCK);
            delay_us(delay);
            output_low(CLOCK);
            delay_us(delay);
         }
         

         
         output_high(DIR);
         delay_ms(500);
         
         //+++++++++++++++++++++++++++++++++++++++++++++
    
         for(i=2000; i>delay; i=i-5)
         {
            output_high(CLOCK);
            delay_us(i);
            output_low(CLOCK);
            delay_us(i);
         }

         //+++++++++++++++++++++++++++++++++++++++++++++
         for(i=0; i<distance_step; i++)
         {
            output_high(CLOCK);
            delay_us(delay);
            output_low(CLOCK);
            delay_us(delay);
         }
         
         output_low(ENABLE);
      }
               

   }
}
