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
#USE RS232(stream=RS232,xmit=PIN_B2,rcv=PIN_B1,baud=38400,parity=N,bits=8,stop=1)     // Set UART2 as RS232 stream
// Pin assignments
#DEFINE DBG_LED      PIN_A3                                          // Debug LED output pin
#DEFINE MD_MS1       PIN_B0                                          // Step resolution select output MS1
#DEFINE MD_MS2       PIN_A4                                          // Step resolution select output MS2
#DEFINE MD_STEP      PIN_B3                                          // Step output pin
#DEFINE MD_ENABLE    PIN_B6                                          // Enable output pin (active low)
#DEFINE MD_DIR       PIN_A2                                          // Direction output pin
#DEFINE MD_HOME      PIN_B7                                          // Home input pin
#DEFINE LM_UP        PIN_B4                                          // Up limit switch input pin
#DEFINE LM_DOWN      PIN_B5                                          // Down limit switch input pin
#DEFINE ENABLE       PIN_A1                                          // Enable Output
#DEFINE CLOCK        PIN_A0                                          // Clock Output

enum  State     {OFF = 0, ON   = 1};                                 // Motor states
enum  Direction {UP  = 0, DOWN = 1};                                 // Direction of motion
enum  Motion    {ACC = 0, WALK = 1, RUN  = 2, DEC = 3, STEADY = 4};  // State of the motion

int1           reg_rs232_message    = 0;
int16          frequency            = 0;
int16          delay                = 0;
int1           enable_state         = 0;
int16          up_distance          = 0;
int16          down_distance        = 0;
int1           up_state             = 0;
int1           down_state           = 0;
int32          down_distance_step   = 0;
int32          up_distance_step     = 0;


// RS232 receive byte interrupt
#INT_RDA
void isr_rs232_message()
{
   // Receive the RS232 message
   reg_rs232_message = 1;  
} 
// Handles the messages of RS232 connection
void data_set_frequency()
{
   unsigned int8 i=0;
   unsigned int8 input[5];
   for(i=0; i<5; i++)
   {
      input[i]=(unsigned)(fgetc(RS232)-48);
      fputc(input[i]+48,RS232);
   }
   frequency=((input[0]*10000)+(input[0]*1000)+(input[0]*100)+(input[0]*10)+(input[0]));
   delay=1000/(2*frequency);
}

void data_set_enable()
{
   unsigned int8 i=0;
   unsigned int8 input[1];
   for(i=0; i<1; i++)
   {
      input[i]=(unsigned)(fgetc(RS232)-48);
      fputc(input[i]+48,RS232);
   }
   enable_state=input[0];
   if (enable_state==1)
   {
   output_high(ENABLE);
   }
   else
   {
   output_low(ENABLE);
   }
}

void data_set_distance_upwards()
{
   unsigned int8 i=0;
   unsigned int8 input[3];
   for(i=0; i<3; i++)
   {
      input[i]=(unsigned)(fgetc(RS232)-48);
      fputc(input[i]+48,RS232);
   }
   up_distance=((input[0]*100)+(input[0]*10)+(input[0]));
   up_distance_step=up_distance*25600;
   up_state=1;


}

void data_set_distance_downwards()
{
   unsigned int8 i=0;
   unsigned int8 input[3];
   for(i=0; i<3; i++)
   {
      input[i]=(unsigned)(fgetc(RS232)-48);
      fputc(input[i]+48,RS232);
   }
   down_distance=((input[0]*100)+(input[0]*10)+(input[0]));
   down_distance_step= down_distance*25600;
   down_state=1;


}

// Handles the messages of RS232 connection// Handles the messages of RS232 connection
void rs232_message()
{
   char input;
   
   input=fgetc(RS232);
   switch (input){
      case 'c':fprintf(RS232,"\n\rSetted Frequency and State of Enable\t(i)");
               fprintf(RS232,"\n\rGo Upwards (mm): \t(u)");
               fprintf(RS232,"\n\rGo Downwards (mm): \t(d)");
//!               fprintf(RS232,"\n\rSet Frequecy Value\t(f)");
//!               fprintf(RS232,"\n\rSet Enable Status\t(e)");
               fprintf(RS232,"\n\n\r");
               break;
               
//!      case 'f':fprintf(RS232,"\n\rFrequecy Value (5 Characters): )");
//!               data_set_frequency();
//!               fprintf(RS232,"\n\n\r");
//!               break;
//!
//!      case 'e':fprintf(RS232,"\n\rEnable State (1 or 0): )");
//!               data_set_enable();
//!               fprintf(RS232,"\n\n\r");
//!               break;
               
      case 'i':fprintf(RS232,"Frequency: %Lu\n\r)",frequency);
               fprintf(RS232,"Enable State: %u\n\r)",enable_state);
               fprintf(RS232,"\n\n\r");
               break;
               
      case 'u':fprintf(RS232,"Upwards Distance (3 Caracters (mm)): )");
               data_set_distance_upwards();
               fprintf(RS232,"\n\n\r");
               
               fprintf(RS232,"\n\rFrequecy Value (5 Characters): )");
               data_set_frequency();
               fprintf(RS232,"\n\n\r");
               
               fprintf(RS232,"\n\rEnable State (1 or 0): )");
               data_set_enable();
               fprintf(RS232,"\n\n\r");
               break;
               
      case 'd':fprintf(RS232,"Downwards Distance (3 Characters (mm)): )");
               data_set_distance_downwards();
               fprintf(RS232,"\n\n\r");
               
               fprintf(RS232,"\n\rFrequecy Value (5 Characters): )");
               data_set_frequency();
               fprintf(RS232,"\n\n\r");
               
               fprintf(RS232,"\n\rEnable State (1 or 0): )");
               data_set_enable();
               fprintf(RS232,"\n\n\r");
               break;
               
               
               
   }
   
   return;
}
// Main method
void main()
{
   //           76543210
   set_tris_a(0b11100000);                                              // Set I/O states of the ports
   set_tris_b(0b10110010);
   
   enable_interrupts(INT_RDA);
   //fprintf(RS232,"\n\n\rMODESIS LASER POSITIONING STAGE\n\n\r");
   output_high(DBG_LED);
   delay_ms(500);
   
   fprintf(RS232,"\n\r");
   fprintf(RS232,"______________________________________________________________\n\r");
   fprintf(RS232,"\n\n\rMODESIS LASER POSITIONING SYSTEM\n\n\r");
   fprintf(RS232,"Press 'c' for command list\n\n\r");
   
   while(true)
{
      
      if(reg_rs232_message)
      {
         // Disable RS232 receive byte interrupt
         disable_interrupts(INT_RDA);
         
         reg_rs232_message = 0;
         rs232_message();
         
         // Enable RS232 receive byte interrupt
         clear_interrupt(INT_RDA);
         enable_interrupts(INT_RDA);
      }
      while(enable_state==1)
      {
         if(frequency!=0)
         {
            if(up_state==1)
            {
               disable_interrupts(INT_RDA);
               unsigned int8 i=0;
               for(i=0; i<up_distance_step; i++)
               {
               output_high(CLOCK);
               delay_ms(delay);
               output_low(CLOCK);
               delay_ms(delay);
               }
               up_state=0;
               enable_interrupts(INT_RDA);
            }
            else if(down_state==1)
            {
            disable_interrupts(INT_RDA);
               unsigned int8 i=0;
               for(i=0; i<down_distance_step; i++)
            {
               output_high(CLOCK);
               delay_ms(delay);
               output_low(CLOCK);
               delay_ms(delay);
            }
            down_state=0;
            enable_interrupts(INT_RDA);
            }
         }   
      }
}
   
   //md_init();                                                           // Initialize motor driver

   //motion_cycle();                                                      // Start motion cycle
}
