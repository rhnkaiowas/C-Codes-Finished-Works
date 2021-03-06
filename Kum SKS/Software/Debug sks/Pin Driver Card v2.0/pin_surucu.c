#INCLUDE <16f1946.h> 

#FUSES INTRC_IO      // Internal RC clock (OSC1 and OSC2 pins are normal I/O)
#FUSES NOWDT         // Watch Dog Timer disabled
#FUSES PUT           // Power Up Timer enabled
#FUSES NOMCLR        // Master Clear pin is used for I/O
#FUSES PROTECT       // Code protected from reads
#FUSES CPD           // Data EEPROM code protected
#FUSES NOBROWNOUT    // No Brown_Out reset
#FUSES NOCLKOUT      // Disable clock output on OSC2
#FUSES NOIESO        // Internal External Switch Over Mode disabled
#FUSES NOFCMEN       // Fail-safe clock monitor disabled
#FUSES WRT           // Program memory write protected                                              
#FUSES NOLVP         // Low Voltage Programming disabled
#FUSES NODEBUG       // No debug mode for ICD

#USE  DELAY(clock = 16000000)
#USE  RS232(stream=RS485,UART2,baud=38400,parity=N,bits=8,stop=1)

//TX and RX control pins
#DEFINE TX_Enable           PIN_G3      //If high Transmit enabled.
#DEFINE RX_Disable          PIN_G4      //If low Receive enabled.

// Led pin
#DEFINE LED          PIN_E2             //output               // Led used in debugging

// Pins used to control motor drivers
#DEFINE M1_IN1    PIN_B1  //output    //1  
#DEFINE M1_IN2    PIN_B0  //output

#DEFINE M2_IN1    PIN_B3  //output    //2
#DEFINE M2_IN2    PIN_B2  //output

#DEFINE M3_IN1    PIN_C3  //output    //3
#DEFINE M3_IN2    PIN_C4  //output

#DEFINE M4_IN1    PIN_C7  //output    //4
#DEFINE M4_IN2    PIN_C2  //output

#DEFINE M5_IN1    PIN_D5  //output    //5
#DEFINE M5_IN2    PIN_D4  //output

#DEFINE M6_IN1    PIN_D7  //output    //6
#DEFINE M6_IN2    PIN_D6  //output

#DEFINE M7_IN1    PIN_C1  //output    //7
#DEFINE M7_IN2    PIN_C0  //output

#DEFINE M8_IN1    PIN_A5  //output    //8
#DEFINE M8_IN2    PIN_A4  //output

#DEFINE M9_IN1    PIN_D3  //output    //9
#DEFINE M9_IN2    PIN_D2  //output

#DEFINE M10_IN1    PIN_D1  //output    //10
#DEFINE M10_IN2    PIN_D0  //output

#DEFINE M11_IN1    PIN_A3  //output    //11
#DEFINE M11_IN2    PIN_A2  //output

#DEFINE M12_IN1    PIN_A1  //output    //12
#DEFINE M12_IN2    PIN_A0  //output

#DEFINE M13_IN1    PIN_E7  //output    //13
#DEFINE M13_IN2    PIN_E6  //output

#DEFINE M14_IN1    PIN_E5  //output    //14
#DEFINE M14_IN2    PIN_E4  //output

#DEFINE M15_IN1    PIN_F3  //output    //15
#DEFINE M15_IN2    PIN_F2  //output

#DEFINE M16_IN1    PIN_F1  //output    //16
#DEFINE M16_IN2    PIN_F0  //output

// Pins used to identify adress
#DEFINE ID_0      PIN_F4 //input
#DEFINE ID_1      PIN_F5 //input
#DEFINE ID_2      PIN_F6 //input
#DEFINE ID_3      PIN_F7 //input

// States of the motors
#DEFINE STOP      0
#DEFINE UP        1
#DEFINE DOWN      2

// Variable initialization
int1           pin_states     [16]  = { 0 };                // States of the pin connections
unsigned int16 motor_runtimes [16]  = { 0 };                // Run time of the motors (in ms)
unsigned int8  motor_states   [16]  = { STOP };             // States of the pin motors
unsigned int16 motor_ports    [32]  = { M1_IN1, M1_IN2,     // Motor control pins
                                        M2_IN1, M2_IN2, 
                                        M3_IN1, M3_IN2,
                                        M4_IN1, M4_IN2, 
                                        M5_IN1, M5_IN2, 
                                        M6_IN1, M6_IN2, 
                                        M7_IN1, M7_IN2, 
                                        M8_IN1, M8_IN2, 
                                        M9_IN1, M9_IN2, 
                                        M10_IN1, M10_IN2, 
                                        M11_IN1, M11_IN2, 
                                        M12_IN1, M12_IN2, 
                                        M13_IN1, M13_IN2, 
                                        M14_IN1, M14_IN2, 
                                        M15_IN1, M15_IN2, 
                                        M16_IN1, M16_IN2};

unsigned int16 down_time         = 600;
unsigned int16 up_time           = 600;
unsigned int8  card_ID           = 0;                       // Communication ID of the pin card
int1           reg_rs485_message = 0;                       // RS232 message flag

// Timer used to stop motors (1 ms timer count)
void set_motor_timer()
{
   enable_interrupts(INT_TIMER0);
   setup_timer_0(RTCC_INTERNAL | RTCC_DIV_16 | RTCC_8_BIT); // 1 ms timer count
   set_timer0(0);
}
// Reads jumper pin states and sets the card_ID variable
void set_address()
{
   card_ID  =  8*input(ID_3) + 4*input(ID_2) + 2*input(ID_1) + input(ID_0);
}

// Set the state of the given motor
void set_motor_state(unsigned int8 motor_ID, unsigned int state)
{
   // Motor ID cannot be larger than number of pins
   if(motor_ID > 15) 
      motor_ID = 15;

   // UP = 0, DOWN = 1, STOP = 2, 
   switch (state)
   {
      case STOP:  output_low(motor_ports[2*motor_ID]);
                  output_low(motor_ports[2*motor_ID + 1]);
                  // Initialize motor runtime and set motor state
                  motor_runtimes[motor_ID] = 0;
                  motor_states[motor_ID] = STOP;
                  break;
      case DOWN:  if(motor_states[motor_ID] == STOP)
                  {    
                     output_low(motor_ports[2*motor_ID]);
                     output_high(motor_ports[2*motor_ID + 1]);
                     motor_states[motor_ID] = DOWN;
                  }
                  break;
      case UP:    if(motor_states[motor_ID] == STOP)
                  {
                     output_high(motor_ports[2*motor_ID]);
                     output_low(motor_ports[2*motor_ID + 1]);
                     motor_states[motor_ID] = UP;
                  }
                  break;
   } 
}
// Reset all motor states to STOP
void reset_motor_states()
{
   for(int i=0; i<16; i++)
      set_motor_state(i, STOP);
}

// Sets the states of all pins (if the new state is different than the current one)
void set_pins(unsigned int16 value)
{
   int1 new_state;
   
   for(int i=0; i<4; i++)
   {
      for(int j=0; j<4; j++)
      {
         unsigned int8 index = i * 4 + j; 
         int1 new_state = bit_test(value, index);
         
         //fprintf(RS485,"old: %u, new: %u, mstate: %u\n\r", pin_states[index], new_state, motor_states[index]);
         if((pin_states[index] == !new_state) && (motor_states[index] == STOP))
         {
            if(new_state)
               set_motor_state(index, DOWN);
            else
               set_motor_state(index, UP);
               
            pin_states[index] = new_state;
         }
      }
      delay_ms(100);
   }
}

// Handles the messages of RS485 connection
void rs485_message()
{
   char input = fgetc(RS485);
   
   unsigned int high_byte;
   unsigned int low_byte;
   
   switch (input)
   {
      case 'p':   fputc('r', RS485);
                  high_byte = (unsigned)(fgetc(RS485));
                  fputc('r', RS485);
                  low_byte = (unsigned)(fgetc(RS485));
                  fputc('r', RS485);
                  
                  unsigned int16 value = make16(high_byte, low_byte);                  
                  //fprintf(RS485,"state: %Lu\n\r", value);
                  
                  set_pins(value);
                  break;
      case 'u':   fputc('r', RS485);
                  high_byte = (unsigned)(fgetc(RS485));
                  fputc('r', RS485);
                  low_byte = (unsigned)(fgetc(RS485));
                  fputc('r', RS485);
                  
                  up_time = make16(high_byte, low_byte);                  
                  //fprintf(RS485,"up_time: %Lu\n\r", up_time);
                  break;
      case 'd':   fputc('r', RS485);
                  high_byte = (unsigned)(fgetc(RS485));
                  fputc('r', RS485);
                  low_byte = (unsigned)(fgetc(RS485));
                  fputc('r', RS485);
                  
                  down_time = make16(high_byte, low_byte);                  
                  //fprintf(RS485,"down_time: %Lu\n\r", down_time);
                  break;
   }
}

// RS485 receive byte interrupt
#INT_RDA2
void isr_rs485_message() 
{
   // Disable RS485 receive byte interrupt
   disable_interrupts(INT_RDA2);
   // Receive the RS485 message
   reg_rs485_message = 1;           
}
// Timer0 overflow interrupt (1ms)
#INT_TIMER0
void isr_timer0() 
{
   /*
   // Update motor states
   for(int i=0; i<16; i++)
   {
      unsigned int8 state = motor_states[i];
      if(state != STOP)
      {
         motor_runtimes[i]++;
         if((state == UP && motor_runtimes[i] > up_time) || (state == DOWN && motor_runtimes[i] > down_time))
         {
            set_motor_state(i, STOP);
         }
      }
   }
   */
   
   unsigned int8 state;
   
   state = motor_states[0];
   if(state != STOP)
   {
      motor_runtimes[0]++;
      if((state == UP && motor_runtimes[0] > up_time) || (state == DOWN && motor_runtimes[0] > down_time))
      {
         output_low(M1_IN1);
         output_low(M1_IN2);
         // Initialize motor runtime and set motor state
         motor_runtimes[0] = 0;
         motor_states[0] = STOP;
      }
   }
   
   state = motor_states[1];
   if(state != STOP)
   {
      motor_runtimes[1]++;
      if((state == UP && motor_runtimes[1] > up_time) || (state == DOWN && motor_runtimes[1] > down_time))
      {
         output_low(M2_IN1);
         output_low(M2_IN2);
         // Initialize motor runtime and set motor state
         motor_runtimes[1] = 0;
         motor_states[1] = STOP;
      }
   }
   
   state = motor_states[2];
   if(state != STOP)
   {
      motor_runtimes[2]++;
      if((state == UP && motor_runtimes[2] > up_time) || (state == DOWN && motor_runtimes[2] > down_time))
      {
         output_low(M3_IN1);
         output_low(M3_IN2);
         // Initialize motor runtime and set motor state
         motor_runtimes[2] = 0;
         motor_states[2] = STOP;
      }
   }
   
   state = motor_states[3];
   if(state != STOP)
   {
      motor_runtimes[3]++;
      if((state == UP && motor_runtimes[3] > up_time) || (state == DOWN && motor_runtimes[3] > down_time))
      {
         output_low(M4_IN1);
         output_low(M4_IN2);
         // Initialize motor runtime and set motor state
         motor_runtimes[3] = 0;
         motor_states[3] = STOP;
      }
   }
   
   state = motor_states[4];
   if(state != STOP)
   {
      motor_runtimes[4]++;
      if((state == UP && motor_runtimes[4] > up_time) || (state == DOWN && motor_runtimes[4] > down_time))
      {
         output_low(M5_IN1);
         output_low(M5_IN2);
         // Initialize motor runtime and set motor state
         motor_runtimes[4] = 0;
         motor_states[4] = STOP;
      }
   }
   
   state = motor_states[5];
   if(state != STOP)
   {
      motor_runtimes[5]++;
      if((state == UP && motor_runtimes[5] > up_time) || (state == DOWN && motor_runtimes[5] > down_time))
      {
         output_low(M6_IN1);
         output_low(M6_IN2);
         // Initialize motor runtime and set motor state
         motor_runtimes[5] = 0;
         motor_states[5] = STOP;
      }
   }
   
   state = motor_states[6];
   if(state != STOP)
   {
      motor_runtimes[6]++;
      if((state == UP && motor_runtimes[6] > up_time) || (state == DOWN && motor_runtimes[6] > down_time))
      {
         output_low(M7_IN1);
         output_low(M7_IN2);
         // Initialize motor runtime and set motor state
         motor_runtimes[6] = 0;
         motor_states[6] = STOP;
      }
   }
   
   state = motor_states[7];
   if(state != STOP)
   {
      motor_runtimes[7]++;
      if((state == UP && motor_runtimes[7] > up_time) || (state == DOWN && motor_runtimes[7] > down_time))
      {
         output_low(M8_IN1);
         output_low(M8_IN2);
         // Initialize motor runtime and set motor state
         motor_runtimes[7] = 0;
         motor_states[7] = STOP;
      }
   }
   
   state = motor_states[8];
   if(state != STOP)
   {
      motor_runtimes[8]++;
      if((state == UP && motor_runtimes[8] > up_time) || (state == DOWN && motor_runtimes[8] > down_time))
      {
         output_low(M9_IN1);
         output_low(M9_IN2);
         // Initialize motor runtime and set motor state
         motor_runtimes[8] = 0;
         motor_states[8] = STOP;
      }
   }
   
   state = motor_states[9];
   if(state != STOP)
   {
      motor_runtimes[9]++;
      if((state == UP && motor_runtimes[9] > up_time) || (state == DOWN && motor_runtimes[9] > down_time))
      {
         output_low(M10_IN1);
         output_low(M10_IN2);
         // Initialize motor runtime and set motor state
         motor_runtimes[9] = 0;
         motor_states[9] = STOP;
      }
   }
   
   state = motor_states[10];
   if(state != STOP)
   {
      motor_runtimes[10]++;
      if((state == UP && motor_runtimes[10] > up_time) || (state == DOWN && motor_runtimes[10] > down_time))
      {
         output_low(M11_IN1);
         output_low(M11_IN2);
         // Initialize motor runtime and set motor state
         motor_runtimes[10] = 0;
         motor_states[10] = STOP;
      }
   }
   
   state = motor_states[11];
   if(state != STOP)
   {
      motor_runtimes[11]++;
      if((state == UP && motor_runtimes[11] > up_time) || (state == DOWN && motor_runtimes[11] > down_time))
      {
         output_low(M12_IN1);
         output_low(M12_IN2);
         // Initialize motor runtime and set motor state
         motor_runtimes[11] = 0;
         motor_states[11] = STOP;
      }
   }
   
   state = motor_states[12];
   if(state != STOP)
   {
      motor_runtimes[12]++;
      if((state == UP && motor_runtimes[12] > up_time) || (state == DOWN && motor_runtimes[12] > down_time))
      {
         output_low(M13_IN1);
         output_low(M13_IN2);
         // Initialize motor runtime and set motor state
         motor_runtimes[12] = 0;
         motor_states[12] = STOP;
      }
   }
   
   state = motor_states[13];
   if(state != STOP)
   {
      motor_runtimes[13]++;
      if((state == UP && motor_runtimes[13] > up_time) || (state == DOWN && motor_runtimes[13] > down_time))
      {
         output_low(M14_IN1);
         output_low(M14_IN2);
         // Initialize motor runtime and set motor state
         motor_runtimes[13] = 0;
         motor_states[13] = STOP;
      }
   }
   
   state = motor_states[14];
   if(state != STOP)
   {
      motor_runtimes[14]++;
      if((state == UP && motor_runtimes[14] > up_time) || (state == DOWN && motor_runtimes[14] > down_time))
      {
         output_low(M15_IN1);
         output_low(M15_IN2);
         // Initialize motor runtime and set motor state
         motor_runtimes[14] = 0;
         motor_states[14] = STOP;
      }
   }
   
   state = motor_states[15];
   if(state != STOP)
   {
      motor_runtimes[15]++;
      if((state == UP && motor_runtimes[15] > up_time) || (state == DOWN && motor_runtimes[15] > down_time))
      {
         output_low(M16_IN1);
         output_low(M16_IN2);
         // Initialize motor runtime and set motor state
         motor_runtimes[15] = 0;
         motor_states[15] = STOP;
      }
   }
}

void main()
{
   // Set I/O states of the ports
   //           76543210
   set_tris_a(0b00000000);
   set_tris_b(0b11000000);
   set_tris_c(0b00000000);
   set_tris_d(0b00000000);
   set_tris_e(0b00000000);
   set_tris_f(0b11110000);
   set_tris_g(0b11100100);

   // Enable RS485 communication
   output_low(RX_Disable);    
   output_high(TX_Enable);

   // Turn on debug led
   output_high(LED);
   
   // Sets the card_ID variable
   set_address();
   // Resets all motors to STOP state
   reset_motor_states();
   // Setups the motor timer
   set_motor_timer();

   // Enable RS485 receive byte interrupt
   enable_interrupts(global);
   enable_interrupts(INT_RDA2);
   
   // Sent ready signal to the control card
   fprintf(RS485, "SKS Pin Card - Code Version V1.1\n\r");

   while(true)
   {
      if(reg_rs485_message)
      {
         reg_rs485_message = 0;
         rs485_message();
         
         // Enable RS485 receive byte interrupt
         clear_interrupt(INT_RDA2);
         enable_interrupts(INT_RDA2);
      }
   }
}
