#include <16f1826.h>   

#fuses intrc_io,NOWDT,NOPROTECT,NOBROWNOUT,NOLVP,NOPUT,NOWRT,NODEBUG,NOCPD
#use delay (clock=32000000) 
#use rs232 (baud=38400, xmit=pin_b5, rcv=pin_b2,parity=N,bits=8,stop=1)

/*
#byte APFCON0 =0X11D//7=0
#byte ANSELB=0X18D// 1,2=0
#byte RCSTA = 0X19D//4,7=1 6=0
#byte TXSTA = 0X19E//4=0

#bit APFCON0_RXDTSEL   = APFCON0.7
#bit ANSELB_ANSB2     = ANSELB.2
#bit RCSTA_SPEN      = RCSTA.7
#bit TXSTA_SYNC      = TXSTA.4
#bit RCSTA_RX9       = RCSTA.6

void set_pins(){
APFCON0_RXDTSEL    =  1;
ANSELB_ANSB2      =  0;
RCSTA_SPEN        =  1;
TXSTA_SYNC        =  0;
RCSTA_RX9         =  0;
}
*/
void main()
{
   // Set I/O states of the ports
   //           76543210                  
   set_tris_a(0b00101000);       
   set_tris_b(0b11001111);
  // set_pins();

   while(true){
     //putc('c');
      delay_ms(1000);
   }  
}
