/* rs323.cpp
   serial port routines for the autonomous obc
   modified 11/16/2007 for PD0 ADCP messages
*/   
   
#include <stdlib.h>
#include <stdio.h>
#include <dos.h>
#include <conio.h>
#include <string.h>
#include <graph.h>
#include <iostream.h>
#include "pc.h"
#include "rs232.h"
#include "dmmtest.h"
#include "dmm32.h"                                                  

// adcp variables
unsigned char tx1_message[80];
unsigned tx1_index = 0;
unsigned tx1_counter = 0;
unsigned tx1_len = 0;
unsigned rx1_index = 0;
unsigned char rx1_message[1024];
//unsigned char adcp_test[88];

// desert star variables
unsigned char tx2_message[80];
unsigned tx2_index = 0;
unsigned tx2_counter = 0;
unsigned tx2_len = 0;
unsigned rx2_index = 0;
unsigned char rx2_message[1024];

// gps variables
unsigned char tx4_message[80];
unsigned tx4_index = 0;
unsigned tx4_counter = 0;
unsigned tx4_len = 0;
unsigned rx4_index = 0;
unsigned char rx4_message[80];

// global variables
void (interrupt far * com1_old_vector)();
void (interrupt far * com2_old_vector)();
void (interrupt far * com4_old_vector)();

// global flags


// external variables
//extern unsigned char adcp_buffer[88];
extern unsigned char adcp_buffer[213];  // for PD0 file format db & jtm 11/16/2007
extern unsigned      adcp_new_data;
extern unsigned long adcp_data_frame;
extern unsigned char ds_buffer[80];
extern unsigned      ds_new_data;
extern unsigned long ds_data_frame;
extern unsigned char gps_buffer[80];
extern unsigned      gps_new_data;
extern unsigned long gps_data_frame;
extern unsigned      com1_tx;
extern unsigned      com2_tx;
extern unsigned      com4_tx;
extern unsigned long frame;

struct TIMER
  {
  unsigned long high;
  unsigned      low;
  };

extern TIMER sys_timer;
extern TIMER adcp_time;
extern TIMER ds_time;
extern TIMER gps_time;


void init_com1(void)
  {
  cout << "Begin Com1 initialization.   ";
  // disable cpu interrupts
  _disable();
  
  // set vector for COM1, irq4
  com1_old_vector = _dos_getvect(4+8); // irq4
  _dos_setvect(4+8, com1_isr);
     
  // initialize COM1 16550
  _outp( COM1_LCR, (DLAB1 | NO | STOP1 | EIGHT));
  _outp( COM1_DLL, BR_9600);
  //_outp( COM1_DLL, BR_115200);
  //_outp( COM1_DLL, BR_57600);
  //_outp( COM1_DLL, BR_19200);
  _outp( COM1_DLM, 0);
  _outp( COM1_LCR, (DLAB0 | NO | STOP1 | EIGHT));
  //_outp( COM1_FCR, 0xc1); // enable FIFOs to 14, reset FIFOs
  _outp( COM1_FCR, 0xc7); // enable FIFOs to 14, reset FIFOs
    
  // unmask all com1 uart interrupts
  //_outp( COM1_IER, 0x0f);  //jtm
  _outp( COM1_IER, 0x07);
  
  // clear garbage from the com1 iir
  while((_inp(COM1_IIR) & 0x01) != 1)
    {
    _inp(COM1_RX);
    _inp(COM1_LSR);
    _inp(COM1_MSR);
    }
  
  // enable com1 irq4 interrupts at the pic
  _outp(0x21, _inp(0x21) & 0xef);

  // turn on the com1 mcr gate (out2)
  _outp( COM1_MCR, _inp(COM1_MCR) | 0x08); 
  
  // enable cpu interrupts          
  _enable();
  cout << "Com1 initialized." << endl;
  // done 
  } // end init_com1()



void init_com2(void)
  {
  cout << "Begin Com2 initialization.   ";
  // disable cpu interrupts
  _disable();
  
  // set vector for COM2, irq3
  com2_old_vector = _dos_getvect(3+8);
  _dos_setvect(3+8, com2_isr);
     
  // initialize COM2 16550
  _outp( COM2_LCR, (DLAB1 | NO | STOP1 | EIGHT));
  _outp( COM2_DLL, BR_19200);  // for acoustic modem
  //_outp( COM2_DLL, BR_9600);  // for acoustic modem
  //_outp( COM2_DLL, BR_4800);  // for desert star
  //_outp( COM2_DLL, BR_57600);  // for crossbow
  _outp( COM2_DLM, 0);
  _outp( COM2_LCR, (DLAB0 | NO | STOP1 | EIGHT));
//sjc  _outp( COM2_FCR, 0xc1); // enable FIFOs to 14, reset FIFOs
  _outp( COM2_FCR, 0xc7); // enable FIFOs to 14, reset FIFOs
    
  // unmask all com1 uart interrupts
//sjc  _outp( COM2_IER, 0x0f);
  _outp( COM2_IER, 0x07);
  
  // clear garbage from the com1 iir
  while((_inp(COM2_IIR) & 0x01) != 1)
    {
    _inp(COM2_RX);
    _inp(COM2_LSR);
    _inp(COM2_MSR);
    }
  
  // enable com2 irq3 interrupts at the pic
  _outp(0x21, _inp(0x21) & 0xf7);

  // turn on the com2 mcr gate (out2)
  _outp( COM2_MCR, _inp(COM2_MCR) | 0x08); 
  
  // enable cpu interrupts          
  _enable();
  cout << "Com2 initialized." << endl;
  // done 
  } // end init_com2()

void init_com4(void)
  {
  cout << "Begin Com4 initialization.   ";
  // disable cpu interrupts
  _disable();
  
  // set vector for COM4, irq9
  com4_old_vector = _dos_getvect(9+0x70-8);
  _dos_setvect(9+0x70-8, com4_isr);
     
  // initialize COM4 16550
  _outp( COM4_LCR, (DLAB1 | NO | STOP1 | EIGHT));
  _outp( COM4_DLL, BR_9600);  // for gps
  //_outp( COM4_DLL, BR_4800);  //
  //_outp( COM4_DLL, BR_57600);
  _outp( COM4_DLM, 0);
  _outp( COM4_LCR, (DLAB0 | NO | STOP1 | EIGHT));
  _outp( COM4_FCR, 0xc1); // enable FIFOs to 14, reset FIFOs
    
  // unmask all com4 uart interrupts
  _outp( COM4_IER, 0x0f);
  
  // clear garbage from the com4 iir
  while((_inp(COM4_IIR) & 0x01) != 1)
    {
    _inp(COM4_RX);
    _inp(COM4_LSR);
    _inp(COM4_MSR);
    }
  
  // enable com4 irq9 interrupts at the pic
  _outp(0xa1, _inp(0xa1) & 0xfd);

  // turn on the com4 mcr gate (out2)
  _outp( COM4_MCR, _inp(COM4_MCR) | 0x08); 
  
  // enable cpu interrupts          
  _enable();
  cout << "Com4 initialized." << endl;
  // done 
  } // end init_com4()



void interrupt far com1_isr(void)
  {
  // performs rs232 functions on com1
  unsigned intr;
  unsigned i;
  //unsigned lsb;
  //unsigned msb;
    
  _enable();
    
  // disable com1 uart interrupts      
  _outp(COM1_MCR, _inp(COM1_MCR) & 0xf7);
  
  start_com1_isr:
  
  // check contents of COM1_IIR  
  intr = _inp(COM1_IIR);
  intr = intr & 0x0f; // mask off b7 to b4
  
  if((intr & 0x01) == 0) // if a valid com1 interrupt is pending
    {   
    // shift IIR value right one bit and test to see which routine is required
    intr = intr >> 1;
  
    // do rs232 input interrupt routine
    if(intr == 0)
      {
      // read the rs232 status register
      unsigned msr_char = _inp(COM1_MSR);
      goto start_com1_isr;
      }
     
    // do tx buffer empty interrupt routine
    if(intr == 1)
      {
      // either write to tx FIFO or read int id register again
      
      // the idea here is to continue a tx once it has started,
      // not to start one in the first place
      
      // the tx is started by the main() program writing the first
      // char to the tx buffer, which will send the char out
      
      // once the tx fifo has drained, the tbe bit will be
      // set in the iir, indicating that the tx buffer is ready for more
      
      if(com1_tx==0)  // no message to tx
        {
        _inp(COM1_IIR);
        goto start_com1_isr;
        }
      if(com1_tx==1)  // tx remaining message
        {
        for(i=0; i<16;i++) // load 16 characters into the tx fifo
          {
          if(com1_tx==0) break;
          else  // only if more message needs to be sent
            {
            _outp(COM1_TX, tx1_message[tx1_index]); // write the next character
            tx1_index++;
            tx1_counter++;
            if(tx1_index >= tx1_len)  // entire message has been sent
              {
              tx1_index = 0;
              com1_tx = 0;
              tx1_len = 0;
              goto start_com1_isr;  
              }
            }
          }
        }
      goto start_com1_isr;    
      }
      
    // rx data received interrupt routine (>= 14 bytes in receive buffer)
    if(intr == 2)
      {
      // read rx register
            
      // read 12 bytes from the rx register (still at least 2 bytes left)
      for(i = 0; i < 12; i++)
        {
        rx1_message[rx1_index] = _inp(COM1_RX);
        rx1_index++;
        if(rx1_index >= sizeof(rx1_message))
          rx1_index = 0;
        }
        
      goto start_com1_isr;
      }
    
    // else do serialization error/break routine  
    if(intr == 3)
      {
      // read serialization/line status register
      unsigned lsr_char = _inp(COM1_LSR);
      goto start_com1_isr;
      }
    
    // do rx character timeout routine: message end
    if(intr == 6)
      {
      // drain rx fifo
      while((_inp(COM1_LSR) & 0x01) == 1)
        {
        rx1_message[rx1_index] = _inp(COM1_RX);
        rx1_index++;
        if(rx1_index >= sizeof(rx1_message)) rx1_index = 0;
        }
      
      //if(rx1_index >= 88) // complete frame received
      if(rx1_index >= 213) // complete frame received  // for PD0 format db & jtm 11/16/2007
        {
        // timestamp the adcp message
        _disable();
        time_stamp(&adcp_time);
        _enable();

        //memcpy(adcp_buffer, rx1_message, 88);
        //_fmemcpy(adcp_buffer, rx1_message, sizeof(adcp_buffer));//for PD0 format db & jtm 11/16/2007
        _fmemcpy(adcp_buffer, rx1_message, 213);//for PD0 format db & jtm 11/16/2007
        //rx1_index = 0;
        adcp_new_data = 1;
        adcp_data_frame = frame;
        }
      rx1_index = 0;
      goto start_com1_isr;      
      } // end rx character timeout routine
        
    goto start_com1_isr;
    }  // end if((intr & 0x01) == 0)
  
  //_disable();  
  // enable com1 uart tx interrupts    
  _outp(COM1_MCR, _inp(COM1_MCR) | 0x08);
  _outp(0x20, 0x20);  // non-specific return from interrupt
  _disable();
  
  }  // end com1_isr()



void interrupt far com2_isr(void)
  {
  // performs rs232 functions on com2
  unsigned intr;
  unsigned i;
    
  _enable();
    
  // disable com2 uart interrupts      
  _outp(COM2_MCR, _inp(COM2_MCR) & 0xf7);
  
  start_com2_isr:
  
  // check contents of COM2_IIR  
  intr = _inp(COM2_IIR);
  intr = intr & 0x0f; // mask off b7 to b4
  
  if((intr & 0x01) == 0) // if a valid com2 interrupt is pending
    {   
    // shift IIR value right one bit and test to see which routine is required
    intr = intr >> 1;
  
    // do rs232 input interrupt routine
    if(intr == 0)
      {
      // read the rs232 status register
      unsigned msr_char = _inp(COM2_MSR);
      goto start_com2_isr;
      }
     
    // do tx buffer empty interrupt routine
    if(intr == 1)
      {
      // either write to tx FIFO or read int id register again
      
      // the idea here is to continue a tx once it has started,
      // not to start one in the first place
      
      // the tx is started by the main() program writing the first
      // char to the tx buffer, which will send the char out
      
      // once the tx fifo has drained, the tbe bit will be
      // set in the iir, indicating that the tx buffer is ready for more
      
      // we will refill the tx fifo, up to 127 chars, for a total of 128
      if(com2_tx==0)  // no message to tx
        {
        _inp(COM2_IIR);
        goto start_com2_isr;
        }
      if(com2_tx==1)  // tx remaining message
        {
        for(i=0; i<16;i++) // load 16 characters into the tx fifo
          {
          if(com2_tx==0) break;
          else  // only if more message needs to be sent
            {
            _outp(COM2_TX, tx2_message[tx2_index]); // write the next character
            tx2_index++;
            tx2_counter++;
            if(tx2_index >= tx2_len)  // entire message has been sent
              {
              tx2_index = 0;
              com2_tx = 0;
              tx2_len = 0;
              goto start_com2_isr;  
              }
            }
          }
        }
      goto start_com2_isr;    
      }
      
    // rx data received interrupt routine (>= 14 bytes in receive buffer)
    if(intr == 2)
      {
      // read rx register
            
      // read 12 bytes from the rx register (still at least 2 bytes left)
      for(i = 0; i < 12; i++)
        {
        rx2_message[rx2_index] = _inp(COM2_RX);
        rx2_index++;
        if(rx2_index >= sizeof(rx2_message))
          rx2_index = 0;
        }
        
      goto start_com2_isr;
      }
    
    // else do serialization error/break routine  
    if(intr == 3)
      {
      // read serialization status register
      unsigned lsr_char = _inp(COM2_LSR);
      goto start_com2_isr;
      }
    
    // do rx character timeout routine: message end
    if(intr == 6)
      {
      // drain rx register first
      while((_inp(COM2_LSR) & 0x01) == 1)
        {
        rx2_message[rx2_index] = _inp(COM2_RX);
        rx2_index++;
        if(rx2_index >= sizeof(rx2_message))
          rx2_index = 0;
        }
      
      if(rx2_index >= 49) // complete frame received
        {
        _disable();   //sjc added to match com1
        time_stamp(&ds_time);                  
        _enable();    //sjc added to match com1
        //memcpy(ds_buffer, rx2_message, sizeof(ds_buffer));
        // First fill with 0's then put in 49 byte msg
        //memcpy(ds_buffer, 0, sizeof(ds_buffer));
        //sjc memcpy(ds_buffer, rx2_message, 49);
        _fmemcpy(ds_buffer, rx2_message, 49);
        ds_new_data = 1;
        ds_data_frame = frame;
        //sjc rx2_index = 0;
        //for(i=0; i<sizeof(rx2_message); i++)
        //  rx2_message[i] = 0;
        //sjc _fmemset(rx2_message, 0, sizeof(rx2_message));
        }
      rx2_index = 0; //sjc added to match com1

      goto start_com2_isr;      
      } // end rx character timeout routine
        
    goto start_com2_isr;
    }  // end if((intr & 0x01) == 0)
  
  //sjc _disable();  
  // enable com2 uart interrupts    
  _outp(COM2_MCR, _inp(COM2_MCR) | 0x08);
  _outp(0x20, 0x20);  // non-specific return from interrupt
  _disable();  //sjc added to match com1
  }  // end com2_isr()



void interrupt far com4_isr(void)
  {
  // performs rs232 functions on com4
  unsigned intr;
  unsigned i;
    
  _enable();
    
  // disable com4 uart interrupts      
  _outp(COM4_MCR, _inp(COM4_MCR) & 0xf7);
  
  start_com4_isr:
  
  // check contents of COM4_IIR  
  intr = _inp(COM4_IIR);
  intr = intr & 0x0f; // mask off b7 to b4
  
  if((intr & 0x01) == 0) // if a valid com4 interrupt is pending
    {   
    // shift IIR value right one bit and test to see which routine is required
    intr = intr >> 1;
  
    // do rs232 input interrupt routine
    if(intr == 0)
      {
      // read the rs232 status register
      unsigned msr_char = _inp(COM4_MSR);
      goto start_com4_isr;
      }
     
    // do tx buffer empty interrupt routine
    if(intr == 1)
      {
      // either write to tx FIFO or read int id register again
      
      // the idea here is to continue a tx once it has started,
      // not to start one in the first place
      
      // the tx is started by the main() program writing the first
      // char to the tx buffer, which will send the char out
      
      // once the tx fifo has drained, the tbe bit will be
      // set in the iir, indicating that the tx buffer is ready for more
      
      // we will refill the tx fifo, up to 127 chars, for a total of 128
      if(com4_tx==0)  // no message to tx
        {
        _inp(COM4_IIR);
        goto start_com4_isr;
        }
      if(com4_tx==1)  // tx remaining message
        {
        for(i=0; i<16;i++) // load 16 characters into the tx fifo
          {
          if(com4_tx==0) break;
          else  // only if more message needs to be sent
            {
            _outp(COM4_TX, tx4_message[tx4_index]); // write the next character
            tx4_index++;
            tx4_counter++;
            if(tx4_index >= tx4_len)  // entire message has been sent
              {
              tx4_index = 0;
              com4_tx = 0;
              tx4_len = 0;
              goto start_com4_isr;  
              }
            }
          }
        }
      goto start_com4_isr;    
      }
      
    // rx data received interrupt routine (>= 14 bytes in receive buffer)
    if(intr == 2)
      {
      // read rx register
            
      // read 12 bytes from the rx register (still at least 2 bytes left)
      for(i = 0; i < 12; i++)
        {
        rx4_message[rx4_index] = _inp(COM4_RX);
        rx4_index++;
        if(rx4_index >= 80)
          rx4_index = 0;
        }
        
      goto start_com4_isr;
      }
    
    // else do serialization error/break routine  
    if(intr == 3)
      {
      // read serialization status register
      unsigned lsr_char = _inp(COM4_LSR);
      goto start_com4_isr;
      }
    
    // do rx character timeout routine: message end
    if(intr == 6)
      {
      // drain rx register first
      while((_inp(COM4_LSR) & 0x01) == 1)
        {
        rx4_message[rx4_index] = _inp(COM4_RX);
        rx4_index++;
        if(rx4_index >= 80)
          rx4_index = 0;
        }
      
      if(rx4_index >= 77) // complete frame received
        {
        time_stamp(&gps_time);
        memcpy(gps_buffer, rx4_message, sizeof(gps_buffer));
        gps_new_data = 1;
        gps_data_frame = frame;
        rx4_index = 0;
        //for(i=0; i<sizeof(rx4_message); i++)
        //  rx4_message[i] = 0;
        _fmemset(rx4_message, 0, sizeof(rx4_message));
        }

      goto start_com4_isr;      
      } // end rx character timeout routine
        
    goto start_com4_isr;
    }  // end if((intr & 0x01) == 0)
  
  _disable();  
  // enable com4 uart interrupts    
  _outp(COM4_MCR, _inp(COM4_MCR) | 0x08);
  _outp(0xa0, 0x20);  // non-specific return from interrupt slave pic
  _outp(0x20, 0x20);  // non-specific return from interrupt master pic
  }  // end com4_isr()


void shutdown_com1(void)
  {
  _disable();
  // disable com1 interrupts at the uart
  _outp( COM1_MCR, 0X00);
  _outp( COM1_IER, 0x00);
  // mask irq4 interrupts at pic
  _outp(0x21, _inp(0x21) | ~0xef);
  // restore original COM1 vector
  _dos_setvect(4+8, com1_old_vector);
  _enable();
  }
  
  
void shutdown_com2(void)
  {
  _disable();
  // disable com2 interrupts at the uart
  _outp( COM2_MCR, 0x00);
  _outp( COM2_IER, 0x00);
  // mask irq3 interrupts at pic
  _outp(0x21, _inp(0x21) | ~0xf7); 
  // restore original COM2 vector
  _dos_setvect(3+8, com2_old_vector);
  _enable();
  }
  

void shutdown_com4(void)
  {
  _disable();
  // disable com4 interrupts at the uart
  _outp( COM4_MCR, 0x00);
  _outp( COM4_IER, 0x00);
  // mask irq9 interrupts at pic
  _outp(0xa1, _inp(0xa1) | ~0xfd); 
  // restore original COM4 vector
  _dos_setvect(9+0x70-8, com4_old_vector);
  _enable();
  }
  
