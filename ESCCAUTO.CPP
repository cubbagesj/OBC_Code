// esccauto.cpp

// modified 2/25/2004 by jtm
// removed some code for unused modes
// improved code style
// added comments
// added application-specific code for autonomous model
// search for "jtm" to find changes from original

// original code supplied by Carl George
// of Commtech, Inc., Wichita, KS
// as a general purpose driver for 
// the ESCC-104 synchronous serial board
// check their web site for additional information

#include "esccauto.h"           
#include "conio.h"
#include "stdlib.h"
#include "malloc.h"
#include "dos.h"
#include "memory.h"
#include "dmmtest.h"

struct TIMER
  {
  unsigned long high;
  unsigned int  low;
  };

struct LN200
  {
  unsigned int  new_data;
  long unsigned frame;
  unsigned long time_high;
  unsigned      time_low;
  unsigned      data[16];
  };

extern unsigned char ln200_buffer[32];
extern unsigned ln200_new_data;
extern unsigned long ln200_data_frame;
extern char pcm[96];

int odd = 0;

Cescc *t1; // makes the instance of Cescc class object visible to the ISR
          
Cescc::Cescc()  // constructor
  {
  unsigned i,j;
  
  t1 = this;
  
  for(i=0;i<MAX_PORTS;i++)
    {
    port_list[i] = 0;
    port_open_list[i] = 0;                 
    interrupt_list[i] = 0;
    for(j=0;j<MAX_RBUFS;j++)rxbuffer[i][j] = 0;
    for(j=0;j<MAX_TBUFS;j++)txbuffer[i][j] = 0;
    timer_status[i] = 0;
    current_rxbuf[i] = 0;
    current_txbuf[i] = 0;
    max_rxbuf[i] = 0;
    max_txbuf[i] = 0;
    tx_type[i] = 0x08;
    istxing[i] = 0; //==1 if a frame is being sent ,==0 if no txing is going on
    port_dmat_list[i] = 0;
    port_dmar_list[i] = 0;
    port_status[i] = 0;
    channel[i] = 0;
    eopmode[i] = 0;
    }
    
  for(i=0;i<16;i++)
    {
    hooked_irqs[i] = 0;
    old_service_routines[i] = NULL;
    }
    
  next_port = 0;
  next_irq =0;
  upper_irq = 0;

  // not needed once fixed settings PVR reg in init_escc for channela in
  // ln200com.cpp  
  //  outp(0x240 + PCR,inp(0x240 + PCR) & ~0x08);//set DTR to output
  //  outp(0x240 + PVR,inp(0x240 + PVR) | 0x08);//set DTR

  } // end constructor



Cescc::~Cescc()  // destructor
  {
  unsigned i,j;
  
  //check if any open ports left if so then unhook unterrupts and kill them free any buffers etc
  //printf("in destructor\n\r");
  //walk the port list and turn off ints at the uarts
  for(i=0;i<next_port;i++)
    {
    if(port_list[i]!=0)
      {
      outp(port_list[i]+PVR,(inp(port_list[i]+PVR)&0xfe)+channel[i]); //set current channel (a or b)
      outp(port_list[i]+IMR0,0xff);//disable interrupts at escc!!!
      outp(port_list[i]+IMR1,0xff);
      outp(port_list[i]+PIM,0xff);
      //if the port is open we need to free all the allocated buffer spaces
      //ffree ignores NULL arguments so we should be safe freeing all of these pointers
      //as the ones that were allocated will not be null, all others should be NULL!!!
      for(j=0;j<max_rxbuf[i];j++) _ffree(rxbuffer[i][j]);
      for(j=0;j<max_txbuf[i];j++) _ffree(txbuffer[i][j]);
      
      if(port_dmat_list[i]!=0)
        {
        if(port_dmat_list[i]==1) outp(DMA_MASK_LO,5);
        if(port_dmat_list[i]==3) outp(DMA_MASK_LO,7);
        if(port_dmat_list[i]==5) outp(DMA_MASK_HI,5);
        if(port_dmat_list[i]==7) outp(DMA_MASK_HI,7);
        outp(port_list[i]+XBCH,0);//turn DMA off
        }
        
      if(port_dmar_list[i]!=0)
        {
        if(port_dmar_list[i]==1) outp(DMA_MASK_LO,5);
        if(port_dmar_list[i]==3) outp(DMA_MASK_LO,7);
        if(port_dmar_list[i]==5) outp(DMA_MASK_HI,5);
        if(port_dmar_list[i]==7) outp(DMA_MASK_HI,7);
        }
      }
    }
  
  //walk the hooked IRQ list and replace the ISR's
  for(i=0;i<next_irq;i++)
    {
    if(hooked_irqs[i]<8)
      {
      _dos_setvect(hooked_irqs[i]+8,old_service_routines[i]);
      //be nice and mask the IRQ at the PIC...could be bad I suppose... but it is nicer
      j = inp(0x21);
      if(hooked_irqs[i] ==3) j = j|0x08;
      if(hooked_irqs[i] ==4) j = j|0x10;
      if(hooked_irqs[i] ==5) j = j|0x20;
      if(hooked_irqs[i] ==6) j = j|0x40;
      if(hooked_irqs[i] ==7) j = j|0x80;
      outp(0x21,j);
      }
        
    if(hooked_irqs[i]>8)
      {    
      _dos_setvect(hooked_irqs[i]-8+0x70,old_service_routines[i]);
      j = inp(0xa1);
      if(hooked_irqs[i] ==9)  j = j|0x02;
      if(hooked_irqs[i] ==10) j = j|0x04;
      if(hooked_irqs[i] ==11) j = j|0x08;
      if(hooked_irqs[i] ==12) j = j|0x10;
      if(hooked_irqs[i] ==15) j = j|0x80;
      outp(0xa1,j);
      } 
    }
  }  // end destructor





unsigned Cescc::add_port(unsigned base, unsigned chan, unsigned irq, unsigned dmar, unsigned dmat)
  {
  unsigned i;
  unsigned avail;
  
  avail = next_port;      // next_port is reset to 0 in Cescc() contructor
                          // then incremented at end of this function  
  for(i=0;i<=next_port;i++)
    {
    if(port_list[i]==0)   //port_list is the table of escc base addresses for each port
      {                   //find first unused (available) slot in port_list table
      avail = i;          //and use the slot for the port you are adding now
      break;
      }
    }
    
  port_list[avail] = base;            //store port base address
  interrupt_list[avail] = irq;        //store port irq
  port_dmar_list[avail] = dmar;       //store port rx dma channel
  port_dmat_list[avail] = dmat;       //store port tx dma channel                     
  channel[avail] = chan;              //store port channel number 0 or 1 (A or B)

  //set up the ESCC port register so we can access both channels   
  //note this *****MUST***** be the first access to the escc or we can't rely on talking to a specific
  //channel (defaults to channel 1, or the upper channel when bit 0 is defined as an input)
  //this is because bit 0 of the escc universal port is wired to select either channel A or B
  outp(base+PCR,0xe0);  //bit 0 = channel select  (output; 0 selects channel A; 1 selects channel B)
            //bit 1 = not used      (output)
            //bit 2 = not used      (output)
            //bit 3 = DTR channel 0   (output)
            //bit 4 = DTR channel 1   (output)
            //bit 5 = DSR channel 0   (input)
            //bit 6 = DSR channel 1   (input)
            //bit 7 = TC for DMA (off ISA bus)(input)
            
  outp(base+IPC,0x03);  //escc int pin must be slave mode active high push pull to work.
  
  //check current interrupts against ones that are hooked and hook new interrupt if necessary
  for(i=0;i<16;i++)
    {
    if(hooked_irqs[i]==irq) goto skip_irqsetup;
    }                                          

  _disable();
  
  //next_irq reset to 0 at end of Cescc constructor then incremented at end of add_port()
  if(irq<8)
    {
    old_service_routines[next_irq] = _dos_getvect(irq+8); //get old vector and save
    _dos_setvect(irq+8,escc_isr);   //put in our routine
    outp(base+PVR,(inp(base+PVR)&0xfe)+chan); //set current channel (a or b)
    outp(base+IMR0,0xff);//disable ints from uart
    outp(base+IMR1,0xff);//disable ints from uart
    outp(base+PIM,0xff);//disable ints from universal port
    i = inp(0x21);
    if(irq ==3) i = i&0xf7;
    if(irq ==4) i = i&0xef;
    if(irq ==5) i = i&0xdf;
    if(irq ==6) i = i&0xbf;
    if(irq ==7) i = i&0x7f;
    outp(0x21,i);
    }

  if(irq>8)
    {
    old_service_routines[next_irq] = _dos_getvect(irq+0x70 -8);
    _dos_setvect(irq+0x70-8,escc_isr);
    i = inp(0xa1);
    if(irq ==9)         i = i&0xfd;
    if(irq ==10)        i = i&0xfb;
    if(irq ==11)        i = i&0xf7;
    if(irq ==12)        i = i&0xef;
    if(irq ==15)        i = i&0x7f;
    outp(0xa1,i);
    upper_irq = 1;
    }
    
  _enable();

  hooked_irqs[next_irq] = irq;
  next_irq++;

  //note we will wait until we get the init_port call before enabling the ints at the uart
  //this will prevent bad things from happening like not having a buffer to put data in !!:)  
  skip_irqsetup:
  if(avail==next_port) next_port++;
  return (avail);
  }     //  end add_port()




unsigned Cescc::kill_port(unsigned port)
  {
  //here we should set the port list stuff to 0, and unhook the irq
  //walk the port list and check the irq to see if any other ports are using it, if not then 
  //unhook the irq, and mask it at the pic
  
  unsigned i;
  
  //free the buffers for this port
  for(i=0;i<max_rxbuf[port];i++)
    {
    _ffree(rxbuffer[port][i]);
    rxbuffer[port][i] = NULL; //wipe them since they are now gone
    }
    
  for(i=0;i<max_txbuf[port];i++)
    {
    _ffree(txbuffer[port][i]);
    txbuffer[port][i] = NULL;
    }
  
  //we are done with the base address so kill the links
  outp(port_list[port]+PVR,(inp(port_list[port]+PVR)&0xfe)+channel[port]); //set current channel (a or b)
  outp(port_list[port]+IMR0,0xff); //turn off channel specific interrupts
  outp(port_list[port]+IMR1,0xff);
  //should also turn off DSR/DTR interrupts here...since we will no longer have the port address
  //after the next line and if it fires we will have a stuck int line.
  //need Interrupt masks stored somewhere since we cannot read them back (write only)

  port_list[port] = 0;
  port_open_list[port] = 0;
  port_dmar_list[port] = 0;
  port_dmat_list[port] = 0;
  interrupt_list[port] = 0;
  
  if(port == (next_port-1)) next_port--; //if it was the last port then we can safely decrement the
                                         //next_port specifier
  return TRUE;
  }  // end kill_port()




unsigned Cescc::init_port(  unsigned port,
              unsigned opmode,
              struct escc_regs *esccregs,
              unsigned rbufs,
              unsigned tbufs)
  {
  //set the port (from the list, make sure it is in the list...) to the settings given
  //probably a good idea to verify all params before continuing
  //set up the registers and reset the 82526 here
  if(rbufs<2) return FALSE;
  if(rbufs>MAX_RBUFS) return FALSE;
  if(tbufs<2) return FALSE;        
  if(tbufs>MAX_TBUFS) return FALSE;
  if((opmode!=OPMODE_HDLC)&&(opmode!=OPMODE_BISYNC)&&(opmode!=OPMODE_ASYNC)) return FALSE;
  
  //allocate the memory for the buffers
  //unsigned long datasize;      
  unsigned i;
  for(i=0;i<max_rxbuf[port];i++)
    {
    _ffree(rxbuffer[port][i]);//just in case this isn't the first call to here
    rxbuffer[port][i]=NULL;   
    }                       
  
  for(i=0;i<max_txbuf[port];i++)
    {
    _ffree(txbuffer[port][i]);//just in case this isn't the first call to here
    txbuffer[port][i]=NULL;
    }

  //datasize = (unsigned long)rbufs * sizeof(struct buf);
  for(i=0;i<rbufs;i++)
    {
    rxbuffer[port][i] = (struct buf huge *)_fmalloc(sizeof(struct buf)); //create rxbuffers for this port
    if(rxbuffer[port][i]==NULL) return FALSE;                            //something failed
    }
  
  //datasize = (unsigned long)tbufs * sizeof(struct buf);
  for(i=0;i<tbufs;i++)
    {
    txbuffer[port][i] = (struct buf huge *)_fmalloc(sizeof(struct buf)); //create txbuffers for this port     
    if(txbuffer[port][i]==NULL) return FALSE;
    }
    
  current_rxbuf[port] = 0;
  current_txbuf[port] = 0;
  max_rxbuf[port] = rbufs;
  max_txbuf[port] = tbufs;

  for(i=0;i<rbufs;i++)
    {
    rxbuffer[port][i]->valid = 0;
    rxbuffer[port][i]->no_bytes = 0;
    rxbuffer[port][i]->max = 0;
    }
  
  for(i=0;i<tbufs;i++)
    {
    txbuffer[port][i]->valid = 0;      //valid indicates data (1) or garbage (0)
    txbuffer[port][i]->no_bytes = 0;   //no_bytes is the number of bytes in the frame <= FRAME_SIZE
    txbuffer[port][i]->max = 0;        //max is the number of bytes to be tx'ed this time <= 32
    }

  //set channel for this port; PVR bit0 selects channel A (bit0 = 0) or B (bit0 = 1)
  outp(port_list[port]+PVR,(inp(port_list[port]+PVR)&0xfe)+channel[port]);  //select channel A or B

  eopmode[port] = opmode;

  if(opmode == OPMODE_ASYNC)    //ASYNC not used by LN200 or biphase-M 5/18/98
    {
    outp(port_list[port]+MODE,esccregs->mode);
    outp(port_list[port]+TIMR,esccregs->timr);
    outp(port_list[port]+TCR,esccregs->tcr);
    outp(port_list[port]+DAFO,esccregs->dafo);
    outp(port_list[port]+RFC,esccregs->rfc);
    outp(port_list[port]+XBCL,esccregs->xbcl);
    outp(port_list[port]+XBCH,esccregs->xbch);
    outp(port_list[port]+CCR0,esccregs->ccr0);
    outp(port_list[port]+CCR1,esccregs->ccr1);
    outp(port_list[port]+CCR2,esccregs->ccr2);
    outp(port_list[port]+BGR,esccregs->bgr);
    outp(port_list[port]+PVR,(esccregs->pvr&0xfe)+channel[port]);
    }

  if(opmode==OPMODE_HDLC)
    {
    //this is mode specific to hdlc/sdlc...need to do various things for different modes
    //copy the regset to the escc in question
    //outp(port_list[port]+register,esccregs->register);

    outp(port_list[port]+CCR4,esccregs->ccr4); //code to access CCR4 for biphase-M
    
    outp(port_list[port]+MODE,esccregs->mode);
    outp(port_list[port]+TIMR,esccregs->timr);
    outp(port_list[port]+XAD1,esccregs->xad1);
    outp(port_list[port]+XAD2,esccregs->xad2);
    outp(port_list[port]+RAH1,esccregs->rah1);
    outp(port_list[port]+RAH2,esccregs->rah2);
    outp(port_list[port]+RAL1,esccregs->ral1);
    outp(port_list[port]+RAL2,esccregs->ral2);
    outp(port_list[port]+XBCL,esccregs->xbcl);
    outp(port_list[port]+XBCH,esccregs->xbch);
    outp(port_list[port]+CCR0,esccregs->ccr0);
    outp(port_list[port]+CCR1,esccregs->ccr1);
    outp(port_list[port]+CCR2,esccregs->ccr2);
    outp(port_list[port]+CCR3,esccregs->ccr3);
    outp(port_list[port]+BGR,esccregs->bgr);
    outp(port_list[port]+RLCR,esccregs->rlcr);
    outp(port_list[port]+PRE,esccregs->pre);
    outp(port_list[port]+PVR,(esccregs->pvr&0xfe)+channel[port]);
    }

  if(opmode == OPMODE_BISYNC)
    {
    outp(port_list[port]+MODE,esccregs->mode);
    outp(port_list[port]+TIMR,esccregs->timr);
    outp(port_list[port]+XAD1,esccregs->synl);
    outp(port_list[port]+XAD2,esccregs->synh);
    outp(port_list[port]+RAH1,esccregs->tcr);
    outp(port_list[port]+RAH2,esccregs->dafo);
    outp(port_list[port]+RAL1,esccregs->rfc);
    outp(port_list[port]+XBCL,esccregs->xbcl);
    outp(port_list[port]+XBCH,esccregs->xbch);
    outp(port_list[port]+CCR0,esccregs->ccr0);
    outp(port_list[port]+CCR1,esccregs->ccr1);
    outp(port_list[port]+CCR2,esccregs->ccr2);
    outp(port_list[port]+CCR3,esccregs->ccr3);
    outp(port_list[port]+BGR,esccregs->bgr);
    outp(port_list[port]+PRE,esccregs->pre);
    outp(port_list[port]+PVR,(esccregs->pvr&0xfe)+channel[port]);
    }

  //not really necessary because it will be done in txframe but just in case....
  if(port_dmar_list[port]!=0) setupdmar(port,rxbuffer[port][current_rxbuf[port]]->frame);
  if(port_dmat_list[port]!=0) setupdmat(port,txbuffer[port][current_txbuf[port]]->frame,FRAME_SIZE);
  
  outp(port_list[port]+CMDR,XRES);//reset transmitter
  
  unsigned long timeout_cntr = 0;

  while((inp(port_list[port]+STAR)&CEC)==CEC)
    {
    timeout_cntr++;
    if(timeout_cntr>1000000)
      {
      //bad error condition here must indicate that we are not ready for business
      outp(port_list[port]+IMR0,0xff);
      outp(port_list[port]+IMR1,0xff);
      return FALSE;
      }
    }//wait for CEC = 0//need a timeout loop here (only wait so long...now it could be infinite if no txclock input to hscx)
  
  outp(port_list[port]+CMDR,RHR); //reset receiver
  timeout_cntr=0;
  while((inp(port_list[port]+STAR)&CEC)==CEC)
    {
    timeout_cntr++;
    if(timeout_cntr>1000000)
      {
      //bad error condition here must indicate that we are not ready for business
      outp(port_list[port]+IMR0,0xff);//mask all interrupts 
      outp(port_list[port]+IMR1,0xff);
      return FALSE;
      }
    }//wait for CEC = 0//need a timeout loop here (only wait so long...now it could be infinite if no txclock input to hscx)

  port_open_list[port] = 1;  //flag this port open
  
  outp(port_list[port]+IMR0,esccregs->imr0);//flame on, unmask escc interrupts
  outp(port_list[port]+IMR1,esccregs->imr1);
  outp(port_list[port]+PIM,esccregs->pim);
  
  if(opmode==OPMODE_BISYNC) outp(port_list[port]+CMDR,HUNT);//start receive engine...search for SYN
  if(esccregs->cmdr !=0);//could write the command if we wanted to but we won't for now
  return TRUE;
  }  //  end init_port()






unsigned Cescc::rx_port(unsigned port,char far *buf, unsigned num_bytes)
  {
  //returns the number of bytes transfered to buf (one frame worth)
  //retval = rxbuffer[port][next_rbuf].no_bytes
  //buf = rxbuffer[port][next_rbuf].frame
  //rxbuffer[port][next_rbuf].valid = 0;
  // next_rbuf++;
  // if (next_rbuf > max_rbufs) next_rbuf =0 ;
  unsigned i,j;
  
  // select channel A or B
  outp(port_list[port]+PVR,(inp(port_list[port]+PVR)&0xfe)+channel[port]);
  
  // this is the section for autonomous model jtm
  if((eopmode[port]==OPMODE_HDLC)||(eopmode[port]==OPMODE_BISYNC))
    {
    // select the escc buffer pool assigned to this channel
    i = current_rxbuf[port];
    i++;
    if(i==max_rxbuf[port]) i = 0;
    
    // step through the buffer pool to see if any has a message
    do{
      if(rxbuffer[port][i]->valid == 1)
        {
        // this escc buffer claims it has a message
        // check for error conditions
        // fail if the escc buffer holds more than the user's buffer can hold
        if(rxbuffer[port][i]->no_bytes > num_bytes) return 0;
        
        // user's buffer is ok so copy the message over
        for(j=0;j<rxbuffer[port][i]->no_bytes;j++)
          {
          buf[j] = (rxbuffer[port][i]->frame[j]);
          }
        
        // release the escc buffer to be used again  
        rxbuffer[port][i]->valid = 0;
        
        // tell the user how many chars were in the message
        return j;
        }
      
      i++; // this escc buffer has no message so try the next one
      
      //wrap back to 0 if at end of buffer pool
      if(i==max_rxbuf[port])i = 0;
      
      // repeat the do loop until you've checked all the buffers
      // or until you find one with a message
      }while(i!=current_rxbuf[port]);
    
    // tell the user that no messages were found
    return 0;
    }

  if(eopmode[port]==OPMODE_ASYNC)
    {
    i = current_rxbuf[port];
    i++;
    if(i==max_rxbuf[port]) i = 0;
    do{
      if(rxbuffer[port][i]->valid ==1)
        {
        //we got a frame so copy the .frame to the buf and invalidate it 
        //always assume that async will store both byte and status 
        //such that data is allways multiples of 2
        if((rxbuffer[port][i]->no_bytes) > num_bytes) return 0;
        for(j=0;j<(rxbuffer[port][i]->no_bytes);j++)
        buf[j] = (rxbuffer[port][i]->frame[j]);//copy to user buffer
        rxbuffer[port][i]->valid = 0;//invalidate so it can be used again
        return j; //give back the # bytes copied
        }
        
      i++;  //try the next one
      if(i==max_rxbuf[port])i = 0;//wrap to 0 if at end of buffers
      
      }while(i!=current_rxbuf[port]);
  
    return 0; //no received frames so no bytes xfred
    }
  return 0;
  }   //  end rx_port(); returns # bytes transfered, 0 if fails





  // tx_port() does only a small portion of the actual tx processing.
  // escc_isr() does most of the work.
unsigned Cescc::tx_port(unsigned port,char far *buf, unsigned num_bytes)
  {
  unsigned far *bloc;
  if(num_bytes==0) return  -1;
  if(num_bytes>FRAME_SIZE) return -2;
  
  unsigned i,j;
  
  bloc = (unsigned far*)buf;
  
  outp(port_list[port]+PVR,(inp(port_list[port]+PVR)&0xfe)+channel[port]); //select channel A or B
  
  //all of this is operating mode specific,...need different cases for async, hdlc, bisync
  i = current_txbuf[port];
  
  // The following do loop cycles through this port's txbuffers once,
  // starting with current_txbuf[], then (in the first if statement)
  // examines each txbuffer until it finds an an empty one
  do  //while(i!=current_txbuf[port]) //Search through txbuffers.....
    {
    if(txbuffer[port][i]->valid ==0)  //.....and find an empty one.
      {                               
      if(port_dmat_list[port]==0)     //If we are not using DMA transfers to FIFO.....
        {
        if((istxing[port]==0)&&(num_bytes<=32))//.....and this port is not currently txing a frame.....
          {                                    //.....and we only need to fill the FIFO once (<=32 bytes)......
                                             //.....then we can complete the send here and now.....
          //printf("sending =<32\n\r");        //.....WITHOUT using a txbuffer.
          txbuffer[port][i]->valid = 0;        //???  So declare this txbuffer free.....
          for(j=0;j<num_bytes;j++)
            outp(port_list[port]+FIFO,buf[j]);//and write directly to FIFO from user's buffer
          istxing[port] = 1;   //flag that this port is actively txing
          outp(port_list[port]+CMDR,tx_type[port]+XME); //send XTF+XME command to escc
          return num_bytes; //done
          }
        
        // If the message is too big to fit in the FIFO (> 32 bytes)
        // or if this port is currently txing, 
        // then we need to store the message in a txbuffer.
        for(j=0;j<num_bytes;j++)   // so load the message into a txbuffer
          txbuffer[port][i]->frame[j] = buf[j];
        txbuffer[port][i]->valid = 1; //flag the isr that this txbuffer has a pending message,
        txbuffer[port][i]->max = num_bytes; //and record how many bytes are to be tx'ed.
        txbuffer[port][i]->no_bytes = 0;    //no_bytes is number actually sent
        //current_txbuf[port]++;
        //if(current_txbuf[port]==max_txbuf[port]) current_txbuf[port] = 0;
        if(istxing[port]==1) //If this port is currently txing, we will let the isr
          {          //reload the FIFO and tx the data when an XPR occurs.
          //printf("txing queued\n\r");
          return j;//return number of bytes xfred       //j==num_bytes after previous for loop
          }
        else // If this port is not currently txing
             // we will load the FIFO and issue the tx command.
          {
          //printf("initiating transfer\n\r");
          //outp(port_list[port]+CMDR,XRES);           //will force a tx interrupt and send the frame
          if(num_bytes>32)
            {
            //printf("sending =32\n\r");
            txbuffer[port][i]->no_bytes = 32;
            for(j=0;j<16;j++)
              outpw(port_list[port]+FIFO,bloc[j]);   //load next 32 bytes into fifo
            istxing[port]=1;
            outp(port_list[port]+CMDR,tx_type[port]);               //command escc to tx
            }
          return num_bytes; //return number of bytes xfrd
          }  // end else this port is not txing
        }  // end not dma
      
      else  // do a DMA style transfer
        {
        // printf("txing DMA style\n\r");
        //
        // copy the contents of the char string supplied to
        // the tx_port function to the next available tx buffer
        for(j=0;j<num_bytes;j++)
          txbuffer[port][i]->frame[j] = buf[j];
        txbuffer[port][i]->valid = 1;  // mark the tx buffer as containing valid data
        txbuffer[port][i]->max = num_bytes;  // max stores number of bytes to be tx'ed
        txbuffer[port][i]->no_bytes = 0;   // 
        
        if(istxing[port]==1)  // if this port is already txing
          { 
          //printf("txing queued\n\r");
          return j;// just return the number of bytes xfred
          }
          
        else   // this port is NOT already txing
          {
          //ok here we initiate the dma transfer
          //the num_bytes passed to the dma controller needs to be modified as the 
          //16 bit dma's use half the transfers for the same num_bytes...
          setupdmat(port,txbuffer[port][i]->frame,num_bytes);
          istxing[port] = 1;
          outp(port_list[port]+CMDR,tx_type[port]);
          return j;
          }  // end else
        }  // end dma style
      }  // end if this txbuffer avaiable
    i++; //look at next txbuffer
    if(i==max_txbuf[port]) i = 0; //if this is highest buffer, then reset to first buffer
    
    }while(i!=current_txbuf[port]); //continue looking for available txbuffer until cycled through buffers once
  
  //if here then there are no txbufs available so indicate no bytes xfred
  return -3;
  }         //  end tx_port(); returns # bytes transfered,0 if fails





unsigned Cescc::set_control_lines(unsigned port, unsigned dtr, unsigned rts)
  {
  
  if(port_open_list!=0)
    {                                                                         
    outp(port_list[port]+PVR,(inp(port_list[port]+PVR)&0xfe)+channel[port]); //select channel A or B
    //needs modified for correct registers on escc
    if(dtr ==1) outp(port_list[port]+PVR,inp(port_list[port]+PVR)|(1<<(3+channel[port])));//set DTR
    if(channel[port]==0) if(dtr ==0) outp(port_list[port]+PVR,inp(port_list[port]+PVR)&0xf7);//clear DTR
    if(channel[port]==1) if(dtr ==0) outp(port_list[port]+PVR,inp(port_list[port]+PVR)&0xef);
    if(rts ==1) outp(port_list[port]+MODE,inp(port_list[port]+MODE)|0x04);//set the rts bit in MODE
    if(rts ==0) outp(port_list[port]+MODE,inp(port_list[port]+MODE)&0xfb);//clear the RTS bit in mode
    return TRUE;
    }
  else return FALSE;
  }






unsigned Cescc::get_control_lines(unsigned port)
  {
  unsigned cts,dsr,dcd,dtr,rts;
  unsigned retval;
  
  if(port_open_list!=0)
    {
    outp(port_list[port]+PVR,(inp(port_list[port]+PVR)&0xfe)+channel[port]); //select channel A or B
    //needs modified for correct registers on escc
    dsr = (inp(port_list[port]+PVR)>>5+channel[port])&0x01;
    dtr = (inp(port_list[port]+PVR)>>3+channel[port])&1;
    rts = (inp(port_list[port]+MODE)>>2)&1;
    dcd = (inp(port_list[port]+VSTR)>>7)&1;
    cts = (inp(port_list[port]+STAR)>>1)&1;
    retval = (dsr<<4)+(dtr<<3)+(dcd<<2)+(cts<<1)+rts;
    return retval;  //return  = bit flags  X X X DSR DTR DCD CTS RTS
    }
  else return 0;
  }






unsigned Cescc::get_port_status(unsigned port)
  {
  unsigned st;
  
  if(port_open_list[port]!=0)
    {
    st = port_status[port];
    port_status[port] = 0;
    return st;
    }
  else return 0;
  }






unsigned Cescc::clear_rx_buffer(unsigned port)
  {
  //set all rxbuffer[port][..].valid = 0 here                             
  //and reset the receiver on the 82526
  unsigned i;
  outp(port_list[port]+PVR,(inp(port_list[port]+PVR)&0xfe)+channel[port]);  //select channel A or B
  if(port_open_list[port] != 0)
    {
    if(rxbuffer[port]!=NULL)
      {
      for(i=0;i<max_rxbuf[port];i++) rxbuffer[port][i]->valid = 0;//wipe all received frames
      current_rxbuf[port] = 0;  
      port_status[port] = port_status[port]&0xfdff;//reset the rxready condition
      outp(port_list[port]+CMDR,RHR); //reset the HDLC receiver
      return TRUE;                    //success 
      }
    }
    
  return FALSE;         //no rxbuffer pointer so port not open (failed)
  }





unsigned Cescc::clear_tx_buffer(unsigned port)
  {          
  unsigned i;
  //set all txbuffer[port][..].valid = 0 here
  //and reset the transmitter on the 82526
  outp(port_list[port]+PVR,(inp(port_list[port]+PVR)&0xfe)+channel[port]); //select channel A or B
  if(port_open_list[port] != 0)                                             
    {
    if(txbuffer[port]!=NULL)
      {          
      for(i=0;i<max_txbuf[port];i++) txbuffer[port][i]->valid = 0; //wipe all data frames to be sent
      istxing[port] = 0;  //not sending anymore
      current_txbuf[port] = 0;
      outp(port_list[port]+CMDR,XRES);//reset the transmitter
      return TRUE;
      }
    }
  return FALSE; //port not open or buffer not allocated return false (failed)
  }






unsigned Cescc::set_tx_type(unsigned port,unsigned type)
  {
  //quite specific to hdlc/sdlc
  if(type==AUTO_MODE) tx_type[port]=0x04;
  if(type==TRANSPARENT_MODE) tx_type[port] = 0x08;
  return TRUE;
  }
  



unsigned Cescc::set_tx_address(unsigned port,unsigned address)
  {
  //xad1 is high byte
  //xad2 is low byte
  outp(port_list[port]+PVR,(inp(port_list[port]+PVR)&0xfe)+channel[port]);
  if(port_open_list[port] != 0)
    {
    outp(port_list[port]+XAD2,address&0xff);
    outp(port_list[port]+XAD1,address>>8);
    return TRUE;
    }
  else return FALSE;
  }






unsigned Cescc::set_rx_address1(unsigned port,unsigned address)
  {                                                                         
  outp(port_list[port]+PVR,(inp(port_list[port]+PVR)&0xfe)+channel[port]);  //select channel A or B
  if(port_open_list[port] != 0)
    {
    outp(port_list[port]+RAL1,address&0xff);
    outp(port_list[port]+RAH1,address>>8);
    return TRUE;
    }
  else return FALSE;
  }






unsigned Cescc::set_rx_address2(unsigned port,unsigned address)
  {                                                                         
  outp(port_list[port]+PVR,(inp(port_list[port]+PVR)&0xfe)+channel[port]);
  if(port_open_list[port] != 0)
    {
    outp(port_list[port]+RAL2,address&0xff);
    outp(port_list[port]+RAH2,address>>8);
    return TRUE;
    }
  else return FALSE;
  }






unsigned Cescc::start_timer(unsigned port)
  {                                                                         
  outp(port_list[port]+PVR,(inp(port_list[port]+PVR)&0xfe)+channel[port]);
  if(port_open_list[port] != 0)
    {
    timer_status[port] = 0;
    outp(port_list[port]+CMDR,STI);//send start timer command
    return TRUE;
    }
  else return FALSE;
  }                 






unsigned Cescc::stop_timer(unsigned port)                                 
  {
  outp(port_list[port]+PVR,(inp(port_list[port]+PVR)&0xfe)+channel[port]);
  if(port_open_list[port] != 0)
    {
    outp(port_list[port]+TIMR,inp(port_list[port]+TIMR));//writing the timr register stops the timer
    return TRUE;
    }
  else return FALSE;
  }                 






unsigned Cescc::is_timer_expired(unsigned port)
  {
  if(port_open_list[port] != 0)
    {
    if(timer_status[port]==1)
      {
      timer_status[port] =0;
      port_status[port] = port_status[port]&(~TIMER_INTERRUPT);
      return TRUE;
      }
    else return FALSE;
    }
  return FALSE;
  }







unsigned Cescc::wait_for_timer_expired(unsigned port)
  {
  unsigned long timer_timeout;
  timer_timeout = 0;
  
  if(port_open_list[port]!=0)
    {
    while(timer_status[port]==0)
      {
      timer_timeout++;
      if(timer_timeout>1000000)
        {
        return FALSE;
        }
      }
    timer_status[port] = 0;
    port_status[port] = port_status[port]&(~TIMER_INTERRUPT);
    return TRUE;
    }
    
  else return FALSE;
  }






void interrupt far Cescc::escc_isr(void)
  {
  unsigned isr0;
  unsigned isr1;
  unsigned pis;
  unsigned i; 
  unsigned chanstor;
  unsigned j,k;
  unsigned inthit;
  unsigned lsb = 0;
  unsigned msb = 0;
  unsigned far *bloc;
  unsigned ok_to_tx = 0;

  extern unsigned long frame;
  extern LN200 ln200;
  extern TIMER ln200_time;
  extern TIMER sys_timer; 
  extern unsigned long ln200_rx_num;

  struct buf far *irxbuf;
  struct buf far *itxbuf;
  
  //_disable();
  //_enable();      //12/8/03 rem out
  
  isr0 = 0;
  isr1 = 0;
  pis = 0;
  j = 0;
  

  startisr:
  
  inthit = 0; // clear any lingering garbage
  
  for(i=0;i<t1->next_port;i++) // for all opened ports
    {
    // store the active channel (A or B) to be restored when ISR terminates
    chanstor = inp(t1->port_list[i]+PVR)&0x1;
  
    // set the active channel to A or B as selected by the for-loop index                                    
    outp(t1->port_list[i]+PVR,(inp(t1->port_list[i]+PVR)&0xfe)+t1->channel[i]); //set channel A or B

    // shut this channel down if it was never properly initialized
    if(t1->port_open_list[i]==0)
      {
      if(t1->port_list[i]!=0) // the channel was never assigned a base address
        {
        outp(t1->port_list[i]+IMR0,0xff);  // so shut off all its interrupts
        outp(t1->port_list[i]+IMR1,0xff);
        goto nextpt; //skip to end of the big for() loop
        }
      }
  
    // read this channel's interrupt status registers and store locally  
    isr0 = inp(t1->port_list[i]+ISR0);
    isr1 = inp(t1->port_list[i]+ISR1);
    pis = inp(t1->port_list[i]+PIS); // universal port interrupts
    
    // skip to next channel if no interrupts set for this channel
    if((isr0+isr1+pis)==0) goto nextpt;
  
    // or else service this channel's interrupts
    inthit = 1;

    // check each interrupt event and service them as needed
    // ISR0 first
    if((isr0&RME)==RME)  //  RME Receive Message End
      {
      if(t1->eopmode[i]==OPMODE_HDLC)
        {
        if(t1->port_dmar_list[i]==0)  // if not using DMA
          {
          
          // get "k", the total number of bytes received in this message
          
          // then fill the rbuf frame (char frame[]) with the rest of the
          // chars still remaining in the rx fifo
          
          // remember that the rx fifo is only 32 chars deep
          
          // the low 4 bits of RBCH and 8 bits of RBCL indicate the total length in chars
          // of the received message (0 to 4095)
          
          // additionally, the low 4 bits of RBCL indicate the number of chars currently
          // in the rx fifo
          
          // read total number of chars received in this message
          k = (unsigned)((inp(t1->port_list[i]+RBCH)&0x0f) <<8) +
              (unsigned) (inp(t1->port_list[i]+RBCL)&0xff);
          
          // get the right buffer to use
          irxbuf = (t1->rxbuffer[i][t1->current_rxbuf[i]]);     
          
          //the reads are done as byte access so that odd frame lengths will be received correctly
          //the fifo can not be accessed both as word and byte in same pool, meaning 
          //if you start doing 16 bit xfrs you must do all 16 bit xfrs...
          //note it might be quicker to just do an extra 16 bit xfr as opposed to doing it byte wise
          //but that only stands a chance of working for rx, txing you must do byte transfers
          //as the number of writes to the fifo determines what gets sent out.
          
          // read the final message chars from the rx fifo and add them to the buffer
          for(j=irxbuf->no_bytes;j<k;j++) // j = num bytes received before this fifo dump
            {
            irxbuf->frame[j] = inp(t1->port_list[i]+FIFO);
            }
            
          outp(t1->port_list[i]+CMDR,RMC);// release fifo
          irxbuf->valid = 1;              // validate the rbuf we just used
          irxbuf->no_bytes = k;           // update the total number of bytes received
          
          // new code specific to receiving ln200 messages in the autonomous model
          // added 3/25/2004 jtm
          if(k==33)
            {
            if(odd) // keep only every other ln200 message since they are 
              {     // coming in at 200 Hz
              ln200_rx_num++;
              
              _fmemcpy(ln200_buffer, irxbuf->frame, k);  // copy message to app buffer
              ln200_new_data = 1;                      // flag new data available
              ln200_data_frame = frame;                     // record frame number
              time_stamp(&ln200_time);
              }
            odd = !odd;  
            }  
          irxbuf->valid = 0;                       // release the escc buffer to be used again  
          irxbuf->no_bytes = 0;
          // end autonomous-specific code

                                 
          // select the next buffer for the next reception
          t1->current_rxbuf[i]++;
          // rollover to the first buffer if we've reached the end of available buffers
          if(t1->current_rxbuf[i]==t1->max_rxbuf[i]) 
            {
            t1->current_rxbuf[i]=0;
            }
          
          // make sure the next selected buffer is not in use (an error condition)
          if(t1->rxbuffer[i][t1->current_rxbuf[i]]->valid ==1) 
            {
            // An error occured and the main program isn't done with 
            // the frame that is about to be used for received data.
            // Set the receive buffers overflowed bit of the status word for this port here.
            t1->port_status[i] = t1->port_status[i] | RX_BUFFER_OVERFLOW;
            }
            
          // prep the next selected buffer for use  
          t1->rxbuffer[i][t1->current_rxbuf[i]]->no_bytes = 0;
          t1->rxbuffer[i][t1->current_rxbuf[i]]->valid = 0;
          
          // flag this port as holding a received message
          //t1->port_status[i] = t1->port_status[i] | RX_READY; // jtm 3/31/2004
          }  
        else
          {
          //this is the DMA code for RME interrupt
          //if we are here then the DMA reception has gone to completion
          //(we have a complete frame)
          //get the number of bytes received (total)
          k = ((inp(t1->port_list[i]+RBCH)&0x0f)<<8)+inp(t1->port_list[i]+RBCL);
    
          t1->rxbuffer[i][t1->current_rxbuf[i]]->valid = 1;         //validate the frame buffer
          t1->rxbuffer[i][t1->current_rxbuf[i]]->no_bytes = k;      //indicate the number of bytes received
          t1->current_rxbuf[i]++;
          if(t1->current_rxbuf[i]==t1->max_rxbuf[i]) t1->current_rxbuf[i] = 0;
          if(t1->rxbuffer[i][t1->current_rxbuf[i]]->valid ==1) 
            {
            //   an error occured and the 
            //   main program isn't done with 
            //   the frame that is about
            //   to be used for received data
            t1->port_status[i] = t1->port_status[i] | RX_BUFFER_OVERFLOW;
            }
          t1->rxbuffer[i][t1->current_rxbuf[i]]->no_bytes = 0;  //prep buf for use
          t1->rxbuffer[i][t1->current_rxbuf[i]]->valid = 0;    //invalidate it for use
          setupdmar(i,t1->rxbuffer[i][t1->current_rxbuf[i]]->frame); //sets address of rbuf.frame[] to dma channel and enables it
          outp(t1->port_list[i]+CMDR,RMC);     //release the FIFO
          t1->port_status[i] = t1->port_status[i] | RX_READY;
          }
        }
    
      if(t1->eopmode[i]==OPMODE_ASYNC)
        {
        //TCD interrupt
        //do check for dma..in which case we don't need to get any data here...just need to figure out
        //how many bytes are in the buffer        
        if(t1->port_dmar_list[i]==0)  // no DMA
          {
          k = inp(t1->port_list[i]+RBCL);//get num bytes in fifo
          k = k&0x1f;
          if(k==0) k=32;
        
          //assumes that k is allways a multiple of 2 (status byte included mode)
          irxbuf = (t1->rxbuffer[i][t1->current_rxbuf[i]]);     
          //bloc = (unsigned far *)irxbuf->frame;
          for(j=irxbuf->no_bytes;j<irxbuf->no_bytes+k;j++)  irxbuf->frame[j] = inp(t1->port_list[i]+FIFO);  //******inpw?
        
          outp(t1->port_list[i]+CMDR,RMC);     //release fifo
          //could move this stuff elsewhere (such that the buffers get more than 32 bytes in em)
          irxbuf->valid = 1;//validate rbuf
          irxbuf->no_bytes = irxbuf->no_bytes + k; //number of bytes received in rbuf
          t1->current_rxbuf[i]++;  
          //irxbuf is no longer valid
          if(t1->current_rxbuf[i]==t1->max_rxbuf[i]) t1->current_rxbuf[i]=0;                
          if(t1->rxbuffer[i][t1->current_rxbuf[i]]->valid ==1) 
            {
            // an error occured and the 
            //   main program isn't done with 
            //   the frame that is about
            //   to be used for received data
            // set the receive buffers overflowed bit of the status word for this port here
            t1->port_status[i] = t1->port_status[i] | RX_BUFFER_OVERFLOW;
            }
          t1->rxbuffer[i][t1->current_rxbuf[i]]->no_bytes = 0;  //prep buf for use
          t1->rxbuffer[i][t1->current_rxbuf[i]]->valid = 0;    //invalidate it for use
          t1->port_status[i] = t1->port_status[i] | RX_READY;
          }                                                 
        else  // use DMA
          {
          //get dmar count here and make buffer valid
          k = getdmarcount(i);
          //assumes that the number of bytes transfered(dma) = number of bytes received
          //this isn't allways the case...
    
          //assumes that k is allways a multiple of 2 (status byte included mode)
          irxbuf = (t1->rxbuffer[i][t1->current_rxbuf[i]]);     
          irxbuf->valid = 1;//validate rbuf
          irxbuf->no_bytes = k; //number of bytes received in rbuf
          t1->current_rxbuf[i]++;  
          //irxbuf is no longer valid
          if(t1->current_rxbuf[i]==t1->max_rxbuf[i]) t1->current_rxbuf[i]=0;                
          if(t1->rxbuffer[i][t1->current_rxbuf[i]]->valid ==1) 
            {
            // an error occured and the 
            //   main program isn't done with 
            //   the frame that is about
            //   to be used for received data
            //set the receive buffers overflowed bit of the status word for this port here
            t1->port_status[i] = t1->port_status[i] | RX_BUFFER_OVERFLOW;
            }
          t1->rxbuffer[i][t1->current_rxbuf[i]]->no_bytes = 0;  //prep buf for use
          t1->rxbuffer[i][t1->current_rxbuf[i]]->valid = 0;    //invalidate it for use
          t1->port_status[i] = t1->port_status[i] | RX_READY;
          setupdmar(i,t1->rxbuffer[i][t1->current_rxbuf[i]]->frame);
          inp(t1->port_list[i]+RBCL);
          outp(t1->port_list[i]+CMDR,RMC);     //release fifo
          }   // end use DMA
        } // end ASYNC

      if(t1->eopmode[i]==OPMODE_BISYNC)
        {
        //TCD interrupt
        
        k = inp(t1->port_list[i]+RBCL);//get num bytes in fifo
        k = k&0x1f;
        if(k==0) k=32;
        
        //assumes that k is allways a multiple of 2 (status byte included mode)
        irxbuf = (t1->rxbuffer[i][t1->current_rxbuf[i]]);     
        //bloc = (unsigned far *)irxbuf->frame;
        for(j=irxbuf->no_bytes;j<irxbuf->no_bytes+k;j++)
          irxbuf->frame[j] = inp(t1->port_list[i]+FIFO); //****inpw?
        while((inp(t1->port_list[i]+STAR)&CEC)==CEC);
        outp(t1->port_list[i]+CMDR,RMC);     //release fifo
        //could move this stuff elsewhere (such that the buffers get more than 32 bytes in em)
        irxbuf->valid = 1;//validate rbuf
        irxbuf->no_bytes = irxbuf->no_bytes + k; //number of bytes received in rbuf
        t1->current_rxbuf[i]++;  
        //irxbuf is no longer valid
        if(t1->current_rxbuf[i]==t1->max_rxbuf[i]) t1->current_rxbuf[i]=0;                
        if(t1->rxbuffer[i][t1->current_rxbuf[i]]->valid ==1) 
          {
          // an error occured and the 
          //   main program isn't done with 
          //   the frame that is about
          //   to be used for received data
          // set the receive buffers overflowed bit of the status word for this port here
          t1->port_status[i] = t1->port_status[i] | RX_BUFFER_OVERFLOW;
          }
        t1->rxbuffer[i][t1->current_rxbuf[i]]->no_bytes = 0;  //prep buf for use
        t1->rxbuffer[i][t1->current_rxbuf[i]]->valid = 0;    //invalidate it for use
        t1->port_status[i] = t1->port_status[i] | RX_READY;
        while((inp(t1->port_list[i]+STAR)&CEC)==CEC);
        outp(t1->port_list[i]+CMDR,HUNT);     //go back to hunt mode
        }  //  end BISYNC                                                 
      } // end RME
      
    
    if((isr0&RFS)==RFS)   //  RFS Receive Frame Start
      {
      if(t1->eopmode[i]==OPMODE_HDLC)
        {     
        //RFS interrupt time to start the coffee
        t1->port_status[i] = t1->port_status[i] | RFS_INTERRUPT;
        }
        
      if(t1->eopmode[i]==OPMODE_ASYNC)
        {
        //TIME out interrupt
        //I think this would be a good time to pull the data from the fifo, issuing a 
        //RFRD command (0x20) to force the fifo open, the book says that it will force
        //a TCD interrupt which will collect the data
        
        if((inp(t1->port_list[i]+STAR)&0x20)==0x20) outp(t1->port_list[i]+CMDR,0x20);
        else
          {
          //here would be a good time to close out the current rxbuf and
          //start in on a new one...sending rxready...no more data is 
          //currently coming in so pass what we have on.
          irxbuf = (t1->rxbuffer[i][t1->current_rxbuf[i]]);     
          irxbuf->valid = 1;//validate rbuf
          t1->current_rxbuf[i]++;  
          //irxbuf is no longer valid
          if(t1->current_rxbuf[i]==t1->max_rxbuf[i]) t1->current_rxbuf[i]=0;                
          if(t1->rxbuffer[i][t1->current_rxbuf[i]]->valid ==1) 
            {
            // an error occured and the 
            // main program isn't done with 
            // the frame that is about
            // to be used for received data
            // set the receive buffers overflowed bit of the status word for this port here
            t1->port_status[i] = t1->port_status[i] | RX_BUFFER_OVERFLOW;
            }
            
          t1->rxbuffer[i][t1->current_rxbuf[i]]->no_bytes = 0;  //prep buf for use
          t1->rxbuffer[i][t1->current_rxbuf[i]]->valid = 0;    //invalidate it for use
          t1->port_status[i] = t1->port_status[i] | RX_READY;
          }
          
        t1->port_status[i] = t1->port_status[i] | REC_TIMEOUT;
        }  // end ASYNC                             
        //shouldn't happen in bisync mode
      } // end RFS
      
    
    if((isr0&RSC)==RSC)     //  RSC Receive Status Change.  Significant in auto-mode only.
      {
      if(t1->eopmode[i]==OPMODE_HDLC)
        {     
        //RSC interrupt (either got a RR or a RNR from HDLC link in auto mode
        t1->port_status[i] = t1->port_status[i] | RSC_INTERRUPT;
        }
        
      if(t1->eopmode[i]==OPMODE_ASYNC)
        {
        //parrity error interrupt         
        t1->port_status[i] = t1->port_status[i] | PARITY_ERROR;
        }
          
      if(t1->eopmode[i]==OPMODE_BISYNC)
        {
        //parrity error interrupt         
        t1->port_status[i] = t1->port_status[i] | PARITY_ERROR;
        }
      }  // end RSC
      
    
    if((isr0&PCE)==PCE)  //  PCE Protocol Error.  Significant in auto-mode only.
      {
      if(t1->eopmode[i]==OPMODE_HDLC)
        {     
        //protocol error (auto mode only)            
        t1->port_status[i] = t1->port_status[i] | PCE_INTERRUPT;
        }
        
      if(t1->eopmode[i]==OPMODE_ASYNC)
        {
        //framing error interrupt                         
        t1->port_status[i] = t1->port_status[i] | FRAMING_ERROR;
        } 
        
      if(t1->eopmode[i]==OPMODE_BISYNC)
        {
        //SYN detected
        t1->port_status[i] = t1->port_status[i] | SYN_DETECTED;
        } 
      } // end PCE
      
    
    if((isr0&PLLA)==PLLA)  //  PLLA DPLL Asynchronous.
      {
      if(t1->eopmode[i]==OPMODE_HDLC)
        {
        //DPLL async interrupt (lost clock sync)
        }
        
      if(t1->eopmode[i]==OPMODE_ASYNC)
        {
        //DPLL async interrupt (lost clock sync)
        }
        
      if(t1->eopmode[i]==OPMODE_BISYNC)
        {
        //DPLL async interrupt (lost clock sync)
        }
      } // end PLLA
      
    
    if((isr0&CDSC)==CDSC)  //  CDSC  Carrier Detect Status Change.
      {
      if(t1->eopmode[i]==OPMODE_HDLC)
        {
        //carrier detect changed state
        }
        
      if(t1->eopmode[i]==OPMODE_ASYNC)
        {
        //carrier detect changed state
        }
        
      if(t1->eopmode[i]==OPMODE_BISYNC)
        {
        //carrier detect changed state
        }
      }  // end CDSC
        
    
    if((isr0&RFO)==RFO)  //  RFO Receive Frame Overflow.
      {
      if(t1->eopmode[i]==OPMODE_HDLC)
        {
        //RFO interrupt                              
        t1->port_status[i] = t1->port_status[i] | RFO_INTERRUPT;
        outp(t1->port_list[i]+CMDR,RHR); // reset the HDLC receiver and RFIFO
        }
          
      if(t1->eopmode[i]==OPMODE_ASYNC)
        {
        //receive overflow                                      
        t1->port_status[i] = t1->port_status[i] | RFO_INTERRUPT;
        //must be reset here or will be locked forever
        }
        
      if(t1->eopmode[i]==OPMODE_BISYNC)
        {
        //receive overflow                                      
        t1->port_status[i] = t1->port_status[i] | RFO_INTERRUPT;
        //must be reset here or will be locked forever
        }
      }  // end RFO
      
      
    if((isr0&RPF)==RPF)  //  RPF Receive Pool Full.  Interrupt mode only.
      {
      if(t1->eopmode[i]==OPMODE_HDLC)
        {                                                            
        //RPF interrupt, exactly 32 bytes are ready
        //do this for all but DMA (we shouldn't get here in DMA mode)
        irxbuf = (t1->rxbuffer[i][t1->current_rxbuf[i]]);
        bloc = (unsigned far *)irxbuf->frame;
        for(j=(irxbuf->no_bytes>>1);j<(irxbuf->no_bytes>>1)+16;j++)
          {
          bloc[j] = inpw(t1->port_list[i]+FIFO);
          }
          
        outp(t1->port_list[i]+CMDR,RMC);     //release rfifo
        irxbuf->no_bytes += 32;
        } 
      
      if(t1->eopmode[i]==OPMODE_ASYNC)
        {
        //DATA IS READY                                             
        k = inp(t1->port_list[i]+RBCL);//get num bytes in fifo
        k = k&0x1f;
        if(k==0) k=32;
        //assumes that k is allways a multiple of 2 (status byte included mode)
        irxbuf = (t1->rxbuffer[i][t1->current_rxbuf[i]]);     
        bloc = (unsigned far *)irxbuf->frame;
        for(j=irxbuf->no_bytes>>1;j<(irxbuf->no_bytes+k)>>1;j++)  bloc[j] = inpw(t1->port_list[i]+FIFO);
        outp(t1->port_list[i]+CMDR,RMC);     //release fifo
        //could move this stuff elsewhere (such that the buffers get more than 32 bytes in em)
        irxbuf->no_bytes = irxbuf->no_bytes + k; //number of bytes received in rbuf
        if(irxbuf->no_bytes >= (FRAME_SIZE - 32))
          {
          irxbuf->valid = 1;//validate rbuf
          t1->current_rxbuf[i]++;  
          //irxbuf is no longer valid
          if(t1->current_rxbuf[i]==t1->max_rxbuf[i]) t1->current_rxbuf[i]=0;                
          if(t1->rxbuffer[i][t1->current_rxbuf[i]]->valid ==1) 
            {
            // an error occured and the 
            //   main program isn't done with 
            //   the frame that is about
            //   to be used for received data
            //set the receive buffers overflowed bit of the status word for this port here
            t1->port_status[i] = t1->port_status[i] | RX_BUFFER_OVERFLOW;
            }
            
          t1->rxbuffer[i][t1->current_rxbuf[i]]->no_bytes = 0;  //prep buf for use
          t1->rxbuffer[i][t1->current_rxbuf[i]]->valid = 0;    //invalidate it for use
          t1->port_status[i] = t1->port_status[i] | RX_READY;
          }
        }     
      
      if(t1->eopmode[i]==OPMODE_BISYNC)
        {
        //DATA IS READY                                             
        k = inp(t1->port_list[i]+RBCL);//get num bytes in fifo
        k = k&0x1f;
        if(k==0) k=32;
        
        //assumes that k is allways a multiple of 2 (status byte included mode)
        //this will hold true unless the programmed threshold = 1 in non parity store mode
        //ie this code will not work for threshold level 1, in no parity store mode!!!!!
        irxbuf = (t1->rxbuffer[i][t1->current_rxbuf[i]]);     
        bloc = (unsigned far *)irxbuf->frame;
        for(j=irxbuf->no_bytes>>1;j<(irxbuf->no_bytes+k)>>1;j++)
          {
          bloc[j] = inpw(t1->port_list[i]+FIFO);
          }
          
        outp(t1->port_list[i]+CMDR,RMC);     //release fifo
        //could move this stuff elsewhere (such that the buffers get more than 32 bytes in em)
        irxbuf->no_bytes = irxbuf->no_bytes + k; //number of bytes received in rbuf
        if(irxbuf->no_bytes >=( FRAME_SIZE - 32))
          {
          irxbuf->valid = 1;//validate rbuf
          t1->current_rxbuf[i]++;  
          //irxbuf is no longer valid
          if(t1->current_rxbuf[i]==t1->max_rxbuf[i]) t1->current_rxbuf[i]=0;                
          if(t1->rxbuffer[i][t1->current_rxbuf[i]]->valid ==1) 
            {
            // an error occured and the 
            //   main program isn't done with 
            //   the frame that is about
            //   to be used for received data
            //set the receive buffers overflowed bit of the status word for this port here
            t1->port_status[i] = t1->port_status[i] | RX_BUFFER_OVERFLOW;
            }
            
          t1->rxbuffer[i][t1->current_rxbuf[i]]->no_bytes = 0;  //prep buf for use
          t1->rxbuffer[i][t1->current_rxbuf[i]]->valid = 0;    //invalidate it for use
          t1->port_status[i] = t1->port_status[i] | RX_READY;
          }
        }     
      } // end RPF
    
    
    
    //check ISR1 next  
    if((isr1&EOP)==EOP)  //EOP End of Poll Sequence Detected.  Valid only in SLDC Loop mode.
      { 
      if(t1->eopmode[i]==OPMODE_HDLC)
        {
        //end of poll
        }            
        
      if(t1->eopmode[i]==OPMODE_ASYNC)
        {
        //break detect interrupt                                  
        t1->port_status[i] = t1->port_status[i] | BREAK_DETECTED;
        }       
      }  // end EOP
      
    
    
    if((isr1&0x40)==0x40) // OLP/RDO On Loop(SDLC Loop mode) / Receive Data Overflow(n/a in SDLC Loop)
      {               
      if(t1->eopmode[i]==OPMODE_HDLC)  //  OLP
        {
        //on loop interrupt
        }
      if(t1->eopmode[i]==OPMODE_ASYNC)  //  RDO
        {                  
        //break terminated interrupt
        t1->port_status[i] = t1->port_status[i] | BREAK_TERMINATED;
        }                  
      }  // end OLP/RDO
      
    
    if((isr1&0x20)==0x20)  //  AOLP/ALLS Active On Loop(SDLC Loop mode) / All Sent(non-SDLC Loop)
      {               
      if(t1->eopmode[i]==OPMODE_HDLC)
        {
        //all sent interrupt (tx machine is empty)
        }                                         
        
      if(t1->eopmode[i]==OPMODE_ASYNC)
        {
        //all characters are sent out the fifo and out the txd pin
        t1->port_status[i] = t1->port_status[i] | ALLSENT_INTERRUPT;
        }
        
      if(t1->eopmode[i]==OPMODE_BISYNC)
        {
        //all characters are sent out the fifo and out the txd pin
        t1->port_status[i] = t1->port_status[i] | ALLSENT_INTERRUPT;
        }
      } // end AOLP/ALLS
      
    
    if((isr1&0x10)==0x10) //  XDU/EXE Transmit Data Underrun/Extended Transmission End
      {
      if(t1->eopmode[i]==OPMODE_HDLC)
        {
        //EXE interupt                               
        t1->port_status[i] = t1->port_status[i] | EXE_INTERRUPT;
        //outp(t1->port_list[i]+CMDR,XRES);
        //need something here to reset the dma channel if operating in Extended trasparent mode
        //as this is the normal transmit end message for that mode
        }
          
      if(t1->eopmode[i]==OPMODE_BISYNC)
        {
        //EXE interupt                               
        t1->port_status[i] = t1->port_status[i] | EXE_INTERRUPT;
        }
      } // end XDU/EXE

        
    if((isr1&TIN)==TIN)  // Timer Interrupt
      {
      if(t1->eopmode[i]==OPMODE_HDLC)
        {        
        if(i==1)
          {
          //frame++;  // jtm used for frame timing in cb2000
          }

        t1->port_status[i] = t1->port_status[i] | TIMER_INTERRUPT;
        t1->timer_status[i] = 1;
        
        //outp(t1->port_list[i]+TIMR,inp(t1->port_list[i]+TIMR));//stops timer    
        }
        
      if(t1->eopmode[i]==OPMODE_ASYNC)
        {
        t1->port_status[i] = t1->port_status[i] | TIMER_INTERRUPT;
        t1->timer_status[i] = 1;
        }
    
      if(t1->eopmode[i]==OPMODE_BISYNC)
        {
        t1->port_status[i] = t1->port_status[i] | TIMER_INTERRUPT;
        t1->timer_status[i] = 1;
        }
      } // end TIN
      
      
    if((isr1&CSC)==CSC)  // Clear to Send Status Change
      {
      if(t1->eopmode[i]==OPMODE_HDLC)
        {     
        //cts status change                                    
        t1->port_status[i] = t1->port_status[i] | CTS_INTERRUPT;
        }
        
      if(t1->eopmode[i]==OPMODE_ASYNC)
        {
        //CTS changed state
        t1->port_status[i] = t1->port_status[i] | CTS_INTERRUPT;
        }
        
      if(t1->eopmode[i]==OPMODE_BISYNC)
        {
        //CTS changed state
        t1->port_status[i] = t1->port_status[i] | CTS_INTERRUPT;
        }
      } // end CSC
      
    
    if((isr1&0x02)==0x02)  // XMR  Transmit Message Repeat
      {
      if(t1->eopmode[i]==OPMODE_HDLC)
        {       
        //XMR interrupt
        t1->port_status[i] = t1->port_status[i] | XMR_INTERRUPT;
        //could do an auto resend here ...possibly (indicates transmit message repeat)
        }
    
      if(t1->eopmode[i]==OPMODE_BISYNC)
        {       
        //XMR interrupt
        t1->port_status[i] = t1->port_status[i] | XMR_INTERRUPT;
        //could do an auto resend here ...possibly (indicates transmit message repeat)
        }
      } // end XMR
      
  
    if((isr1&XPR)==XPR)  //  Transmit Pool Ready
      {
      if((t1->eopmode[i]==OPMODE_HDLC)||(t1->eopmode[i]==OPMODE_BISYNC))
        {
        //XPR interrupt (time to send the bytes out)            
        if(t1->port_dmat_list[i]==0)  // no dma, so do interrupt xpr here  
          {
          //try_again_tx:                                
          itxbuf = (t1->txbuffer[i][t1->current_txbuf[i]]);
          bloc = (unsigned far *)itxbuf->frame;
          
          if(itxbuf->valid==1)
            {  // note: ->max is the total num bytes in frame
               //       ->no_bytes is num bytes already sent
            if((itxbuf->max - itxbuf->no_bytes) > 32)
              { // if more than 32 bytes still to be sent
                // this is somewhere in the middle of the frame
                // load exactly 32 bytes into TFIFO (word writes since even number bytes)
              for(j=(itxbuf->no_bytes)>>1;j<(itxbuf->no_bytes>>1)+16;j++)
                {
                outpw(t1->port_list[i]+FIFO,bloc[j]);      // load the fifo 
                }
                
              outp(t1->port_list[i]+CMDR,t1->tx_type[i]);  // tx the bytes
              itxbuf->no_bytes += 32;                      // update num bytes sent
              t1->istxing[i] = 1;                          // flag that we're still tx'ing
              }
            else
              { // there are 32 or fewer bytes remaining to tx
                // this is the end of the frame
                // load remaining bytes into TFIFO (byte writes since may be odd number)
              for(j=itxbuf->no_bytes;j<itxbuf->max;j++)
                {
                outp(t1->port_list[i]+FIFO,itxbuf->frame[j]);  // load the fifo
                }
              
                // tell the transmitter to tx the bytes, the crc, and the closing flag
              outp(t1->port_list[i]+CMDR,t1->tx_type[i]+XME);
              itxbuf->no_bytes = itxbuf->max;    // update num bytes sent
              itxbuf->valid = 0;                 // mark the buffer as unused
              t1->current_txbuf[i]++;            // select the next buffer, rollover if necessary
              if(t1->current_txbuf[i]==t1->max_txbuf[i]) t1->current_txbuf[i] = 0;
              //goto try_again_tx;
              }
            }
            
          else
            {
            t1->istxing[i] = 0; //no valid frames we are done txing
            }
          } // end interrupt, no DMA
  
        else
          {
          
          //  do DMA xpr here
          if(t1->txbuffer[i][t1->current_txbuf[i]]->valid==1)
            {
            t1->txbuffer[i][t1->current_txbuf[i]]->valid = 0;   
            t1->txbuffer[i][t1->current_txbuf[i]]->no_bytes =
                t1->txbuffer[i][t1->current_txbuf[i]]->max;
            t1->istxing[i] = 0;
            t1->current_txbuf[i]++;
            if(t1->current_txbuf[i]==t1->max_txbuf[i])
              {
              t1->current_txbuf[i]=0;
              }
            if(t1->txbuffer[i][t1->current_txbuf[i]]->valid ==1)
              {
              setupdmat(i,t1->txbuffer[i][t1->current_txbuf[i]]->frame,t1->txbuffer[i][t1->current_txbuf[i]]->max);
              t1->istxing[i] = 1;
              outp(t1->port_list[i]+CMDR,t1->tx_type[i]);
              }
            }  // end if current_txbuf[] == valid         
          //t1->tx_port(1, pcm, 96);  
          } // end DMA
        }  // end of HDLC
    
    
      if(t1->eopmode[i]==OPMODE_ASYNC)
        {
        //transmit data ready interrupt
        if(t1->port_dmat_list[i]==0)
          {
          itxbuf = (t1->txbuffer[i][t1->current_txbuf[i]]);
          bloc = (unsigned far *)itxbuf->frame;
          if(itxbuf->valid==1)
            {
            if((itxbuf->max - itxbuf->no_bytes) > 32)
              {
              //do send 32
              for(j=(itxbuf->no_bytes)>>1;j<(itxbuf->no_bytes>>1)+16;j++)
              outpw(t1->port_list[i]+FIFO,bloc[j]); 
              outp(t1->port_list[i]+CMDR,0x08);
              itxbuf->no_bytes += 32;
              t1->istxing[i] = 1;
              }
            else
              {
              //do send <=32 //sending as byte accesses such that odd bytes can go out
              for(j=itxbuf->no_bytes;j<itxbuf->max;j++)
              outp(t1->port_list[i]+FIFO,itxbuf->frame[j]);
              outp(t1->port_list[i]+CMDR,0x08);
              itxbuf->no_bytes = itxbuf->max;
              itxbuf->valid = 0;
              t1->current_txbuf[i]++;
              if(t1->current_txbuf[i]==t1->max_txbuf[i])
                t1->current_txbuf[i] = 0;
              //goto try_again_tx;
              }
            }
          else
            {
            t1->istxing[i] = 0; //no valid frames we are done txing
            }
          }
        else
          {
          //do DMA xpr here
          if(t1->txbuffer[i][t1->current_txbuf[i]]->valid==1)
            {
            t1->txbuffer[i][t1->current_txbuf[i]]->valid = 0;   
            t1->txbuffer[i][t1->current_txbuf[i]]->no_bytes = t1->txbuffer[i][t1->current_txbuf[i]]->max;
            t1->istxing[i] = 0;
            t1->current_txbuf[i]++;
            if(t1->current_txbuf[i]==t1->max_txbuf[i]) t1->current_txbuf[i]=0;
            if(t1->txbuffer[i][t1->current_txbuf[i]]->valid ==1)
              {
              setupdmat(i,t1->txbuffer[i][t1->current_txbuf[i]]->frame,t1->txbuffer[i][t1->current_txbuf[i]]->max);
              t1->istxing[i] = 1;
              outp(t1->port_list[i]+CMDR,0x08);
              }  // end if()
            }  // end if()
          }  // end else do DMA
        } // end do ASYNC
      }  // end XPR
        
    
    if((pis&0x80)==0x80)
      {
      //DMA TC reached
      }               
      
    
    if((pis&0x40)==0x40)
      {
      if((_inp(0x240+PVR) & 0x40) == 0x40);
        //com_request = 1;     //DSR channel 1 changed
      }
    
    
    if((pis&0x20)==0x20)
      {
      //DSR channel 0 changed
      }
        
    nextpt:
    
  
    // restore escc ports to channel
    //  that was active when ISR started
    outp(t1->port_list[i]+PVR,(inp(t1->port_list[i]+PVR)&0xfe)+chanstor); //set channel
    
    }  // end for all open ports
  
  if(inthit==1) goto startisr;
  
  //_disable();  // rem out 12/08/03
  
/*
  if(ok_to_tx)
    {
    ok_to_tx = 0;
    t1->tx_port(1, pcm,96);
    }
*/  
  // no more interrupts set
  // do eoi's and leave
  outp(0x20,0x20);
  if(t1->upper_irq!=0) outp(0xa0,0x20);
  } //  end isr






void Cescc::setupdmat(unsigned port, void far *address,unsigned numbytes)
  {
  unsigned i,size;
  unsigned long linaddress;
  unsigned chanstor;
  
  if (numbytes == 0) size = FRAME_SIZE;
  else size = numbytes;
  
  if(t1->port_dmat_list[port]>3)
    {
    size = (size/2) +(numbytes&1);//word transfers...half the size..add 1 for odd bytelenghts
    }

  if(t1->port_dmat_list[port]==1)
    {
    outp(DMA_COMMAND_LO,4);             //disables 8 bit dma
    outp(DMA_MODE_LO,0x19);             //sets demand xfer on ch1
    outp(DMA_CLEAR_FFLO,0);             //reset count hi/lo
    outp(DMA_COUNT_CH1,(size-1)&0xff);    //program byte count 
    outp(DMA_COUNT_CH1,(size-1)>>8);      //controller will do 
    linaddress = (((unsigned long)address & 0xffff0000)>>12) + ((unsigned long)address & 0xffff);
    outp(DMA_PAGE_CH1,(char)(linaddress>>16));             //calculate linear address of buffer
    outp(DMA_CLEAR_FFLO,0);                     //and store in DMA controller
    outp(DMA_ADD_CH1,(char)(linaddress&0xff));
    outp(DMA_ADD_CH1,(char)(linaddress>>8));
    outp(DMA_MASK_LO,1);                        //enable ch1 mask
    outp(DMA_COMMAND_LO,0);                     //enable 8bit dma
    }
  
  if(t1->port_dmat_list[port]==3)
    {
    outp(DMA_COMMAND_LO,4);
    outp(DMA_MODE_LO,0x1B);
    outp(DMA_CLEAR_FFLO,0);
    outp(DMA_COUNT_CH3,(size-1)&0xff);
    outp(DMA_COUNT_CH3,(size-1)>>8);
    linaddress = (((unsigned long)address & 0xffff0000)>>12) + ((unsigned long)address & 0xffff);
    outp(DMA_PAGE_CH3,(char)(linaddress>>16));
    outp(DMA_CLEAR_FFLO,0);
    outp(DMA_ADD_CH3,(char)(linaddress&0xff));
    outp(DMA_ADD_CH3,(char)(linaddress>>8));
    outp(DMA_MASK_LO,3);
    outp(DMA_COMMAND_LO,0);
    }
  
  if(t1->port_dmat_list[port]==5)
    {
    outp(DMA_COMMAND_HI,4);     //disable dma
    outp(DMA_MODE_HI,0x19);     //demand xfr on ch5
    outp(DMA_CLEAR_FFHI,0);     //reset flip-flop
    outp(DMA_COUNT_CH5,(size-1)&0xff);
    outp(DMA_COUNT_CH5,(size-1)>>8);
    linaddress = (((unsigned long)address & 0xffff0000)>>12) + ((unsigned long)address & 0xffff);
    outp(DMA_PAGE_CH5,(char)(linaddress>>16));
    linaddress = linaddress>>1; //adjust for 16bit dma
    outp(DMA_CLEAR_FFHI,0);
    outp(DMA_ADD_CH5,(char)(linaddress&0xff));
    outp(DMA_ADD_CH5,(char)(linaddress>>8));
    outp(DMA_MASK_HI,1);        //enable chan5
    outp(DMA_COMMAND_HI,0);     //enable dma
    }

  if(t1->port_dmat_list[port]==7)
    {
    outp(DMA_COMMAND_HI,4);     //disable dma
    outp(DMA_MODE_HI,0x1b);     //demand xfr on ch7
    outp(DMA_CLEAR_FFHI,0);     //reset flip-flop
    outp(DMA_COUNT_CH7,(size-1)&0xff);
    outp(DMA_COUNT_CH7,(size-1)>>8);
    linaddress = (((unsigned long)address & 0xffff0000)>>12) + ((unsigned long)address & 0xffff);
    outp(DMA_PAGE_CH7,(char)(linaddress>>16));
    linaddress = linaddress>>1; //adjust for 16bit dma
    outp(DMA_CLEAR_FFHI,0);
    outp(DMA_ADD_CH7,(char)(linaddress&0xff));
    outp(DMA_ADD_CH7,(char)(linaddress>>8));
    outp(DMA_MASK_HI,3);        //enable chan7
    outp(DMA_COMMAND_HI,0);     //enable dma
    }

  if(numbytes ==0) size =FRAME_SIZE;
  else size = numbytes;
  i = size-1;
  i = i&0x0fff;
  i = i|0x8000;
  chanstor = inp(t1->port_list[port]+PVR)&0x1;
  outp(t1->port_list[port]+PVR,(inp(t1->port_list[port]+PVR)&0xfe)+t1->channel[port]); //set channel
  outp(t1->port_list[port]+XBCL,i&0xff);
  outp(t1->port_list[port]+XBCH,i>>8);    //set for dma mode and FRAME_SIZE bytes per frame
  outp(t1->port_list[port]+PVR,(inp(t1->port_list[port]+PVR)&0xfe)+chanstor); //set channel
  
  }





void Cescc::setupdmar(unsigned port,void far *address)
  {
  unsigned long linaddress;
  unsigned size;

  size = 0;
  //size = 32; //so max received is framesize+32
  //the extra transfers are added to the frame size so if FRAME_SIZE+1 
  //( the +1 is for the status bytes that is pushed into the RXFIFO) is set to 
  //a non multiple of 32 the receive[] array will be valid and not overwritten
  //by the unexpected transfers at the end of a frame. and subsequent frames
  //will be received correctly
  //in other words if a frame size of say 40 is used:
  // frame_size+1 = 41
  // then ther @ @    ÿÿÿ›    yte DMA cycle and leaving 9 bytes to be transfered
  // 9 bytes will force a 16 byte DMA cycle giving us 7 extra bytes
  // so we add 7 to the frame size to get the total number of cycles in a
  // received frame( the received length will be 48 bytes 41 of which are valid)
  if(t1->port_dmar_list[port]==1)
    {
    outp(DMA_COMMAND_LO,4);     //disable dma
    outp(DMA_MODE_LO,0x15);     //demand xfer ch1
    outp(DMA_CLEAR_FFLO,0);     //clear ctrlr flip flop
    outp(DMA_COUNT_CH1,(FRAME_SIZE+size-1)&0xff);//write corrected word count
    outp(DMA_COUNT_CH1,(FRAME_SIZE+size-1)>>8);
    linaddress = (((unsigned long)address & 0xffff0000)>>12) + ((unsigned long)address & 0xffff);
    outp(DMA_PAGE_CH1,(char)(linaddress>>16));
    outp(DMA_CLEAR_FFLO,0);
    outp(DMA_ADD_CH1,(char)(linaddress&0xff));     //and store it in the controller
    outp(DMA_ADD_CH1,(char)(linaddress>>8));
    outp(DMA_MASK_LO,1);                //unmask ch1
    outp(DMA_COMMAND_LO,0);             //enable DMA
    }
  
  if(t1->port_dmar_list[port]==3)
    {
    outp(DMA_COMMAND_LO,4);
    outp(DMA_MODE_LO,0x17);
    outp(DMA_CLEAR_FFLO,0);
    outp(DMA_COUNT_CH3,(FRAME_SIZE+size-1)&0xff);
    outp(DMA_COUNT_CH3,(FRAME_SIZE+size-1)>>8);
    linaddress = (((unsigned long)address & 0xffff0000)>>12) + ((unsigned long)address & 0xffff);
    outp(DMA_PAGE_CH3,(char)(linaddress>>16));
    outp(DMA_CLEAR_FFLO,0);
    outp(DMA_ADD_CH3,(char)(linaddress&0xff));
    outp(DMA_ADD_CH3,(char)(linaddress>>8));
    outp(DMA_MASK_LO,3);
    outp(DMA_COMMAND_LO,0);
    }
  
  if(t1->port_dmar_list[port]==5)
    {
    outp(DMA_COMMAND_HI,4);     //disable dma
    outp(DMA_MODE_HI,0x15);     //demand xfr on ch5
    outp(DMA_CLEAR_FFHI,0);     //reset flip-flop
    outp(DMA_COUNT_CH5,(((FRAME_SIZE+size)/2)-1)&0xff);
    outp(DMA_COUNT_CH5,(((FRAME_SIZE+size)/2)-1)>>8);
    linaddress = (((unsigned long)address & 0xffff0000)>>12) + ((unsigned long)address & 0xffff);
    outp(DMA_PAGE_CH5,(char)(linaddress>>16));
    linaddress = linaddress>>1; //adjust for 16bit dma
    outp(DMA_CLEAR_FFHI,0);
    outp(DMA_ADD_CH5,(char)(linaddress&0xff));
    outp(DMA_ADD_CH5,(char)(linaddress>>8));
    outp(DMA_MASK_HI,1);        //enable chan5
    outp(DMA_COMMAND_HI,0);     //enable dma
    }
  
  if(t1->port_dmar_list[port]==7)
    {
    outp(DMA_COMMAND_HI,4);     //disable dma
    outp(DMA_MODE_HI,0x17);     //demand xfr on ch7
    outp(DMA_CLEAR_FFHI,0);     //reset flip-flop
    outp(DMA_COUNT_CH7,(((FRAME_SIZE+size)/2)-1)&0xff);
    outp(DMA_COUNT_CH7,(((FRAME_SIZE+size)/2)-1)>>8);
    linaddress = (((unsigned long)address & 0xffff0000)>>12) + ((unsigned long)address & 0xffff);
    outp(DMA_PAGE_CH7,(char)(linaddress>>16));
    linaddress = linaddress>>1; //adjust for 16bit dma
    outp(DMA_CLEAR_FFHI,0);
    outp(DMA_ADD_CH7,(char)(linaddress&0xff));
    outp(DMA_ADD_CH7,(char)(linaddress>>8));
    outp(DMA_MASK_HI,3);        //enable chan7
    outp(DMA_COMMAND_HI,0);     //enable dma
    }
  
  //this really isn't necessary except for the DMA bit of XBCH
  //but the transmit count is programmed in setup_dmat()
  //so make sure that setup_dmar() is called before setup_dmat() if an
  //odd size frame is being sent..or you won't get what you expect
  //i = FRAME_SIZE-1;       //transmit XBCH:XBCL +1 bytes so set XBCH:XBCL to framesize-1
  //i = i&0x0fff;
  //i = i|0x8000;
  //outp(XBCL,i&0xff);
  //outp(XBCH,i>>8);    //set for dma mode and FRAME_SIZE bytes per frame
  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  //SETUPDMAT must be called before the receive section will work, preferably in init!!!
  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  //this is the only way to set the dma bit of XBCH without knowing the transmitted size
  }





unsigned Cescc::getdmarcount(unsigned port)
  {
  unsigned size;
  unsigned lo,hi;
  size = 0;
  if(t1->port_dmar_list[port]==1)
    {
    outp(DMA_CLEAR_FFLO,0);     //clear ctrlr flip flop
    lo = inp(DMA_COUNT_CH1);
    hi = inp(DMA_COUNT_CH1);
    size = (hi<<8) + lo;
    size = size +1;
    size = FRAME_SIZE - size;
    }

  if(t1->port_dmar_list[port]==3)
    {
    outp(DMA_CLEAR_FFLO,0);
    lo = inp(DMA_COUNT_CH3);
    hi = inp(DMA_COUNT_CH3);
    size = (hi<<8) + lo;
    size = size +1;
    size = FRAME_SIZE - size;
    }

  if(t1->port_dmar_list[port]==5)
    {
    outp(DMA_CLEAR_FFHI,0);     //reset flip-flop
    lo = inp(DMA_COUNT_CH5);
    hi = inp(DMA_COUNT_CH5);
    size = (hi<<8) + lo;
    size = size +1;
    size = size <<1;
    size = FRAME_SIZE - size;
    }

  if(t1->port_dmar_list[port]==7)
    {
    outp(DMA_CLEAR_FFHI,0);     //reset flip-flop
    lo = inp(DMA_COUNT_CH7);
    hi = inp(DMA_COUNT_CH7);
    size = (hi<<8) + lo;
    size = size +1;
    size = size <<1;
    size = FRAME_SIZE - size;
    }

  return size;
  }






//sets the onboard Cypress ICD2053B Programmable Clock Generator
//to divide the 18.432 MHz system oscillator to your desired oscillator speed
//see data sheet at Cypress Web site, and Cypress' BitCalc program.
void Cescc::set_clock_generator(unsigned port, unsigned long hval,unsigned nmbits)
  {
  unsigned base;
  unsigned curval;
  unsigned long tempval;
  unsigned i;

  base = port_list[port];
  curval = 0;
  //curval bit 2 = data bit      (DATA on ICD2053B, parallel port bit2 on ESCC)
  //curval bit 1 = clock toggle  (SCLK on ICD2053B, parallel port bit1 on ESCC)
  outp(base+PVR,curval);   //writing to PVR (0x1c) writes to ESCC universal port (parallel port)

  tempval = STARTWRD;      //0x1e05
  for(i=0;i<14;i++)        //writes 14 LSBs of STARTWRD into ICD2053 one bit at a time, LSB first
    {
    curval = 0;
    curval = (char)(tempval&0x1)<<2;  //select LSB and rotate up to bit2 position (DATA)
    outp(base+PVR,curval);              
    curval = curval |0x02;    //force rising edge on SCLK (bit1 on parallel port)
    outp(base+PVR,curval);    //clock data into ICD 2053
    curval = curval &0x04;    //force falling edge
    outp(base+PVR,curval);    //set clock low
    tempval = tempval >> 1;   //get next bit
    }
  
  tempval = hval;
  for(i=0;i<nmbits;i++)
    {
    curval = 0;
    curval = (char)(tempval&0x1)<<2;  //set bit
    outp(base+PVR,curval);
    curval = curval |0x02;    //force rising edge
    outp(base+PVR,curval);    //clock in data
    curval = curval &0x04;    //force falling edge
    outp(base+PVR,curval);    //set clock low
    tempval = tempval >> 1;   //get next bit
    }

  tempval = MIDWRD;        //0x1e04
  for(i=0;i<14;i++)
    {
    curval = 0;
    curval = (char)(tempval&0x1)<<2;  //set bit
    outp(base+PVR,curval);
    curval = curval |0x02;    //force rising edge
    outp(base+PVR,curval);    //clock in data
    curval = curval &0x04;    //force falling edge
    outp(base+PVR,curval);    //set clock low
    tempval = tempval >> 1;   //get next bit
    }
  
  //pause for >10ms --should be replaced with a regulation pause routine
  for(i=0;i<32000;i++) inp(0x20);

  tempval = ENDWRD;        //0x1e00
  for(i=0;i<14;i++)
    {
    curval = 0;
    curval = (char)(tempval&0x1)<<2;  //set bit
    outp(base+PVR,curval);
    curval = curval |0x02;    //force rising edge
    outp(base+PVR,curval);    //clock in data
    curval = curval &0x04;    //force falling edge
    outp(base+PVR,curval);    //set clock low
    tempval = tempval >> 1;   //get next bit
    }
  }
  
  
  
unsigned Cescc::wait_for_tx(unsigned port)
  {
  unsigned long i = 0;
  if(port_open_list[port]!=0)
    {
    while(istxing[port]==1)
      {
      // The screen writes inserted here are a desperation move to avoid
      // system crashes.  Perhaps it is a timing issue and the screen
      // writes provide a small delay.  They do reduce the likelihood
      // of system crashes substantially, but not completely.  The
      // problem may also be due to sporadic misbehavior by the Tech 80
      // motion controllers.  It was noted during development that
      // occasional screen writes eliminated the Tech 80 anomalies.
      // Go figure.  Let me know if you ever solve this problem.  JTM.
      /*if(i++ > 10000000)  // jtm remove for autonomous
        {
        cout << "\t\t\t\t\t\t\t\t\tX\r" << flush;
        return TRUE;
        }
      */  
      }
    return TRUE;
    }
  else 
    {
    //printf("L\r");
    return FALSE;
    }
  }
