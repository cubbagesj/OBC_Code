//  dmm32.cpp
//  Diamond MM-32 A/D card source module
//
//  JTM
//  01/10/01

#include "dmm32.h"

Dmm32::Dmm32(void)
  {  
  _outp(BASE + 2, LOW_CHANNEL);  // low channel
  _outp(BASE + 3, HIGH_CHANNEL);  // high channel
  _outp(BASE + 7, FIFOEN + SCANEN + FIFORST); //fifo control
  _outp(BASE + 9, 0x00);  // intr/clk control
  _outp(BASE + 11, 0x00);  // analog configuration, +- 5V input, gain = 1
  
  while(_inp(BASE + 11) & 0x80);   // wait for settling time
  }
  
Dmm32::~Dmm32(void)
  {
  }  
  
void Dmm32::trigger(void)
  {
  _outp(BASE, 0x00);  // write anything to base address to start conversions
  }
  
int Dmm32::read_data(void)
  {
  return(_inp(BASE) + (_inp(BASE + 1) << 8));     // low byte, high byte
  }
  
int Dmm32::data_ready(void)
  {
/*
  if(_inp(BASE + 8) && 0x80) return(1);
  return(0);
*/
  if(_inp(BASE + 8) & 0x80) return(0);//jtm bug fix (no more need for KKKKK)
  return(1);                          // 11/6/2006

  }


void Dmm32::da_out(int value, int channel)
  {
  //while(_inp(DALSB) & 0x80);     // wait till not busy
  _outp(DALSB, value & 0x00ff);  // load low byte
  //while(_inp(DALSB) & 0x80);     // wait
  _outp(DAMSB, (value / 256) + (channel * 64)); // load high nibble and channel number
  while(_inp(DALSB) & 0x80);     // wait
  _inp(DAMSB);                   // enable update
  }
      
void Dmm32::da_lsb(int lsb_value)
  {
  while(_inp(DALSB) & 0x80);
  _outp(DALSB, lsb_value);
  }
  
void Dmm32::da_msb(int msb_value)
  {
  while(_inp(DALSB) & 0x80);
  _outp(DAMSB, msb_value);
  }
  
void Dmm32::da_update(void)
  {
  while(_inp(DALSB) & 0x80);
  _inp(DAMSB);
  }
    