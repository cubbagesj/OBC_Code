// dmm32.h
// 01/11/01
// jtm


#include <stdlib.h>
#include <conio.h>
#include <dos.h>
#include <iostream.h>

#define BASE 0x200
#define LOW_CHANNEL 0
#define HIGH_CHANNEL 31
#define FIFOEN 0x08
#define SCANEN 0x04
#define FIFORST 0x02
#define ADINTE 0x80
#define DINTE 0x40
#define TINTE 0x20
#define DALSB BASE+4
#define DAMSB BASE+5
#define DAUPDATE BASE+5


class Dmm32
  {
  private:
  public:
  Dmm32(void);
  ~Dmm32(void);
  
  void trigger(void);
  int read_data(void);
  int data_ready(void);
  void da_lsb(int);
  void da_msb(int);
  void da_update(void);
  void da_out(int, int);
    
  };
  
