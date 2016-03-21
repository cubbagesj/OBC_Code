// ether.cpp module for autonomous obc project
// JTM Code 5600
// 2/27/2004



#include <dos.h>
#include <stdio.h>
#include <conio.h>
#include <string.h>
#include <stdlib.h>
#include "pktdrv.h"
#include "ether.h"
#include "dmmtest.h"

//unsigned udp_checksum;
//unsigned char rxbuf[MAX_RXBUF+200];
unsigned char rxbuf[2000];
unsigned char far * prxbuf = rxbuf;
//unsigned char far app_buffer[MAX_RXBUF+200];
extern unsigned char far cmd_buffer[2008];
extern unsigned char obc_mac[6];
extern unsigned char far data_pkt[MAX_RXBUF+200];

//extern commands;
extern unsigned long frame;
extern int time_since_last_command;
//extern struct SYS sys;
//extern struct OBS obs;
//extern struct DYNO dyno;
//extern struct PROP prop;
//extern struct LN200 ln200;
//extern struct ECHO echo;
//extern struct DS ds;
//extern struct ADCP adcp; 

struct COMMANDS  // new 12/8/2004
  {
  unsigned int  sync_word;
  unsigned int  mode;
  unsigned int  rpm;
  unsigned int  rudder;
  unsigned int  stern1;
  unsigned int  stern2;
  unsigned int  fore;  
  unsigned int  ballast;
  unsigned int  ln200_mode;
  unsigned int  ln200_init_heading;
  unsigned int  ln200_init_latitude;
/*
  unsigned int  adcp_reset;
  unsigned int  adcp_heading;
  unsigned int  ds_send_message;
  unsigned int  ds_dmgx;
  unsigned int  ds_dmgy;
  unsigned int  ds_dmgz;        
  unsigned int  spare1;
*/
  unsigned int  ds_send_message;   // new 8/9/06 JTM
  unsigned int  ds_dmgx1;
  unsigned int  ds_dmgx2;
  unsigned int  ds_dmgy1;
  unsigned int  ds_dmgy2;
  unsigned int  ds_dmgz1;
  unsigned int  ds_dmgz2;
  };  
 
extern COMMANDS commands;

struct TIMER
  {
  unsigned long high;
  unsigned int  low;
  };
  
struct ECHO            // new 12/08/2004
  {
  unsigned int  new_data;
  unsigned long frame;
  unsigned long time_high;
  unsigned int  time_low;
  unsigned char data[sizeof(commands)];
  };
  

//extern TIMER cmd_packet_time;
extern TIMER sys_timer;
extern TIMER cmds_time;

extern ECHO  echo;

char name[80];

unsigned char addr_buf[6];
unsigned char pc1_addr[6]       = {0x08, 0x00, 0x17, 0x08, 0x87, 0x4d}; // pc1
unsigned char pc2_addr[6]       = {0x08, 0x00, 0x17, 0x08, 0x6d, 0x91}; // pc2
unsigned char pc3_addr[6]       = {0x08, 0x00, 0x17, 0x08, 0x78, 0x50}; // pc3
unsigned char jim_pc[6]         = {0x00, 0x08, 0x74, 0x3c, 0x07, 0xe6}; // laptop
unsigned char broadcast_addr[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
//unsigned char control_mac[6]    = {0x00, 0x01, 0x45, 0x00, 0x8a, 0x21};
//unsigned char micron_mac[6]     = {0x08, 0x00, 0x17, 0x08, 0x87, 0x4d};
//unsigned char obc_mac[6]        = {0x00, 0x01, 0x45, 0x00, 0x7a, 0x84};

char ethertype[5];
int merror = 0;
int error_num = 0;
int carry_flag = 0;
int version = 0;
int etherclass = 0;
int type = 0;
int number = 0;
int setmode = 0;
int getmode = 0;
int functionality = 0;
int handle;
unsigned txlen;
int rx_flag =0;
int function = 0;
int last_function = 0;
int packet_len;
static int i, j;
int msg_rcvd;
int done = 0;
unsigned long packets_sent = 0;
unsigned long packets_rcvd = 0;
unsigned long receiver_calls = 0;
unsigned long ax0_counter = 0;
unsigned long ax1_counter = 0;
unsigned long discards = 0;
//unsigned ip_id = 0;
unsigned checksum16 = 0;



int driver_info(unsigned dhandle)
  {
  union REGS regs;
  struct SREGS sregs;
  int version;
  int iclass;
  int type;
  int number;
  char name[80];
  int functionality;

  regs.h.ah = DRIVER_INFO;
  regs.h.al = 255;
  regs.x.bx = dhandle;  //jtm
  int86x(PKTDVR, &regs, &regs, &sregs);
  version = regs.x.bx;
  iclass = regs.h.ch;
  type  = regs.x.dx;
  number = regs.h.cl;
  _fstrcpy(name, (char far *)MK_FP(sregs.ds, regs.x.si));
  functionality = regs.h.al;
  
  printf("Driver_info()\n");
  printf("Version: %d\n", version);
  printf("Ifclass: %d", iclass);
  if(iclass == CL_ETHERNET) printf(", DIX Bluebook\n");
  if(iclass == CL_IEEE8023) printf(", IEEE 802.3 w/802.2 headers\n");
  printf("Type: %d\n", type);
  printf("Number: %d\n", number);
  printf("Name: %s\n", name);
  printf("Functionality: %d", functionality);
  if(functionality == 1) printf(", basic\n");
  if(functionality == 2) printf(", basic, extended(*)\n");
  if(functionality == 5) printf(", basic, high performance(+)\n");
  if(functionality == 6) printf(", basic, high performance(+), extended(*)\n");
  if(functionality == 255) printf(", not installed\n");
  //last_function = function;
  
  if(regs.x.cflag) error_num = regs.h.dh;
  else error_num = 0;
  return(regs.x.cflag);
  // end driver_info()
  }

int access_type(int if_class, int if_type, int if_number, char far *type,
                     unsigned typelen,   void (interrupt far *a_receiver)
                     (unsigned, unsigned, unsigned, unsigned,
                      unsigned, unsigned, unsigned, unsigned,
                      unsigned, unsigned, unsigned, unsigned,
                      unsigned) )
  {
  union  _REGS regs;
  struct _SREGS sregs;

  _segread(&sregs);
  regs.h.ah = ACCESS_TYPE;
  regs.h.al = if_class;
  //regs.x.bx = if_type;
  regs.x.bx = 0xffff;
  regs.h.dl = if_number;
  regs.x.si = FP_OFF(type);
  sregs.ds = FP_SEG(type);
  //regs.x.cx = typelen;
  regs.x.cx = 0;
  sregs.es =  _FP_SEG(a_receiver);
  regs.x.di = _FP_OFF(a_receiver);
  _int86x(PKTDVR, &regs, &regs, &sregs);
  handle = regs.x.ax;    // handle is global
  //printf("handle: %d\n", handle);
  if(regs.x.cflag) error_num = regs.h.dh;
  else error_num = 0;
  return(regs.x.cflag);
  }

int release_type(int rhandle)
  {
  union REGS regs;
  struct SREGS sregs;

  regs.h.ah = 3;
  regs.x.bx = rhandle;
  int86x(PKTDVR, &regs, &regs, &sregs);
  last_function = function;
  if(regs.x.cflag) error_num = regs.h.dh;
  else error_num = 0;
  return(regs.x.cflag);
  }

int terminate(int thandle)
  {
  union REGS regs;

  regs.h.ah = 5;
  regs.x.bx = handle;
  int86(PKTDVR, &regs, &regs);
  last_function = function;
  if(regs.x.cflag) error_num = regs.h.dh;
  else error_num = 0;
  return(regs.x.cflag);
  }


int get_address(int ghandle, unsigned char far * pad_buf, int len)
  {
  union REGS regs;
  struct SREGS sregs;

  for(i=0;i<6;i++)
    {
    addr_buf[i] = 0;
    }
  regs.h.ah = 6;
  regs.x.bx = ghandle;
  sregs.es = FP_SEG(pad_buf);
  regs.x.di = FP_OFF(pad_buf);
  regs.x.cx = 6;
  int86x(PKTDVR, &regs, &regs, &sregs);
  /*printf("Address length: %d\n", regs.x.cx);
  printf("ES: %d\n", sregs.es);
  printf("DI: %d\n", regs.x.di);
  printf("FP SEG: %u\n", (unsigned char) FP_SEG(pad_buf));
  printf("FP OFF: %u\n", (unsigned char) FP_OFF(pad_buf));
  printf("Address: ");
  for(i=0; i<6; i++)
    {
    printf("%02x ", addr_buf[i]);
    }
  printf("\n");
  last_function = function;
  */
  if(regs.x.cflag) error_num = regs.h.dh;
  else error_num = 0;
  return(regs.x.cflag);
  }
  

int send_pkt(unsigned char far *ptbuffer, unsigned tlength)
  {
  union REGS regs;
  struct SREGS sregs;

  segread(&sregs);
  regs.h.ah = 4;
  sregs.ds = FP_SEG(ptbuffer);
  regs.x.si = FP_OFF(ptbuffer);
  regs.x.cx = tlength;
  int86x(PKTDVR, &regs, &regs, &sregs);
  //last_function = function;
  if(regs.x.cflag) error_num = regs.h.dh;
  else error_num = 0;
  return(regs.x.cflag);
  }



int reset_interface(int rhandle)
  {
  union REGS regs;
  struct SREGS sregs;

  regs.h.ah = 7;
  regs.x.bx = rhandle;
  int86x(PKTDVR, &regs, &regs, &sregs);
  last_function = function;
  if(regs.x.cflag) error_num = regs.h.dh;
  else error_num = 0;
  return(regs.x.cflag);
  }



int get_parameters(void)
  {
  union REGS regs;
  struct SREGS sregs;
  char far * param;

  regs.h.ah = 10;
  int86x(PKTDVR, &regs, &regs, &sregs);
  param = (char far *)MK_FP(sregs.es, regs.x.di);
  /*
  printf("major rev: %d\n",          (unsigned char) param[0]);
  printf("minor rev: %d\n",          (unsigned char) param[1]);
  printf("param struct length: %d\n",(unsigned char) param[2]);
  printf("addr length length: %d\n", (unsigned char) param[3]);
  printf("mtu: %d\n",                (unsigned char) param[4] +
                                    ((unsigned char) param[5]) *256);
  printf("multicast aval: %d\n",     (unsigned char) param[6] +
                                    ((unsigned char) param[7]) *256);
  printf("rcv bufs: %d\n",           (unsigned char) param[8] +
                                    ((unsigned char) param[9]) *256);
  printf("xmt bufs: %d\n",           (unsigned char) param[10] +
                                    ((unsigned char) param[11]) *256);
  printf("int num: %d\n",            (unsigned char) param[12] +
                                    ((unsigned char) param[13]) *256);
  last_function = function;
  */
  if(regs.x.cflag) error_num = regs.h.dh;
  else error_num = 0;
  return(regs.x.cflag);
  }


int as_send_pkt(unsigned char far *ptbuffer, unsigned tlength, void (far *upcall)(char far *,int))
  {
  union REGS regs;
  struct SREGS sregs;

  segread(&sregs);
  regs.h.ah = 11;
  sregs.ds = FP_SEG(ptbuffer);
  regs.x.si = FP_OFF(ptbuffer);
  regs.x.cx = tlength;
  sregs.es = FP_SEG(upcall);
  regs.x.di = FP_OFF(upcall);
  int86x(PKTDVR, &regs, &regs, &sregs);
  //last_function = function;
  if(regs.x.cflag) error_num = regs.h.dh;
  else error_num = 0;
  //return(regs.x.cflag);
  return error_num;
  }

void upcall(char far *buffer, int result)
  {
  }

void interrupt far receiver(unsigned es, unsigned ds, unsigned di, unsigned si,
                            unsigned bp, unsigned sp, unsigned bx, unsigned dx,
                            unsigned cx, unsigned ax, unsigned ip, unsigned cs,
                            unsigned flags)
  {
  unsigned int lsb = 0;
  unsigned int msb = 0;
  
  receiver_calls++;
  packet_len = 0;
  if(packet_len || (cx > sizeof(rxbuf)/sizeof(rxbuf[0])))
    {
    es = di = 0;
    discards++;
    }
  else
    {
    if(ax == 0)
      {
      es = _FP_SEG(prxbuf);
      di = _FP_OFF(prxbuf);
      ax0_counter++;
      }
    if(ax == 1)
      {
      packet_len = cx;
      if(  (_fstrnicmp((char *)&rxbuf[0], (char *)&obc_mac[0], 6)==0)
             && (packet_len < 200) )
        {
        _fmemcpy(cmd_buffer, rxbuf, packet_len);
        packet_len = 0;
        packets_rcvd++;
        ax1_counter++;
        echo.new_data = 1;
        echo.frame = frame;
        time_stamp(&cmds_time);
        time_since_last_command = 0;
        }
      _fmemset(rxbuf, 0, sizeof(rxbuf));
      }
    }
  }

                           
int set_rcv_mode(int set_rcv_handle, int set_rcv_mode)
  {
  union REGS regs;

  regs.h.ah = SET_RCV_MODE;
  regs.x.bx = set_rcv_handle;
  regs.x.cx = set_rcv_mode;
  int86(PKTDVR, &regs, &regs);
  //last_function = function;
  if(regs.x.cflag) error_num = regs.h.dh;
  else error_num = 0;
  return(regs.x.cflag);
  }


int get_rcv_mode(int get_rcv_handle)
  {
  union REGS regs;

  regs.h.ah = GET_RCV_MODE;
  regs.x.bx = get_rcv_handle;
  int86(PKTDVR, &regs, &regs);
  //last_function = function;
  if(regs.x.cflag) error_num = regs.h.dh;
  else error_num = 0;
  return(regs.x.cflag);
  }


int set_multicast_list(unsigned char far * addrlst, int len)
  {
  union REGS regs;
  struct SREGS sregs;

  segread(&sregs);
  regs.h.ah = SET_MULTICAST_LIST;
  regs.x.cx = len;
  sregs.es = FP_SEG(addrlst);
  regs.x.di = FP_OFF(addrlst);
  int86x(PKTDVR, &regs, &regs, &sregs);
  //last_function = function;
  if(regs.x.cflag) error_num = regs.h.dh;
  else error_num = 0;
  return(regs.x.cflag);
  }


int get_multicast_list(void)
  {
  union REGS regs;
  struct SREGS sregs;

  regs.h.ah = GET_MULTICAST_LIST;
  int86x(PKTDVR, &regs, &regs, &sregs);
  //last_function = function;
  if(regs.x.cflag) error_num = regs.h.dh;
  else error_num = 0;
  return(regs.x.cflag);
  }


void print_error(int err_num)
  {
  printf("%s\n", error_msg[err_num]);
  err_num = 0;
  printf("\n");
  }

