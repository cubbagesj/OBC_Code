/*
 dmmtest.cpp
 original 02/18/2004
 updated  03/02/2007 for auto2 centerbody
 modified 12/07/2007 enlarge char strings for ADCP PD0 messages
*/

#include <bios.h>
#include <time.h>
#include <process.h>
#include <stdlib.h>
#include "dmmtest.h"
#include "esccauto.h"
#include "dmm32.h"
#include "rs232.h"
#include "uio48.h"
#include "ether.h"
#include "pc.c"
#include "commands.h"
#include <conio.h>
#include <stdio.h>
#include <iostream.h>
#include <dos.h>
#include <string.h>
extern "C"
{
 //#include "c:\tech80\c\te5650.h"
 #include "te5650.h"
}

struct SYS
  {
  unsigned int  packet_type;
  unsigned int  packet_number;
  unsigned long frame;
  unsigned int  fs_status;
  unsigned int  op_status;
  unsigned long time_high;
  unsigned int  time_low;
  } sys;
  
struct OBS
  {
  unsigned int  new_data;
  long unsigned frame;
  unsigned long time_high;
  unsigned int  time_low;
  unsigned int  data[32];
  } obs;  

struct DYNO
  {
  unsigned int  new_data;
  long unsigned frame;
  unsigned long time_high;
  unsigned int  time_low;
  unsigned int  data[128];
  } dyno;
  
struct PROP
  {
  unsigned int  new_data;
  long unsigned frame;
  unsigned long rpm_time_high;
  unsigned int  rpm_time_low;
  unsigned long position_time_high;
  unsigned int  position_time_low;
  unsigned long adc_time_high;
  unsigned int  adc_time_low;
  int           rpm;
  long int      position;
  unsigned int  data[6];    
  } prop;
  
struct LN200
  {
  unsigned int  new_data;
  long unsigned frame;
  unsigned long time_high;
  unsigned int  time_low;
  int  data[16];
  } ln200;

struct COMMANDS
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
//  unsigned int  actuator5;
  
  unsigned int  adcp_reset;     // for future use
  unsigned int  adcp_heading;
  unsigned int  ds_send_message;
  unsigned int  ds_dmgx;
  unsigned int  ds_dmgy;
  unsigned int  ds_dmgz;
  unsigned int  spare1;

/*
  unsigned int  ds_send_message;
  unsigned int  ds_dmgx1;
  unsigned int  ds_dmgx2;
  unsigned int  ds_dmgy1;
  unsigned int  ds_dmgy2;
  unsigned int  ds_dmgz1;
  unsigned int  ds_dmgz2;
*/
  } commands;  

struct ECHO
  {
  unsigned int  new_data;
  unsigned long frame;
  unsigned long time_high;
  unsigned int  time_low;
  unsigned char data[sizeof(commands)];
  } echo;

struct DS
  {
  unsigned int  new_data;
  long unsigned frame;
  unsigned long time_high;
  unsigned int  time_low;
  unsigned char  data[80];
  } ds;
  
struct ADCP
  {
  unsigned int  new_data;
  long unsigned frame;
  unsigned long time_high;
  unsigned int  time_low;
  //unsigned char  data[88];  //for PD? file format   
  unsigned char  data[213];  //for PD0 file format 11/16/2007 db & jtm
  } adcp;

struct GPS
  {
  unsigned int  new_data;
  long unsigned frame;
  unsigned long time_high;
  unsigned int  time_low;
  unsigned char  data[80];
  } gps;
 
struct TIMER
  {
  unsigned long high;
  unsigned int  low;
  }     sys_timer,
        frame_time,
        obs_time,
        dyno_time,
        prop_rpm_time,
        prop_position_time,
        prop_position_time_previous,
        prop_adc_time,        
        ln200_time,
        cmds_time,
        ds_time,
        adcp_time,
        gps_time;

struct escc_regs settings;
Cescc escc1;

void (interrupt far * old_dmm32a_vect)();
void (interrupt far * old_dmm32b_vect)();
void (interrupt far * rtc_vect)();

//char          pcm[96];                    // pcm data string
char          pcm[240];                    // pcm data string
unsigned      pcm_sent = 0;               // flag; pcm data was sent or not
char          s[81];                      // for screen writes

char          acoustic_message[12];       // acoustic modem
int           send_message = 0;           // flag; valid acoustic message is ready to send

unsigned char obc_mac[6]       = {0x00, 0x01, 0x45, 0x00, 0x7a, 0x9c};
unsigned char control_mac[6]   = {0x00, 0x01, 0x45, 0x00, 0x8a, 0x21};
unsigned char broadcast_mac[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
unsigned char shore_mac[6]     = {0x00, 0x0e, 0xa6, 0xb6, 0xc0, 0x4e};

unsigned char far cmd_buffer[2008];       // command packet buffer
unsigned char data_pkt[1514];             // data packet buffer

long unsigned sys_frame  = 0;
unsigned int  sys_status = 0;

int           time_since_last_command = 0;

unsigned      channela, channelb;         // escc104 channels; a = LN200, b = pcm data stream
unsigned long frame, next_frame;          // frame counter, etc.
unsigned      reading_prop;               // flag; a/d's reading either prop or mux boxes
unsigned      mux_addr = 0;               // mux box channel addresses

unsigned      obs_new_data;
unsigned long obs_data_frame;
unsigned      obs_array[32];
unsigned char obs_buffer[64];

unsigned      dyno_new_data;
unsigned long dyno_data_frame;
unsigned      dyno_array[128];
unsigned char dyno_buffer[256];

unsigned      prop_new_data;
unsigned long prop_data_frame;
int           prop_rpm;
long int      prop_position = 0;
long int      prop_report;
long int      index_report;
long int      prop_index = 0;
long int      prop_position_absolute_current = 0;
long int      prop_position_absolute_previous = 0;
float         rpm_sf;
unsigned      prop_array[6];
unsigned char prop_buffer[12];

unsigned      ln200_new_data;
unsigned long ln200_data_frame;
unsigned char ln200_buffer[32];
char          ln200_command[14];
unsigned long ln200_rx_num = 0;

unsigned      echo_new_data;
unsigned long echo_data_frame;
unsigned      echo_array[8];
unsigned char echo_buffer[16];

unsigned      ds_new_data;
unsigned long ds_data_frame;
unsigned      ds_array[4];
unsigned char ds_buffer[80];

unsigned      adcp_new_data;
unsigned long adcp_data_frame;
//unsigned char adcp_array[88];
//unsigned char adcp_buffer[88];
//unsigned char adcp_test[88];
unsigned char adcp_array[213];  // for PD0 file format 11/16/2007 db & jtm
unsigned char adcp_buffer[213];
unsigned char adcp_test[213];

unsigned      gps_new_data;
unsigned long gps_data_frame;
unsigned      gps_array[4];
unsigned char gps_buffer[80];

unsigned      stern1_cmd = 0;
unsigned      stern2_cmd = 0;
unsigned      fore_cmd   = 0;
unsigned      rudder_cmd = 0;
unsigned      prop_cmd   = 0;
unsigned      com1_tx;
unsigned      com2_tx;
unsigned      com4_tx;
unsigned      data_ready = 0;
unsigned int  ip_id;

extern int           handle;              // packet driver and enet variables
extern unsigned long packets_rcvd;        // from other source modules
extern unsigned long ax0_counter;
extern unsigned long ax1_counter;
extern unsigned long discards;
extern unsigned long receiver_calls;
extern unsigned char pc1_addr[6];
extern unsigned char pc2_addr[6];
extern unsigned char pc3_addr[6];
extern unsigned char jim_pc[6];
extern unsigned char control_mac[6];
extern unsigned char broadcast_addr[6];
extern unsigned tx2_len;
extern unsigned tx2_counter;
extern unsigned tx2_index;

void main(void)
  {
  configure_obc();
  wait_for_time_tick();
  while(1)
    {
    get_frame_time();
    send_data_packet();
    start_data_collection();    
    apply_commands();
    heartbeat();
    send_acoustic_message();
    wait_for_time_tick();
    if(_kbhit()) break;
    }
  terminate();
  exit(0);
  }


void send_acoustic_message(void)
  {
  if(send_message != 0)
    {
    send_message = 0;
    if(_inp(COM2_LSR) & TXB_EMPTY)                   // if com2 tx is idle
      {                                              // send the message
      for(int i=0; i<sizeof(acoustic_message); i++)  
        {
        _outp(COM2_TX, acoustic_message[i]);
        }
      }
    }    
  }


void send_pcm(void)
  {
  if(escc1.istxing[channelb]==0)
    {
    pack_pcm();
    //escc1.tx_port(channelb, pcm, 96);
    escc1.tx_port(channelb, pcm,240);
    }
  }
  


void pack_pcm(void)  // new decom
  {
    
  // Prepare pcm data to be sent up the telemetry link in biphase-m (76.8 KHz)
  // The telemetry frame is 120 16-bit words x 12.
  //
  // All ln200 and a/d data was first collected and stored in separate structures
  // in a total of 48 16-bit words.  For pcm transmission, the data must be:
  // 1) scaled, converted to offset-binary and placed in proper channel slots
  // 2) shifted down from 16-bit to 12-bit (shift and mask) (keep high byte 0000 hex
  // 3) converted from from lsb first to msb first for the bit synchronizer
  // 4) repacked for the escc from 64 12-bit words to 96 8-bit bytes (shift and mask) for the escc 
  
  unsigned data[120];
  //unsigned i = 0;
/*  
    // first scale the numbers, add the offset, place in slots
  data2[0] = 0x0faf;         // decom sync 1
  data2[1] = 0x0321;         // decom sync 2
  data2[2] = 5 << 1;         // CB ID, 5 << 1
  data2[7] = (sys.frame >> 4) & 0x0ffe; // most significant 11 bits of 16 bit frame counter
  data2[8] = (sys.frame << 7) & 0x0f80; // least sig 5 bits of 16 bit frame counter, LEFT ADJUSTED

  //  "R"
  data2[10] = ((long)ln200.data[5] * 2)    + d_offset;       // z axis angular rate low
  data2[9]  = (data2[10] >> 12) * 2 + d_offset;  // high
  
  //  "Q"
  data2[12] = ((long)ln200.data[3] * 2)    + d_offset;       // y axis angular rate low
  data2[11] = (data2[12] >> 12) * 2 + d_offset;  // high
    
  // "P"
  data2[14] = ((long)ln200.data[4] * 2)    + d_offset;       // x axis angular rate low
  data2[13] = (data2[14] >> 12) * 2 + d_offset;  // high
    
  
  //new 2/10/2005
  //data2[16] = ((long)ln200.data[2] * 2)    + d_offset;       // x axis linear accel low
  //data2[15] = (data2[16] >> 12) * 2 + d_offset;  // high

  //new 2/10/2005
  //data2[18] = ((long)ln200.data[1] * 2)    + d_offset;       // y axis linear accel low
  //data2[17] = (data2[18] >> 12) * 2 + d_offset;  // high

  // compensate LN200 Z accel reading for standard gravity: (32.17405 / 2^-6 counts )
  //new 2/10/2005
  //data2[20] = (((long)ln200.data[0] + ((long)(32.17405 * 64))) * 2)    + d_offset;       // z axis linear accel
  //data2[19] =  (data2[20] >> 12) * 2 + d_offset;

  data2[15] = 0;
  data2[16] = 0;
  data2[17] = 0;
  data2[18] = 0;
  data2[19] = 0;
  data2[20] = 0;

  data2[21] = 0;
  data2[22] = 0;
  data2[23] = 0;
  data2[24] = 0;
  data2[25] = 0;
  data2[26] = 0;
    
 //  ln200 single channel scale factors
 //  convert 16-bit LN200 native data to 12-bit pcm data
 //  to provide desired full-scale range for each channel
 //  scale factor = ln200 lsb value * 2^12 counts / desired full-scale range EUs
 
    // roll_accel (Pdot), ln200 lsb = 2^-4 deg/sec^2
  //data2[27] = data[7];
    // limit roll_accel to 12-bits and max +- 128 deg/sec^2 range
  //if(data2[27] >  2047) data2[27] =  2047;
  //if(data2[27] < -2048) data2[27] = -2048;
  //data2[27] = data2[27] + s_offset;
  //new 2/10/2005
  //data2[27] = ln200.data[7];
    // limit roll_accel to 12-bits and max +- 128 deg/sec^2 range
  //if(data2[27] >  2047) data2[27] =  2047;
  //if(data2[27] < -2048) data2[27] = -2048;
  //data2[27] = data2[27] + s_offset;
    
    // pitch_accel (Qdot), lsb = 2^-6 deg/sec^2/count, range = +-70 deg/sec^2    
//  data2[28] = int(data[6]  * 0.457142857) + s_offset; // pitch_FAST_ACCEL
  //data2[28] = data[6];
  // limit pitch_accel to 12-bits and max +- 128 deg/sec^2 range  
  //if(data2[28] >  8191) data2[28] =  8191;
  //if(data2[28] < -8192) data2[28] = -8192;
  //data2[28] = (data2[28] >> 2) + s_offset;
  //new 2/10/2005
  //data2[28] = (long)ln200.data[6];
  // limit pitch_accel to 12-bits and max +- 128 deg/sec^2 range  
  //if(data2[28] >  8191) data2[28] =  8191;
  //if(data2[28] < -8192) data2[28] = -8192;
  //data2[28] = (data2[28] >> 2) + s_offset;

    // yaw_accel (Rdot), lsb = 2^-7 deg/sec^2/count, range = +-70 deg/sec^2    
  //data2[29] = int(data[8]  * 0.228571429) + s_offset; // yaw_FAST_ACCEL
  //data2[29] = data[8];
  // limit yaw_accel to 12-bits and max +- 128 deg/sec^2 range  
  //if(data2[29] >  16383) data2[29] =  16383;
  //if(data2[29] < -16384) data2[29] = -16384;
  //data2[29] = (data2[29] >> 3) + s_offset;
  //new 2/10/2005
  //data2[29] = (long)ln200.data[8];
  // limit yaw_accel to 12-bits and max +- 128 deg/sec^2 range  
  //if(data2[29] >  16383) data2[29] =  16383;
  //if(data2[29] < -16384) data2[29] = -16384;
  //data2[29] = (data2[29] >> 3) + s_offset;
  
  data2[26] = 0;
  data2[27] = 0;
  data2[28] = 0;
  data2[29] = 0;
  
    // scale 16-bit data to 12-bit by dividing by 2^4
  data2[30] = ((long)obs.data[21] >> 4)      + s_offset; // prop_cmd
  data2[31] = ((long)obs.data[25] >> 4)      + s_offset; // fwd_cmd
  data2[32] = ((long)obs.data[22] >> 4)      + s_offset; // strn_cmd
  data2[33] = ((long)obs.data[24] >> 4)      + s_offset; // rudd_cmd
  data2[34] = ((long)obs.data[28] >> 4)      + s_offset; // failsafe
  data2[35] = ((long)obs.data[26] >> 4)      + s_offset; // cage/mode
  data2[36] = ((long)obs.data[16] >> 4)      + s_offset; // e_bat
  
    // yaw, ln200 lsb = (2^-15) * 180 deg/count, range = +-180 deg    
  data2[37] = ((long)ln200.data[11] >> 4)      + s_offset; // yaw180  

    // yaw_exp, lsb = (2^-15) * 180 deg/count, range = +-45 deg      
  data2[38] = 0;
  
    // pitch_exp, ln200 lsb = (2^-15) * 180 deg/count, range = +-22.5 deg      
  data2[39] = 0;
  
    // roll, ln200 lsb = (2^-15) * 180 deg/count, range = +-180 deg      
  data2[40] = ((long)ln200.data[10] >> 3) + s_offset; // roll  

    // roll_rate_exp, lsb = 2^-6 deg/sec/count, range = +-14 deg/sec  
    // deleted 2/21/2002 jtm    
  
    // pitch_rate_exp, lsb = 2^-8 deg/sec/count, range = +-14 deg/sec    
    // deleted 2/21/2002 jtm    
  
  // new depth gauges 2/17/2005 jtm
  data2[41] = ((long)obs.data[0] >> 4) + s_offset; // depth 1
  data2[42] = ((long)obs.data[1] >> 4) + s_offset; // depth 2
  data2[43] = ((long)obs.data[2] >> 4) + s_offset; // depth 3
  
  // new status bits
  data2[44] = sys.fs_status & 0x0fff;
  data2[45] = commands.mode  & 0x0fff;

    // yaw_rate_exp, lsb = 2^-9 deg/sec/count, range = +-14 deg/sec  
    // deleted 2/21/2002 jtm
  
    // x_accel_exp, lsb = (2^-6) ft/sec^2/count, range = +-17 ft/sec^2      
    // deleted 2/21/2002 jtm    
    
    // y_accel_exp, lsb = (2^-6) ft/sec^2/count, range = +-17 ft/sec^2        
    // deleted 2/21/2002 jtm    
  
    // z_accel_exp, lsb = (2^-6) ft/sec^2/count, range = +-17 ft/sec^2
    // NOTE:  LN200 reading is compensated for standard gravity: (32.17405 / 2^-6 counts)
    // deleted 2/21/2002 jtm    
  data2[46] = 0;
  
    // Pdot_exp, lsb = 2^-4 deg/sec^2, range = +-17 deg/sec^2
    // deleted 2/21/2002 jtm    
  data2[47] = 0;

  status_word = 0;
  data2[48] = status_word;                          

    // Qdot_exp, lsb = (2^-6) ft/sec^2/count, range = +-17 ft/sec^2                                
    // deleted 2/21/2002 jtm    
  data2[49] = 0;
  
    // Rdot_exp, lsb = (2^-7) ft/sec^2/count, range = +-17 ft/sec^2        
    // deleted 2/21/2002 jtm    
  data2[50] = 0;
    
    //fore plane lvdt
  data2[52] = (long)obs.data[7];
  data2[52] = (data2[52] >> 4) + s_offset;

    //rudder lvdt
  data2[53] = obs.data[6];
  data2[53] = (data2[53] >> 4) + s_offset;

    //stern lvdt
  data2[54] = obs.data[4];  
  data2[54] = (data2[54] >> 4) + s_offset;
  
    // stern tips lvdt
  data2[55] = obs.data[5];
  data2[55] = (data2[55] >> 4) + s_offset;

  
   // prop_voltage, sc1 ch3, mm32 adc ch3; mc output is +- 5V, 40V/V; module is +-10V
   // NOTE:  data is shifted by 3 instead of 4 because 5B module has
   // +/-10 volt input instead of +/- 5 volt input.  ??????????
  //new 2/10/2005
  data2[56] = ((long)obs.data[18] >> 4) + s_offset;  
  //data2[56] = 0;
  
   // prop_current, sc1 ch2, mm32 adc ch2; mc output is +- 5V, 20V/V; module is +-10V
   // NOTE:  data is shifted by 3 instead of 4 because 5B module has
   // +/-10 volt input instead of +/- 5 volt input.  
  //new 2/10/2005
  data2[57] = ((long)obs.data[19] >> 4) + s_offset;
  //data2[57] = 0;

    // pitch, lsb = (2^-15) * 180 deg/count, range = +-90 deg        
  data2[58] = ((long)ln200.data[9] >> 2) + s_offset; // pitch

    // paddle_wheel, module is 500Hz 0-5Volt
  data2[59] = obs.data[3];
  if(data2[59] < 0) data2[59] = 0;    
  data2[59] = (data2[59] >> 4) + s_offset;

    // rpm, 2540 or 5000 pulse/rev, module is 100KHz 0-5Volt
  data2[60] = obs.data[20];
  if(data2[60] < 0) data2[60] = 0;  
  data2[60] = (data2[60] >> 4) + s_offset;  
  
*/    
  
  data[0]  = 0x7e7e;        // sync word
  data[1]  = sys.fs_status;
  //data[1]  = 0xa5a5;
  data[2]  = sys.op_status;
  data[3]  = obs.data[0];  // depth 1
  data[4]  = obs.data[1];  // depth 2
  data[5]  = obs.data[2];  // depth 3
  data[6]  = obs.data[3];  // paddle wheel
  data[7]  = obs.data[4];  // stern1 lvdt
  data[8]  = obs.data[5];  // stern2 lvdt
  data[9]  = obs.data[6];  // rudder lvdt
  data[10] = obs.data[7];  // fore lvdt
  data[11] = obs.data[8];  // spare lvdt
  data[12] = obs.data[16];  // ebat volts
  data[13] = obs.data[17];  // pbat volts
  data[14] = obs.data[18];  // prop volts
  data[15] = obs.data[19];  // prop amps
  data[16] = obs.data[20];  // 5B module rpm (unsigned)
  data[17] = obs.data[21];  // prop discriminator
  data[18] = obs.data[22];   // stern1 discriminator
  data[19] = obs.data[23];   // stern2 discriminator
  data[20] = obs.data[24];   // rudder discriminator
  data[21] = obs.data[25];   // fore discriminator
  data[22] = obs.data[26];   // mode discriminator
  data[23] = obs.data[27];   // ballast discriminator
  data[24] = obs.data[28];   // failsafe discriminator
  data[25] = obs.data[29];   // radio mode discriminator
  data[26] = obs.data[30];   // key discriminator
  data[27] = prop.rpm;
  data[28] = (int)prop.position;           
  data[29] = ln200.data[0];  // z linear accel 
  data[30] = ln200.data[1];  // y linear accel
  data[31] = ln200.data[2];  // x linear accel
  data[32] = ln200.data[3];  // y angular rate
  data[33] = ln200.data[4];  // x angular rate
  data[34] = ln200.data[5];  // z angular rate
  data[35] = ln200.data[6];  // y angular accel
  data[36] = ln200.data[7];  // x angular accel
  data[37] = ln200.data[8];  // z angular accel
  data[38] = ln200.data[9];  // pitch
  data[39] = ln200.data[10]; // roll 
  data[40] = ln200.data[11]; // heading 
  data[41] = commands.mode;
  data[42] = commands.rpm;
  data[43] = commands.rudder;
  data[44] = commands.stern1;
  data[45] = commands.stern2;
  data[46] = commands.fore;
  data[47] = commands.ballast;
  data[48] = commands.ds_dmgx;
  data[49] = commands.ds_dmgy;
  data[50] = commands.ds_dmgz;
  data[51] = commands.spare1;
  memcpy(&data[52],&adcp.data[154],2); // y_vel_btm;
  memcpy(&data[53],&adcp.data[158],2); // z_vel_btm;
  memcpy(&data[54],&adcp.data[156],2); // x_vel_btm;
  memcpy(&data[55],&adcp.data[146],2); // bm1_rng_to_btm;
  memcpy(&data[56],&adcp.data[148],2); // bm2_rng_to_btm;
  memcpy(&data[57],&adcp.data[150],2); // bm3_rng_to_btm;
  memcpy(&data[58],&adcp.data[152],2); // bm4_rng_to_btm;
  data[59] = 0;
  _fmemcpy(&data[60],&dyno.data,2*60);
  _fmemcpy(&pcm,&data,sizeof(pcm));
  }  


void get_frame_time(void)
  {
  _disable();
  time_stamp(&frame_time);
  _enable();
  }
  
  
void send_data_packet(void)
  {
  unsigned int lsb = 0;
  unsigned int msb = 0;
  
  build_packet();
  send_pkt(data_pkt, 1514);  // 1514 bytes in enet packet w/o checksum
  }
  
  
void apply_commands(void)
  {
  unsigned int lsb = 0;
  unsigned int msb = 0;

  // get commands from buffered udp packet
  _disable();
  _fmemcpy(&commands, &cmd_buffer[42], sizeof(commands));
  _enable();
  
  // zero out the commands if no response from control for 1 second
  if(++time_since_last_command > 100)  
    {                                
    commands.rpm = 0;
    commands.rudder    = 0x800;
    commands.stern1    = 0x800;
    commands.stern2    = 0x800;
    commands.fore      = 0x800;
//    commands.actuator5 = 0x800;  // May 2007 jtm
    }

/*  
  //  blow the ERS if no response from control for 2 minutes
  if(time_since_last_command > 12000)
    {                                
    commands.ballast = commands.ballast | ERS_BLOW;
    time_since_last_command = 12000;
    }
*/

//  8/30/2007 jtm rem'ed out for sam
  //  blow the ERS if no response from control for 5 seconds
  if(packets_rcvd > 100 && time_since_last_command > 500)
    {                                
    commands.ballast = commands.ballast | ERS_BLOW;
    time_since_last_command = 500;
    }
    
  // apply the commands
  // stern1
  sprintf(s, "Stern1:    %04x", commands.stern1);
  PC_DispStr(20,16, (unsigned char *)s, DISP_FGND_BLACK + DISP_BGND_RED);
  while(_inp(OBS_BASE+DACLSB) & 0x80);
  _outp(OBS_BASE+DACLSB, commands.stern1 & 0xff);
  _outp(OBS_BASE+DACMSB, (commands.stern1>>8) & 0x0f | STERN1_CHAN);
  while(_inp(OBS_BASE+DACLSB) & 0x80);
  _inp( OBS_BASE+DACMSB);

  // stern2
  sprintf(s, "Stern2:    %04x", commands.stern2);
  PC_DispStr(20,17, (unsigned char *)s, DISP_FGND_BLACK + DISP_BGND_RED);
  while(_inp(OBS_BASE+DACLSB) & 0x80);
  _outp(OBS_BASE+DACLSB, commands.stern2 & 0xff);
  _outp(OBS_BASE+DACMSB, (commands.stern2>>8) & 0x0f | STERN2_CHAN);
  while(_inp(OBS_BASE+DACLSB) & 0x80);
  _inp( OBS_BASE+DACMSB);
  
  // forward
  while(_inp(OBS_BASE+DACLSB) & 0x80);
  _outp(OBS_BASE+DACLSB, commands.fore & 0xff);
  _outp(OBS_BASE+DACMSB, (commands.fore>>8) & 0x0f | FORE_CHAN);
  while(_inp(OBS_BASE+DACLSB) & 0x80);
  _inp( OBS_BASE+DACMSB);
  
  // rudder
  sprintf(s, "Rudder:    %04x", commands.rudder);
  PC_DispStr(20,19, (unsigned char *)s, DISP_FGND_BLACK + DISP_BGND_RED);
  while(_inp(OBS_BASE+DACLSB) & 0x80);
  _outp(OBS_BASE+DACLSB, commands.rudder & 0xff);
  _outp(OBS_BASE+DACMSB, (commands.rudder>>8) & 0x0f | RUDDER_CHAN);
  while(_inp(OBS_BASE+DACLSB) & 0x80);
  _inp( OBS_BASE+DACMSB);

// actuator5
/*  while(_inp(DYNO_BASE+DACLSB) & 0x80);
  _outp(DYNO_BASE+DACLSB, commands.actuator5 & 0xff);
  _outp(DYNO_BASE+DACMSB, (commands.actuator5>>8) & 0x0f | 0x00);
  while(_inp(DYNO_BASE+DACLSB) & 0x80);
  _inp(DYNO_BASE+DACMSB);
*/    
  // prop
  sprintf(s, "Prop:      %04x", commands.rpm);
  PC_DispStr(20,18, (unsigned char *)s, DISP_FGND_BLACK + DISP_BGND_RED);
  te5650SetMotor(commands.rpm);
  te5650Update();

  // ers blow  
  sprintf(s, "Ballast:   %04x", commands.ballast);
  PC_DispStr(20,20, (unsigned char *)s, DISP_FGND_BLACK + DISP_BGND_RED);
  if(commands.ballast & ERS_BLOW)
    {
    write_bit(PUMP,   PUMPOFF);
    write_bit(VALVE1, VALVEOFF);
    write_bit(VALVE2, VALVEOFF);
    write_bit(VALVE3, VALVEOFF);
    write_bit(VALVE4, VALVEOFF);
    write_bit(VALVE5, VALVEOFF);
    write_bit(VALVE6, VALVEOFF);
    write_bit(ERS,    ERSBLOW);
    }
  
  // pfs blow
  if(commands.ballast & PFS_BLOW)
    {
    write_bit(PFSVENTBLOW, PFSBLOW);
    }
  
  // pfs flood
  if(commands.ballast & PFS_FLOOD)
    {
    write_bit(PFSVENTBLOW, PFSVENT);    
    }
  
  // turn pump and valves off if no blow/flood command
  if(!(commands.ballast & 0x00ff))
    {
    write_bit(PUMP,   PUMPOFF);
    write_bit(VALVE1, VALVEOFF);
    write_bit(VALVE2, VALVEOFF);
    write_bit(VALVE3, VALVEOFF);
    write_bit(VALVE4, VALVEOFF);
    write_bit(VALVE5, VALVEOFF);
    write_bit(VALVE6, VALVEOFF);
    }

  // ballast blow
  if(commands.ballast & ALL_BLOW)
    {
    write_bit(PUMP,   PUMPON);
    write_bit(VALVE1, VALVEON);
    write_bit(VALVE2, VALVEOFF);
    write_bit(VALVE3, VALVEOFF);
    write_bit(VALVE4, VALVEON);
    write_bit(VALVE5, VALVEON);
    write_bit(VALVE6, VALVEOFF);
    }

  // ballast flood
  if(commands.ballast & ALL_FLOOD)
    {
    write_bit(PUMP,   PUMPON);
    write_bit(VALVE1, VALVEOFF);
    write_bit(VALVE2, VALVEON);
    write_bit(VALVE3, VALVEON);
    write_bit(VALVE4, VALVEOFF);
    write_bit(VALVE5, VALVEOFF);
    write_bit(VALVE6, VALVEON);
    }
    
  // trim forward
  if(commands.ballast & TRIM_FORWARD)
    {
    write_bit(PUMP,   PUMPON);
    write_bit(VALVE1, VALVEOFF);
    write_bit(VALVE2, VALVEON);
    write_bit(VALVE3, VALVEOFF);
    write_bit(VALVE4, VALVEOFF);
    write_bit(VALVE5, VALVEON);
    write_bit(VALVE6, VALVEOFF);
    }
    
  // trim aft
  if(commands.ballast & TRIM_AFT)
    {
    write_bit(PUMP,   PUMPON);
    write_bit(VALVE1, VALVEOFF);
    write_bit(VALVE2, VALVEOFF);
    write_bit(VALVE3, VALVEON);
    write_bit(VALVE4, VALVEON);
    write_bit(VALVE5, VALVEOFF);
    write_bit(VALVE6, VALVEOFF);
    }
  
  // templates for future ballast features when implemened
  // forward blow
  if(commands.ballast & FORWARD_BLOW)
    {
    }
  
  // forward flood
  if(commands.ballast & FORWARD_FLOOD)
    {
    }
  
  // aft blow
  if(commands.ballast & AFT_BLOW)
    {
    }
    
  // aft flood
  if(commands.ballast & AFT_FLOOD)
    {
    }
 
  if(commands.ds_send_message != 0)
    {     
      send_message = 0;
    //sjc send_message = 1;
//    _fmemcpy(&acoustic_message, &commands.ds_dmgx1, sizeof(acoustic_message));
    }
  }
        
  
void configure_obc(void)
  {
  //for(unsigned i=0; i<88; i++) adcp_test[i] = (unsigned char) i;
  
  rpm_sf = 60.0f * 1000000.0f / 0.838097f / 10160.0f;
  
  sys_timer.high = 0;
  sys_timer.low = 0;
  
  frame = 0;
  next_frame = frame;
  
  sys.packet_type = 0xf0c1;   // obc data packet identifier
  sys.packet_number = 0xfffe;
  sys.fs_status = 0;
  sys.op_status = 0;
  
  obs.new_data = 0;
  obs.frame = frame;
  
  dyno.new_data = 0;
  obs.frame = frame;
  
  prop.new_data = 0;
  prop.frame = frame;
  
  ln200.new_data = 0;
  ln200.frame = frame;
  
  echo.new_data = 0;
  echo.frame = frame;
  
  ds.new_data = 0;
  ds.frame = frame;
  
  adcp.new_data = 0;
  adcp.frame = frame;
  
  init_io(UIO48BASE);
  write_bit(PUMP,   PUMPOFF);
  write_bit(VALVE1, VALVEOFF);
  write_bit(VALVE2, VALVEOFF);
  write_bit(VALVE3, VALVEOFF);
  write_bit(VALVE4, VALVEOFF);
  write_bit(VALVE5, VALVEOFF);
  write_bit(VALVE6, VALVEOFF);
  write_bit(ERS,    ERSNOBLOW);
  write_bit(PFSVENTBLOW, PFSBLOW);

  configure_escc();
  //while(!kbhit());  getch();
  init_tech80();
  //while(!kbhit());  getch();
  //init_sys_timer();
  init_com1();
  //while(!kbhit());  getch();
  init_com2();
  //while(!kbhit());  getch();
  init_com4();
  //while(!kbhit());  getch();
  configure_enet();
  //while(!kbhit());  getch();
  configure_dmm32a(); // onboard sensor adc
  //while(!kbhit());  getch();
  configure_dmm32b(); // dyno adc
  //while(!kbhit());  getch();
  init_screen();
  }


void init_tech80(void)
  {
  cout << "te5650InitSw:    " << hex << te5650InitSw() << dec << endl;
  cout << "te5650InitServo: " << te5650InitServo(0x400, TE5650TYPE_DAC16) << endl;
  cout << "te5650SetAxis:   " << te5650SetAxis(1,1) << endl;
  te5650PhasesPolarity(1,1,1);  // prop encoder A,B,I phase polarity
  te5650Update();
  te5650CaptureIndex();  
  te5650Update();
  te5650ResetCapture();
  te5650Update();
  te5650MotorOff();
  te5650Update();
  te5650SetMotor(0);
  te5650Update();
  cout << "Tech80 5650 Installed." << endl;
  }
  
  
void init_sys_timer(void)
  {
  rtc_vect = _dos_getvect(0x08);               // save the pc's default rtc isr vector
  _disable();                                  // so we can chain into it later
  _dos_setvect(0x08, sys_timer_isr);           // hook our system timer isr
  _outp(0x43, CTC0 | LSBMSB | MODE2 | BINARY); // change the pc's 8254 to mode2 operation
  _outp(0x40, 0xff);                           // load timer divisor LSB, then MSB
  _outp(0x40, 0xff);
  _enable();
  }


void configure_enet(void)
  { 
  cout << "Initializing packet driver." << endl;
  char packet_type[2] = {0x08, 0x00};
  char far *ptype = packet_type;

/*
  // use at/lantic packet driver
  system("c:\\ne2000\\pktdrv\\atdrive -i 0x7e -b 0x300 -q 10");
  cout << "Access_type() error: " << access_type(1, 52, 0, ptype, 2, receiver) << endl;
  cout << "Handle: " << handle << endl;
  cout << "Set_rcv_mode() error: " << set_rcv_mode(handle, 6) << endl;
*/  
  
  // use ne2000 packet driver
  system("c:\\packet\\pktdrv\\ne2000 0x7e 10 0x300");
  cout << "Access_type() error: " << access_type(1, 52, 0, NULL, 0, receiver) << endl;
  cout << "Handle: " << handle << endl;
  cout << "Driver_info() error: " << driver_info(handle) << endl;
  cout << "Rpm_sf: " << rpm_sf << endl;
  cout << "Packet driver initialized." << endl;
  }

  
void configure_escc(void)
  {
  cout << "Initializing ESCC." << endl;
  for(int i=0;i<96;i++) pcm[i] = 'a';
  
  // open the two escc communications ports using add_port() function
  // parameters: (base address of escc board,
  //              escc channel 0 or 1,
  //              IRQ setting,   
  //              dma channel for receive,  
  //              dma channel for transmit)

  // channel A to communicate with the LN200                  
  channela = escc1.add_port(0x240,0,5,0,0);  //1st channel, IRQ 5, no dma
  cout << "Channel A = " << channela << endl;
  
  // channel B to transmit the pcm telemetry
  channelb = escc1.add_port(0x240,1,5,1,5);  //2nd channel IRQ 5, Rx=DMA Ch1, Tx=DMA Ch5
  cout << "Channel B = " << channelb  << endl;
  
  // assemble configuration data for channel_a for LN200 SDLC operation.
  // see Siemens SAB 82532 ESCC User's Manual page 110.
  settings.mode = 0x88;   // transparent 0 frames, receiver active
  settings.timr = 0x1f;   // for cb98
  settings.xbcl = 0x00;   // n/a for interrupt mode
  settings.xbch = 0x00;   // interrupt mode (no dma)
  settings.ccr0 = 0xc0;   // power up, master clock enabled
  settings.ccr1 = 0x10;   // txd push-pull, one's insertion, clk mode 0 (use LN200 clock)
  settings.ccr2 = 0x00;   // normal txd/rxd, ssel=0 (clk mode 0a), crc-ccitt, no inversion 
  settings.ccr3 = 0x00;   // no preamble, rx crc off, tx crc generated internally
  settings.ccr4 = 0x00;   
  settings.bgr  = 0x00;    
  settings.iva  = 0x00;    
  settings.ipc  = 0x03;   // masked interrupts NOT visible, pin INT = push/pull active high
  settings.imr0 = 0x04;   // cdsc disabled
  settings.imr1 = 0x00;   // all interrupts enabled 
  settings.pvr  = 0x00;   
  settings.pim  = 0xff;
  settings.pcr  = 0xe0;
  settings.xad1 = 0xff;
  settings.xad2 = 0xff;
  settings.rah1 = 0xff;
  settings.rah2 = 0xff;
  settings.ral1 = 0xff;
  settings.ral2 = 0xff;
  settings.rlcr = 0x00;
  settings.pre  = 0x00;
    
  // write configuration data to escc channel a registers
  // parameters:(port to intialize,
  //             operating mode,
  //             settings defined above,
  //             number of receive buffers,
  //             number of transmit buffers)
  cout << "initializing channel A" << endl;
  if(escc1.init_port(channela,OPMODE_HDLC,&settings,2,2)==TRUE)
    cout << "Intialize Channel A OK." << endl;
  else
    {
    cout << "Initialize Channel A FAILED!" << endl;
    terminate();
    //exit(0);
    }
  
  // test channel a to make sure it opened
  if(escc1.clear_rx_buffer(channela)==TRUE)
    cout << "Channel A rx buffer cleared OK.  " << endl;
  else
    {
    cout << "Channel A rx buffer clear FAILED!" << endl;
    terminate();
    //exit(0);
    }

  if(escc1.clear_tx_buffer(channela)==TRUE)
    cout << "Channel A tx buffer cleared OK." << endl;
  else
    {
    cout << "Channel A tx buffer clear FAILED!" << endl;
    terminate();
    }
  
  // set channel a frame transmission type
  if(escc1.set_tx_type(channela, TRANSPARENT_MODE)==TRUE)
    cout << "Channel A set to Transparent Mode.  " << endl;
  else
    {
    cout << "Channel A Transparent Mode not set!  Channel not open!" << endl;
    terminate();
    }

  // Channel b runs off the escc internal oscillator.
  // Configure the escc frequency generator to produce a 9.8304 MHz clock signal
  // from the onboard 18.432 MHz TTL oscillator.  The 9.8304 MHz clock is 
  // then divided by 128 and used as the master clock for the 76.8 KHz pcm data stream
  
  // See escc.cpp, Cypress ICD2053B clock-generator datasheet and BitCalc software
  // available at the Cypress Web site to calculate the clock scaling factor.
  
  // void Cescc::set_clock_generator(unsigned port,     //either channel A or B will work
  //                                 unsigned long hval,//stuffed hex value from BitCalc
  //                                 unsigned nmbits)   //number of bits in stuffed value
  
  //escc1.set_clock_generator(channela, 0x5d31c0L, 0x18);
  
  // new 2.4576 MHz clock for 19.2 KHz PCM bit rate 12/19/2007 jtm
  escc1.set_clock_generator(channela, 0x5d51c0L, 24);


  // Configure channel_b for biphase-m operation for pcm data telemetry.
  // see Siemens SAB 82532 ESCC User's Manual page 110 for register descriptions.
  settings.mode = 0xC8;  // extended transparent mode 0, rcvr active, extern timer k=32768
  settings.timr = 0xe2;  // cnt=7, value=2, continuous 100 Hz interrupts when tcp=1/9.8304MHz
  settings.xbcl = 0x00;  // set up by setupdmat() when transmission is started
  settings.xbch = 0x80;  // dma data transfer mode
  // Note:  ESCC User's manual p 92/129 has switched FM0 & FM1?  Actually FM0=101, FM1=100 ?
  settings.ccr0 = 0xD0;  // power-up, master clk=xtal, FM1(biphase-m), hdlc/sdlc
  settings.ccr1 = 0x17;  // set txd push-pull, set clk mode 7b
  settings.ccr2 = 0x38;  // br9&8=0, bdf=1, ssel=1, toe=1, rwx=0, crc-ccitt, no inversion
  settings.ccr3 = 0x02;  // no preamble, radd=0, crl=0, rcrc=off, xcrc=off, psd=na
  settings.ccr4 = 0x00;  // added to implement MCK4 and EBRG, but not used
  settings.bgr  = 0x3f;  // brg divisor=63 (0x3F), 76.8 KHz = 9.8304 MHz /((63+1)*2)
  settings.iva  = 0x00;  // not set here
  settings.ipc  = 0x03;  // masked interrupts not visible, pin INT = push/pull active high
  settings.imr0 = 0x00;  // all interrupts enabled
  settings.imr1 = 0x00;  // all interrupts enabled
  settings.pvr  = 0x00;  
  settings.pim  = 0xff;
  settings.pcr  = 0xe0;
  settings.xad1 = 0xff;
  settings.xad2 = 0xff;
  settings.rah1 = 0xff;
  settings.rah2 = 0xff;
  settings.ral1 = 0xff;
  settings.ral2 = 0xff;
  settings.rlcr = 0x00;
  settings.pre  = 0x00;
  
  //Write configuration to channelb registers
  //Parameters: (port to intialize,
  //             operating mode,
  //             settings defined above,
  //             number of receive buffers,
  //             number of transmit buffers)
  if(escc1.init_port(channelb,OPMODE_HDLC,&settings,MAX_RBUFS,MAX_TBUFS)==TRUE)
    cout << "Intialize Channel B OK." << endl;
  else
    {
    cout << "Initialize Channel B FAILED!" << endl;
    terminate();
    }
  
  // test channelb to make sure it opened
  if(escc1.clear_rx_buffer(channelb)==TRUE)
    cout << "Channel B rx buffer cleared OK.  " << endl;
  else
    {
    cout << "Channel B rx buffer clear FAILED!" << endl;
    terminate();
    }
  
  if(escc1.clear_tx_buffer(channelb)==TRUE)
    cout << "Channel B tx buffer cleared OK." << endl;
  else
    {
    cout << "Channel B tx buffer clear FAILED!" << endl;
    terminate();
    }
  
  //set channelb frame transmission type
  if(escc1.set_tx_type(channelb, TRANSPARENT_MODE)==TRUE)
    cout << "Channel B set to Transparent Mode.  " << endl;
  else
    {
    cout << "Channel B Transparent Mode not set!  Channel not open!" << endl;
    terminate();
    }
  cout << "ESCC initialized." << endl;
  }

  
void wait_for_time_tick(void)            
  {
  /*
    Generates the 100 Hz frame rate,
    sends the pcm data frame,
    queries the LN200,
    writes data to screen.
  */
  static unsigned toggle = 0;
  unsigned pcm_lockup = 1;
  
  pcm_sent = 0;
  
  while(frame < next_frame)              // idle loop waits until beginning of next frame
    {                                    // frame is incremented in dmm32a isr by 100 Hz timer
    if(data_ready)
      {                                  // wait until adc's are done before querying the ln200
      data_ready = 0;
      query_ln200();
      }
    if(escc1.istxing[channelb]==0)
      {
      pcm_sent = 0;
      send_pcm();
      pcm_lockup = 0;
      }  
    }
  
  //if(pcm_lockup) escc1.clear_tx_buffer(channelb);
    
  sprintf(s, "Pitch:   %04x  %10.2f deg", ln200.data[9], (ln200.data[9] / 182.044));   // Pitch
  PC_DispStr(0,6, (unsigned char *)s, DISP_FGND_BLACK + DISP_BGND_RED);
  sprintf(s, "Roll:    %04x  %10.2f deg", ln200.data[10], (ln200.data[10] / 182.044));  // Roll
  PC_DispStr(0,7, (unsigned char *)s, DISP_FGND_BLACK + DISP_BGND_RED);
  sprintf(s, "Heading: %04x  %10.2f deg", ln200.data[11], (ln200.data[11] / 182.044));  // Heading
  PC_DispStr(0,8, (unsigned char *)s, DISP_FGND_BLACK + DISP_BGND_RED);
  
  sprintf(s, "Stern 1: %04x", echo.data[4]);
  PC_DispStr(0,9, (unsigned char *)s, DISP_FGND_BLACK + DISP_BGND_RED);     // Stern 1
  sprintf(s, "Stern 2: %04x", echo.data[5]);
  PC_DispStr(0,10, (unsigned char *)s, DISP_FGND_BLACK + DISP_BGND_RED);     // Stern 2
  sprintf(s, "Rudder:  %04x", echo.data[3]);
  PC_DispStr(0,11, (unsigned char *)s, DISP_FGND_BLACK + DISP_BGND_RED);     // Rudder
  sprintf(s, "Fore:    %04x", echo.data[6]);
  PC_DispStr(0,12, (unsigned char *)s, DISP_FGND_BLACK + DISP_BGND_RED);     // Fore
  
  sprintf(s, "RawPos:    %08ld", prop_report);
  PC_DispStr(0,13, (unsigned char *)s, DISP_FGND_BLACK + DISP_BGND_RED);     // Prop position
  
  sprintf(s, "RawIndex:    %08ld", index_report);
  PC_DispStr(0,14, (unsigned char *)s, DISP_FGND_BLACK + DISP_BGND_RED);     // Prop position
  
  sprintf(s, "PPos:    %08d", prop.position);
  PC_DispStr(0,15, (unsigned char *)s, DISP_FGND_BLACK + DISP_BGND_RED);     // Prop position
    
  next_frame = frame + 1;                
  }

  
void start_data_collection(void)  
  {
  trigger_dmm32a();
  trigger_dmm32b();
  }


void query_ln200(void)
  {
  prep_ln200_command();                        // build the ln200 initialization command
  escc1.tx_port(channela, ln200_command, 14);  // and send it to the ln200
  }
  

void configure_dmm32a(void)
  {
  cout << "Initializing DMM32a." << endl;
   //io = 0x340, irq=11, bottom board, reads the two banks of 5B modules
  _disable();
  _outp(OBS_BASE+FCR, FIFORST);                   // reset the fifo

  _outp(OBS_BASE+MCR, PAGE1);                     // set dio ports for outputs, mode0
  _outp(OBS_BASE+15,  0x80);
  _outp(OBS_BASE+MCR, PAGE0);

  _outp(OBS_BASE+LCR,  0);                         // set the range of channels to scan
  _outp(OBS_BASE+HCR,  31);                        // all channels for the 2 5B module backplanes
  _outp(OBS_BASE+FCR, SCANEN);                     // enable scan interrupts
  _outp(OBS_BASE+ACR, KHZ200 | BIPOLAR5 | GAIN1);  // 200KHz scans, +-5Volt inputs
  while(_inp(OBS_BASE+ARR) & WAIT);                // wait for the adc circuits to settle

  _outp(OBS_BASE+MCR, PAGE2);                      // configure ctc0 for falling edge trigger
  _outp(OBS_BASE+15 , 1);
  _outp(OBS_BASE+MCR, PAGE0);
  _outp(OBS_BASE+CTCCR, 0x42);                     // set CTC0 to 10 KHz input
  _outp(OBS_BASE+MCR, PAGE0);                      // access the 82C54's registers
  _outp(OBS_BASE+15, CTC0 | LSBMSB | MODE2 | BINARY);// program the 82C54's mode
  _outp(OBS_BASE+12, 0x64);                        // load divisor LSB (100 or 0x0064)
  _outp(OBS_BASE+12, 0x00);                        // load divisor MSB
                                                   // 10 KHz / 100 = 100 Hz frame rate
  old_dmm32a_vect = _dos_getvect(11-8+0x70);       // save old irq11 vector
  _dos_setvect(11-8+0x70, isr_dmm32a);             // install dmm32a isr vector
  _outp(0xa1, _inp(0xa1) & 0xf7);                  // unmask irq11 at pic

  _outp(OBS_BASE+ICR, ADINTE | TINTE);             // enable adc and timer interrupts
  _enable();
  cout << "DMM32a initialized." << endl;
  }
  

void trigger_dmm32a(void)  
  {
  while(_inp(OBS_BASE+ARR) & WAIT);        // wait for the adc inputs to settle
  _outp(OBS_BASE, 0);                      // start the scan of the onboard sensors
  }
  
  
void interrupt isr_dmm32a(void)
  {
  // the dmm32 A board generates two types of interrupts:
  // 1) 82C54 ctc0 timeouts, TINT, the 100 Hz frame timer
  // 2) adc scan complete, ADINT, when onboard sensor data is ready
  
  unsigned i;
  unsigned lsb = 0;
  unsigned msb = 0;

  // section 1, a TINT has occurred, the 100 Hz ctc0 signals the beginning of a new frame
  if(_inp(OBS_BASE+ISR) & TINT)
    {
    frame++;                                       // update the frame counter
    }
  
  // section 2, an ADINT has occurred, the onboard sensor data is ready
  if(_inp(OBS_BASE+ISR) & ADINT)
    {
    time_stamp(&obs_time);
    for(i=0; i<32; i++) obs_array[i] = _inpw(OBS_BASE);  // read the data from the dmm32a fifo
    _fmemcpy(obs_buffer, obs_array, sizeof(obs_array));  // buffer the data
    obs_new_data = 1;                                    // flag the system new data available
    obs_data_frame = frame;                              // record the frame number
    }
  _outp(OBS_BASE+MCR, INTRST);                           // reset the dmm32's interrupt circuit    
  _outp(OBS_BASE+FCR, FIFORST);                          // reset the fifo
  _outp(OBS_BASE+LCR,  0);                               // reset the range of channels to scan
  _outp(OBS_BASE+HCR,  31);
  _outp(OBS_BASE+FCR, SCANEN);                           // enable scan interrupts

  _outp(0xa0, 0x20);                                     // slave EOI
  _outp(0x20, 0x20);                                     // master EOI  
  }
  

void configure_dmm32b(void)
  {
  cout << "Initializing DMM32b." << endl;
  //io = 0x380, irq=12, top board, reads the mux boxes and the prop dynos
  _disable();

  _outp(DYNO_BASE+FCR, FIFORST);                         // reset the adc fifo
  _outp(DYNO_BASE+LCR,  0);                              // set the range of channels to scan
  _outp(DYNO_BASE+HCR,  7);                              // initially set for 8 channels for mux boxes
  _outp(DYNO_BASE+FCR, SCANEN);                          // enable adc scan interrupts
  _outp(DYNO_BASE+ACR, KHZ200 | BIPOLAR5 | GAIN1);       // 200KHz scans, +-5Volt inputs
  while(_inp(DYNO_BASE+ARR) & WAIT);                     // wait for adc inputs to settle

  _outp(DYNO_BASE+MCR, PAGE1);                           // set the 8255 dio ports for outputs, mode0
  _outp(DYNO_BASE+15,  0x80);
  _outp(DYNO_BASE+MCR, PAGE0);

  old_dmm32b_vect = _dos_getvect(12-8+0x70);             // save old irq12 isr vector
  _dos_setvect(12-8+0x70, isr_dmm32b);                   // install new isr_dmm32b vector
  _outp(0xa1, _inp(0xa1) & 0xef);                        // unmask irq12 at pic

  _outp(DYNO_BASE+CTCCR, _inp(DYNO_BASE+CTCCR) | 0x02);  // set 8254 ctc0 for 10 MHz input
  _outp(DYNO_BASE+MCR, PAGE2);                           // set ctc0 for falling edge trigger
  _outp(DYNO_BASE+15 , 1);
  _outp(DYNO_BASE+MCR, PAGE0);
  _outp(DYNO_BASE+ICR, ADINTE | TINTE);                  // enable adc & ctc0 timer interrupts

  _enable();
  cout << "DMM32b initialized." << endl;
  }


void trigger_dmm32b(void)  
  {                                  // start dmm32b for the entire mux/scan/read routine
  reading_prop = 0;                  // start with mux box scans, not prop scans
  mux_addr = 0;

  _disable();
  
  // initialize the mux address and put it on the wires
  _outp(DYNO_BASE+MCR, PAGE1);
  _outp(DYNO_BASE+PORTA , mux_addr);
  _outp(DYNO_BASE+MCR, PAGE0);

  // start a 200 usec settling period for the mux box data 
  // with a 10MHz input to ctc0 and desired delay of 200 usec the divisor = 2000 = 0x07d0
  _outp(DYNO_BASE+15, CTC0 | LSBMSB | MODE0 | BINARY);   // program ctc0
  _outp(DYNO_BASE+12, 0xd0);                             // load divisor LSB
  _outp(DYNO_BASE+12, 0x07);                             // load divisor MSB
  _outp(DYNO_BASE+ICR, ADINTE | TINTE);                  // enable adc and timer interrupts
  _enable();
  
  // ctc0 will generate a TINT interrupt 200usec from now
  // we will trigger a scan of the mux boards in the dmm32b isr when the TINT interrupt occurs
  // and read the data in the dmm32b isr when a ADINT interrupt occurs
  }
    

void interrupt isr_dmm32b(void)
  {
  // two modes of operation
  // 1) reading the prop dynos      (no multiplexing)
  // 2) reading the mux boxes dynos (with multiplexing)
  
  // for each mode there are two types of interrupts
  // a) settling time complete (TINT, 82C54 ctc0 timeouts)
  // b) adc scan complete (ADINT)
  
  unsigned int i;
  unsigned int lsb      = 0;
  unsigned int msb      = 0;
  long int     prop_pos = 0;
  
  // mode 1, reading prop dynos
  if(reading_prop)
    {
    if(_inp(DYNO_BASE+ISR) & TINT)                        // 200 usec settling time is complete
      {                                                   // trigger a scan of the prop dynos
      _outp(DYNO_BASE+ICR, ADINTE & ~TINTE);              // disable further timer interrupts
      _outp(DYNO_BASE+FCR, _inp(DYNO_BASE+FCR) | FIFORST);// reset the fifo
      while(_inp(DYNO_BASE+ARR) & WAIT);                  // ensure the adc is ready and then
      _outp(DYNO_BASE, 0);                                // trigger the scan
      // we will read the prop dyno adc's when an ADINT interrupt occurs about 30 usec from now
      }
    else                                                  // not a TINT, must be an ADINT
      {                                                   
      if(_inp(DYNO_BASE+ISR) & ADINT)                     // ADINT occurred so it is 
        {                                                 // time to read the prop data
        time_stamp(&prop_adc_time);
        if(te5650IsCapture()==1)
          {
          te5650CapturePos(&prop_index);                  // get prop index
          }
        te5650ActPos(&prop_pos);                          // get prop position
        prop_report = prop_pos;
        index_report = prop_index;
        time_stamp(&prop_position_time);
        time_stamp(&prop_rpm_time);
        prop_position_absolute_current = prop_pos;
        prop_position = (prop_pos - prop_index) - 19409;  // 4/25/2006 jtm prop zero
        
        for(i=0; i<6; i++)                                // read and buffer the dyno data  
          {
          prop_array[i] = _inpw(DYNO_BASE);
          }
        _fmemcpy(prop_buffer, prop_array, sizeof(prop_array));
                                                                                                
        prop_new_data = 1;                            // flag new data is available
        prop_data_frame = frame;                      // record the frame number      
        
        reading_prop = 0;                             // reset flag so we scan the mux boxes next
        _outp(DYNO_BASE+FCR, _inp(DYNO_BASE+FCR) | FIFORST); // reset the fifo
        _outp(DYNO_BASE+LCR,  0);                     // set the range of channels to scan
        _outp(DYNO_BASE+HCR,  7);
        
        data_ready = 1;
        }
      }
    } // end of the prop section
  
  
  // mode 2, reading mux boxes 
  else
    {
    if(_inp(DYNO_BASE+ISR) & TINT)                      // the 200 usec settling time is complete
      {                                                 // so start a mux box scan
      _outp(DYNO_BASE+ICR, ADINTE & ~TINTE);            // disable further TINT interrupts
      while(_inp(DYNO_BASE+ARR) & WAIT);                // wait for converters to get ready
      _outp(DYNO_BASE, 0);                              // trigger the scan
      // we will read the mux box scan about 40 usec from now when an ADINT irq occurs
      }
      
    else                                                  
      {
      if(_inp(DYNO_BASE+ISR) & ADINT)                     // the mux box adc data is ready
        {
        for(i=0; i<8; i++)                                // read the data  
          {
          dyno_array[i + (mux_addr*8)] = _inpw(DYNO_BASE);
          }
        
        mux_addr++;                                       // set mux address for the next scan
        
        if(mux_addr < 16)                                 // not done scanning all 16 addresses
          {
          _outp(DYNO_BASE+MCR, PAGE1);                    // put address on the dio port
          _outp(DYNO_BASE+PORTA , mux_addr);
          
          // start another 200 usec settling period
          _outp(DYNO_BASE+MCR, PAGE0);                         // access the 82C54 timer
          _outp(DYNO_BASE+15, CTC0 | LSBMSB | MODE0 | BINARY); // set up ctc0
          _outp(DYNO_BASE+12, 0xd0);                           // divisor LSB (2000 or 0x07d0)
          _outp(DYNO_BASE+12, 0x07);                           // divisor MSB 
          _outp(DYNO_BASE+ICR, ADINTE | TINTE);                // enable adc & timer interrupts
          // we will scan the mux data again when ctc0 generates a TINT irq 200 usec from now
          }
          
        else                                            // all 16 mux addresses have been scanned
          {
          time_stamp(&dyno_time);
          
          _fmemcpy(dyno_buffer, dyno_array, sizeof(dyno_array));// buffer the data
          dyno_data_frame = frame;                              // set system flags
          dyno_new_data = 1;
          
          mux_addr = 0;                                   // reset the mux addess for next frame 
          _outp(DYNO_BASE+MCR, PAGE1);
          _outp(DYNO_BASE+PORTA, mux_addr);
          _outp(DYNO_BASE+MCR, PAGE0);
          
          reading_prop = 1;                                   // set flag to do a prop scan next
          _outp(DYNO_BASE+FCR, _inp(DYNO_BASE+FCR) | FIFORST);// reset the fifo
          _outp(DYNO_BASE+LCR, 16);                           // set range of prop channels
          _outp(DYNO_BASE+HCR, 21);

          // start another 200 usec settling period
          _outp(DYNO_BASE+MCR, PAGE0);                        // access the 82C54's registers
          _outp(DYNO_BASE+15, CTC0 | LSBMSB | MODE0 | BINARY);// set up ctc0
          _outp(DYNO_BASE+12, 0xd0);                          // load divisor LSB (2000 or 0x07d0)
          _outp(DYNO_BASE+12, 0x07);                          // load divisor MSB
          _outp(DYNO_BASE+ICR, ADINTE | TINTE);               // enable adc & timer interrupts
          // we will scan the prop dynos when ctc0 generates a TINT irq 200 usec from now
          }
        }
      }
    } // end mux box section

  _outp(DYNO_BASE+MCR, INTRST); // reset the dmm32's interrupt circuit    
  _outp(0xa0, 0x20);            // slave  EOI
  _outp(0x20, 0x20);            // master EOI  
  // end dmm32b isr
  }
  

void interrupt sys_timer_isr(void) 
  {                                // every pc timer tick (approx 55 msec)
  sys_timer.high++;                // increment the high long word of the obc system timer
  _chain_intr(rtc_vect);           // chain back into the pc's original rtc isr
  }
  

void terminate(void)
  {
  char s[81];
  
  _getch();
  _disable();
  shutdown_tech80();
  shutdown_enet();
  shutdown_com1();
  shutdown_com2();
  shutdown_com4();
  shutdown_dmm32a();
  shutdown_dmm32b();
  shutdown_escc();
//  shutdown_sys_timer();
  shutdown_uio48();
  _enable();
  
  // put final data on screen
        sprintf(s, "%s%8lu %5u%s%8lu", "Frame start:   ",
          frame_time.high, frame_time.low, "    Frame # ", frame);
        PC_DispStr(0,6, (unsigned char *)s, DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);  

        sprintf(s, "%s%8lu %5u", "OBS data:      ",
          obs_time.high, obs_time.low);
        PC_DispStr(0,7, (unsigned char *)s, DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);

        sprintf(s, "%s%8lu %5u", "Dyno data:     ",
          dyno_time.high, dyno_time.low);
        PC_DispStr(0,8, (unsigned char *)s, DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);

        sprintf(s, "%s%8lu %5u", "Prop adc:      ",
          prop_adc_time.high, prop_adc_time.low);
        PC_DispStr(0,9, (unsigned char *)s, DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);

        sprintf(s, "%s%8lu %5u", "Prop position: ",
          prop_position_time.high, prop_position_time.low);
        PC_DispStr(0,10, (unsigned char *)s, DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);

        sprintf(s, "%s%8lu %5u", "Commands rcvd: ",
          cmds_time.high, cmds_time.low);
        PC_DispStr(0,11, (unsigned char *)s, DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);

        sprintf(s, "%s%8lu %5u", "ADCP rcvd:     ",
          adcp_time.high, adcp_time.low);
        PC_DispStr(0,12, (unsigned char *)s, DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);

        sprintf(s, "%s%8lu %5u", "DS rcvd:       ",
          ds_time.high, ds_time.low);
        PC_DispStr(0,13, (unsigned char *)s, DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
        
        sprintf(s, "%s%8lu %5u%s%8lu ", "LN200 rcvd:    ",
          ln200_time.high, ln200_time.low, "    LN200 # ", ln200_rx_num);
        PC_DispStr(0,14, (unsigned char *)s, DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
        
        sprintf(s, "%s%lu", "Received packets: ", packets_rcvd);          
        PC_DispStr(0,15, (unsigned char *)s, DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);

        sprintf(s, "%s%lu", "Ax0 count:        ", ax0_counter);          
        PC_DispStr(0,16, (unsigned char *)s, DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
        
        sprintf(s, "%s%lu", "Ax1 count:        ", ax1_counter);          
        PC_DispStr(0,17, (unsigned char *)s, DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
        
        sprintf(s, "%s%lu", "Discards:         ", discards);          
        PC_DispStr(0,18, (unsigned char *)s, DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);

        sprintf(s, "%s%lu", "Receiver calls:   ", receiver_calls);          
        PC_DispStr(0,19, (unsigned char *)s, DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
  }
  
  
void shutdown_uio48(void)
  {
  for(int i=1;i<49;i++) set_bit(i);
  }     


void shutdown_sys_timer(void)
  {  // restore pc's default system timer isr
  _disable();
  _dos_setvect(0x08, rtc_vect);
  _enable();
  }


void shutdown_enet(void)
  {
  
  // use with at/lantic packet driver  
/*  while(handle > 0)
    {
    release_type(handle);
    handle--;
    }
*/

  // use with the ne2000 packet driver    
  terminate(handle);  
  }    
  
  
void shutdown_tech80(void)
  {
  te5650SetMotor(0);
  }


void shutdown_dmm32a(void)
  {                                           // onboard sensors, irq 11
  _outp(OBS_BASE+ICR, 0x00);                  // disable interrupts on the dmm32
  _disable();
  _outp(OBS_BASE+MCR, 0x20);                  // apply dmm32 master reset
  _outp(0xa1, _inp(0xa1) | ~0xf7);            // mask off irq 11 at the pic
  _dos_setvect(11-8+0x70, old_dmm32a_vect);   // rehook previous irq handler
  _enable();
  }


void shutdown_dmm32b(void)
  {                                           // dyno, irq 12
  _outp(DYNO_BASE+ICR, 0x00);                 // disable interrupts on the dmm32
  _disable();
  _outp(DYNO_BASE+MCR, 0x20);                 // apply dmm32 master reset
  _outp(0xa1, _inp(0xa1) | ~0xef);            // mask off irq 12 at the pic
  _dos_setvect(12-8+0x70, old_dmm32b_vect);   // rehook previous irq handler
  _enable();
  }


void shutdown_escc(void)
  {
  escc1.kill_port(channela);
  escc1.kill_port(channelb);
  }


void build_packet(void)
  {
  unsigned i,j,value;
  unsigned checksum16 = 0;
  unsigned long checksum32 = 0;
  unsigned char udp_pseudo[20];
  long prop_delta_position = 0;
  //unsigned long prop_delta_time_high = 0;
  long prop_delta_time_high = 0;
  //unsigned long prop_delta_time_low = 0;
  long prop_delta_time_low = 0;
  unsigned long prop_delta_time = 0;

/*
  for(j=0;j<1514;j++)
    {
    data_pkt[j]=0;
    }
*/    
    
  _fmemset(data_pkt, 0, sizeof(data_pkt));
  
  _disable();             // turn off other services while we copy data
  
  sys.packet_number++;
  sys.frame = frame - 1;  // data is from last frame--not this one
  sys.fs_status = 0;
  sys.fs_status = (read_port(FSHIGH) << 8) | read_port(FSLOW);
  sys.op_status = read_port(FSHIGH2);
  sys.time_high = frame_time.high;
  sys.time_low  = frame_time.low;
  
  obs.new_data = obs_new_data;
  obs_new_data = 0;
  obs.frame = obs_data_frame;
  obs.time_high = obs_time.high;
  obs.time_low  = obs_time.low;
  _fmemcpy(&obs.data, &obs_buffer, sizeof(obs.data));  
  
  dyno.new_data = dyno_new_data;
  dyno_new_data = 0;
  dyno.frame = dyno_data_frame;
  dyno.time_high = dyno_time.high;
  dyno.time_low  = dyno_time.low;
  _fmemcpy(&dyno.data, &dyno_buffer, sizeof(dyno.data));  
  
  prop.new_data = prop_new_data;
  prop_new_data = 0;
  prop.frame = prop_data_frame;
  prop.rpm_time_high = prop_rpm_time.high;
  prop.rpm_time_low  = prop_rpm_time.low;
  prop.position_time_high = prop_position_time.high;
  prop.position_time_low  = prop_position_time.low;
//  while(prop_position < 0) prop_position += 10160;  // old encoder normalize the prop position
  while(prop_position < 0) prop_position += 20000;    // new encoder normalize the prop position
//  while(prop_position > 10159) prop_position -= 10160; // old encoder
  while(prop_position > 19999) prop_position -= 20000;   // new encoder
  prop.position = prop_position;
  prop_delta_position = prop_position_absolute_current - prop_position_absolute_previous;

/*
  prop_delta_time_high = prop_position_time.high - prop_position_time_previous.high;
  prop_delta_time_low = prop_position_time.low - prop_position_time_previous.low;
  if(prop_delta_time_low < 0)
    {
    prop_delta_time_high--;
    prop_delta_time_low += 65536;
    }
  prop_delta_time = (65535 * prop_delta_time_high) + prop_delta_time_low;
  
  if(prop_delta_time != 0)
     prop.rpm = (int) (rpm_sf * ((float)prop_delta_position / (float)prop_delta_time));
*/
  // rpm scale factor = 60 sec/min  x  rev/numcounts  x  1/.01 sec
  //prop.rpm = (int) ((float)prop_delta_position * 0.590551f);  // old encoder
  prop.rpm = (int) ((float)prop_delta_position * 0.3f);  // new encoder
  
  prop_position_absolute_previous = prop_position_absolute_current;
  prop_position_time_previous.high = prop_position_time.high;
  prop_position_time_previous.low  = prop_position_time.low;

  prop.adc_time_high = prop_adc_time.high;
  prop.adc_time_low  = prop_adc_time.low;
  _fmemcpy(&prop.data, &prop_buffer, sizeof(prop.data));  
  
  ln200.new_data = ln200_new_data;
  ln200_new_data = 0;
  ln200.frame = ln200_data_frame;
  ln200.time_high = ln200_time.high;
  ln200.time_low  = ln200_time.low;
  _fmemcpy(&ln200.data, &ln200_buffer, sizeof(ln200.data));  
                                      
  echo.new_data = echo_new_data;
  echo_new_data = 0;
  echo.frame = echo_data_frame;
  echo.time_high = cmds_time.high;
  echo.time_low  = cmds_time.low;
  _fmemcpy(&echo.data, &commands, sizeof(echo.data));  
  
  ds.new_data = ds_new_data;
  ds_new_data = 0;
  ds.frame = ds_data_frame;
  ds.time_high = ds_time.high;
  ds.time_low  = ds_time.low;
  _fmemcpy(&ds.data, &ds_buffer, sizeof(ds.data));  
  
  adcp.new_data = adcp_new_data;
  adcp_new_data = 0;
  adcp.frame = adcp_data_frame;
  adcp.time_high = adcp_time.high;
  adcp.time_low  = adcp_time.low;
  _fmemcpy(&adcp.data, &adcp_buffer, sizeof(adcp.data));  
  
  gps.new_data = gps_new_data;
  gps_new_data = 0;
  gps.frame = gps_data_frame;
  gps.time_high = gps_time.high;
  gps.time_low  = gps_time.low;
  _fmemcpy(&gps.data, &gps_buffer, sizeof(gps.data));  

  _enable();                               // finished copying data, restore other services
  
  _fmemcpy(data_pkt, broadcast_mac, 6);    // insert destination mac addr;
  _fmemcpy(data_pkt+6, obc_mac, 6);        // insert source mac addr

  // for DIX insert packet type = 0x0800 (ip packet)
  data_pkt[12] = 0x08;
  data_pkt[13] = 0x00;

  // insert ip header
  // insert version (4 bit) and header length (4 bit)
  data_pkt[14] = 0x45; // version = 0, hdr len = 5 (ie: 5 x 32-bit words) 69

  // insert type of service (TOS) (8 bit)
  data_pkt[15] = 0;

  // insert total length (16 bit)
  data_pkt[16] = 0x05;  // high byte 1500 bytes to fit in Ethernet data field
  data_pkt[17] = 0xdc;  // low byte  

  // insert id (16 bit)
  data_pkt[18] = high_byte(ip_id);     // high byte
  data_pkt[19] = low_byte(ip_id);      // low byte
  ip_id++;

  // insert flags (3 bit) and fragment offset (13 bit)
  data_pkt[20] = 0;
  data_pkt[21] = 0;

  // insert time to live (TTL) (8 bit)
  data_pkt[22] = 64;

  // insert protocol (8 bit)
  data_pkt[23] = 17;  // UDP protocol = type 17

  // insert dummy ip checksum
  data_pkt[24] = 0;
  data_pkt[25] = 0;

  // insert source ip address (32 bit)
  data_pkt[26] = 192;
  data_pkt[27] = 168;
  data_pkt[28] = 1;
  data_pkt[29] = 4;

/*
  // insert destination ip address (32 bit) sam
  data_pkt[30] = 192;
  data_pkt[31] = 168;
  data_pkt[32] = 1;
  data_pkt[33] = 8;
*/

/*
  // insert destination ip address (32 bit) jim
  data_pkt[30] = 192;
  data_pkt[31] = 168;
  data_pkt[32] = 1;
  data_pkt[33] = 5;
*/

/*  // insert destination ip address (32 bit) shore
  data_pkt[30] = 192;
  data_pkt[31] = 168;
  data_pkt[32] = 1;
  data_pkt[33] = 30;
*/

/*
  // insert destination ip address (32 bit) control
  data_pkt[30] = 192;
  data_pkt[31] = 168;
  data_pkt[32] = 1;
  data_pkt[33] = 8;
*/  

  // insert destination ip address (32 bit) broadcast ip
  data_pkt[30] = 192;
  data_pkt[31] = 168;
  data_pkt[32] = 1;
  data_pkt[33] = 255;

  // insert true ip header checksum
  // checksum is the 1's complement of the 16-bit 1's complement sum
  // each pair of 8-bit bytes is first converted to a 16-bit word which
  // must be converted from 2's complement to 1's complememt representation
  // use 32-bit integer math to capture the 1's complement carry bit
  checksum32=0;        
  for(j=14;j<34;)
    {
    value = (data_pkt[j] << 8) + data_pkt[j+1];  // form next 16-bit word
    checksum32 += value;                         // add it to the checksum and
    if(checksum32 > 0x0000ffff)                  // perform end-around carry if required
      {                                          
      checksum32 = (checksum32 & 0x0000ffff) + 1;
      }
    j = j+2;
    }
  checksum16 = (unsigned)(checksum32 & 0x0000ffff);// convert back to 16-bit
  checksum16 = ~checksum16;                        // take the 1's complement
  data_pkt[24] = high_byte(checksum16);            // and insert into header
  data_pkt[25] = low_byte(checksum16);

  // insert udp packet into ip data field (up to 1480 bytes)
  // start with the udp header
  // insert udp source port
  data_pkt[34] = high_byte(UDP_SOURCE_PORT);
  data_pkt[35] = low_byte(UDP_SOURCE_PORT);

  // insert udp destination port
  data_pkt[36] = high_byte(UDP_DEST_PORT);
  data_pkt[37] = low_byte(UDP_DEST_PORT);

  // insert udp length
  data_pkt[38] = high_byte(UDP_LENGTH);
  data_pkt[39] = low_byte(UDP_LENGTH);

  // insert dummy udp checksum
  // must wait until after udp data is loaded to calculate the real udp checksum
  data_pkt[40] = 0;
  data_pkt[41] = 0;
  
  /*
  // insert dummy udp data for trouble shooting
  for(i=42; i<1514; i++) // insert packet data
    {
    data_pkt[i] = (unsigned char) i & 0xf;
    }
  */
  
  // insert real obc data
  i = 42;
  
  _fmemcpy(&data_pkt[i], &sys, sizeof(sys));
  i += sizeof(sys);

  _fmemcpy(&data_pkt[i], &obs, sizeof(obs));
  i += sizeof(obs);

  _fmemcpy(&data_pkt[i], &dyno, sizeof(dyno));
  i += sizeof(dyno);
  
  _fmemcpy(&data_pkt[i], &prop, sizeof(prop));
  i += sizeof(prop);
  
  _fmemcpy(&data_pkt[i], &ln200, sizeof(ln200));
  i += sizeof(ln200);
  
  _fmemcpy(&data_pkt[i], &echo, sizeof(echo));
  i += sizeof(echo);
  
  _fmemcpy(&data_pkt[i], &ds, sizeof(ds));
  i += sizeof(ds);
  
  _fmemcpy(&data_pkt[i], &adcp, sizeof(adcp));
  i += sizeof(adcp);
  
  _fmemcpy(&data_pkt[i], &gps, sizeof(gps));
  i += sizeof(gps);

  // now calculate and insert the real udp checksum
  // from the udp pseudo-header and true header
  for(j=0;j<8;j++) udp_pseudo[j] = data_pkt[j+26];  // source & dest ip addr
  udp_pseudo[8] = 0;
  udp_pseudo[9] = data_pkt[23];   // protocol id
  udp_pseudo[10] = data_pkt[38];  // udp length high byte
  udp_pseudo[11] = data_pkt[39];  // udp length low byte

  // calculate the 1's complement checksum
  checksum32 = 0;
  for(j=0;j<12;)
    {
    value = (udp_pseudo[j] << 8) + udp_pseudo[j+1];
    checksum32 += value;                         // add them up
    if(checksum32 > 0x0000ffff)                  // perform end-around carry
      {
      checksum32 = (checksum32 & 0x0000ffff) + 1;
      }
    j = j+2;
    }

  // insert the true udp header and data next
  for(j=34;j<1514;)                              // true udp header and data
    {
    value = (data_pkt[j] << 8) + data_pkt[j+1];
    checksum32 += value;                         // add them up
    if(checksum32 > 0x0000ffff)                  // perform end-around carry
      {
      checksum32 = (checksum32 & 0x0000ffff) + 1;
      }
    j = j+2;
    }
  checksum16 = (unsigned)(checksum32 & 0x0000ffff);// convert back to 16-bit
  checksum16 = ~checksum16;                      // take the 1's complement
  if(checksum16 == 0) checksum16 = 0xffff;       // do this only for udp/tcp packets
  data_pkt[40] = high_byte(checksum16);
  data_pkt[41] = low_byte(checksum16);

  // end build_packet()
  }


unsigned char high_byte(unsigned int word)
  {
  return (unsigned char) ((word>>8)&0xff);
  }


unsigned char low_byte(unsigned int word)
  {
  return (unsigned char) (word&0xff);
  }
    

void init_screen(void)
{
  char s[81];

   PC_DispClrScr(DISP_FGND_LIGHT_GRAY + DISP_BGND_RED);
//  PC_DispClrRow(0, DISP_BGND_LIGHT_GRAY);
//  PC_DispClrRow(1, DISP_BGND_LIGHT_GRAY);
  
  sprintf(s, "%s", 
"                       Autonomous Model Onboard Computer                        ");
  PC_DispStr(0,0, (unsigned char *)s, DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);

  sprintf(s, "%s", 
"                               NSWC-CD Code 5600                                ");
  PC_DispStr(0,1, (unsigned char *)s, DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);  
    
  sprintf(s, "%s",
"                             March 8, 2004 Rev 0.8                              ");
  PC_DispStr(0,2, (unsigned char *)s, DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
  
}


void prep_ln200_command(void)
  {
  #define YES 1
  //#define X_AXIS 0.0f
  #define X_AXIS -0.48f
  #define Y_AXIS 0.0f
  #define Z_AXIS 0.0f
  #define LATITUDE 38.7f

  int i;
  int heading;
  unsigned latitude_valid = 0;
  unsigned boresights_valid = 0;
  unsigned heading_valid = 0;
  unsigned bit = 0;                               
  unsigned mode = 0;
  int      latitude;
  int      x_axis;
  int      y_axis;
  int      z_axis;
  
  unsigned icmd[7];
  
  //static unsigned do_once = 1;
  
  //heading = (int) (-52.0 * 182.044);   // along wavemaker?
//  heading =  (int) (-53.5 * 182.044);    // along wavemaker? 5/21/08
//  heading =  (int) (26.5 * 182.044);   // Aberdeen Alignment 8/28/06 sjc
//  latitude = (int) (LATITUDE * 182.044); // operating latitude,+-degrees, see ln200 ICD 5/21/08
  x_axis =   (int) (X_AXIS * 182.044);   // boresight rotations,+-degrees
  y_axis =   (int) (Y_AXIS * 182.044);
  z_axis =   (int) (Z_AXIS * 182.044);

    
  //mode = 0;      //LN200 modes are 0 (free inertial) and 1 (fast leveling)
  //mode = 1;
    
/*  latitude_valid   = YES; // 5/21/08
  heading_valid    = YES;  5/21/08
  boresights_valid = YES;
  bit = NO;
*/  
  //ln200_mode = 0;
  //if (commands.ln200_mode & 0x0010) commands.ln200_mode=0;               
//commented out 5/21/08
/*  if(commands.ln200_mode & 0x0010) commands.ln200_mode = commands.ln200_mode | 0x0001;               
  
  if(do_once)   // set heading to presets at model startup only
    {
    do_once = 0;
    commands.ln200_mode = 0x0011;
    }
*/                        
/*
if (commands.mode & 0x2000) latitude_valid =1;
if (commands.mode & 0x4000) heading_valid =1;

if (commands.ln200_mode & 0x8000)
  {latitude = (int) (command.ln200_init_latitude * 182.044);
   heading = (int) (command.ln200_init_heading * 182.044);
  }

if (send_once == 1)
  {heading = (int) (+128.0 * 182.044);
   latitude = (int) (LATITUDE * 182.044);
   mode = 1;
   latitude_valid = 1;
   heading_valid = 1;
  }
   
if (ready == 1)
  {heading = (int) (+128.0 * 182.044);
   heading = (int) (commands.ln200_init_heading * 182.044);
   latitude = (int) (LATITUDE *182.044);
   mode = (commands.ln200_mode & 0x0007);
   latitude_valid = 1;
   heading_valid = 1;
  } 
   
        
*/
/*  icmd[0]=0x0000;                   // assemble the command word icmd[]
  icmd[0]=icmd[0]|commands.ln200_mode & 0x0007;
  icmd[0]=icmd[0]|(boresights_valid<<3);
  icmd[0]=icmd[0]|(bit<<4);
  icmd[0]=icmd[0]|(latitude_valid<<8);
  icmd[0]=icmd[0]|((commands.ln200_mode & 0x0010) << 5);
*/

//  icmd[1]=latitude;
//  icmd[2]=heading;
  icmd[0] = commands.ln200_mode;
  icmd[1] = commands.ln200_init_latitude; //5/21/08
  icmd[2] = commands.ln200_init_heading;
  icmd[3] = x_axis;
  icmd[4] = y_axis;
  icmd[5] = z_axis;
  icmd[6] = 0x0;                                // calculate icmd[] checksum
  for (i=0;i<6;i++) icmd[6]=icmd[6]+icmd[i];  // sum icmd[0] to icmd[5]
    
  icmd[6]=~icmd[6];                           // take bit-wise 1's complement

  _fmemcpy(ln200_command, icmd, 14);
  } // end prep_ln200_command()
  

void time_stamp(struct TIMER * t_struct)
  {  // uses the pc's internal timer, resolution ~880 nsec
  unsigned int lsb;
  unsigned int msb;
  
   _outp(0x43, 0xd2);
   lsb = _inp(0x40);
   msb = _inp(0x40);
   t_struct->low = ~((msb << 8) | lsb);
   t_struct->high = sys_timer.high;
   }


void heartbeat(void)
  {
  write_bit(HEARTBEAT, (!read_bit(HEARTBEAT)));
  }
  
