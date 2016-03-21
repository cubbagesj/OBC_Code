/*    cb2000.cpp
  
      Main module for CB2000 project

      John Maurer
      NSWC-CD Code 5600
      RCM Lab
      01/08/2001

      Notes:
      1)  Compile and link with MSVC++ v1.52c, huge memory model.
      2)  Link with UIO48, CESCC, LN200, DMM32, TE5650.

*/

#include <string.h>
#include <conio.h>
#include <stdio.h>
#include <stdlib.h>
#include <graph.h>
#include <time.h>
#include <malloc.h>
#include <dos.h>
#include <bios.h>
#include <iostream.h>
#include <math.h>

#include "cb2000.h"
#include "cescc.h"     // communications controller
#include "dmm32.h"     // adc
#include "ln200.h"     // gyro
#include "uio48.h"     // dio
extern "C"
  {
  #include "te5650.h"  // motion controller
  }

Cescc escc1;                     // Comtech escc object for data communications
Dmm32 dmm32;                     // Diamond dmm32 A/D object
Uio48 uio48;                     // LBC586 DIO functions

//float in_old, out1old, out2old;

int c;
  
int data[65];         // raw data collected from ln200 and adc's
int data2[144];       // scaled data to be sent thru pcm stream
int data3[144];       // left adjusted data to be stored in obc disk file

char buffer[80];      // general purpose text string
char tbuffer1[64];    // data to be tx'd thru escc to ln200
char rbuffer[64];     // data received from ln200 thru escc
char pcm[96];         // data to be tx'd thru escc to pcm stream
char header[2048];    // header info for zeros and run files

int icmd[7];          // used to build ln200 query commands
int latitude, heading;   // initialization parameters for ln200
int x_axis, y_axis, z_axis;   // boresight rotations for ln200
int latitude_valid, heading_valid, boresights_valid;    //more for ln200
int mode = 0;   // ln200 operating mode
int bit  = 0;   // built-in-test flag for ln200
int direction;  // heading east/west, north/south
char bearing0[8], bearing1[8];  // ln200 startup bearing stored on disk

long unsigned frame = 0;  // data frame counter
long unsigned zeros_terminate = 0;
long unsigned run_terminate = 0;
unsigned pinger_count = 0;
unsigned channela, channelb;  // escc channel identifiers

int status_word    = 0; // system status word to be written to disk data file
int in_fm_failsafe = 1; // flags when fm failsafe signal is out of range
int in_hw_failsafe = 0; // flags when fs logic signals in-fs
int zeros_complete = 0; // flags that zeros have been taken but data run has not occured
int data_complete  = 1; // flags that data run has been completed
int cage_complete  = 0; // flags that gyro has been caged after a completed data run
int taking_zeros   = 0; // flags systems to begin taking zeros
int taking_data    = 0; // flags system to take data
int print_index_done = 0;

 // tech80 variables
int   axis1_step_1     = 0;  // sequences the actions of the 5950b motion controller
int   axis1_step_2     = 0;  // while capturing the index pulses
int   axis1_step_3     = 0;
long  axis1_index      = 0; // position count when index pulse is captured
long  axis1_limit_a    = 0; // position count when actuator reaches first stop
long  axis1_limit_b    = 0; // position count when opposite (2nd) stop is reached
long  axis1_center     = 0; // caculated mean between two actuator limits
long  axis1_bias       = 0; // calculated distance from index position to mechanical center
int   axis1_index_done = 0;
long  axis1_position   = 0;
int   axis1_command1_given = 0;
int   axis1_command2_given = 0;
short axis1_error      = 0;
float stern_out       = 0.0;  // terms for digital filter
float stern_out_old   = 0.0;
float stern_in_old    = 0.0;
float stern_in        = 0.0;

int   proceed = 0;
int   axis2_step_1     = 1;
int   axis2_step_2     = 0;
int   axis2_step_3     = 0;
long  axis2_index      = 0; // position count when index pulse is captured
long  axis2_limit_a    = 0; // position count when actuator reaches first stop
long  axis2_limit_b    = 0; // position count when opposite (2nd) stop is reached
long  axis2_center     = 0; // caculated mean between two actuator limits
long  axis2_bias       = 0; // calculated distance from index position to mechanical center
int   axis2_index_done = 0;
long  axis2_position   = 0;
int   axis2_command1_given = 0;
int   axis2_command2_given = 0;
short axis2_error      = 0;
long  rudder_history[100];
int   rudder_counter   = 0;
int   rudder_i         = 0;
float rudder_out       = 0.0;
float rudder_out_old   = 0.0;
float rudder_in_old    = 0.0;
float rudder_in        = 0.0;

int   axis3_step_1     = 0;
int   axis3_step_2     = 0;
int   axis3_step_3     = 0;
long  axis3_index      = 0; // position count when index pulse is captured
long  axis3_limit_a    = 0; // position count when actuator reaches first stop
long  axis3_limit_b    = 0; // position count when opposite (2nd) stop is reached
long  axis3_center     = 0; // caculated mean between two actuator limits
int   axis3_index_done = 0;
long  axis3_position   = 0;
short axis3_error      = 0;
long  axis3_bias       = 0; // calculated distance from index position to mechanical center
float fore_out       = 0.0;
float fore_out_old   = 0.0;
float fore_in_old    = 0.0;
float fore_in        = 0.0;


int   axis4_step_1     = 0;
int   axis4_step_2     = 0;
int   axis4_step_3     = 0;
long  axis4_index      = 0; // position count when index pulse is captured
long  axis4_limit_a    = 0; // position count when actuator reaches first stop
long  axis4_limit_b    = 0; // position count when opposite (2nd) stop is reached
long  axis4_center     = 0; // caculated mean between two actuator limits
int   axis4_index_done = 0;
long  axis4_position   = 0;
short axis4_error      = 0;
long  axis4_bias       = 0; // calculated distance from index position to mechanical center
      
int   axis5_step_1     = 0;
int   axis5_step_2     = 0;
int   axis5_step_3     = 0;
long  axis5_index      = 0; // position count when index pulse is captured
long  axis5_limit_a    = 0; // position count when actuator reaches first stop
long  axis5_limit_b    = 0; // position count when opposite (2nd) stop is reached
long  axis5_center     = 0; // caculated mean between two actuator limits
int   axis5_index_done = 0;
long  axis5_position   = 0;
short axis5_error      = 0;
long  axis5_bias       = 0; // calculated distance from index position to mechanical center

long  axis6_index      = 0;
long  axis6_position   = 0;
 
int actuator_index_done = 0;

float  mode_in = 0;    // used to filter fm mode commands
float  mode_out = 0;
float  mode_out_old = 0;
float  mode_in_old = 0;
float  mode_command = 0;


long act_pos = 0; // current position count
long command = 0; // position command for actuator
long target  = 0; // ?
short dumpit = 0;
float degrees = 0.0;
//int set_counter =0;    // for rudder calibration

char data_name[80];     //string to hold name of current data file
char zero_name[80];     //string to hold name of current zeros file
char datebuffer[9];     //system date in ascii format
char timebuffer[9];     //system time in ascii format

char d_name[50][20];
char z_name[50][20];

unsigned value; 
int run_number = 1;

FILE *testf1;   // 
//FILE *testf2;   // 
FILE *framefile;  // not used
FILE *axis1_bias_file;          //
FILE *axis2_bias_file;
FILE *axis3_bias_file;
FILE *datafile, *data_namefile; // file handles for the data storage files
                                // and the data storage name file   
FILE *zerofile, *zero_namefile; // file handles for the zeros storage files                          
                                // and zeros storage name file
//FILE *errorfile;                // write error messages to disk file

//FILE *rudder_cmnd;               // track stern commands to te5650

FILE *bearingfile;          // stores startup heading for ln200

void main()
  {
  
//  testf1 = fopen("c:\\testf1.txt", "wt");
//  fprintf(testf1, "%s", "position, command, out_raw, out, dac, frame\n");
//  fprintf(testf1, "%s", "out1old, out1new, in_old, in_new, out\n\n");
  
//  testf2 = fopen("c:\\testf2.txt", "wt");
//  fprintf(testf2, "%s", "position, command, data[30], msb, dac\n");  
//  fprintf(testf2, "%s", "out2old, out2new, in_old, in_new, out\n\n");

  unsigned i, j, k;
 
  struct escc_regs settings;      // local mirror of the escc registers 

  // get the bias settings for stern planes, rudder, fore planes
  if((axis1_bias_file = fopen("c:\\axis1.txt", "rt")) == NULL)
    {
    cout << "Failed to open file: axis1.txt." << endl;
    terminate_program();    
    }
  else
    {
    cout << "Opening file: axis1.txt." << endl;
    }    
      
    fscanf(axis1_bias_file, "%s\n",  buffer);
    axis1_bias = atol(buffer);
    fclose(axis1_bias_file);

  if((axis2_bias_file = fopen("c:\\axis2.txt", "rt")) == NULL)
    {
    cout << "Failed to open file: axis2.txt." << endl;
    terminate_program();    
    }
  else
    {
    cout << "Opening file: axis2.txt." << endl;
    }    
      
    fscanf(axis2_bias_file, "%s\n",  buffer);
    axis2_bias = atol(buffer);
    fclose(axis2_bias_file);

  if((axis3_bias_file = fopen("c:\\axis3.txt", "rt")) == NULL)
    {
    cout << "Failed to open file: axis3.txt." << endl;
    terminate_program();
    }
  else
    {
    cout << "Opening file: axis3.txt." << endl;
    }    
  fscanf(axis3_bias_file, "%s\n",  buffer);
  axis3_bias = atol(buffer);
  fclose(axis3_bias_file);

  // get the initial run headings for the LN200 
  if((bearingfile = fopen("c:\\bearings.txt", "rt")) == NULL)
    {
    cout << "Failed to open bearings file" << endl;
    terminate_program();
    }
  fscanf(bearingfile, "%s%s", bearing0, bearing1);
  cout << "Bearing0 = " << bearing0 << endl << "Bearing1 = " << bearing1 << endl;
  fclose(bearingfile);
  
  _clearscreen(_GCLEARSCREEN);             
  
  cout << "Program CB2000" << endl << endl;
  
  //  get the names of the run files from an ascii file on disk 
  if((data_namefile = fopen("c:\\dnames.txt", "rt"))==NULL)
    {
    cout << "Error opening Data Names File" << endl; 
    terminate_program();
    }
  else
    {
    cout << "Open Data Names File." << endl;
    }
  for(i=0; i<50; i++)
    {
    fscanf(data_namefile, "%s", d_name[i]);
    }  
  fclose(data_namefile);   
   
  //  get the names of the zeros files
  if((zero_namefile = fopen("c:\\znames.txt", "rt"))==NULL)
    {
    cout << "Error opening Zero Names File" << endl;
    terminate_program();
    }
  else
    {
    cout << "Open Zero Names File." << endl;
    }
  for(i=0; i<50; i++)
    {
    fscanf(zero_namefile, "%s", z_name[i]);
    }  
  fclose(zero_namefile);   
   
  reset_ln200();        // apply a Power On Reset to the ln200
  
  zero_buffers();       // overwrite lingering garbage with zeros
  
  init_tech80();        // configure two TECH-80 5950b motion controllers

  // configure the Comtech ESCC communications controller
  // to connect to the LN200 (channel A) and the pcm telemetry stream (channel B).
  // open the two escc communications ports using add_port() function
  // parameters: (base address of escc board,
  //              escc channel 0 or 1,
  //              IRQ setting,   
  //              dma channel for receive,  
  //              dma channel for transmit)
  // channel A first, to communicate with the LN200                  
  channela = escc1.add_port(0x240,0,5,0,0);  //IRQ 5, no dma
  cout << "Channel A = " << hex << channela << dec <<endl;
  // channel B next, to transmit the pcm telemetry
  channelb = escc1.add_port(0x240,1,5,1,5);  //IRQ 5, Rx=DMA1, Tx=DMA5
  cout << "Channel B = " << hex << channelb << dec <<endl;
  
  // assemble configuration data for channel_a for LN200 SDLC operation.
  // see Siemens SAB 82532 ESCC User's Manual page 110.
  settings.mode = 0x88;   // transparent 0 frames, receiver active
  settings.timr = 0x1f;   // for cb98
  settings.xbcl = 0x00;   // n/a for interrupt mode
  settings.xbch = 0x00;   // interrupt mode (no dma)
  settings.ccr0 = 0x80;   // for cb98
  settings.ccr1 = 0x10;   // txd push-pull, one's insertion, clk mode 0 (use LN200 clock)
  settings.ccr2 = 0x00;   // normal txd/rxd, ssel=0 (clk mode 0a), crc-ccitt, no inversion (LN200 mode) 
  settings.ccr3 = 0x00;   // no preamble, rx crc off, tx crc generated internally
  settings.ccr4 = 0x00;   // 
  settings.bgr  = 0x00;   // 
  settings.iva  = 0x00;   // 
  settings.ipc  = 0x03;   // masked interrupts NOT visible, pin INT = push/pull active high
  settings.imr0 = 0x04;   // cdsc disabled
  settings.imr1 = 0x00;   // all interrupts enabled 
  settings.pvr  = 0x00;   //
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
  if(escc1.init_port(channela,OPMODE_HDLC,&settings,2,2)==TRUE)
    cout << "Intialize Channel A OK." << endl;
  else
    {
    cout << "Initialize Channel A FAILED!" << endl;
    terminate_program();
    }
  
  // test channel a to make sure it opened
  if(escc1.clear_rx_buffer(channela)==TRUE)
    cout << "Channel A rx buffer cleared OK.  " << endl;
  else
    {
    cout << "Channel A rx buffer clear FAILED!" << endl;
    terminate_program();
    }

  if(escc1.clear_tx_buffer(channela)==TRUE)
    cout << "Channel A tx buffer cleared OK." << endl;
  else
    {
    cout << "Channel A tx buffer clear FAILED!" << endl;
    terminate_program();
    }
  
  // set channel a frame transmission type
  if(escc1.set_tx_type(channela, TRANSPARENT_MODE)==TRUE)
    cout << "Channel A set to Transparent Mode.  " << endl;
  else
    {
    cout << "Channel A Transparent Mode not set!  Channel not open!" << endl;
    terminate_program();
    }

  // Channel b will run off the escc internal oscillator, so must first
  // configure the escc frequency generator to produce a 9.8304 MHz clock signal
  // from the onboard 18.432 MHz TTL oscillator.
  //
  // The 9.8304 MHz clock is used as the master clock for the pcm
  // data stream and controls the overall data frame timing in the model.
  //
  // See escc.cpp, Cypress ICD2053B datasheet and BitCalc software
  // available at the Cypress Web site to calculate the clock scaling factor
  // to be loaded into the clock generator.
  //
  // void Cescc::set_clock_generator(unsigned port,    //either channel A or B
  //                                unsigned long hval,//stuffed hex value obtained from BitCalc
  //                                unsigned nmbits)   //number of bits in stuffed value from BitCalc
  escc1.set_clock_generator(channela, 0x5d31c0L, 0x18);
  
  // Configure channel_b for biphase-m operation for pcm telemetry.
  // see Siemens SAB 82532 ESCC User's Manual page 110.
  settings.mode = 0xC8;  // extended transparent mode 0, rcvr inactive, extern timer k=32768
  settings.timr = 0xe2;  // cnt=7, value=2, continuous 100 Hz interrupts when tcp=1/9.8304MHz
  settings.xbcl = 0x00;  // n/a in interrupt mode
  settings.xbch = 0x80;  // n/a in interrupt mode (no dma)
  settings.ccr0 = 0xD4;  // power-up, master clk=xtal, FM1(biphase-m), hdlc/sdlc
  settings.ccr1 = 0x17;  // set txd push-pull, set clk mode 7
  settings.ccr2 = 0x38;  // br9&8=0, bdf=1, ssel=1, toe=1, rwx=0, crc-ccitt, no inversion
  settings.ccr3 = 0x02;  // no preamble, radd=0, crl=0, rxrc=off, xcrc=1(off), psd=na
  settings.ccr4 = 0x00;  // added to implement MCK4 and EBRG, but not used
  settings.bgr  = 0x3f;  // brg divisor=63 (3F hex) 76.8 KHz = 9.8304 MHz /( (63+1)*2 )
  settings.iva  = 0x00;  // not set here?
  settings.ipc  = 0x03;  // masked (disabled) interrupts NOT visible, pin INT = push/pull active high
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
    terminate_program();
    }
  
  // test channelb to make sure it opened
  if(escc1.clear_rx_buffer(channelb)==TRUE)
    cout << "Channel B rx buffer cleared OK.  " << endl;
  else
    {
    cout << "Channel B rx buffer clear FAILED!" << endl;
    terminate_program();
    }
  
  if(escc1.clear_tx_buffer(channelb)==TRUE)
    cout << "Channel B tx buffer cleared OK." << endl;
  else
    {
    cout << "Channel B tx buffer clear FAILED!" << endl;
    terminate_program();
    }
  
  //set channelb frame transmission type
  if(escc1.set_tx_type(channelb, TRANSPARENT_MODE)==TRUE)
    cout << "Channel B set to Transparent Mode.  " << endl;
  else
    {
    cout << "Channel B Transparent Mode not set!  Channel not open!" << endl;
    terminate_program();
    }
   
  // The escc communications controller is now ready to go.
  // Query the ln200 to set initial latitude, heading, boresights, gyro mode
  mode = 1;  // fast levelling mode, used while CAGED
  prep_ln200_command(HEADING_VALID); // build the ln200 initialization command
  j = escc1.tx_port(channela, tbuffer1, 14); // and send it to the ln200
  cout << "Query LN200" << endl;
  
  // wait for ln200 response
  i = 0;    
  while((j = escc1.rx_port(channela, rbuffer, 64)) != 33)
    {
    if(++i>100000)  // time-out loop provides soft crash if no response
      {             // from LN200
      cout << "No response from LN200." << endl;
      terminate_program();
      }
     }
  k = j;  // k is the number of bytes received in rbuffer
  cout << "LN200 received." << endl;

//******************************IDLE LOOP********************************
//  This 10msec loop executes repeatedly after configuration is complete.
//  Loop timing is crystal controlled by the rate at which the escc can transmit pcm
//  data up the telemetry link.
  while(!kbhit())  // program will execute until you hit any keystroke
    {
  // locate the encoder index positions for the three control plane actuators
    if(!actuator_index_done)
      {
      index_actuators();
      }
      
    if(actuator_index_done && !print_index_done)
      {
      print_index(); 
      }  
      
    cout << "1\r" << flush;
      
    perform_common_tasks();  // transmit pcm data, trigger a/d converters,
                             // collect command data, process fm commands,
                             // control ballast and planes
    
    cout << "2\r" << flush;
    
    // wait for the pcm tx fifo to drain before starting new data frame;
    // this provides the timing for the 100 Hz frame rate.    
    escc1.wait_for_tx(channelb);
    
    heartbeat();
        
    cout << "3\r" << flush;
    
    frame++;  // update the frame counter

      // tell the DAQ system what frame this is
    uio48.port_write(FRAME_LOW,frame); // LBC586 DIO ports connected to DAQ
    uio48.port_write(FRAME_HIGH, frame >> 8);

      // tell the DAQ system the number of the impending run
    value = ~uio48.port_read(RUN_NUMBER); // read data on the port 
    uio48.port_write(RUN_NUMBER, (value & 0x00e0) | (run_number & 0x001f)); // and insert the run number
    
    uio48.clr_bit(COMMAND0);  // send stop command to DAQ system "00"
    uio48.clr_bit(COMMAND1);


// ****************************TAKING ZEROS******************************
// This loop executes whenever the TAKE_ZEROS command is asserted
    if(taking_zeros)
      {
      uio48.set_bit(COMMAND0);  // send taking_zeros command to DAQ system "01"
      uio48.clr_bit(COMMAND1);
        
      //open a file to store this run's zeros
      if((zerofile = fopen(z_name[run_number], "wb")) == NULL)
        {
        printf("\nCan't open Zeros file.  Exiting.\n");// exit on error
        terminate_program();
        }
       
      _strdate(datebuffer);  // get system date
      _strtime(timebuffer);  // and time in ascii string format
      
        // build a file header to be written to the zeros file 
        // write 1536 ff's to file header
      for(i = 0; i < 1536; i++)
        {
        header[i] = '\xff';
        }
      
      // write system date to header in ascii mo/da/yr
      for(i=1536; i< 1544; i++)
        {
        header[i] = datebuffer[i-1536];
        }
      
      // write 56 spaces to header
      for(i = 1544; i < 1600; i++)
        {
        header[i] = ' ';
        }

      // write system time to header
      for(i=1600; i<1608; i++)
        {
        header[i] = timebuffer[i-1600]; 
        }

      // 53 more spaces
      for(i = 1608; i < 1661; i++)
        {
        header[i] = ' ';
        }

      // write file name to header
      for(i = 1661; i< 1674; i++)
        {
        header[i] = z_name[run_number][i-1661];  
        }
      
      // 374 more spaces
      for(i = 1674; i < 2048; i++)
        {
        header[i] = ' ';
        }
      
      // header is ready, write it to disk
      fwrite(header, sizeof(header), 1, zerofile);
        
      // set up counter to take zeros for next 200 data frames
      zeros_terminate = frame + 200;

      while(frame < zeros_terminate)
        {
        perform_common_tasks();
        
        fwrite(data3, sizeof(data3), 1, zerofile);   // write pcm data to file
                                                     // data3[] is left-adjusted version of data2[]
        cout << "5\r" << flush;
        
        // wait for the pcm tx fifo to drain creating the 100Hz timing
        escc1.wait_for_tx(channelb);
        
        heartbeat();        
        
        cout << "6\r" << flush;
        
        frame++;  // declare a new frame
        
          // tell the DAQ system what frame this is
        uio48.port_write(FRAME_LOW,frame);
        uio48.port_write(FRAME_HIGH, frame >> 8);

          // tell the DAQ system the number of the impending run
        value = ~uio48.port_read(RUN_NUMBER); // read data on the port 
        uio48.port_write(RUN_NUMBER, (value & 0x00e0) | (run_number & 0x001f)); // and "or" in the run number
    
        } // end while(frame < zeros_terminate)
        
      // zeros are done, flag the system and close the zeros file  
      taking_zeros = 0;
      fclose(zerofile);
      zeros_complete = 1;
      data_complete = 0;

      cout << "7\r" << flush;
     
      } // end if(taking_zeros)
      
      uio48.clr_bit(COMMAND0);  // command DAQ to stop "00"
      uio48.clr_bit(COMMAND1);

      
//******************************TAKING DATA**********************************          
// This loop runs repeatedly while the TAKE_DATA command is asserted
    if(taking_data && actuator_index_done)  //
      {
      uio48.clr_bit(COMMAND0);  // send taking_data command to DAQ system "10"
      uio48.set_bit(COMMAND1);
      
        //open a data file       
      if((datafile = fopen(d_name[run_number], "wb")) == NULL)
        {
        printf("\nCan't open Data file.  Exiting.\n");  // exit on error
        terminate_program();
        }
      
      cout << "8\r" << flush;
      
      _strdate(datebuffer);  // get system date
      _strtime(timebuffer);  // and time in ascii string format

        // build a header for the data file, same as for the zeros file
      for(i = 0; i < 1536; i++)
        {
        header[i] = '\xff';
        }

      for(i=1536; i< 1544; i++)
        {
        header[i] = datebuffer[i-1536];
        }
      
      for(i = 1544; i < 1600; i++)
        {
        header[i] = ' ';
        }

      for(i=1600; i<1608; i++)
        {
        header[i] = timebuffer[i-1600]; 
        }
      
      for(i = 1608; i < 1661; i++)
        {
        header[i] = ' ';
        }

      for(i = 1661; i< 1674; i++)
        {
        header[i] = d_name[run_number][i-1661];  
        }
      
      for(i = 1674; i < 2048; i++)
        {
        header[i] = ' ';
        }
      
      // write the header to disk
      fwrite(header, sizeof(header), 1, datafile);

      run_terminate = frame + 60000;   // set up a counter to limit the
                                       // data run to 60000 frames in case of a runaway
      int tracker_sync=0;

      while(taking_data)
        {
        // pinger will output freq. B, sync'd with msb=0 and lsb =0
        if(!tracker_sync && ((frame & 0x000f)==0x0002))
         { 
            reset_pinger();
            tracker_sync=1;
         }
                                     
        perform_common_tasks();
        
        fwrite(data3, sizeof(data3), 1, datafile);  // store run data to disk
                                                    // data3[] is left-adjusted version of data2[]
        cout << "9\r" << flush;
        
        if(tracker_sync == 1)
          { 
            run_pinger();
          }
          
        cout << "A\r" << flush;

        // wait for the pcm tx fifo to drain before starting new data frame;
        // this provides the timing for the 100 Hz frame rate.
        escc1.wait_for_tx(channelb);
        
        heartbeat();
        
        cout << "B\r" << flush;
        
        frame++;  // update the frame counter
        
          // tell the DAQ system what frame this is
        uio48.port_write(FRAME_LOW,frame);
        uio48.port_write(FRAME_HIGH, frame >> 8);

          // tell the DAQ system the number of the impending run
        value = ~uio48.port_read(RUN_NUMBER); // read data on the port 
        uio48.port_write(RUN_NUMBER, (value & 0x00e0) | (run_number & 0x001f)); // and "or" in the run number
    

//        if(frame > run_terminate)    // used in final code to stop recording
//          {                          // data if rcm "runs away" for 10 minutes
//          cout << "Data run exceeds 10 minutes." << endl;
//          terminate_program();     
//          }
                                 
              
        } // end of while(taking_data)      
        
        // taking data is done, flag the system and close the data file
        taking_data = 0;
        fclose(datafile);

        cout << "C\r" << flush;
        
        data_complete = 1;
        zeros_complete = 0;
        run_number++;   // declare end of current run, indicate # of next run
      
      } // end if(taking_data)      
      
      uio48.clr_bit(COMMAND0);  // command DAQ to stop "00"
      uio48.clr_bit(COMMAND1);
      
      test();

//      if(!(data[30] == 0))
//        {
//        terminate_program();
//        }

    }//*************************END OF IDLE LOOP**************************
  
  // arrive here when operator hits a key to terminate the program
  c = _getch();         // flush keyboard buffer
  terminate_program();  // time to quit
  
  }//**************************END OF MAIN***********************************
  

void test(void)
  {
  // ******** test *********
	static int initialized = 0;
	long int position, command;
	int lsb, msb, dac;
	float out, out_raw, out1new, out2new, in_new;
	const float t1 = 0.005;
	const float t2 = 0.006;
	static float in_old, out1old, out2old;
	const float k1 = 50;
	const float k2 = 0.00001;
//	const float a = (2.0 * k1) + (-100.0 * k1 * k2); 

//	const float b = (100.0 * k1 * k2) + (-1.0 * k1);
	const float b = (k1 * (k2 - t2)) / (t1-t2);
	const float a = k1 - b; 	
	struct _rccoord oldpos;
	
	long int command_filter;
	static long int command_filter_old, command_old;;
    const float tau = 0.1;     // filter time constant
    const float time_step = 0.01;            // filter time step 
    int dac_out;
 	
    // rudder is on fm ch3, aim2 ch7 (dmm32 ch22), data[38], 5950b axis2
    // rudder_adc = data[38] / full_scale_counts * input_range * scale_factor; //degrees

	command  = long int(data[38]);     // current rudder command from discriminator


    //rudder_out = (time_step*(rudder_in + rudder_in_old) / (2*tau+time_step))  - rudder_out_old *((time_step - 2*tau)/(2*tau + time_step));
    //rudder_out_old = rudder_out;
    //rudder_in_old = rudder_in;
    
    command_filter = long int((time_step * (command + command_old) / (2*tau + time_step)) - command_filter_old * ((time_step - 2*tau) / (2*tau + time_step)));
    command_filter_old = command_filter;
    command_old = command;
	
	dac_out = (int(command_filter) / 16) + 2048 - 4;   // dac output in 12-bit offset binary    

	if(dac_out > 4095) dac_out = 4095;
	if(dac_out < 0)    dac_out = 0;
    
	dmm32.da_out(dac_out,0);  // send it out on channel 0
	dmm32.da_out(dac_out,1);  // send it out on channel 0
	dmm32.da_out(dac_out,2);  // send it out on channel 0
	dmm32.da_out(dac_out,3);  // send it out on channel 0			
	

	
/*	oldpos = _settextposition(25,1);
	cout << "command: " << (5.0*command/32767.0) << "\tposition: "
	                    << ((5.0*float(position))/32767.0) << "\tdac: "
//	                    << position << "\tdac: "
	                    << (5.0*out/32767.0) << '\r' << flush; 
    _settextposition(25,1);
*/    
//	  }

// *******end test *******       
  }



  
int sign(float number)  // returns the sign (+-) of a floating point number
  {
  if(number < 0)
    {
    return -1;
    }
  else
    {
    return 1;
    }
  }  


void terminate_program(void)  // shutdown system at user request
  {
  
  if(c == 't')
    {
    uio48.set_bit(COMMAND0);  // command DAQ system to stop "11"
    uio48.set_bit(COMMAND1);
    }
    
  escc1.kill_port(channela);  //shut down the ESCC board 
  escc1.kill_port(channelb);
  
    // send final sync pulse to the DAQ system to latch in the stop command
  uio48.set_bit(SYNC);
  uio48.clr_bit(SYNC);
  
  _fcloseall();               // general file cleanup
  
  te5650DisableIRQ();         // release Tech-80 resources
  
  printf("\n\n");
  
  exit(0);
  
  }  



void sleep(clock_t wait)   //timing loop based on system clock ticks
  {                        //minimum resolution = 55 msec, or one pc clock tick
  clock_t goal;            //works poorly
  goal = wait + clock();
  while(goal > clock());
  }
    


void prep_data(void)
  {
  // Prepare pcm data to be sent up the telemetry link in biphase-m (76.8 KHz)
  // The telemetry frame is 64 words x 12 bits.
  //
  // All ln200 and a/d data was first collected and stored in array data[]
  // as 48 16-bit words.  For pcm transmission, the data must be:
  // 1) scaled, converted to offset-binary and placed in proper channel slots
  // 2) shifted down from 16-bit to 12-bit (shift and mask) (keep high byte 0000 hex
  // 3) converted from from lsb first to msb first (with lookup[] table) for the bit synchronizer
  // 4) 1's complemented (~) (also for the bit synchronizer)
  // 5) repacked for the escc from 64 12-bit words to 96 8-bit bytes (shift and mask) (for the escc) 
  
  // so, first scale the numbers, add the offset, place in slots
  data2[0] = 0x0faf;         // decom sync 1
  data2[1] = 0x0321;         // decom sync 2
  data2[2] = 0x6;            // 3 << 1 for CB2000 ID
  data2[3] = (data[64] >> 12) & 0x000f;   //  4 msb of prop position
  data2[4] = data[64] & 0x0fff;           // 12 lsb of prop position
  data2[5] = 0x0000;
  data2[6] = 0x0000;
   
  data2[7] = (frame >> 4) & 0x0ffe; // most significant 11 bits of 16 bit frame counter
  data2[8] = (frame << 7) & 0x0f80; // least sig 5 bits of 16 bit frame counter, LEFT ADJUSTED
  
  const int d_offset = 0x0800;
  
  data2[10] = (data[5] * 2)    + d_offset;       // z axis angular rate
  data2[9]  = (data2[10] >> 12) * 2 + d_offset;  
  
  data2[12] = (data[3] * 2)    + d_offset;       // y axis angular rate
  data2[11] = (data2[12] >> 12) * 2 + d_offset;
    
  data2[14] = (data[4] * 2)    + d_offset;       // x axis angular rate
  data2[13] = (data2[14] >> 12) * 2 + d_offset;
    
  data2[16] = (data[2] * 2)    + d_offset;       // x axis linear accel
  data2[15] = (data2[16] >> 12) * 2 + d_offset;
      
  data2[18] = (data[1] * 2)    + d_offset;       // y axis linear accel
  data2[17] = (data2[18] >> 12) * 2 + d_offset;

    // compensate LN200 Z accel reading for standard gravity: (32.17405 / 2^-6 counts )
  data2[20] = ((data[0] + (32.17405 * 64)) * 2)    + d_offset;       // z axis linear accel
  data2[19] = (data2[20] >> 12) * 2 + d_offset;
  
  data2[22] = (data[32] * 2)    + d_offset;       // depth 1
  data2[21] = (data2[22] >> 12) * 2 + d_offset;
  
  data2[26] = (data[33] * 2)    + d_offset;       // depth 2
  data2[25] = (data2[26] >> 12) * 2 + d_offset;
  
     
  const int s_offset = 0x0800;
    
  //data2[25] = (data[13] >> 4) & 0x0ff0;              // ln200 status word high byte
  //data2[26] = (data[13] << 4) & 0x0ff0;              // ln200 status word low byte
    // moved ln200 status from ch 26/27 to 24/25 on 9/10/99 jtm
  data2[23] = (data[13] >> 4) & 0x0ff0;              // ln200 status word high byte
  data2[24] = (data[13] << 4) & 0x0ff0;              // ln200 status word low byte
  
  
 //  ln200 single channel scale factors
 //  converts 16-bit native data to 12-bit pcm data
 //  scaled to provide desired full-scale range for each channel
 //
 //  scale factor = ln200 lsb value * 2^12 counts / desired full-scale range EUs
 
    // roll_accel (Pdot), lsb = 2^-4 deg/sec^2/count, range = +-70 deg/sec^2  
  data2[27] = int(data[7]  * 1.828571429) + s_offset;
  
    // pitch_accel (Qdot), lsb = 2^-6 deg/sec^2/count, range = +-70 deg/sec^2    
  data2[28] = int(data[6]  * 0.457142857) + s_offset; // pitch_FAST_ACCEL
  
    // yaw_accel (Rdot), lsb = 2^-7 deg/sec^2/count, range = +-70 deg/sec^2    
  data2[29] = int(data[8]  * 0.228571429) + s_offset; // yaw_FAST_ACCEL
  
    // scale 16-bit data to 12-bit by dividing by 2^4
  data2[30] = int(data[36] * 0.0625)      + s_offset; // prop_cmd
  data2[31] = int(data[39] * 0.0625)      + s_offset; // fwd_cmd
  data2[32] = int(data[37] * 0.0625)      + s_offset; // strn_cmd
  data2[33] = int(data[38] * 0.0625)      + s_offset; // rudd_cmd
  data2[34] = int(data[42] * 0.0625)      + s_offset; // failsafe
  data2[35] = int(data[40] * 0.0625)      + s_offset; // cage/mode
  data2[36] = int(data[16] * 0.0625)      + s_offset; // e_bat
  
    // yaw180, lsb = (2^-15) * 180 deg/count, range = +-180 deg    
  data2[37] = int(data[11] * 0.0625)      + s_offset; // yaw180
  
    // yaw_exp, lsb = (2^-15) * 180 deg/count, range = +-45 deg      
  data2[38] = int(data[11] * 0.25)        + s_offset; // yaw_exp
  
    // pitch_exp, lsb = (2^-15) * 180 deg/count, range = +-22.5 deg      
  data2[39] = int(data[9]  * 0.5)         + s_offset; // pitch_exp
  
    // roll, lsb = (2^-15) * 180 deg/count, range = +-90 deg      
  data2[40] = int(data[10] * 0.125)       + s_offset; // roll

    // roll_rate_exp, lsb = 2^-6 deg/sec/count, range = +-14 deg/sec  
  data2[41] = int(data[4]  * 2.285714286) + s_offset;
  
    // pitch_rate_exp, lsb = 2^-8 deg/sec/count, range = +-14 deg/sec    
  data2[42] = int(data[3]  * 0.571428571) + s_offset; // pitch_rate_exp
  
    // yaw_rate_exp, lsb = 2^-9 deg/sec/count, range = +-14 deg/sec  
  data2[43] = int(data[5]  * 0.285714286) + s_offset; // yaw_rate_exp
  
    // x_accel_exp, lsb = (2^-6) ft/sec^2/count, range = +-17 ft/sec^2      
  data2[44] = int(data[2]  * 1.882352941) + s_offset; // x_accel_exp
  
    // y_accel_exp, lsb = (2^-6) ft/sec^2/count, range = +-17 ft/sec^2        
  data2[45] = int(data[1]  * 1.882352941) + s_offset; // y_accel_exp
  
    // z_accel_exp, lsb = (2^-6) ft/sec^2/count, range = +-17 ft/sec^2
    // NOTE:  LN200 reading is compensated for standard gravity: (32.17405 / 2^-6 counts)
  data2[46] = int((data[0] + (32.17405 * 64)) * 1.882352941) + s_offset; // z_accel_exp
  
    // Pdot_exp, lsb = 2^-4 deg/sec^2, range = +-17 deg/sec^2
  data2[47] = int(data[7]  * 7.529411765) + s_offset; // Pdot_exp

  status_word = 0;
                          
  data2[48] = status_word | !in_hw_failsafe << 11
                          | (!(taking_data | taking_zeros) <<8)
                          | 0x680
                          | taking_data << 3
                          | taking_zeros << 2
                          | mode << 1
                          | direction;
                          

    // Qdot_exp, lsb = (2^-6) ft/sec^2/count, range = +-17 ft/sec^2                                
  data2[49] = int(data[6] * 1.882352941) + s_offset; // Qdot_exp
  
    // Rdot_exp, lsb = (2^-7) ft/sec^2/count, range = +-17 ft/sec^2        
  data2[50] = int(data[8] * 0.941176471) + s_offset; // Rdot_exp
  
  
  data2[51] = in_fm_failsafe | (in_hw_failsafe << 4); //0;                                     // unused
  
  data2[52] = ((int)(axis3_position - (axis3_index - axis3_bias))) + s_offset; // fwd_encoder
  //data2[52] = ((int)(16 * (axis3_position - (axis3_index - axis3_bias)))) + s_offset; // fwd_encoder
  
  data2[53] = ((int)((axis2_position - (axis2_index - axis2_bias)))) + s_offset; // rudd_encoder
  //data2[53] = ((int)(16 * (axis2_position - (axis2_index - axis2_bias)))) + s_offset; // rudd_encoder
  
  data2[54] = ((int)(axis1_position - (axis1_index - axis1_bias))) + s_offset; // strn_encoder
  //data2[54] = ((int)(16 * (axis1_position - (axis1_index - axis1_bias)))) + s_offset; // strn_encoder
  
  data2[55] = ((int)(axis4_position - (axis4_index - axis4_bias))) + s_offset; // spare_encoder
  //data2[55] = ((int)(16 * (axis4_position - (axis4_index - axis4_bias)))) + s_offset; // spare_encoder
  
  data2[56] = int(data[19] * 0.023809524)+ s_offset; // prop_volt
  
  data2[57] = int(data[18] * 0.024875622)+ s_offset; // prop_current

    // pitch, lsb = (2^-15) * 180 deg/count, range = +-45 deg        
  data2[58] = int(data[9]  * 0.25)       + s_offset; // pitch
  
  data2[59] = (4096 - 500) - data[34]/8;               // paddle_wheel
  
  data2[60] = (4096 - 500) - int(data[35] * 0.018454724)*5; // rpm
   //use next line when ready to re-calibrate rpm
   //data2[60] = (4096 - 250) - int(data[35] * 0.125); // rpm

  data2[61] = int(data[32] * 0.074482759) + s_offset; // depth1
  
  data2[62] = int(data[33] * 0.074482759) + s_offset; // depth2
    
    // roll_rate, lsb = 2^-6, range = +-55 deg/sec
  data2[63] = int(data[4]  * 0.581818182) + s_offset; // roll_rate
  
  data2[64] = data[64];  // prop position 
//  data2[64] = data[64] / 4;  // prop position, adjust channel gain 4/27/00 jtm
  


    // now left adjust the data that will be written to onboard data file
  for(int i = 0; i < 64; i++)
    {
    data3[i] = data2[i] << 4;
    }
    
  data3[64] = data[64];  // jtm 03/15/00 add 16-bit prop position   
  data3[69] = frame;  // jtm 03/02/00 16-bit frame counter added for daq sync
  data3[70] = axis6_index; // % 10160;   // jtm 05/18/00 store prop index position; 
  data3[71] = axis6_index; // % 10160;   // jtm 05/18/00 store prop index position;
  
  //The scaled and offset data is now stored in array pcm[] in 8 bit bytes.
  //The following for-loop converts 2 16-bit data2[] words into 3 8-bit pcm[] bytes
  //(shift and mask), reverses the word msb for lsb (lookup[]), and inverts each bit(~).
  for(int j=0;j<32;j++)
    {
    pcm[3*j]   = ~lookup[(data2[2*j] >> 4) & 0x00ff];
    pcm[3*j+1] = ~lookup[((data2[2*j]<<4) & 0x00f0) | ((data2[2*j+1]>>8)&0x000f)];
    pcm[3*j+2] = ~lookup[(data2[2*j+1] & 0x00ff)];
    }
        
  }  // end prep_data()
  
  
  
void reset_ln200(void)      // LN200 Power On Reset
  // note negative logic level on the LBC586 DIO ports
  // 0 volts = logic high
  {
  cout << "Resetting LN200." << endl; 
  uio48.set_bit(POR);  // pull POR low
  sleep(2000);         // wait 2 seconds
  uio48.clr_bit(POR);  // return POR high
  sleep(1000);         // wait 1 second
  cout << "LN200 reset." << endl << endl;
  }  //  end reset_ln200()
  
  
  
void zero_buffers(void)  // clear garbage, just in case
  {
  int i;    
  for(i=0;i<64;i++) rbuffer[i] = tbuffer1[i] = 0;
  for(i=0;i<65;i++) data[i]  = 0;
  for(i=0;i<144;i++) data2[i] = 0;
  for(i=0;i<96;i++) pcm[i]   = 0;
  for(i=0;i<100;i++) rudder_history[i] = 0;
  for(i=0;i<80;i++)
   {
   data_name[i] = '0';
   zero_name[i] = '0';
   }
  }  // end zero_buffers()  
  
  

void heartbeat(void) // pulse the heartbeat signal to the failsafe system
  {
  uio48.set_bit(HEARTBEAT);
  uio48.clr_bit(HEARTBEAT);
  }   // end heartbeat()
  
  
  
void prep_ln200_command(int heading_valid)  // build LN200 query frame
  {
  int i, j;
  
//  if(mode == 1 && !data_complete && zeros_complete) // fast levelling in idle loop
//    {
//    head[0] = data_name[8];
//    }
  
  if((direction = ( run_number % 2))==0)     // if impending run # is even
    {                                        // use the 0th bearing from disk
    heading = (int) (atof(bearing0) * 182.044);
    }
  else                                       // if impending run # is odd
    {                                        // use the 1st bearing from disk
    heading = (int) (atof(bearing1) * 182.044);
    }
  
  latitude = (int) (LATITUDE * 182.044); // LN200 operating latitude,+-degrees, see ln200 ICD
  x_axis = (int) (X_AXIS * 182.044);     // LN200 boresight rotations,+-degrees
  y_axis = (int) (Y_AXIS * 182.044);
  z_axis = (int) (Z_AXIS * 182.044);
  //mode = 0;      //LN200 modes are 0 (free inertial) and 1 (fast leveling)
  //mode = 1;
  latitude_valid  = YES;               // latitude valid?
  //heading_valid = YES;               // heading valid?
  boresights_valid = YES;              // boresight rotations valid?
  bit = NO;                            // run Built In Test?

  icmd[0]=0x0000;                      // assemble the command word icmd[]
  icmd[0]=icmd[0]|mode;
  icmd[0]=icmd[0]|(boresights_valid<<3);
  icmd[0]=icmd[0]|(bit<<4);
  icmd[0]=icmd[0]|(latitude_valid<<8);
  icmd[0]=icmd[0]|(heading_valid<<9);
  icmd[1]=latitude;
  icmd[2]=heading;
  icmd[3]=x_axis;
  icmd[4]=y_axis;
  icmd[5]=z_axis;
  icmd[6]=0x0;                      // calculate icmd[] checksum
  for (i=0;i<6;i++)                 // sum icmd[0] to icmd[5]
    icmd[6]=icmd[6]+icmd[i];
  icmd[6]=~icmd[6];                 // then take bit-wise 1's complement
    
  //convert the word-oriented icmd[] into the byte-oriented tbuffer1[]
  for(j=0;j<7;j++)
    {
    tbuffer1[2*j] = icmd[j]&0x00ff;          //mask off high byte and load low byte first
    tbuffer1[(2*j)+1] = (icmd[j]>>8)&0x00ff; //then shift high byte down, mask any sign bits,
    }                                        //and load high byte
  
  } // end prep_ln200_command()
  
  
  
void get_fm_commands(void)  // process fm commands from the a/d converter
  {
  float rudder_adc     = 0.0;
  float stern_adc      = 0.0;
  float fairwater_adc  = 0.0;
  float ballast_adc    = 0.0;
  float mode_adc       = 0.0;
  float failsafe_adc   = 0.0;
  float spare_adc      = 0.0;
  const float full_scale_counts = 65535.0; // dmm32 full scale counts = 2^16 - 1
  const float input_range       = 10.0;    // dmm32 input range = +-10.0 volts
  const float scale_factor      = 20.0;    // 20.0 degrees/volt
  const float tau               = 0.1;     // filter time constant
  const float time_step = 0.01;            // filter time step 
  const float limit = 10160 / 360 * 0.5;   // maximum delta for plane command in one data frame
                                           // for 0.5 degree
/*
  float rudder_set[20];
  char cc;//used for rudder calibration
  rudder_set[1]=0.0;
  rudder_set[2]=10.0;
  rudder_set[3]=20.0;
  rudder_set[4]=30.0;
  rudder_set[5]=40.0;
  rudder_set[6]=30.0;
  rudder_set[7]=20.0;
  rudder_set[8]=10.0;
  rudder_set[9]=0.0 ;
  rudder_set[10]=-10.0;
  rudder_set[11]=-20.0;
  rudder_set[12]=-30.0;
  rudder_set[13]=-40.0;
  rudder_set[14]=-30.0;
  rudder_set[15]=-20.0;
  rudder_set[16]=-10.0;
  rudder_set[17]=0.0;
  rudder_set[18]=0.0;
  rudder_set[19]=0.0;
  rudder_set[20]=0.0;
  */
  
  // failsafe is on fm ch7, aim2 ch11, data[42]
  failsafe_adc = data[42] / full_scale_counts * input_range;
  if(failsafe_adc >= (3.450) && failsafe_adc <= (3.850))  // 3.560+-200mV grey console
    {     // fm failsafe signal is within range           // 2.960+-200mV desired
    in_fm_failsafe = 0;
    }
  else    // fm failsafe signal is out of range
    {
    in_fm_failsafe = 1;
    }
  in_hw_failsafe = uio48.read_bit(IN_HW_FAILSAFE);  // check if hardware failsafe is asserted

  // rudder is on fm ch3, aim2 ch7, data[38], 5950b axis2
  rudder_adc = data[38] / full_scale_counts * input_range * scale_factor; //degrees

/*
  while(kbhit()) // used for rudder calibration
  { 
    set_counter = set_counter + 1;
    cout << "Rudder set[" << set_counter << "] to:  " << rudder_set[set_counter] << endl;
    cc=_getch();
   }    
    rudder_adc=rudder_set[set_counter];
*/
   
  //cout << rudder_adc << '\r';
  
  long rudder_command;
  te5650SetAxis(BOARD1, AXIS2);

/*  longcheckindex=0.;// used to verify no glitches on rudder readout
  if(te5650IsCapture()==1)//if next index pulse
    {
    te5650CapturePos(&checkindex);
    te5650ResetCapture();
    cout<<"New Index:"<<checkindex<<endl;
    }   */

  te5650ActPos(&axis2_position);
  
  if(axis2_index_done && !in_fm_failsafe && actuator_index_done)   //jtm 2/29/00
    {
    if(fabs(rudder_adc) > 39.0)
      {
      rudder_adc = sign(rudder_adc) * 39.0;
      }
      // get current rudder command
    rudder_in = (rudder_adc / 360.0 * 10160.0) + (axis2_index - axis2_bias);
    
      // limit rate to 0.5 degrees per data frame
    if(abs(rudder_in - rudder_in_old) > limit)   // 14.111 counts per 0.5 degree  
      {
      rudder_in = rudder_in_old + ( sign(rudder_in - rudder_in_old) * limit);
      }
    
      // digital filter
    rudder_out = (time_step*(rudder_in + rudder_in_old) / (2*tau+time_step))  - rudder_out_old *((time_step - 2*tau)/(2*tau + time_step));
    rudder_out_old = rudder_out;
    rudder_in_old = rudder_in;
    rudder_command = (long) rudder_out;
    
    te5650ProfileTrap(rudder_command, FAST_VELOCITY, FAST_ACCEL); 
    te5650Update();
    }
  
/*  if(axis2_index_done && in_fm_failsafe)
    {
    rudder_command = axis2_index + axis2_bias;
    te5650ProfileTrap(rudder_command, FAST_VELOCITY, FAST_ACCEL); 
    te5650Update();
    }
*/      
  


  // stern planes is on fm ch2, aim2 ch6, data[37], 5950b axis1
  stern_adc = (data[37] / full_scale_counts * input_range * scale_factor);  // + 24.0
  //stern_adc=0.0;//debug line to check for movement
  //cout << stern_adc << '\r';
  long stern_command; // = (long) ((stern_adc / 360.0 * 10160.0) + (axis1_index + axis1_bias));
  
  te5650SetAxis(BOARD1, AXIS1);

// added -3.1 offset
  te5650ActPos(&axis1_position);
  if(axis1_index_done && !in_fm_failsafe)
  //if(axis1_index_done)  
    {
    if(fabs(stern_adc) > 28.0)
      {
      stern_adc = sign(stern_adc) * 28.0; // -3.1
      }
            
    stern_in = (stern_adc / 360.0 * 10160.0) + (axis1_index - axis1_bias);
    
    
      // stern plane maximum rate limiter
    if(abs(stern_in - stern_in_old) > limit)   // 14.111 counts per 0.5 degree  
      {
      stern_in = stern_in_old + sign(stern_in - stern_in_old) * limit;
      }
        
    stern_out = (time_step*(stern_in + stern_in_old) / (2*tau+time_step))  - stern_out_old *((time_step - 2*tau)/(2*tau + time_step));
    stern_out_old = stern_out;
    stern_in_old = stern_in;
    stern_command = (long) stern_out;

    te5650ProfileTrap(stern_command, FAST_VELOCITY, FAST_ACCEL); 
    te5650Update();
    }
  else
    {
    if(in_fm_failsafe)
      {
      //cout << "In failsafe." << '\r';
      }
    }
  
  // fairwater planes is on fm ch4, aim2 ch8, data[39], 5950b axis3
  fairwater_adc = data[39] / full_scale_counts * input_range * scale_factor;
  //cout << fairwater_adc << '\r';
  //long fairwater_command = (long) (fairwater_adc / 360.0 * 10160.0) + (axis3_index + axis3_bias);

  fore_in = (fairwater_adc / 360.0 * 10160.0) + (axis3_index - axis3_bias);
  
      // fore plane rate limiter
  if(abs(fore_in - fore_in_old) > limit)   // 14.111 counts per 0.5 degree  
    {
    fore_in = fore_in_old + sign(fore_in - fore_in_old) * limit;
    }
    
  fore_out = (time_step*(fore_in + fore_in_old) / (2*tau+time_step))  - fore_out_old *((time_step - 2*tau)/(2*tau + time_step));
  fore_out_old = fore_out;
  fore_in_old = fore_in;
  
  long fairwater_command = (long) fore_out;
    
  te5650SetAxis(BOARD1, AXIS3);

  te5650ActPos(&axis3_position);
  //cout << " Index3 done: " << axis3_index_done << "   FS: " << in_fm_failsafe << "   FW comm: " << fairwater_command << '\r';
  //buffer = "Actual position";
  //_settextposition(24,1);
  //_outtext("Actual position.");//cout << "*****IsComplete" << endl;
  if(axis3_index_done && !in_fm_failsafe) 
    {
    te5650ProfileTrap(fairwater_command, FAST_VELOCITY, FAST_ACCEL); 
    te5650Update();
    }

/*
  // spare is on fm ch?, aim2 ch12, data[43]
  spare_adc = data[43] / full_scale_counts * input_range * scale_factor;
  //cout << fairwater_adc << '\r';
  long spare_command = (long) (spare_adc / 360.0 * 10160.0) + (axis4_index - axis4_bias);
  spare_command = 0;
  te5650SetAxis(BOARD1, AXIS4);
  te5650ActPos(&axis4_position);
  //cout << " Index3 done: " << axis3_index_done << "   FS: " << in_fm_failsafe << "   FW comm: " << fairwater_command << '\r';
  //buffer = "Actual position";
  //_settextposition(24,1);
  //_outtext("Actual position.");//cout << "*****IsComplete" << endl;
  if(axis4_index_done && !in_fm_failsafe) 
    {
    te5650ProfileTrap(spare_command, FAST_VELOCITY, FAST_ACCEL); 
    te5650Update();
    }
  
*/

  // ballast is on fm ch6, aim2 ch10, data[41]
  ballast_adc = data[41] / full_scale_counts * input_range;
  //cout << ballast_adc << '\r';
  
  if(ballast_adc >= 3.475 && !in_fm_failsafe)                           //flood
    {
//    aim1.outpword( aim1.DIOR, (pump + valve2 + valve3 + valve6) );
    uio48.port_write(BALLAST, pump + valve2 + valve3 + valve6);
    }
  
  if(ballast_adc >= 0.976 && ballast_adc < 3.475 && !in_fm_failsafe)     //trim_aft
    {
//    aim1.outpword( aim1.DIOR, (pump + valve3 + valve4) );    
    uio48.port_write(BALLAST, pump + valve3 + valve4);
    }
    
  if(ballast_adc >= -1.675 && ballast_adc < 0.976  && !in_fm_failsafe)    //off
    {
//    aim1.outpword( aim1.DIOR, 0 );    
    uio48.port_write(BALLAST,0);
    }
    
  if(ballast_adc >= -4.175 && ballast_adc < -1.675  && !in_fm_failsafe)   //trim_forward
    {
//    aim1.outpword( aim1.DIOR, (pump + valve2 + valve5) );    
    uio48.port_write(BALLAST, pump + valve2 + valve5);
    }
    
  if(ballast_adc < -4.175  && !in_fm_failsafe)                            //blow
    {
//    aim1.outpword( aim1.DIOR, (pump + valve1 + valve4 + valve5) );    
    uio48.port_write(BALLAST, pump + valve1 + valve4 + valve5);
    }
  
  
  // mode is on fm ch 5, aim2 ch9, data[40]
  mode_adc = data[40] / full_scale_counts * input_range;
  
  mode_in = mode_adc;
  mode_out = (time_step*(mode_in + mode_in_old) / (2*tau+time_step))  - mode_out_old *((time_step - 2*tau)/(2*tau + time_step));
  mode_out_old = mode_out;
  mode_in_old = mode_in;
  
  int mode_command = mode_out;
    
    // take data
  if((mode_command >= 4.00) && zeros_complete
                            && !data_complete  
                            && actuator_index_done
                            && !in_fm_failsafe)
    {
    mode = 0;  // ln200 free inertia 
    taking_zeros = 0;
    taking_data = 1;
    cage_complete = 0;
    }
                          
    
    // take zeros                          
  if((mode_command >= -2.50) && mode_adc < 2.50 
                             && !zeros_complete 
                             && data_complete
                             && cage_complete
                             && actuator_index_done
                             && !in_fm_failsafe)
    {
    mode = 0;  // ln200 free inertia
    taking_zeros = 1;
    taking_data = 0;
    cage_complete = 0;
    }
                          
    
    // cage gyro                      
  if((mode_command < -4.00) || !actuator_index_done && !in_fm_failsafe)
    {
    mode = 1;  // ln200 fast leveling 
    taking_zeros = 0;
    taking_data = 0;
    cage_complete = 1;
    }
  }  // end get_fm_commands
  
  

void perform_common_tasks(void)
  {
  int i = 0;
  int j = 0;
  int k = 0;
  

  
  // transmit the pcm data on channelb
  j = escc1.tx_port(channelb, pcm, 96);

//  heartbeat();


  
  get_fm_commands();
  
  //get_prop_position();    // jtm 05/22/00 moved to cescc isr


    
  // build the ln200 command word and transmit
  prep_ln200_command(HEADING_VALID);
  j = escc1.tx_port(channela, tbuffer1, 14);


  
  // send sync pulse to the DAQ system
  // uio48.set_bit(SYNC);  // sync moved to cescc isr 11/24/99 jtm
  // uio48.clr_bit(SYNC);
      
  // trigger the two AIM16 A/D converters
  //aim1.trigger();
  //aim2.trigger();
        
  // wait to receive ln200 response
  i = 0;    
  while((j = escc1.rx_port(channela, rbuffer, 64)) != 33)
    {
    if(++i>100000)  // time-out loop generates soft crash if no response
      {             // from LN200
      cout << "No response from LN200." << endl;
      terminate_program();
      }
    }
  k = j;  // k is the number of bytes received in the rbuffer

  //cout << "AAA" << flush;
         
  // convert k 8-bit bytes received from ln200 through escc into 16-bit data 
  for(j = 0; j<(k-1)/2; j++)
    {
    data[j] = rbuffer[2*j+1];              // get high byte first
    data[j] = (data[j]<<8) | rbuffer[2*j]; // shift high byte up, insert low byte
    }
      
  // wait for the DMM32 to complete conversions
  
  while(!dmm32.data_ready());
  // read 32 channels of DMM32 data
  for(i=0; i<32; i++)
    data[i+16] = dmm32.read_data();
  
  // pack the pcm data stream
  prep_data();   // this data will be transmitted to console
                 // at the beginning of the next data frame
  
  // Change for CB2000.               
  // A/D data has been stored for transmission up the pcm data stream
  // so go ahead and start the next round of conversions here and let
  // the DMM32 work in the background.
  dmm32.trigger();
  
  }  // end of perform_common_tasks()
  


  // report prop position relative to the encoder index pulse
void get_prop_position(void)
  {
  te5650SetAxis(BOARD2, AXIS2);

  if(te5650IsCapture)   // if an index pulse occured since the last frame
    {
    te5650CapturePos(&axis6_index); // get the position of the index pulse
    }
/*  long checkindex=0;//check for rpm readout glitches
  if(te5650IsCapture()==1)//if next index pulse
    {
    te5650CapturePos(&checkindex);
    te5650ResetCapture();
      cout<<"New Index:"<<(checkindex-10160*int(checkindex/10160))<<endl;
    }   
*/        
  te5650ActPos(&axis6_position);    // get the current encoder position
//  data[64] = axis6_position - axis6_index;  // report the relative prop position 
  //data[64] = axis6_position - axis6_index + 564;  // 4/27/00 bias model Z axis 
  data[64] = axis6_position - axis6_index + 5620 + 254;  // 5/10/00 bias model Z axis
  
  if(data[64] > 10159)     // 4/29/00 adust for overrange condition
    {                      // caused by 4/27/00 bias
    data[64] = data[64] - 10160;
    }
    
  if(data[64] < 0)                  // convert any negative counts to positive
    {
    data[64] = data[64] + 10160;
    }
    
  }
  
  
void init_tech80(void)
  {
    // initialize Tech80 software drivers
  cout << "Tech80 Driver Version " << hex << te5650InitSw() << dec << endl;

  // init first tech80 5950b motion controller  
  if(te5650InitServo(0x1300, TE5650TYPE_DAC16)!=0)  // 0x1300
    {
    cout << "Failed InitServo 1." << endl;
    _fcloseall();
    terminate_program();
    }
  else
    {
    cout << "Board 1 InitServo Done." << endl;
    }

  // init second tech80 5950b motion controller  
  if(te5650InitServo(0x2300, TE5650TYPE_DAC16)!=0)  // 0x2300
    {
    cout << "Failed InitServo 2." << endl;
    _fcloseall();
    terminate_program();
    }
  else
    {
    cout << "Board 2 InitServo Done." << endl;
    }

    // *********** Axis 1 ***************  
  if(te5650SetAxis(BOARD1, AXIS1)!=0)
    {
    cout << "Failed SetAxis1." << endl;
    _fcloseall();
    terminate_program();
    }
  else
    {
    cout << "SetAxis 1 OK.   ";
    }
  te5650Update();
  if(te5650SetSampleTime(SAMPLE_TIME)==0)
    {
    cout << "Axis1 sample time set to " << SAMPLE_TIME << endl;
    }
  else
    {
    cout<< "Axis1 sample time not set." << endl;
    _fcloseall();    
    terminate_program();
    }
  
//  unsigned short stime;
//  te5650GetSampleTime(&stime);
//  cout << "Sample time: " << stime << endl;
    
  if(te5650FilterPID(KP, KI, KD, IL)!=0)
    {
    cout << "Failed FilterPID 1." << endl;
    _fcloseall();    
    terminate_program();
    }
  else
    {
    cout << "FilterPID 1 OK.   ";
    }

  if(te5650Update()!=0)
    {
    cout << "Axis 1 Update Failed." << endl;
    _fcloseall();    
    terminate_program();
    }
  else
    {
    cout << "Axis 1 Update OK." << endl;
    }
   
    // all encoder inputs must be inverted when using DRC M21AAS0BB23-2540 encoder
  te5650PhasesPolarity(INVERT, INVERT, INVERT); 
  
    // 
  te5650SetPosErr(MAX_ERROR, AUTOSTOP_OFF);
  
    // make the index pulse the active capture trigger, not the home switch
  te5650CaptureIndex();
    
    // write the configuration to the 5950b motion controller
  te5650Update();
    
  
  //************* Axis 2 ****************
  if(te5650SetAxis(BOARD1,AXIS2)!=0)
    {
    cout << "Failed SetAxis 2." << endl;
    _fcloseall();    
    terminate_program();
    }
  else
    {
    cout << "SetAxis 2 OK.   ";
    }
    

  te5650Update();
  
  if(te5650SetSampleTime(SAMPLE_TIME)==0)
    {
    cout << "Axis2 sample time set to " << SAMPLE_TIME << endl;
    }
  else
    {
    cout<< "Axis2 sample time not set." << endl;
    _fcloseall();    
    terminate_program();
    }
    
    
//  if(te5650FilterPID(90, 8, 10000, 435)!=0)  // KP KI KD IL
  if(te5650FilterPID(KP+200, KI, KD, IL)!=0)  // jtm 3/13/00 reset KP, was KP-100
    {
    cout << "Failed FilterPID 2." << endl;
    _fcloseall();    
    terminate_program();
    }
  else
    {
    cout << "FilterPID 2 OK.   ";
    }
    
  if(te5650Update()!=0)
    {
    cout << "Axis 2 Update Failed." << endl;
    terminate_program();
    }
  else
    {
    cout << "Axis 2 Update OK." << endl;
    }
   
    // all encoder inputs must be inverted when using DRC M21AAS0BB23-2540 encoder
  te5650PhasesPolarity(INVERT, INVERT, INVERT); 
  
    //
  te5650SetPosErr(MAX_ERROR, AUTOSTOP_OFF);
  
    // make the index pulse the active capture trigger, not the home switch
  te5650CaptureIndex();
  
    // write the configuration to the 5950b motion controller
  te5650Update();
      


  //************* Axis 3 **************
  if(te5650SetAxis(BOARD1, AXIS3)!=0)
    {
    cout << "Failed SetAxis 3." << endl;
    terminate_program();
    }
  else
    {
    cout << "SetAxis 3 OK.   ";
    }
    
    
  te5650Update();
  
  if(te5650SetSampleTime(SAMPLE_TIME)==0)
    {
    cout << "Axis3 sample time set to " << SAMPLE_TIME << endl;
    }
  else
    {
    cout<< "Axis3 sample time not set." << endl;
    terminate_program();
    }
    
    
  if(te5650FilterPID(KP, KI, KD, IL)!=0)
    {
    cout << "Failed FilterPID 3." << endl;
    terminate_program();
    }
  else
    {
    cout << "FilterPID 3 OK.   ";
    }
    
  if(te5650Update()!=0)
    {
    cout << "Axis 3 Update Failed." << endl;
    terminate_program();
    }
  else
    {
    cout << "Axis 3 Update OK." << endl;
    }
   
    // all encoder inputs must be inverted when using DRC M21AAS0BB23-2540 encoder
  te5650PhasesPolarity(INVERT, INVERT, INVERT); 
  
    //
  te5650SetPosErr(MAX_ERROR, AUTOSTOP_OFF);
  
    // make the index pulse the active capture trigger, not the home switch
  te5650CaptureIndex();
  
    // write the configuration to the 5950b motion controller
  te5650Update();
      


/*
  // ************* Axis 4 **************
  if(te5650SetAxis(BOARD1, AXIS4)!=0)
    {
    cout << "Failed SetAxis 4." << endl;
    terminate_program();
    }
  else
    {
    cout << "SetAxis 4 OK.   ";
    }
    
    
  te5650Update();
  
  if(te5650SetSampleTime(SAMPLE_TIME)==0)
    {
    cout << "Axis4 sample time set to " << SAMPLE_TIME << endl;
    }
  else
    {
    cout<< "Axis4 sample time not set." << endl;
    terminate_program();
    }
    
    
  if(te5650FilterPID(KP, KI, KD, IL)!=0)
    {
    cout << "Failed FilterPID 4." << endl;
    terminate_program();
    }
  else
    {
    cout << "FilterPID 4 OK.   ";
    }
    
  if(te5650Update()!=0)
    {
    cout << "Axis 4 Update Failed." << endl;
    terminate_program();
    }
  else
    {
    cout << "Axis 4 Update OK." << endl;
    }
   
    // all encoder inputs must be inverted when using DRC M21AAS0BB23-2540 encoder
  te5650PhasesPolarity(INVERT, INVERT, INVERT); 
  
    // 
  te5650SetPosErr(MAX_ERROR, AUTOSTOP_OFF);
  
    // make the index pulse the active capture trigger, not the home switch
  te5650CaptureIndex();
  
    // write the configuration to the 5950b motion controller
  te5650Update();
    
  // end init_tech80()

*/


/*
  // ************* Axis 5 **************
  if(te5650SetAxis(BOARD2, AXIS1)!=0)
    {
    cout << "Failed SetAxis 5." << endl;
    terminate_program();
    }
  else
    {
    cout << "SetAxis 5 OK.   ";
    }
    
    
  te5650Update();
  
  if(te5650SetSampleTime(SAMPLE_TIME)==0)
    {
    cout << "Axis5 sample time set to " << SAMPLE_TIME << endl;
    }
  else
    {
    cout<< "Axis5 sample time not set." << endl;
    terminate_program();
    }
    
    
  if(te5650FilterPID(KP, KI, KD, IL)!=0)
    {
    cout << "Failed FilterPID 5." << endl;
    terminate_program();
    }
  else
    {
    cout << "FilterPID 5 OK.   ";
    }
    
  if(te5650Update()!=0)
    {
    cout << "Axis 5 Update Failed." << endl;
    terminate_program();
    }
  else
    {
    cout << "Axis 5 Update OK." << endl;
    }
   
    // all encoder inputs must be inverted when using DRC M21AAS0BB23-2540 encoder
  te5650PhasesPolarity(INVERT, INVERT, INVERT); 
  
    // 
  te5650SetPosErr(MAX_ERROR, AUTOSTOP_OFF);
  
    // make the index pulse the active capture trigger, not the home switch
  te5650CaptureIndex();
  
    // write the configuration to the 5950b motion controller
  te5650Update();
*/    


  
  // ************* Axis 6 **************
  if(te5650SetAxis(BOARD2, AXIS2)!=0)
    {
    cout << "Failed SetAxis 6." << endl;
    terminate_program();
    }
  else
    {
    cout << "SetAxis 6 OK.   ";
    }
    
  te5650Update();
  
  if(te5650SetSampleTime(SAMPLE_TIME)==0)
    {
    cout << "Axis6 sample time set to " << SAMPLE_TIME << endl;
    }
  else
    {
    cout<< "Axis6 sample time not set." << endl;
    terminate_program();
    }
    

  if(te5650FilterPID(KP, KI, KD, IL)!=0)
    {
    cout << "Failed FilterPID 6." << endl;
    terminate_program();
    }
  else
    {
    cout << "FilterPID 6 OK.   ";
    }
    

  if(te5650Update()!=0)
    {
    cout << "Axis 6 Update Failed." << endl;
    terminate_program();
    }
  else
    {
    cout << "Axis 6 Update OK." << endl;
    }
   
    // all encoder inputs must be inverted when using DRC M21AAS0BB23-2540 encoder
  te5650PhasesPolarity(INVERT, INVERT, INVERT); 
  
    // 
  te5650SetPosErr(MAX_ERROR, AUTOSTOP_OFF);
  
    // make the index pulse the active capture trigger, not the home switch
  te5650CaptureIndex();
  
    // discard any previous capture event
  te5650ResetCapture();
  
    // write the configuration to the 5950b motion controller
  te5650Update();
  
  } // end init_tech80()
  
  
 

void index_actuators(void)
  {
//  char buffer[80];
  
  // if in failsafe: 1) do not begin to index the actuators
  //                 2) if indexing has already started, terminate it and get ready to start over
  if(in_fm_failsafe || in_hw_failsafe)
    {
    axis1_step_1     = 0;
    axis1_step_2     = 0;
    axis1_step_3     = 0;
    axis1_index      = 0;
    axis1_limit_a    = 0;
    axis1_limit_b    = 0;
    axis1_center     = 0;
    //axis1_bias       = 0;
    axis1_index_done = 0;
    //axis1_position   = 0;
    //axis1_command1_given = 0;
    //axis1_command2_given = 0;
    axis1_error      = 0;
    //stern_out       = 0.0;
    //stern_out_old   = 0.0;
    //stern_in_old    = 0.0;
    //stern_in        = 0.0;


    axis2_step_1     = 1;
    axis2_step_2     = 0;
    axis2_step_3     = 0;
    axis2_index      = 0;
    axis2_limit_a    = 0;
    axis2_limit_b    = 0;
    axis2_center     = 0;
    //axis2_bias       = 0;
    axis2_index_done = 0;
    //axis2_position   = 0;
    //axis2_command1_given = 0;
    //axis2_command2_given = 0;
    axis2_error      = 0;
    //rudder_history[100];
    //rudder_counter   = 0;
    //rudder_i         = 0;
    //rudder_out       = 0.0;
    //rudder_out_old   = 0.0;
    //rudder_in_old    = 0.0;
    //rudder_in        = 0.0;

    axis3_step_1     = 0;
    axis3_step_2     = 0;
    axis3_step_3     = 0;
    axis3_index      = 0;
    axis3_limit_a    = 0;
    axis3_limit_b    = 0;
    axis3_center     = 0;
    axis3_index_done = 0;
    //axis3_position   = 0;
    //axis3_error      = 0;
    //axis3_bias       = 0;

    axis4_step_1     = 0;
    axis4_step_2     = 0;
    axis4_step_3     = 0;
    axis4_index      = 0;
    axis4_limit_a    = 0;
    axis4_limit_b    = 0;
    //axis4_center     = 0;
    axis4_index_done = 0;
    //axis4_position   = 0;
    //axis4_error      = 0;
    //axis4_bias       = 0;
      
    axis5_step_1     = 0;
    axis5_step_2     = 0;
    axis5_step_3     = 0;
    axis5_index      = 0;
    axis5_limit_a    = 0;
    axis5_limit_b    = 0;
    //axis5_center     = 0;
    axis5_index_done = 0;
    //axis5_position   = 0;
    //axis5_error      = 0;
    //axis5_bias       = 0;

    axis6_index      = 0;
    axis6_position   = 0;
 
    actuator_index_done = 0;
    
//    goto leave_index;
    return;
      
    }
    
    

  
  // *****  axis 1 ** stern planes *****
  te5650SetAxis(BOARD1, AXIS2);  // jtm 2/29/00 check for completed rudder motion
                                 // before starting stern planes
  if(axis1_step_1 && !axis1_index_done && (te5650IsComplete() || proceed))  // jtm 2/29/00    // find first limit
    {    
    proceed = 1;
    te5650SetAxis(BOARD1, AXIS1);
    
    //cout << " First move" << '\r';
    //fprintf(errorfile, "Axis 1 first move.\n");
    
    te5650ProfileTrap(LIMIT_A, SLOW_VELOCITY, SLOW_ACCEL); // move toward first limit
    te5650Update();
    
    axis1_error = 0;
    te5650ActPosErr(&axis1_error);// check position error, error wil grow if actuator is against the limit
    if(abs(axis1_error) > SEARCH_ERROR)// first limit is reached, error is growing
      {
      axis1_error = 0;
      te5650ActPos(&axis1_limit_a);   // store first limit position
      te5650ProfileTrap(axis1_limit_a, SLOW_VELOCITY, SLOW_ACCEL);// stay at limit position, stop looking
      te5650Update();
      te5650ResetCapture(); // discard any previously captured index positions
      axis1_step_1 = 0;     // ready for next step
      axis1_step_2 = 1;
      }  // end if(error)
    } // end step_1
    
    
  if(axis1_step_2 && !axis1_index_done)
    {
    te5650SetAxis(BOARD1, AXIS1);
    if(te5650IsComplete())  // **** use in functional code
    //if(1) // *** use in development when axis1 actuator is not installed
      {
      axis1_step_2 = 0;
      axis1_step_3 = 1;
      }
    } // end axis1_step2 
    
    
  if(axis1_step_3 && !axis1_index_done)    // find second limit while capturing index pulse
    {
    te5650SetAxis(BOARD1, AXIS1);    
    te5650ProfileTrap(LIMIT_B, SLOW_VELOCITY, SLOW_ACCEL); // move toward opposite limit
    te5650Update();
        
    if(te5650IsCapture()==1)    // if an index pulse occurs
      {
      te5650CapturePos(&axis1_index); // store the index position
      }
    
    axis1_error = 0;        
    te5650ActPosErr(&axis1_error);    // check the position error
    if(abs(axis1_error) > SEARCH_ERROR)  // second limit is reached, position error is growing
      {
      axis1_error = 0;
      te5650ActPos(&axis1_limit_b);   // store second limit position
      te5650ProfileTrap((axis1_limit_b), SLOW_VELOCITY, SLOW_ACCEL);
      te5650Update();
      axis1_center = (axis1_limit_a + axis1_limit_b) / 2;
      
    
/*  // used to calculate axis bias and write to disk file
    if((axis1_bias_file = fopen("c:\\axis1.txt", "wt")) == NULL)
      {
      cout << "Failed to open file: axis1.txt." << endl;
      terminate_program();
      }
    else
      {
      cout << "Opening file: axis1.txt." << endl;
      }    
      
      fprintf(axis1_bias_file, "%s\n",  _ltoa((axis1_index - axis1_center), buffer, 10));
      
      fclose(axis1_bias_file);       
      
*/

/*      cout << endl << "Stern planes: " << endl;
      cout << "Limit A: " << axis1_limit_a << endl;
      cout << "Limit B: " << axis1_limit_b << endl;
      cout << "Center: " << axis1_center << endl;
      cout << "Index: " << axis1_index << endl;
*/      
      
      axis1_step_3 = 0;       
      
      axis1_index_done = 1; 
      
      //actuator_index_done = 1;   // if all available axes done
      
      axis3_step_1 = 1;            // ready to go to next axis
    
      }  // end if(error)   
    
    }  // end axis1 step_3;  
    
  
  
  //  *****  axis 2 ** rudder  *****
  if(axis2_step_1  && !axis2_index_done)    // find first limit
    {
    te5650SetAxis(BOARD1, AXIS2);
    
    //fprintf(errorfile, "Axis 2 first move.\n");    
    
    te5650ProfileTrap(LIMIT_A, SLOW_VELOCITY, SLOW_ACCEL); // move toward first limit
    te5650Update();
    
    axis2_error = 0;
    te5650ActPosErr(&axis2_error);    // check position error
    if(abs(axis2_error) > SEARCH_ERROR)        // first limit is reached, error is growing
      {
      te5650ActPos(&axis2_limit_a);   // store first limit position
      te5650ProfileTrap(axis2_limit_a, SLOW_VELOCITY, SLOW_ACCEL);// stay at limit position, stop looking
      te5650Update();
      te5650ResetCapture(); // discard any previously captured index positions
      axis2_step_1 = 0;     // ready for next step
      axis2_step_2 = 1;
      }  // end if(error)
    } // end step_1
    
    
  if(axis2_step_2  && !axis2_index_done)
    {
    te5650SetAxis(BOARD1, AXIS2);
    //if(1)
    if(te5650IsComplete())     // allow the 5950 to complete the current move
      {                        // before going to next step
      axis2_step_2 = 0;
      axis2_step_3 = 1;
      }
    }  
    
    
  if(axis2_step_3  && !axis2_index_done)           // find second limit while capturing index pulse
    {
    te5650SetAxis(BOARD1, AXIS2);
    te5650ProfileTrap(LIMIT_B, SLOW_VELOCITY, SLOW_ACCEL); // move toward opposite limit
    te5650Update();

    if(te5650IsCapture()==1)    // if an index pulse occurs
      {
      te5650CapturePos(&axis2_index); // store the index position
      }

    axis2_error = 0;
    te5650ActPosErr(&axis2_error);  // check the position error
    if(abs(axis2_error) > SEARCH_ERROR)  // second limit is reached, position error is growing
      {
      te5650ActPos(&axis2_limit_b);   // store second limit position
      axis2_center = (axis2_limit_a + axis2_limit_b) / 2;
      te5650ProfileTrap(axis2_center, FAST_VELOCITY, FAST_ACCEL);
      te5650Update();

/*      cout << endl << "Rudder: " << endl;
      cout << "Limit A: " << axis2_limit_a << endl;
      cout << "Limit B: " << axis2_limit_b << endl;
      cout << "Center: " << axis2_center << endl;
      cout << "Index: " << axis2_index << endl;
*/  
      
      
      
        axis2_step_3 = 0;             
        axis2_index_done = 1;
        axis1_step_1 = 1;       // go on to next axis      
            
      //actuator_index_done = 1;  // this is the last axis
                                // axis 3 not used in 7/12/99 configuration
      }  // end if(error)   
    
    }  // end axis2 step_3;  
   
  
  // *****  axis 3 ** forward planes  *****
  // ** axis 3 not used in 7/12/99 configuration **
  
  if(axis3_step_1  && !axis3_index_done)    // find first limit
    {
    te5650SetAxis(BOARD1, AXIS3);    
    
    //fprintf(errorfile, "Axis 3 first move.\n");
    
    te5650ProfileTrap(LIMIT_A, SLOW_VELOCITY, SLOW_ACCEL); // move toward first limit
    te5650Update();
    
    axis3_error = 0;
    te5650ActPosErr(&axis3_error);    // check position error
    if(abs(axis3_error) > SEARCH_ERROR)  // first limit is reached, error is growing
      {
      te5650ActPos(&axis3_limit_a);   // store first limit position
      te5650ProfileTrap(axis3_limit_a, SLOW_VELOCITY, SLOW_ACCEL);// stay at limit position, stop looking
      te5650Update();
      te5650ResetCapture(); // discard any previously captured index positions
      axis3_step_1 = 0;       // ready for next step
      axis3_step_2 = 1;
      }  // end if(error)
    } // end axis3_step_1
    
    
  if(axis3_step_2  && !axis3_index_done)
    {
    te5650SetAxis(BOARD1, AXIS3);
    if(te5650IsComplete()) 
    //if(1)
      {
      axis3_step_2 = 0;
      axis3_step_3 = 1;
      //buffer = "Actual position";
      //_settextposition(22,1);
      //_outtext("Actual position.");//cout << "*****IsComplete" << endl;
      }
    }  
    
    
  if(axis3_step_3  && !axis3_index_done)    // find second limit while capturing index pulse
    {
    te5650SetAxis(BOARD1, AXIS3);
    te5650ProfileTrap(LIMIT_B, SLOW_VELOCITY, SLOW_ACCEL); // move toward opposite limit
    te5650Update();
    
    if(te5650IsCapture()==1)    // if an index pulse occurs
      {
      te5650CapturePos(&axis3_index); // store the index position
      }

    axis3_error = 0;    
    te5650ActPosErr(&axis3_error);       // check the position error
    if(abs(axis3_error) > SEARCH_ERROR)  // second limit is reached, position error is growing
      {
      te5650ActPos(&axis3_limit_b);   // store second limit position
      axis3_center = (axis3_limit_a + axis3_limit_b) / 2;
      te5650ProfileTrap(axis3_center, SLOW_VELOCITY, SLOW_ACCEL);
      te5650Update();
      
      axis3_step_3 = 0;       
      
      axis3_index_done = 1;
      
      actuator_index_done = 1;  // this is the last axis
      
      //axis4_step_1 = 1;       // go on to next axis
                                 // axis 4 not used in 7/12/99 configuration
/*
      cout << endl << "Bow planes: " << endl;
      cout << "Limit A: " << axis3_limit_a << endl;
      cout << "Limit B: " << axis3_limit_b << endl;
      cout << "Center: " << axis3_center << endl;
      cout << "Index: " << axis3_index << endl;
*/  
      }  // end if(error)   
    
    }  // end axis3_step_3;  

  
  
  
/* 
  // ***** axis 4 ** spare 1  *****
  // ** axis 4 not used in 7/12/99 configuration **
  
  if(axis4_step_1 && !axis4_index_done)    // find first limit
    {
    te5650SetAxis(BOARD1, AXIS4);    
    te5650ProfileTrap(LIMIT_A, SLOW_VELOCITY, SLOW_ACCEL); // move toward first limit
    te5650Update();
    
    axis4_error = 0;
    
    short code = te5650ActPosErr(&axis4_error);
    
    //cout << "ActPosErr Code: " << code;    // check position error
    
  //  goto end;
    //short a_error = axis4_error;
    //cout << "TTTT"  << endl;
    //cout << axis4_error << endl;
    if(abs(axis4_error) > SEARCH_ERROR)  // first limit is reached, error is growing
      {
      te5650ActPos(&axis4_limit_a);   // store first limit position
      te5650ProfileTrap(axis4_limit_a, SLOW_VELOCITY, SLOW_ACCEL);// stay at limit position, stop looking
      te5650Update();
      te5650ResetCapture(); // discard any previously captured index positions
      axis4_step_1 = 0;       // ready for next step
      axis4_step_2 = 1;
      }  // end if(error)
    } // end axis4_step_1

      
  if(axis4_step_2 && !axis4_index_done)
    {
    te5650SetAxis(BOARD1, AXIS4);    
    if(te5650IsComplete())
      {
      axis4_step_2 = 0;
      axis4_step_3 = 1;
      }
    }  
    
    
  if(axis4_step_3 && !axis4_index_done)    // find second limit while capturing index pulse
    {
    te5650SetAxis(BOARD1, AXIS4);    
    te5650ProfileTrap(LIMIT_B, SLOW_VELOCITY, SLOW_ACCEL); // move toward opposite limit
    te5650Update();
    
    if(te5650IsCapture()==1)    // if an index pulse occurs
      {
      te5650CapturePos(&axis4_index); // store the index position
      }

    axis4_error = 0;
    te5650ActPosErr(&axis4_error);  // check the position error
    if(abs(axis4_error) > SEARCH_ERROR)  // second limit is reached, position error is growing
      {
      te5650ActPos(&axis4_limit_b);   // store second limit position
      axis4_center = (axis4_limit_a + axis4_limit_b) / 2;
      te5650ProfileTrap(axis4_center, SLOW_VELOCITY, SLOW_ACCEL);
      te5650Update();
      
                                
      cout << "Axis4 A: " << axis4_limit_a << "   B: " << axis4_limit_b
           << "   A-B: " << labs(axis4_limit_a - axis4_limit_b)
           << "   Index: " << axis4_index << "   Center: " << axis4_center
           << endl;
      
      
      axis4_step_3 = 0;       
  
      axis4_index_done = 1;
      
      actuator_index_done = 1;  // this is the last axis
      
      //axis5_step_1 = 1;       // ready for next step
                                // axis 5 not used in 7/12/99 configuration
  
  
                                 
      }  // end if(error)   
    
    }  // end axis4_step_3;  

*/


/* 
  // ***** axis 5 ** spare 2  *****
  // ** axis 5 not used in 7/12/99 configuration **
  
  if(axis5_step_1)    // find first limit
    {
    te5650SetAxis(BOARD2, AXIS1);    
    te5650ProfileTrap(LIMIT_A, SLOW_VELOCITY, SLOW_ACCEL); // move toward first limit
    te5650Update();
    
    axis5_error = 0;
    te5650ActPosErr(&axis5_error);    // check position error
    if(abs(axis5_error) > SEARCH_ERROR)  // first limit is reached, error is growing
      {
      te5650ActPos(&axis5_limit_a);   // store first limit position
      te5650ProfileTrap(axis5_limit_a, SLOW_VELOCITY, SLOW_ACCEL);// stay at limit position, stop looking
      te5650Update();
      te5650ResetCapture(); // discard any previously captured index positions
      axis5_step_1 = 0;       // ready for next step
      axis5_step_2 = 1;
      }  // end if(error)
    } // end axis5_step_1
    
    
  if(axis5_step_2)
    {
    te5650SetAxis(BOARD1, AXIS5);
    if(te5650IsComplete())
      {
      axis5_step_2 = 0;
      axis5_step_3 = 1;
      }
    }  
    
    
  if(axis5_step_3)    // find second limit while capturing index pulse
    {
    te5650SetAxis(BOARD1, AXIS5);
    te5650ProfileTrap(LIMIT_B, SLOW_VELOCITY, SLOW_ACCEL); // move toward opposite limit
    te5650Update();
    
    if(te5650IsCapture()==1)    // if an index pulse occurs
      {
      te5650CapturePos(&axis5_index); // store the index position
      }

    axis5_error = 0;
    te5650ActPosErr(&axis5_error);  // check the position error
    if(abs(axis5_error) > SEARCH_ERROR)  // second limit is reached, position error is growing
      {
      te5650ActPos(&axis5_limit_b);   // store second limit position
      axis5_center = (axis5_limit_a + axis5_limit_b) / 2;
      te5650ProfileTrap(axis5_center, SLOW_VELOCITY, SLOW_ACCEL);
      te5650Update();
      
      axis5_step_3 = 0;       
      axis5_index_done = 1;
      
      //axis6_step_1 = 1;       // ready for next step
                                // axis 6 not used in 7/12/99 configuration
      }  // end if(error)   
    
    }  // end axis5_step_3;  
*/



/* 
  // ***** axis 6 ** rpm  *****
  // ** axis 6 used only to decode prop position **
*/

  leave_index: ;
  
  }
    // end index_actuators();    



void print_index(void)
  {
  cout << endl << "Stern planes: " << endl;
  cout << "Limit A: " << axis1_limit_a << endl;
  cout << "Limit B: " << axis1_limit_b << endl;
  cout << "Center: " << axis1_center << endl;
  cout << "Index: " << axis1_index << endl;
      
  cout << endl << "Rudder: " << endl;
  cout << "Limit A: " << axis2_limit_a << endl;
  cout << "Limit B: " << axis2_limit_b << endl;
  cout << "Center: " << axis2_center << endl;
  cout << "Index: " << axis2_index << endl;
  
  cout << endl << "Bow planes: " << endl;
  cout << "Limit A: " << axis3_limit_a << endl;
  cout << "Limit B: " << axis3_limit_b << endl;
  cout << "Center: " << axis3_center << endl;
  cout << "Index: " << axis3_index << endl;
  
  print_index_done = 1;
  }


void sync_pinger(void)
  {
    // toggle pinger sync line low, then high (negative logic)
    // pinger sync signal is on LBC586 DIO bit 8 (port 0, bit 7)
//  aim2.bitset(aim2.DIOR, RESET, 9);   // (register, action, bit number)
  uio48.clr_bit(PINGER);
//  aim2.bitset(aim2.DIOR, SET,   9);
  uio48.set_bit(PINGER);
  }
                                  

void run_pinger(void)
  {
    // set pinger sync line low (negative logic)
    // pinger sync signal is on LBC586 DIO bit 8 (port 0, bit 7)    
//  aim2.bitset(aim2.DIOR, RESET,   9);
  uio48.clr_bit(PINGER);   //for scenix pinger
//  uio48.set_bit(PINGER);   //for legacy pinger  
  }
  
  
void reset_pinger(void)
  {
    // reset pinger sync line high (negative logic)
    // pinger sync signal is on LBC586 DIO bit 8 (port 0, bit 7)
  //aim2.bitset(aim2.DIOR, SET, 9);   // (register, action, bit number)
  uio48.set_bit(PINGER);  // for scenix pinger
//    uio48.clr_bit(PINGER);  // for legacy pinger
  }


/*
void send_data_to_daq(void)
  {
  for(i=0;i<144;i++)
    { // note: uio48 dio uses negative logic: logic 0 = HIGH, logic 1 = LOW
    while(~uio48.read_bit(DATA_ACK);   // make sure daq is ready to receive data
    uio48.port_write(DATA_LOW, data3[i]&0x00ff); // send low byte to daq
    uio48.port_write(DATA_HIGH, data3[i] >> 8);  // send high byte to daq
    uio48.set_bit(READ_DATA);   // flag daq to read data
    while(uio48.read_bit(DATA_ACK);  // wait for daq to read data and acknowledge
    }
  }  // end send_data_to _daq()
*/  