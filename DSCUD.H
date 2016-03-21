/*
 * Diamond Systems Corporation Universal Driver
 *
 * DOS/Win32 version
 *
 * Copyright 1997 Diamond Systems Corporation.  All rights reserved. 
 *
 */

#ifndef _DSCUD_H
#define _DSCUD_H

//#ifdef __cplusplus
//extern "C" {
//#endif


#define DSCUDAPI

#ifdef __BORLANDC__
#define _strnicmp strnicmp
#endif


#ifdef _WIN32
#ifndef _WINDOWS_
#define BYTE unsigned char
#define WORD unsigned short
#define DWORD unsigned long
#define FAR
#endif
#define halloc calloc
#define _fmemset memset
#define DSCUDAPICALL __declspec(dllexport)
#else
#define BYTE unsigned char
#define WORD unsigned int
#define DWORD unsigned long
#define FAR _far
#define DSCUDAPICALL
#endif

#ifndef _WINDOWS_
#define BOOL long
#define TRUE ((BOOL)1)
#define FALSE ((BOOL)0)
#endif

typedef WORD DSCB;

/* board types */
#define DSC_DMM16    0
#define DSC_RMM      1
#define DSC_TMM      2
#define DSC_OPMM     3
#define DSC_DMM      4
#define DSC_SMM      5
#define DSC_GMM      6
#define DSC_QMM      7
#define DSC_ZMM      8
#define DSC_PMM      9
#define DSC_OMM      10
#define DSC_RMM416   11
#define DSC_DMM32    12
#define DSC_EMMDIO   13
#define DSC_RMM1612  14
#define DSC_DMMAT    15
#define DSC_DMM16AT  16
#define DSC_IR104    17
#define DSC_EMM8	 18
#define DSC_DRVR     255 // Used for Driver Init/Free generic functions

#define BIPOLAR 0
#define UNIPOLAR 1

// VERSION CONTROL :  Used to keep dscud(*).lib-dscud.h together
#define DSC_VERSION   0x0300  // 0xVVRR - Version & Revision
WORD  DSCUDAPICALL dscVersion(void);

typedef struct
{
   BYTE device_number; /* must be 0 for dos, and the WinRT device # for win32 */
   BYTE boardtype;  // Equals DSC_DMM32, DSC_RMM, ...
   DSCB boardnum;  // Board number returned by Init Board function

   union
   { // NOTE : base_address must be the first element in each strcuture in the union (see initboard())
        struct
        {
           WORD base_address;  /* 0x100 to 0x3f0 */
           BYTE int_level;     /* 2-7 */
           BYTE dma_level;     /* 1 or 3 */
           DWORD clock_freq; /* 1 MHz or 10 MHz */
        } dmm;

        struct
        {
           WORD base_address;  /* 0x100 to 0x3f0 */
           BYTE int_level;     /* 2-7 */
           BYTE dma_level;     /* 1 or 3 */
           DWORD clock_freq; /* 1 MHz or 10 MHz */
        } dmmat;

        struct
        {
           WORD base_address;  /* 0x100 to 0x3f0 */
           BYTE int_level;     /* 2-7 */
           BYTE dma_level;     /* 1 or 3 */
           DWORD clock_freq;   /* 1 MHz or 10 MHz */
           BYTE polarity;      /* BIPOLAR or UNIPOLAR */
        } dmm16;

        struct
        {
           WORD base_address;  /* 0x100 to 0x3f0 */
           BYTE int_level;     /* 2-7 */
           BYTE dma_level;     /* 1 or 3 */
           DWORD clock_freq;   /* 1 MHz or 10 MHz */
           BYTE polarity;      /* BIPOLAR or UNIPOLAR */
        } dmm16at;

        struct
        {
           WORD base_address;  /* 0x100 to 0x3f0 */
           BYTE int_level;     /* 2-7 */
           BYTE dma_level;     /* 1 or 3 */
           DWORD clock_freq;   /* 1 MHz or 10 MHz */
           BYTE polarity;      /* BIPOLAR or UNIPOLAR */
        } dmm32;

        struct
        {
           WORD base_address;  /* 0x200 to 0x3f8 */
           BYTE int_level1;    /* 2-7 */
           BYTE int_level2;    /* 2-7 */
           DWORD clock_freq; /* 4 MHz usually */
        } gmm;

        struct
        {
           WORD base_address;  /* 0x200 to 0x3f8 */
           BYTE int_level1;    /* 2-7 */
           BYTE int_level2;    /* 2-7 */
           BYTE int_level3;    /* 2-7 */
           DWORD clock_freq; /* 4 MHz usually */
        } omm;

        struct
        {
           WORD base_address;  /* 0x100 to 0x3C0 */
           BYTE int_level;    /* 2-7, 10-15 */
        //   BYTE int_level2;    /* 2-7 */
       //    BYTE int_level3;    /* 2-7 */
      //     DWORD clock_freq; /* 4 MHz usually */
        } emmdio;

        struct
        {
           WORD base_address;  /* 0x200 to 0x3f8 */
           BYTE int_level;     /* 2-7 */
        } qmm;

        struct
        {
           WORD base_address;  /* 0x200 to 0x3c0 */
           BYTE int_level;     /* 2-7 */
           BOOL DAC_0_3_external_enable;
           BOOL DAC_4_7_external_enable;
           BOOL external_trigger_c3;
        } rmm;

        struct
        {
           WORD base_address;  /* 0x240, 0x280, 0x300, or 0x340 */
           BYTE int_level;     /* 2-7 */
           BYTE dma_level;     /* 1 or 3 */
           DWORD clock_freq;   /* 4 MHz usually */
           BYTE AD_resolution; /* 12 or 14 */
        } smm;

        struct
        {
           WORD base_address;  /* 0x100 to 0x3f0 */
           BYTE int_level;     /* 2-7 */
           DWORD clock_freq; /* 4 MHz usually */
        } zmm;

        struct
        {
           WORD base_address;  /* 0x240 to 0x3c0 */
        } opmm;

        struct
        {
           WORD base_address;  /* 0x240 to 0x3c0 */
        } pmm;

        struct
        {
           WORD base_address;  /* 0x220 to 0x3e0 */
           BYTE int_level;     /* 2-7 */
        } tmm;

        struct
        {
           WORD base_address;  /* 0x100 to 0x3c0 */
           BOOL external_trigger;
           BOOL mode_16bit; /* if the D/A ports are in 16 bit mode */
        } rmm416;

        struct
        {
           WORD base_address;  /* 0x100 to 0x3c0 */
           BOOL external_trigger;
           BOOL mode_16bit; /* if the D/A ports are in 16 bit mode */
        } rmm1612;

        struct
        {
           WORD base_address;  /* 0x240 to 0x3c0 */
        } IR104;

		struct 
		{
			WORD base_address;
		    WORD IOAddr[8];
		    WORD Interrupt[8];
		} emm8;

   };
} DSCCB;

/* Analog/Digital input */

#define GAIN_1 0
#define GAIN_2 1
#define GAIN_4 2
#define GAIN_8 3
/* some SMMs have these gains: */
#define GAIN_1 0
#define GAIN_10 1
#define GAIN_100 2
#define GAIN_1000 3

#define RANGE_BIPOLAR_5 0
#define RANGE_BIPOLAR_10 1
#define RANGE_UNIPOLAR_10 2

typedef struct
{
   BYTE current_channel;
//   BYTE low_channel;
//   BYTE high_channel;
   BYTE gain;
   BYTE range;
   BYTE polarity;
   BYTE load_cal;
} DSCADSETTINGS;

typedef struct
{
   BYTE low_channel;
   BYTE high_channel;
   WORD *sample_values;
} DSCADSCAN;

typedef struct
{
   BYTE address;
   BYTE data;
} REGPARAMS;

typedef struct
{
   BYTE ErrCode;
   char *errstring;
} ERRPARAMS;

BYTE DSCUDAPICALL dscADScan(DSCB board,DSCADSCAN *dscadscan,WORD FAR *sample_values);

typedef struct
{
   DWORD num_conversions; /* total # of conversions to write to sample_values, unless cycling. */
   float conversion_rate; /* interrupts per second */
   BOOL cycle; /* when done with num_conversions, restart at the beginning of the buffer? */
   BOOL internal_clock; /* use the internal clock (82c54) ? */
   BYTE low_channel; /* lowest channel number to perform A/D conversion on */
   BYTE high_channel; /* highest channel number to perform A/D conversion on */
   BOOL external_gate_enable; /* A/D dmm, dmm-16 only */
   BOOL internal_clock_gate;
   WORD FAR *sample_values;
   BOOL fifo_enab; // Fifo operation flag for int sample/scan modes.
   WORD fifo_depth; // Fifo depth (if fifo mode used)
} DSCAIOINT;

typedef struct
{
   BYTE adrange;  // A/D Range to calibrate (use 0xFF for all ranges)
   BOOL use_eeprom;  // Use target values in EEPROM for cal
   WORD target_values[8]; // User supplied target values (when use_eeprom = FALSE)
   BYTE boot_adrange;  // When cal'ed, this adrange is stored to BOOT values
   WORD darange;  // D/A range to calibrate
   float ad_offset, ad_gain;  // Difference between target and measured vals.
   float da_offset, da_gain;  // Difference between target and measured vals.
} DSCAUTOCAL;

typedef struct
{
  BYTE chan;
  BOOL cal_enable;
} SETCALPARAMS;

BYTE DSCUDAPICALL dscADAutoCal(DSCB board, DSCAUTOCAL *dscautocal);

/* Digital/Analog output */


typedef struct
{
   BOOL channel_enable[8];
   WORD *output_codes;
} DSCDACS;

typedef struct
{
  BYTE channel;
  WORD output_code;
} DAPARAMS;  // Single D/A channel

BYTE DSCUDAPICALL dscDAAutoCal(DSCB board, DSCAUTOCAL *dscautocal);
BYTE DSCUDAPICALL dscDACalVerify(DSCB board, DSCAUTOCAL *dscautocal);

typedef struct   // D/A cal parameters for one DAC
{
  BOOL internal;  // internal=FALSE means external
  BOOL fixed;     // fixed=FALSE means programmable.
  BOOL polarity;  // Bipolar=FALSE, Unipolar=TRUE
  WORD darange;   // D/A range to calibrate
  float offset;   // Difference between target and measured vals.
  float gain;     // Difference between target and measured vals.
  BYTE  dacnum;   // DAC number, used by Cal Verify
  BOOL  use_eeprom; // if true, use values stored in EEPROM
} DSCDACALPARAMS;  // Single D/A channel

// dscdacalparams points to array of DSCDACALPARAMS for all DACs parametrs
BYTE DSCUDAPICALL dscDAAutoCal2(DSCB board, DSCDACALPARAMS *dscdacalparams);

// dscdacalparams points to DSCDACALPARAMS for one DACs parametrs
BYTE DSCUDAPICALL dscDACalVerify2(DSCB board, DSCDACALPARAMS *dscdacalparams);

/* Digital input/output */
typedef struct
{
   BYTE port;
   WORD digital_value;
   BYTE bit;
} DIOPARAMS;

BYTE DSCUDAPICALL dscDIOSetConfig(DSCB board,BYTE *config_bytes);
BYTE DSCUDAPICALL dscDIOInputByte(DSCB board,BYTE port,BYTE FAR *digital_value);
BYTE DSCUDAPICALL dscDIOInputWord(DSCB board,BYTE port,WORD FAR *digital_value);
BYTE DSCUDAPICALL dscDIOOutputByte(DSCB board,BYTE port,BYTE digital_value);
BYTE DSCUDAPICALL dscDIOOutputWord(DSCB board,BYTE port,WORD digital_value);
BYTE DSCUDAPICALL dscDIOInputBit(DSCB board,BYTE port,BYTE bit,BYTE FAR *digital_value);
BYTE DSCUDAPICALL dscDIOSetBit(DSCB board,BYTE port,BYTE bit);
BYTE DSCUDAPICALL dscDIOClearBit(DSCB board,BYTE port,BYTE bit);

/* modes for GMM */
#define MODE_0_INT11_INT12 0
#define MODE_0_INT21_INT22 1
#define MODE_1_82C55_1_C0 2
#define MODE_1_82C55_1_C3 3
#define MODE_1_82C55_2_C0 4
#define MODE_1_82C55_2_C3 5

/* sources for OMM */
#define SOURCE_82C55_1_C0 0
#define SOURCE_COUNTER_0  1
#define SOURCE_82C55_2_C0 2
#define SOURCE_COUNTER_1  3
#define SOURCE_EXTERNAL   4
#define SOURCE_COUNTER_2  5

typedef struct
{
   WORD num_transfers;
   BYTE port; /* for boards w/ >8 bits of dio */
   BYTE size; /* 0-7 for bits 0-7, 8 for full byte, 16 for word */
   BOOL cycle;
   BOOL internal_clock;
   BYTE mode; /* for Garnet only */
   BYTE source; /* for Onyx only */
   BYTE FAR *digital_values;
} DSCDIOINT;


BYTE DSCUDAPICALL dscUserInt(DSCB board,DSCDIOINT *dscdioint);
BYTE DSCUDAPICALL dscDIOInputInt(DSCB board,DSCDIOINT *dscdioint);
BYTE DSCUDAPICALL dscDIOOutputInt(DSCB board,DSCDIOINT *dscdioint);

#define OP_TYPE_NONE 0
#define OP_TYPE_INT 1
#define OP_TYPE_DMA 2

typedef struct
{
   BYTE op_type;
   DWORD transfers;
   BOOL output;
   WORD base_address;
   BYTE int_level;
   BYTE dma_level;
   BOOL analog;
} DSCS;

BYTE DSCUDAPICALL dscCancelOp(DSCB board);
BYTE DSCUDAPICALL dscPauseOp(DSCB board);
BYTE DSCUDAPICALL dscResumeOp(DSCB board);

/* Counter/Timer control */
typedef struct
{
   WORD value;
   BYTE status;
} DSCCS;  //Counter readback

typedef struct
{
   BYTE control_code;
   BYTE counter_number;
   WORD counter_data; // Counter write data
   DSCCS counter0;  // Counter read data
   DSCCS counter1;
   DSCCS counter2;
} DSCCR;

#define OMM_SOURCE_PIN 0
#define OMM_SOURCE_CLOCK 1
#define OMM_SOURCE_OUT 2

typedef struct
{
   BYTE counter_source0;
   BYTE counter_source1;
   BYTE counter_source2;
} OMMCONFIGPARAMS;

#define GROUP1 0
#define GROUP2 1

#define FOUT_DIVIDER_16 0

#define E1 0
#define SOURCE1 1
#define SOURCE2 2
#define SOURCE3 3
#define SOURCE4 4
#define SOURCE5 5
#define GATE1   6
#define GATE2   7
#define GATE3   8
#define GATE4   9
#define GATE5   10
#define F1 11
#define F2 12
#define F3 13
#define F4 14
#define F5 15

#define TOD_DISABLED 0
#define TOD_DIV_5 1
#define TOD_DIV_6 2
#define TOD_DIV_10 3

typedef struct
{
  BYTE group;
  BYTE Fout_divider;
  BYTE Fout_source;
  BOOL compare2_enable;
  BOOL compare1_enable;
  BYTE time_of_day_mode;
} SET9513MMRPARAMS;

typedef struct
{
  BYTE counter;
  BYTE gating;
  BOOL active_edge_falling;
  BYTE count_source;
  BOOL special_gate_enable;
  BOOL reload_load_or_hold;
  BOOL cycle;
  BOOL bcd;
  BOOL count_up;
  BYTE output;
} SET9513CMRPARAMS;

#define ACTION_NONE 0
#define ACTION_ARM 1
#define ACTION_LOAD 2
#define ACTION_LOAD_ARM 3
#define ACTION_DISARM_SAVE 4
#define ACTION_SAVE 5
#define ACTION_DISARM 6

typedef struct
{
  BYTE group;    // Used by both Control and GetStatus
  BYTE action;
  BYTE selection;
  BYTE *status;  // Used only by get Status
} CTR9513CNTLPARAMS;

typedef struct
{
  BYTE counter;
  WORD data;
} DS9513IOPARAMS;

#define SP_ACTION_CLEAR_TOGGLE_OUTPUT 0
#define SP_ACTION_SET_TOGGLE_OUTPUT 1
#define SP_ACTION_STEP_COUNTER 2
typedef struct
{
  BYTE counter;
  BYTE action;
} DSC9513SPECIALCTRPARAMS;

typedef struct
{
  BYTE counter;
  WORD alarm_value;
} DSC9513SETCTRALARMPARAMS;

typedef struct
{
   WORD num_transfers;
   BYTE counter_select1;
   BYTE counter_select2;
   BOOL cycle;
   WORD FAR *counter_values;
} DSCCNTINT;

typedef struct
{
  BYTE interval;
  BYTE source;
  WORD *pulses;
} DSC9513MEASFREQPARAMS;

typedef struct
{
  BYTE frequency;
  WORD *periods;
} DSC9513MEASPERIODPARAMS;

typedef void (*user_interrupt_type)(void);

BYTE DSCUDAPICALL dscEnableUserInts(DSCB board,user_interrupt_type new_user_interrupt, BYTE mode);
BYTE DSCUDAPICALL dscDisableUserInts(DSCB board);

BYTE GetErrorString(ERRPARAMS *errparams);
/* returns the string describing the error */

/* error codes */
#define DE_NONE 0             /* no error */
#define DE_HW_FAILURE 1       /* hardware failed */
#define DE_SW_FAILURE 2       /* software failed */
#define DE_HW_NOT_SUPPORTED 3 /* could not perform operation on this board */
#define DE_SW_NOT_SUPPORTED 4 /* software does not yet support this operation */
#define DE_INVALID_PARM 5     /* a parameter to the function is invalid */
#define DE_ALTERNATE_IN_PROGRESS 6 /* alternate interrupt function in progress */
#define DE_NONE_IN_PROGRESS 7 /* no interrupt function in progress to pause/cancel/resume */
#define DE_BUFFER_ROLLOVER 8  /* pointer passed in + sizeof data to be written would roll over a segment */
#define DE_ALREADY_PAUSED 10  /* can't pause an operation--it's already paused */
#define DE_OVERFLOW       11  /* Am9513A counter function overflowed */
#define DE_INVALID_FUNC   12  // Function number not recognized by board type
#define DE_DSCUDH_INVALID 13  // Header - Library version mismatch
#endif

////////////////////////////////////////////////////////
// The following defines are used for the new DSCUD driver.
// These constants are passed as the 'BYTE func' parameter to dscud
////////////////////////////////////////////////////////
BYTE DSCUDAPICALL dscud(DSCB board,WORD func,void *params);

//#define DSINIT                54
#define DRVRFUNCTIONS          0x0000
#define A2DFUNCTIONS           0x1000
#define D2AFUNCTIONS           0x2000
#define DIOFUNCTIONS           0x3000
#define A9513FUNCTIONS         0x4000
#define CTRFUNCTIONS           0x5000
#define AUTOCALFUNCTIONS       0x6000
#define OTHERFUNCTIONS         0x7000

#define DSINIT                 DRVRFUNCTIONS+0
#define DSFREE                 DRVRFUNCTIONS+1
#define DSINITBOARD            DRVRFUNCTIONS+2
#define DSFREEBOARD            DRVRFUNCTIONS+3
#define DRVRTEST               DRVRFUNCTIONS+4

#define DSADSETSETTINGS        A2DFUNCTIONS+0
#define DSADSAMPLE             A2DFUNCTIONS+1
#define DSADSCAN               A2DFUNCTIONS+2
#define DSADSAMPLEINT          A2DFUNCTIONS+3
#define DSADSCANINT            A2DFUNCTIONS+4
#define DSADSAMPLEDMA          A2DFUNCTIONS+5

#define DSADAUTOCAL            AUTOCALFUNCTIONS+0
#define DSADCALVERIFY          AUTOCALFUNCTIONS+1
#define DSSETCALCHANNEL        AUTOCALFUNCTIONS+2
#define DSMEASUREREFERENCES    AUTOCALFUNCTIONS+3
#define DSDAAUTOCAL            AUTOCALFUNCTIONS+4
#define DSDACALVERIFY          AUTOCALFUNCTIONS+5
#define DSDAAUTOCAL2           AUTOCALFUNCTIONS+6
#define DSDACALVERIFY2         AUTOCALFUNCTIONS+7

#define DSDACONVERT            D2AFUNCTIONS+0
#define DSDACONVERTSCAN        D2AFUNCTIONS+1
#define DSDACONVERTSCANINT     D2AFUNCTIONS+2

#define DSDIOSETCONFIG         DIOFUNCTIONS+0
#define DSDIOINPUTBYTE         DIOFUNCTIONS+1
#define DSDIOOUTPUTBYTE        DIOFUNCTIONS+2
#define DSDIOINPUTWORD         DIOFUNCTIONS+3
#define DSDIOOUTPUTWORD        DIOFUNCTIONS+4
#define DSDIOINPUTBIT          DIOFUNCTIONS+5
#define DSDIOSETBIT            DIOFUNCTIONS+6
#define DSDIOCLEARBIT          DIOFUNCTIONS+7
#define DSDIOINPUTINT          DIOFUNCTIONS+8
#define DSDIOOUTPUTINT         DIOFUNCTIONS+9

#define DSCOUNTERSETRATE        CTRFUNCTIONS+0
#define DSCOUNTERDIRECTSET      CTRFUNCTIONS+1
#define DSCOUNTERREAD           CTRFUNCTIONS+2
#define DSOMMCONFIGURECOUNTERS  CTRFUNCTIONS+3

#define DS9513SETMMR           A9513FUNCTIONS+0
#define DS9513SETCMR           A9513FUNCTIONS+1
#define DS9513COUNTERCONTROL   A9513FUNCTIONS+2
#define DS9513GETSTATUS        A9513FUNCTIONS+3
#define DS9513SETCLR           A9513FUNCTIONS+4
#define DS9513SETCHR           A9513FUNCTIONS+5
#define DS9513GETCHR           A9513FUNCTIONS+6
#define DS9513SPECIALCOUNTER   A9513FUNCTIONS+7
#define DS9513SETCOUNTERALARM  A9513FUNCTIONS+8
#define DS9513COUNTERINT       A9513FUNCTIONS+9
#define DS9513MEASUREFREQUENCY A9513FUNCTIONS+10
#define DS9513MEASUREPERIOD    A9513FUNCTIONS+11

#define DSUSERINT              OTHERFUNCTIONS+0
#define DSGETSTATUS            OTHERFUNCTIONS+1
#define DSCANCELOP             OTHERFUNCTIONS+2
#define DSPAUSEOP              OTHERFUNCTIONS+3
#define DSRESUMEOP             OTHERFUNCTIONS+4
#define DSENABLEUSERINTS       OTHERFUNCTIONS+5
#define DSDISABLEUSERINTS      OTHERFUNCTIONS+6
#define DSREGISTERWRITE        OTHERFUNCTIONS+7
#define DSREGISTERREAD         OTHERFUNCTIONS+8
#define DSDISABLEINTS          OTHERFUNCTIONS+9
#define DSGETERRORSTRING       OTHERFUNCTIONS+10

#define OPTOISOLATOR           1
#define RELAY                  0

/////////////////////////////////////////////////////////////////////////
// ORIGINAL UNIVERSAL DRIVER FUNCTION CALLS
/////////////////////////////////////////////////////////////////////////

// Driver Level Functions
BYTE DSCUDAPICALL dscInit(WORD version);
void DSCUDAPICALL dscFree(void);

// Board Level Init/Free
BYTE DSCUDAPICALL dscInitBoard(BYTE board_type,DSCCB *createboard_info,DSCB *board);
BYTE DSCUDAPICALL dscFreeBoard(DSCB board);

// A/D Functions
BYTE DSCUDAPICALL dscADSetSettings(DSCB board,DSCADSETTINGS *adsettings);
BYTE DSCUDAPICALL dscADSample(DSCB board,WORD *sample_value);
BYTE DSCUDAPICALL dscADScan(DSCB board,DSCADSCAN *dscadscan,WORD FAR *sample_values);
BYTE DSCUDAPICALL dscADSampleInt(DSCB board,DSCAIOINT *dscaioint);
BYTE DSCUDAPICALL dscADScanInt(DSCB board,DSCAIOINT *dscaioint);
BYTE DSCUDAPICALL dscADSampleDMA(DSCB board,DSCAIOINT *dscaioint);

// D/A Functions
BYTE DSCUDAPICALL dscDAConvert(DSCB board,BYTE channel,WORD output_code);
BYTE DSCUDAPICALL dscDAConvertScan(DSCB board,DSCDACS *dscdacs);
BYTE DSCUDAPICALL dscDAConvertScanInt(DSCB board,DSCAIOINT *dscaioint);

// AutoCal Functions
BYTE DSCUDAPICALL dscADAutoCal(DSCB board, DSCAUTOCAL *dscautocal);
BYTE DSCUDAPICALL dscADCalVerify(DSCB board, DSCAUTOCAL *dscautocal);
BYTE DSCUDAPICALL dscSetCalChannel(DSCB board, BYTE chan, BOOL cal_enable);
BYTE DSCUDAPICALL dscMeasureReferences(DSCB board,WORD *target_values);
BYTE DSCUDAPICALL dscDAAutoCal(DSCB board, DSCAUTOCAL *dscautocal);
BYTE DSCUDAPICALL dscDACalVerify(DSCB board, DSCAUTOCAL *dscautocal);

// Digital I/O Functions
BYTE DSCUDAPICALL dscDIOSetConfig(DSCB board,BYTE *config_bytes);
BYTE DSCUDAPICALL dscDIOInputByte(DSCB board,BYTE port,BYTE FAR *digital_value);
BYTE DSCUDAPICALL dscDIOOutputByte(DSCB board,BYTE port,BYTE digital_value);
BYTE DSCUDAPICALL dscDIOInputWord(DSCB board,BYTE port,WORD FAR *digital_value);
BYTE DSCUDAPICALL dscDIOOutputWord(DSCB board,BYTE port,WORD digital_value);
BYTE DSCUDAPICALL dscDIOInputBit(DSCB board,BYTE port,BYTE bit,BYTE FAR *digital_value);
BYTE DSCUDAPICALL dscDIOSetBit(DSCB board,BYTE port,BYTE bit);
BYTE DSCUDAPICALL dscDIOClearBit(DSCB board,BYTE port,BYTE bit);
BYTE DSCUDAPICALL dscDIOInputInt(DSCB board,DSCDIOINT *dscdioint);
BYTE DSCUDAPICALL dscDIOOutputInt(DSCB board,DSCDIOINT *dscdioint);

// IR104 functions
BYTE DSCUDAPICALL dscIR104InputBit(DSCB board, BYTE unit, BYTE type, BYTE FAR *digital_value);
BYTE DSCUDAPICALL dscIR104SetBit  (DSCB board, BYTE relay);
BYTE DSCUDAPICALL dscIR104ClearBit(DSCB board, BYTE relay);

// Counter/Timer Functions
BYTE DSCUDAPICALL dscCounterSetRate(DSCB board,float hz);
BYTE DSCUDAPICALL dscCounterDirectSet(DSCB board,BYTE control_code,WORD counter_data,BYTE counter_number);
BYTE DSCUDAPICALL dscCounterRead(DSCB board,DSCCR *dsccr);
BYTE DSCUDAPICALL dscOMMConfigureCounters(DSCB board,BYTE counter_source0,BYTE counter_source1,BYTE counter_source2);

// 9513 Functions
BYTE DSCUDAPICALL dsc9513SetMMR(DSCB board,BYTE group,BYTE Fout_divider,BYTE Fout_source,BOOL compare2_enable,
                   BOOL compare1_enable,BYTE time_of_day_mode);
BYTE DSCUDAPICALL dsc9513SetCMR(DSCB board,BYTE counter,BYTE gating,BOOL active_edge_falling,BYTE count_source,
                   BOOL special_gate_enable,BOOL reload_load_or_hold,BOOL cycle,BOOL bcd,BOOL count_up,BYTE output);
BYTE DSCUDAPICALL dsc9513CounterControl(DSCB board,BYTE group,BYTE action,BYTE selection);
BYTE DSCUDAPICALL dsc9513GetStatus(DSCB board,BYTE group,BYTE *status);
BYTE DSCUDAPICALL dsc9513SetCLR(DSCB board,BYTE counter,WORD data);
BYTE DSCUDAPICALL dsc9513SetCHR(DSCB board,BYTE counter,WORD data);
BYTE DSCUDAPICALL dsc9513GetCHR(DSCB board,BYTE counter,WORD FAR *data);
BYTE DSCUDAPICALL dsc9513SpecialCounter(DSCB board,BYTE counter,BYTE action);
BYTE DSCUDAPICALL dsc9513SetCounterAlarm(DSCB board,BYTE counter,WORD alarm_value);
BYTE DSCUDAPICALL dsc9513CounterInt(DSCB board_type,DSCCNTINT *dsccntint);
BYTE DSCUDAPICALL dsc9513MeasureFrequency(DSCB board,BYTE interval,BYTE source,WORD *pulses);
BYTE DSCUDAPICALL dsc9513MeasurePeriod(DSCB board,BYTE frequency,WORD *periods);

// Other Functions
//BYTE DSCUDAPICALL dscUserInt(DSCB board,DSCDIOINT *dscdioint);
BYTE DSCUDAPICALL dscGetStatus(DSCB board,DSCS *dscs);
BYTE DSCUDAPICALL dscCancelOp(DSCB board);
BYTE DSCUDAPICALL dscPauseOp(DSCB board);
BYTE DSCUDAPICALL dscResumeOp(DSCB board);
//BYTE dscEnableUserInts(DSCB board,user_interrupt_type new_user_interrupt);
//BYTE dscDisableUserInts(DSCB board);
DSCUDAPICALL dscRegisterWrite(DSCB board, BYTE addr , BYTE data );
DSCUDAPICALL dscRegisterRead(DSCB board, BYTE addr , BYTE *data );
DSCUDAPICALL dscDisableInts(DSCB board);
DSCUDAPICALL char * dscGetErrorString(BYTE de_error);

//#ifdef __cplusplus
//}
//#endif  
