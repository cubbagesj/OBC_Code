/* motor output types */
#define TE5650TYPE_DAC16   0
#define TE5650TYPE_PWM     1

/* profile modes */
#define TE5650MODE_VEL     0
#define TE5650MODE_TRAP    1
#define TE5650MODE_SCURVE  2
#define TE5650MODE_GEAR    3

/* breakpoint types */
#define TE5650BRK_TIME       0
#define TE5650BRK_DESPOS_GE  1
#define TE5650BRK_DESPOS_LE  2
#define TE5650BRK_ACTPOS_GE  3
#define TE5650BRK_ACTPOS_LE  4

/* configuration groups */
#define TE5650CFG12  0
#define TE5650CFG34  1

/* drive groups */
#define TE5650DRV12   0x01
#define TE5650DRV34   0x02
#define TE5650DRV_ALL (TE5650DRV12 | TE5650DRV34)

/* axis masks */
#define TE5650AXIS_1   0x01
#define TE5650AXIS_2   0x02
#define TE5650AXIS_3   0x04
#define TE5650AXIS_4   0x08
#define TE5650AXIS_ALL 0x0F

/* event flags */
#define TE5650EV_TC  0x01
#define TE5650EV_WA  0x02
#define TE5650EV_BP  0x04
#define TE5650EV_PC  0x08
#define TE5650EV_PE  0x10
#define TE5650EV_PL  0x20
#define TE5650EV_NL  0x40

/* commands */
#define TE5650_SET_1                0x01
#define TE5650_SET_2                0x02
#define TE5650_SET_3                0x03
#define TE5650_SET_4                0x04
#define TE5650_SET_I                0x08
#define TE5650_SET_PRFL_TRAP        0x09
#define TE5650_SET_PRFL_VEL         0x0A
#define TE5650_SET_PRFL_S_CRV       0x0B
#define TE5650_SET_PRFL_GEAR        0x0C

#define TE5650_SET_POS              0x10
#define TE5650_SET_VEL              0x11
#define TE5650_SET_ACC              0x12
#define TE5650_SET_JERK             0x13
#define TE5650_SET_RATIO            0x14
#define TE5650_SET_MAX_ACC          0x15
#define TE5650_SET_BRK_PNT          0x16
#define TE5650_SET_TIME_BRK         0x17
#define TE5650_SET_POS_BRK          0x18
#define TE5650_SET_NEG_BRK          0x19
#define TE5650_UPDATE               0x1A
#define TE5650_SET_ACTL_POS_BRK     0x1B
#define TE5650_SET_ACTL_NEG_BRK     0x1C
#define TE5650_GET_TRGT_POS         0x1D
#define TE5650_GET_TRGT_VEL         0x1E
#define TE5650_GET_TRGT_ACC         0x1F

#define TE5650_SET_FLTR_PID         0x21
#define TE5650_SET_FLTR_PIVFF       0x22
#define TE5650_SET_KP               0x25
#define TE5650_SET_KI               0x26
#define TE5650_SET_KD               0x27
#define TE5650_SET_I_LM             0x28
#define TE5650_SET_POS_ERR          0x29
#define TE5650_SET_KV               0x2A
#define TE5650_SET_KVFF             0x2B
#define TE5650_GET_INTGR            0x2E
#define TE5650_SET_INTRPT_MASK      0x2F

#define TE5650_GET_INTRPT           0x30
#define TE5650_GET_STATUS           0x31
#define TE5650_RST_INTRPT           0x32
#define TE5650_CLR_STATUS           0x33
#define TE5650_RST_STATUS           0x34
#define TE5650_GET_CAPT             0x36
#define TE5650_GET_ACTL_POS         0x37
#define TE5650_SET_SMPL_TIME        0x38
#define TE5650_RESET                0x39
#define TE5650_SET_OUTPUT_DAC16     0x3B
#define TE5650_SET_OUTPUT_PWM       0x3C
#define TE5650_SET_OUTPUT_DAC12     0x3D
#define TE5650_GET_TIME             0x3E
#define TE5650_ZERO_POS             0x3F

#define TE5650_AXIS_OFF             0x40
#define TE5650_AXIS_ON              0x41
#define TE5650_MTR_OFF              0x42
#define TE5650_MTR_ON               0x43
#define TE5650_SET_AUTO_STOP_OFF    0x44
#define TE5650_SET_AUTO_STOP_ON     0x45
#define TE5650_CLR_PRFL             0x46
#define TE5650_SYNC_PRFL            0x47
#define TE5650_GET_MODE             0x48
#define TE5650_GET_POS              0x4A
#define TE5650_GET_VEL              0x4B
#define TE5650_GET_ACC              0x4C
#define TE5650_GET_MAX_ACC          0x4F

#define TE5650_GET_KP               0x50
#define TE5650_GET_KI               0x51
#define TE5650_GET_KD               0x52
#define TE5650_GET_I_LM             0x53
#define TE5650_GET_KVFF             0x54
#define TE5650_GET_POS_ERR          0x55
#define TE5650_GET_INTRPT_MASK      0x56
#define TE5650_GET_BRK_PNT          0x57
#define TE5650_GET_JERK             0x58
#define TE5650_GET_RATIO            0x59
#define TE5650_GET_KV               0x5A
#define TE5650_MULTI_UPDATE         0x5B
#define TE5650_SET_AUTO_UPDATE_ON   0x5C
#define TE5650_SET_AUTO_UPDATE_OFF  0x5D

#define TE5650_GET_ACTL_POS_ERR     0x60
#define TE5650_GET_SMPL_TIME        0x61
#define TE5650_SET_MTR_CMD          0x62
#define TE5650_SET_CAPT_INDEX       0x64
#define TE5650_SET_CAPT_HOME        0x65
#define TE5650_SET_LMT_SENSE        0x66
#define TE5650_GET_LMT_SWTCH        0x67
/* #define TE5650_SET_CNTS             0x68 */
/* #define TE5650_SET_ENC_PRL          0x6B */

#define TE5650_GET_VRSN             0x6C
#define TE5650_SET_BRK_OFF          0x6D
#define TE5650_GET_OUTPUT_MODE      0x6E

#define TE5650_LMTS_ON              0x70
#define TE5650_LMTS_OFF             0x71


#define TE5650_STOP                             0x46
#define TE5650_SMOOTH_STOP                      0x4e
#define TE5650_SET_MTR_LMT                      0x06
#define TE5650_GET_MTR_LMT                      0x07
#define TE5650_SET_MTR_BIAS                     0x0f
#define TE5650_GET_MTR_BIAS                     0x2d
#define TE5650_SET_MTR_CMPLT_BRK                                0x35
#define TE5650_SET_EXT_BRK                      0x5E
#define TE5650_GET_MTR_CMD                      0x3a
#define TE5650_SET_ACTL_POS                     0x4d


/* commutation */
#define TE5650_SET_PHASE_3                              0x73
#define TE5650_SET_PHASE_2                              0x74
#define TE5650_SET_PHASE_HALL                           0x78
#define TE5650_SET_PHASE_ALGOR                          0x79
#define TE5650_SET_PRESCALE_ON                          0x0d
#define TE5650_SET_PRESCALE_OFF                         0x0e
#define TE5650_SET_PHASE_CNTS                   0x75
#define TE5650_SET_PHASE_TIME                   0x72
#define TE5650_SET_PHASE_OFFSET                 0x76
#define TE5650_SET_KPHASE                       0x1F
#define TE5650_SET_HALL_SENSE                   0x23
#define TE5650_INIT_PHASE                       0x7A
#define TE5650_GET_PHASE_INFO                   0x7F
#define TE5650_GET_PHASE_CNTS                   0x7d
#define TE5650_GET_PHASE_TIME                   0x7c
#define TE5650_GET_PHASE_OFFSET                 0x7b
#define TE5650_GET_KPHASE                       0x20
#define TE5650_GET_HALL_SENSE                   0x24
#define TE5650_GET_HALLS                        0x7e
#define TE5650_GET_PHASE                        0x2C
#define TE5650_PRESCALE_ON                      0x0d
#define TE5650_PRESCALE_OFF                     0x0e
#define TE5650_GET_BUF_MTR_CMD                                          0x69
#define TE5650_SET_BUF_MTR_CMD                                          0x77

#define LIMIT_A 3000
#define LIMIT_B -3000

//#define FAST_VELOCITY 90000
//#define SLOW_VELOCITY 10000
#define FAST_VELOCITY 180000
#define SLOW_VELOCITY 20000

//#define FAST_ACCEL 300
//#define SLOW_ACCEL 50
#define FAST_ACCEL 600
#define SLOW_ACCEL 100


#define MAX_ERROR 32000
#define SEARCH_ERROR 100

//#define KP 500            // jtm 02/14/00
#define KP 400
//#define KI 2 // jtm 02/14/00
#define KI 0
#define KD 20000
//#define IL 1000 // jtm 02/14/00
#define IL 0
#define KVFF 0
//#define KP 1000           
//#define KI 4
//#define KD 32767
//#define IL 2000
//#define KVFF 0

#define ADDRESS 0x1300
#define BOARD1 1
#define BOARD2 2
#define AXIS1 1
#define AXIS2 2
#define AXIS3 3
#define AXIS4 4
#define INVERT 1
#define NONINVERT 0
#define AUTOSTOP_ON 1
#define AUTOSTOP_OFF 0
#define SAMPLE_TIME 10

typedef unsigned short WORD;
/* function declarations */
short te5650ActPos(long *plPosition);
short te5650ActPosErr(short *psErr);
short te5650AutoUpdateOff(void);
short te5650AutoUpdateOn(void);
short te5650AxisIsOn(void);
short te5650AxisOff(void);
short te5650AxisOn(void);
short te5650AxisRestore(void);
short te5650AxisSave(void);

short te5650BreakPointOff(void);

short te5650CaptureHome(void);
short te5650CaptureIndex(void);
short te5650CapturePos(long *plPosition);
short te5650CheckSumOff(void);
short te5650CheckSumOn(void);
short te5650ClrProfile(void);
short te5650CurAxis(void);
short te5650CurAxisMask(void);
short te5650CurBoard(void);
double *te5650CvAcc(double *pdCv);
double *te5650CvJerk(double *pdCv);
double *te5650CvPos(double *pdCv);
double *te5650CvVel(double *pdCv);

short te5650DesPos(long *plPosition);
short te5650DesVel(long *plVelocity);
short te5650DisableIRQ(void);
short te5650DrivesOff(short sGroup);
short te5650DrivesOn(short sGroup);

short te5650EnableIRQ(unsigned short wIRQ);

short te5650FilterPID(unsigned short wKP, unsigned short wKI,
  unsigned short wKD, unsigned short wIL);
short te5650FilterPIVFF(unsigned short wKP, unsigned short wKI,
  unsigned short wKV, unsigned short wKVFF, unsigned short wIL);

short te5650GetAcc(long *plAcceleration);
short te5650GetBreakPoint(long *plPoint);
short te5650GetIntrMask(unsigned short *pwMask);
short te5650GetJerk(long *plJerk);
short te5650GetMaxAcc(long *plAcceleration);
short te5650GetMode(unsigned short *pwMode);
short te5650GetPID(unsigned short *pwKP, unsigned short *pwKI,
  unsigned short *pwKD, unsigned short *pwIL);
short te5650GetPIVFF(unsigned short *pwKP, unsigned short *pwKI,
  unsigned short *pwKV, unsigned short *pwKVFF, unsigned short *pwIL);
short te5650GetPos(long *plPosition);
short te5650GetPosErr(unsigned short *pwErr);
short te5650GetRatio(long *plRatio);
short te5650GetSampleTime(unsigned short *pwTime);
short te5650GetVel(long *plVelocity);
short te5650GetVersion(unsigned short *version);

short te5650HomePolarity(short sInvertHome);
short te5650HomeRead(void);
short te5650HookIntr(void (*IntrHook)(void));

short te5650InitServo(unsigned short wBoardAddr, short sType);
short te5650InitSw(void);
short te5650InputIsHigh(short sInputNum);
short te5650InputsRead(void);
short te5650IntrIsActive(void);
unsigned short te5650IntrAxis(void);
unsigned short te5650IntrBoard(void);
short te5650IsBreakPoint(void);
short te5650IsCapture(void);
short te5650IsComplete(void);
short te5650IsLimit(void);
short te5650IsNegLimit(void);
short te5650IsPosErr(void);
short te5650IsPosLimit(void);
short te5650ISum(short *psSum);
short te5650IsWrapAround(void);

short te5650LimitPolarity(unsigned short wActiveLowLimits);
short te5650Limits(unsigned short *pwLimits);
short te5650LimitsOff(void);
short te5650LimitsOn(void);

short te5650MaskIntr(unsigned short wIntrs);
short te5650MotorIsOn(void);
short te5650MotorOff(void);
short te5650MotorOn(void);

short te5650NotBusy(void);

short te5650OutputHigh(short sOutputNum);
short te5650OutputLow(short sOutputNum);
short te5650OutputsReadBack(void);
short te5650OutputsWrite(short sOutputs);

short te5650PhasesPolarity(short sInvertPhaseA, short sInvertPhaseB,
  short sInvertIndex);
short te5650PhasesRead(void);
short te5650PhasesSwapOff(void);
short te5650PhasesSwapOn(void);

short te5650ProfileSCurve(long lPosition, long lVelocity,
  long lAcceleration, long lJerk);
short te5650ProfileTrap(long lPosition, long lVelocity, long lAcceleration);
short te5650ProfileVel(long lVelocity, long lAcceleration);

short te5650ReadIntrs(void);
short te5650Recover(void);
short te5650RegisterServo(unsigned short wBoardAddr, short sType);
short te5650RegRead(short sReg, short sGroup);
short te5650RelPos(long lDeltaPos);
short te5650ResetBoard(void);
short te5650ResetCapture(void);
short te5650ResetEvents(unsigned short wEvents);
short te5650ResetLimits(void);
unsigned short te5650Resolution(void);

unsigned short te5650SampleTime(void);
short te5650SendCmd(unsigned short wCmd, short sNumWords,
  unsigned short *pwData, short sNotAxisCmd);
short te5650SetAcc(long lAcceleration);

short te5650SetAxis(unsigned short wNewBoard, unsigned short wNewAxis);
short te5650SetAxisMask(unsigned short wNewBoard, unsigned short wAxisMask);
short te5650SetBreakPoint(long lPoint, short sMode);
short te5650SetJerk(long lJerk);
short te5650SetMaxAcc(long lAcceleration);
short te5650SetMotor(unsigned short wValue);
short te5650SetPos(long lPosition);
short te5650SetPosErr(unsigned short wErr, unsigned short wAutoStopOn);
short te5650SetProfile(short sMode);
short te5650SetRatio(long lRatio);
short te5650SetResolution(unsigned short wResolution);
short te5650SetSampleTime(unsigned short wTime);
short te5650SetVel(long lVelocity);
short te5650Status(unsigned short *pwStatus);
short te5650SyncProfile(void);

short te5650Time(unsigned long *pdwTime);

short te5650UnmaskIntr(unsigned short wIntrs);
short te5650Update(void);

short te5650VelIsZero(void);

short te5650ZeroPos(void);
short te5650FilterPIDVFF(WORD wKP, WORD wKI, WORD wKD, WORD wKVFF, WORD wIL);
short te5650GetPIDVFF(WORD *pwKP, WORD *pwKI, WORD *pwKD, WORD *pwKVFF, WORD *pwIL);
short te5650GetMtrCmd(unsigned short *pData);
short te5650Stop(void);
short te5650SmoothStop(void);
short te5650SetMotorLimit(unsigned short MtrLmt);
short te5650GetMotorLimit(unsigned short *pData);
short te5650SetMotorBias(WORD MtrBias);
short te5650GetMotorBias(WORD *pData);
short te5650SetActPos(long lPoint);
short te5650SetMtnCmplBrk(void);
short te5650SetExtBrk(void);

short te5650Set3PhaseBrush(void);
short te5650Set2PhaseBrush(void);
short te5650SetPhaseHall(void);
short te5650SetPhaseAlgor(void);
short te5650SetPhaseCnts(unsigned short *pwPhaseCnts);
short te5650SetPhaseTime(unsigned short *pwPhaseTime);
short te5650SetPhaseOffset(unsigned short *pwPhaseOffset);
short te5650SetKPhase(unsigned short *pwKPhase);
short te5650SetHallSense(unsigned short *pwHallSense);
short te5650InitPhase(void);
short te5650GetPhaseInfo(unsigned short *pwPhaseInfo);
short te5650GetKPhase(unsigned short *pwKPhase);
short te5650GetPhase(unsigned short *pwPhase);
short te5650PrescaleOn(void);

short te5650PrescaleOff(void);
short te5650SetBufMtrCmd(unsigned short *pwBufMtrCmd);
short te5650GetBufMtrCmd(unsigned short *pwBufMtrCmd);
short te5650GetPhaseCnts(unsigned short *pwPhaseCnts);
short te5650GetPhaseTime(unsigned short *pwPhaseTime);
short te5650GetPhaseOffset(unsigned short *pwPhaseOffset);
short te5650GetHallSense(unsigned short *pwHallSense);
short te5650GetHalls(unsigned short *pwHalls);
short te5650CommutationAlgorithmic(unsigned short board_num, unsigned short axis_num,
 unsigned short  cnts_per_cycle,        unsigned short prescaler_on, unsigned short  phase_advance_gain,
 unsigned short  init_duration, unsigned short  init_power);
short te5650InitNoReset(unsigned short wBoardAddr, short sType);

void init_tech80(void);
void index_actuators(void);

void IntrHook();