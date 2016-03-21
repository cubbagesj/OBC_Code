#define FRAME_SIZE 1024  //4096 //maximum size of a received frame if you are memory concious change to 4090 (will make buffer structures = 4k and will not waste as much mem)
#define MAX_RBUFS 16 //25      //maximum number of receive buffers allowed
#define MAX_TBUFS 16 //25      //maximum number of transmit buffers allowed
#define MAX_PORTS 2 //6       //maximum number of ports allowed
//CMDR commands
#define XRES 1
#define XME 2
#define XIF 4
#define HUNT 4
#define XTF 8
#define STI 16
#define RNR 32
#define RHR 64
#define RMC 128
//ISR0 codes
#define RME 128
#define RFS 64
#define RSC 32
#define PCE 16
#define PLLA 8
#define CDSC 4
#define RFO 2
#define RPF 1
//ISR1 codes 
#define EOP 128
#define OLP 64
#define ALLS 32
#define EXE 16
#define TIN 8
#define CSC 4
#define XMR 2
#define XPR 1
//STAR codes
#define XDOV 128
#define XFW 64
#define XRNR 32
#define RRNR 16
#define RLI 8
#define CEC 4
#define CTS 2
#define WFA 1             
             
//escc registers the same for all modes
#define FIFO 0x20
#define STAR 0x00
#define CMDR 0x00
#define MODE 0x02
#define TIMR 0x03
#define XBCL 0x0a
#define RBCL 0x0a
#define XBCH 0x0b
#define RBCH 0x0b
#define CCR0 0x0c
#define CCR1 0x0d
#define CCR2 0x0e
#define CCR3 0x0f
#define CCR4 0x1f         //added for biphase-M 5/18/98
#define VSTR 0x14
#define BGR  0x14
#define GIS  0x18
#define IVA  0x18
#define IPC  0x19
#define ISR0 0x1a
#define IMR0 0x1a
#define ISR1 0x1b             
#define IMR1 0x1b
#define PVR  0x1c
#define PIS  0x1d
#define PIM  0x1d
#define PCR  0x1e

//escc register defines for HDLC/SDLC mode
#define RSTA 0x01
#define XAD1 0x04
#define XAD2 0x05
#define RAH1 0x06
#define RAH2 0x07
#define RAL1 0x08
#define RAL2 0x09
#define RHCR 0x09
#define RLCR 0x15
#define PRE  0x01 //used in bisync as well

//escc async register defines (used in bisync as well)
#define TCR 0x06
#define DAFO 0x07
#define RFC 0x08

//escc bisync register defines
#define SYNL 0x04
#define SYNH 0x05


             
//dma controller defines:
#define DMA_PAGE_CH0    0x87
#define DMA_PAGE_CH1    0x83
#define DMA_PAGE_CH2    0x81
#define DMA_PAGE_CH3    0x82
#define DMA_PAGE_CH5    0x8b
#define DMA_PAGE_CH6    0x89
#define DMA_PAGE_CH7    0x8a

#define DMA_BASE_LO     0x00
#define DMA_COMMAND_LO  0x08
#define DMA_MODE_LO     0x0b
#define DMA_REQUEST_LO  0x09
#define DMA_MASK_LO     0x0a
#define DMA_STATUS_LO   0x08
#define DMA_CLEAR_FFLO  0x0c
#define DMA_ADD_CH1     0x02
#define DMA_COUNT_CH1   0x03
#define DMA_ADD_CH3     0x06
#define DMA_COUNT_CH3   0x07

#define DMA_BASE_HI     0xc0
#define DMA_COMMAND_HI  0xd0
#define DMA_MODE_HI     0xd6
#define DMA_REQUEST_HI  0xd2
#define DMA_MASK_HI     0xd4
#define DMA_STATUS_HI   0xd0
#define DMA_CLEAR_FFHI  0xd8
#define DMA_ADD_CH5     0xc4
#define DMA_COUNT_CH5   0xc6
#define DMA_ADD_CH6     0xc8
#define DMA_COUNT_CH6   0xca
#define DMA_ADD_CH7     0xcc
#define DMA_COUNT_CH7   0xce
      
//status function defines
#define XMR_INTERRUPT   0x0001
#define EXE_INTERRUPT   0x0002
#define PCE_INTERRUPT   0x0004
#define RFO_INTERRUPT   0x0008
#define CTS_INTERRUPT   0x0010
#define RFS_INTERRUPT   0x0020
#define RX_BUFFER_OVERFLOW 0x0040
#define RSC_INTERRUPT   0x0080
#define TIMER_INTERRUPT 0x0100
#define RX_READY        0x0200
#define PARITY_ERROR  0x0400
#define FRAMING_ERROR 0x0800
#define SYN_DETECTED  0x0800
#define BREAK_DETECTED  0x1000
#define BREAK_TERMINATED 0x2000
#define ALLSENT_INTERRUPT 0x4000
#define REC_TIMEOUT 0x8000
#define DPLLA_DETECTED  0x0800

#define TRUE 0                                                                  
#define FALSE 1

#define CHANNEL0 0
#define CHANNEL1 1

#define OPMODE_HDLC   0
#define OPMODE_ASYNC  1
#define OPMODE_BISYNC   2

#define AUTO_MODE 0
#define TRANSPARENT_MODE 1

//defines for 2053b
#define STARTWRD 0x1e05
#define MIDWRD   0x1e04
#define ENDWRD   0x1e00


struct escc_regs{
unsigned cmdr;
unsigned mode;
unsigned timr;
unsigned xbcl;
unsigned xbch;
unsigned ccr0;
unsigned ccr1;
unsigned ccr2;
unsigned ccr3;
unsigned ccr4;       //added for biphase-M 5/18/98
unsigned bgr;
unsigned iva;
unsigned ipc;
unsigned imr0;
unsigned imr1;
unsigned pvr;
unsigned pim;
unsigned pcr;
//escc register defines for HDLC/SDLC mode
unsigned xad1;
unsigned xad2;
unsigned rah1;
unsigned rah2;
unsigned ral1;
unsigned ral2;
unsigned rlcr;
unsigned pre;
//escc async register defines (used in bisync as well)
unsigned tcr;
unsigned dafo;
unsigned rfc;
//escc bisync register defines
unsigned synl;
unsigned synh;
};


struct buf{
  unsigned valid;         //indicator 1 = frame[] has data, 0 = frame[] has ???
  unsigned no_bytes;      //number of bytes in frame[]
  unsigned max;           //maximum number of bytes to send/receive
  char frame[FRAME_SIZE];  //data array for received/transmitted data
};


class __far Cescc
{

protected:
//protected variables
unsigned port_list[MAX_PORTS];              //base address list
unsigned port_open_list[MAX_PORTS];         //port has been inited list
unsigned interrupt_list[MAX_PORTS];         //ports associated interrupt level (hardware)
unsigned port_dmat_list[MAX_PORTS];
unsigned port_dmar_list[MAX_PORTS];
unsigned hooked_irqs[16];           //list of irq vectors that are hooked (by number)
void (interrupt far *old_service_routines[16])();//hooked interrupt service vectors previous routines
//void interrupt (far *old_service_routines[16])(...);
unsigned next_port;               //holds the next port to be used (added)
unsigned next_irq;                //holds the next irq to be added
unsigned upper_irq;               //flag for ISR to send upper EOI if irq >8 is being used
struct buf huge *rxbuffer[MAX_PORTS][MAX_RBUFS];              //array of pointers for receive buffering
struct buf huge *txbuffer[MAX_PORTS][MAX_TBUFS];              //array of pointers for transmitt buffering
unsigned current_rxbuf[MAX_PORTS];
unsigned current_txbuf[MAX_PORTS];
unsigned max_rxbuf[MAX_PORTS];
unsigned max_txbuf[MAX_PORTS];
unsigned timer_status[MAX_PORTS];
unsigned tx_type[MAX_PORTS];
unsigned istxing[MAX_PORTS];  //==1 if a frame is being sent ,==0 if no txing is going on
unsigned port_status[MAX_PORTS];
unsigned channel[MAX_PORTS];
unsigned eopmode[MAX_PORTS];


public:
//public variables
Cescc(); 
~Cescc();


// Operations
public:
//user callable functions
unsigned wait_for_tx(unsigned port);
void set_clock_generator(unsigned port, unsigned long hval,unsigned nmbits);
unsigned add_port(unsigned base, unsigned chan, unsigned irq, unsigned dmar, unsigned dmat);//return port# (index into port..arrays)
unsigned kill_port(unsigned port);//true ==success
unsigned init_port( unsigned port,
          unsigned opmode,
          struct escc_regs *esccregs,
          unsigned rbufs,
          unsigned tbufs);//true ==success
unsigned rx_port(unsigned port, char far *buf, unsigned num_bytes);//returns # bytes transfered, 0 if fails
unsigned tx_port(unsigned port, char far *buf, unsigned num_bytes);//returns # bytes transfered,0 if fails
unsigned set_control_lines(unsigned port,unsigned dtr, unsigned rts);//sets or clears dtr/rts 1 = set 0 = clear
unsigned get_control_lines(unsigned port);//return  = bit flags  X X X DSR DTR DCD CTS RTS 
unsigned get_port_status(unsigned port);//returns status flag consisting of one or more of the following
                          //XMR_INTERRUPT   0x0001
                          //EXE_INTERRUPT   0x0002
                          //PCE_INTERRUPT   0x0004
                          //RFO_INTERRUPT   0x0008
                          //CTS_INTERRUPT   0x0010
                          //RFS_INTERRUPT   0x0020
                          //RX_BUFFER_OVERFLOW 0x0040
                          //RSC_INTERRUPT   0x0080
                          //TIMER_INTERRUPT 0x0100
                          //RX_READY        0x0200

unsigned clear_rx_buffer(unsigned port); //False if port not open
unsigned clear_tx_buffer(unsigned port); //False if port not open
unsigned start_timer(unsigned port); //False if port not open
unsigned is_timer_expired(unsigned port); //False if port not open
unsigned wait_for_timer_expired(unsigned port); //False if port not open
unsigned stop_timer(unsigned port);//Fals if port not open

//hdlc/sdlc specific
unsigned set_tx_type(unsigned port,unsigned type); //False if port not open
unsigned set_tx_address(unsigned port , unsigned address); //False if port not open
unsigned set_rx_address1(unsigned port,unsigned address); //False if port not open
unsigned set_rx_address2(unsigned port,unsigned address); //False if port not open


// Implementation
protected:
//class defined functions 
void cdecl interrupt far escc_isr(void);
//void interrupt far escc_isr(void);
//void interrupt far (*escc_isr_ptr)(void);
void setupdmar(unsigned port,void far *address);
void setupdmat(unsigned port,void far *address,unsigned numbytes);
unsigned getdmarcount(unsigned port);
};

