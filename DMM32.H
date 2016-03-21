// dmm32.h
// defines and function prototypes
// for the Diamond Systems DMM32-AT board

  // base addresses for the two boards
#define OBS_BASE  0x340
#define DYNO_BASE 0x380

  // register offsets
#define ADLSB    0
#define ADMSB    1
#define ADO      1
#define LCR      2
#define HCR      3
#define DACLSB   4
#define STATUS   4
#define ADI      4
#define DACMSB   5
#define DAC_CHAN 5
#define UPD_DAC  5
#define FDR      6
#define FCR      7
#define FSR      7
#define MCR      8
#define ASR      8
#define ICR      9
#define ACLKCR   9
#define ISR      9
#define ACLKSR   9
#define CTCCR   10
#define DCR     10
#define ACR     11
#define ARR     11
#define PORTA   12
#define PORTB   13
#define PORTC   14
#define CCR     15

  // bit defines
#define DACBUSY 0x80
#define CALBUSY 0x40
#define FIFOEN  0x08
#define SCANEN  0x04
#define FIFORST 0x02
#define EF      0x80
#define HF      0x40
#define FF      0x20
#define OVF     0x10
#define RESETA  0x20
#define RESETD  0x10
#define INTRST  0x08
#define STS     0x80
#define SD1     0x40
#define SD0     0x20
#define ADINTE  0x80
#define DINTE   0x40
#define TINTE   0x20
#define CLKEN   0x02
#define CLKSEL  0x01
#define ADINT   0x80
#define DINT    0x40
#define TINT    0x20
#define PAGE0   0x00
#define PAGE1   0x01
#define PAGE2   0x02

#define USEC20  0x00
#define USEC15  0x10
#define USEC10  0x20
#define USEC5   0x30
#define KHZ200  USEC5
#define KHZ100  USEC10
#define KHZ67   USEC15
#define KHZ50   USEC20

#define BIPOLAR5   0x00
#define BIPOLAR10  0x08
#define UNIPOLAR10 0x0c

#define GAIN1  0x00
#define GAIN2  0x01
#define GAIN4  0x02
#define GAIN8  0x03

#define WAIT   0x80

// DAC channel defines
#define STERN1_CHAN 0x00
#define STERN2_CHAN 0x40
#define FORE_CHAN   0xc0
#define RUDDER_CHAN 0x80
#define PROP_CHAN   0x00

// 82C54 defines
#define MHZ10  0x00
#define CTC0   0x00
#define LSBMSB 0x30
#define MODE0  0x00
#define MODE1  0x02
#define MODE2  0x04
#define MODE3  0x06
#define MODE4  0x08
#define MODE5  0x0a
#define BINARY 0x00
#define BCD    0x01