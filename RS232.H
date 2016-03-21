// rs232.h

//#define PI (3.14159)

#define COM_1     0x3f8     /* 8250 Port1 */
#define COM1_RX   COM_1     /*RECEIVER PORT */
#define COM1_TX   COM_1     /*TRANSMITTER PORT*/
#define COM1_IER  COM_1+1   /*INTERRUPT ENABLE REGISTER*/
#define COM1_IIR  COM_1+2   /*INTERRUPT IDENTIFICATION REGISTER*/
#define COM1_LCR  COM_1+3   /*LINE CONTROL REGISTER*/
#define COM1_MCR  COM_1+4   /*MODEM CONTROL REGISTER*/
#define COM1_LSR  COM_1+5   /*LINE STATUS REGISTER*/
#define COM1_MSR  COM_1+6   /*MODEM STATUS REGISTER*/
#define COM1_SCR  COM_1+7   /*SCRATCH REGISTER*/
#define COM1_DLL  COM_1     /*DIVISOR LATCH(LSB) DLAB OF LCR (BIT7=1)*/
#define COM1_DLM  COM_1+1   /*DIVISOR LATCH(MSB) DLAB OF LCR (BIT7=1)*/
#define COM1_FCR  COM_1+2   //FIFO CONTROL REGISTER

#define COM_2     0x2f8     /* 8250 Port2 */
#define COM2_RX   COM_2     /*RECEIVER PORT */
#define COM2_TX   COM_2     /*TRANSMITTER PORT*/
#define COM2_IER  COM_2+1   /*INTERRUPT ENABLE REGISTER*/
#define COM2_IIR  COM_2+2   /*INTERRUPT IDENTIFICATION REGISTER*/
#define COM2_LCR  COM_2+3   /*LINE CONTROL REGISTER*/
#define COM2_MCR  COM_2+4   /*MODEM CONTROL REGISTER*/
#define COM2_LSR  COM_2+5   /*LINE STATUS REGISTER*/
#define COM2_MSR  COM_2+6   /*MODEM STATUS REGISTER*/
#define COM2_SCR  COM_2+7   /*SCRATCH REGISTER*/
#define COM2_DLL  COM_2     /*DIVISOR LATCH(LSB) DLAB OF LCR (BIT7=1)*/
#define COM2_DLM  COM_2+1   /*DIVISOR LATCH(MSB) DLAB OF LCR (BIT7=1)*/
#define COM2_FCR  COM_2+2   //FIFO CONTROL REGISTER

#define COM_4     0x2e8     /* 8250 Port4 */
#define COM4_RX   COM_4     /*RECEIVER PORT */
#define COM4_TX   COM_4     /*TRANSMITTER PORT*/
#define COM4_IER  COM_4+1   /*INTERRUPT ENABLE REGISTER*/
#define COM4_IIR  COM_4+2   /*INTERRUPT IDENTIFICATION REGISTER*/
#define COM4_LCR  COM_4+3   /*LINE CONTROL REGISTER*/
#define COM4_MCR  COM_4+4   /*MODEM CONTROL REGISTER*/
#define COM4_LSR  COM_4+5   /*LINE STATUS REGISTER*/
#define COM4_MSR  COM_4+6   /*MODEM STATUS REGISTER*/
#define COM4_SCR  COM_4+7   /*SCRATCH REGISTER*/
#define COM4_DLL  COM_4     /*DIVISOR LATCH(LSB) DLAB OF LCR (BIT7=1)*/
#define COM4_DLM  COM_4+1   /*DIVISOR LATCH(MSB) DLAB OF LCR (BIT7=1)*/
#define COM4_FCR  COM_4+2   //FIFO CONTROL REGISTER

#define EIGHT 0x03           /* eight bits per char*/
#define NO 0x00              /* no parity */
#define STOP1 0x00           /* one stop bit */
#define TXB_EMPTY 0x60       /* transmit buffer empty */
#define RX_DATA 0x01         /* receive data available */
#define DLAB1 0x80           /* divisor latch access bit enabled */
#define DLAB0 0x00           /* divisor latch access bit disable */
#define BR_115200 0x01       /* divisor value for 115200 baud */
#define BR_57600  0x02 
#define BR_19200  0x06
#define BR_9600 0x0c         /* divisor value for 9600 baud */
#define BR_4800 0x18         /* divisor value for 4800 baud */
#define BR_2400 0x30         /* divisor value for 2400 baud */
#define ERBFI 0x01           /* enable received data available interrupt */
#define OUT2 0x08            /* OUT2 - Master interrupt control bit */

#define PIC_BASE 0x20
#define PIC_ICR PIC_BASE
#define EOI 0x20
#define COM1_VECTOR 0x0c     /* com1 interrupt vector, irq4 */
#define COM2_VECTOR 0x0b     /* com2 interrupt vector, irq3 */
#define COM1_MASK 0x10       /* com1 interrupt mask, irq4 */
#define COM2_MASK 0x08       /* com2 interrupt mask, irq3 */
#define COM4_MASK 0x02       /* com4 interrupt mask, irq9 */

void init_com1(void);
void init_com2(void);
void init_com4(void);
void shutdown_com1(void);
void shutdown_com2(void);
void shutdown_com4(void);
void interrupt far com1_isr(void);
void interrupt far com2_isr(void);
void interrupt far com4_isr(void);

