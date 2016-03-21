// ether.h


#define PKTDVR 0x7e
#define MAX_RXBUF 1520
#define MAX_RX_LENGTH 1520
#define UDP_SOURCE_PORT 4951
#define UDP_DEST_PORT 4950
#define UDP_LENGTH 1480

void far interrupt receiver(unsigned, unsigned, unsigned, unsigned,
                        unsigned, unsigned, unsigned, unsigned,
                        unsigned, unsigned, unsigned, unsigned,
                        unsigned);
int driver_info(unsigned);
int access_type(int, int, int, char far *, unsigned,
                     void (far interrupt *receiver)(unsigned, unsigned, unsigned, unsigned,
                     unsigned, unsigned, unsigned, unsigned, unsigned,
                     unsigned, unsigned, unsigned, unsigned));
int release_type(int);
int send_pkt(unsigned char far *, unsigned);
int terminate(int);
int get_address(int, unsigned char far *, int);
int reset_interface(int);
int get_parameters(void);
//int as_send_packet(char far *, unsigned, int (far *)());
int as_send_pkt(unsigned char far *, unsigned, void (far *)(char far *, int));
void upcall(char far *buffer, int result);
int set_rcv_mode(int, int);
int get_rcv_mode(int);
int set_multicast_list(unsigned char far *, int);
int get_multicast_list(void);
int get_statistics(int);
int set_address(char far *, int);
void print_error(int);
void wait_for_sync_high(void);
void wait_for_sync_low(void);
void prompt_user(void);
void prompt_user(void);
void do_function(void);
void print_stats(void);
void build_packet(void);
unsigned char low_byte(unsigned int);
unsigned char high_byte(unsigned int);

