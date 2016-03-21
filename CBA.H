/* cb8.h
   derived from original 2/18/2004 dmmtest.h
   
   VER 1.0
   12/18/2007 jtm
   
*/   
#define ADCPLEN 213
#define PCMLEN  120


void init_tech80(void);
void configure_obc(void);
void configure_escc(void);
void configure_enet(void);
void shutdown_enet(void);
void wait_for_time_tick(void);
void start_data_collection(void);
void send_data_packet(void);
void send_acoustic_message(void);
void apply_commands(void);
void message_adcp(void);
void configure_dmm32a(void);
void configure_dmm32b(void);
void trigger_dmm32a(void);
void trigger_dmm32b(void);
void interrupt isr_dmm32a(void);
void interrupt isr_dmm32b(void);
void interrupt sys_timer_isr(void);
void init_sys_timer(void);
void shutdown_sys_timer(void);
void terminate(void);
void shutdown_dmm32a(void);
void shutdown_dmm32b(void);
void shutdown_tech80(void);
void shutdown_escc(void);
void query_ln200(void);
unsigned char low_byte(unsigned int);
unsigned char high_byte(unsigned int);
void build_packet(void);
void init_screen(void);
void get_frame_time(void);
void prep_ln200_command(void);
void time_stamp(struct TIMER *);
void heartbeat(void);
void send_pcm(void);
void pack_pcm(void);
