// pktdrv.h

// packet driver interface classes
#define CL_NONE		0
#define CL_ETHERNET 1
#define CL_IEEE8023 11

// packet driver interface types
#define NE2000   54
#define ANYTYPE  0xffff

// packet driver function calls from appendix B (AH register)
#define DRIVER_INFO          1
#define ACCESS_TYPE          2
#define RELEASE_TYPE         3
#define SEND_PKT          4
#define TERMINATE            5
#define GET_ADDRESS          6
#define RESET_INTERFACE      7
#define GET_PARAMETERS      10
#define AS_SEND_PKT         11
#define SET_RCV_MODE        20
#define GET_RCV_MODE        21
#define SET_MULTICAST_LIST  22
#define GET_MULTICAST_LIST  23
#define GET_STATISTICS      24
#define SET_ADDRESS         25

//packet driver error return codes from appendix C
#define NO_ERROR             0
#define BAD_HANDLE           1 
#define NO_CLASS             2
#define NO_TYPE              3
#define NO_NUMBER            4
#define BAD_TYPE             5
#define NO_MULTICAST         6
#define CANT_TERMINATE       7
#define BAD_MODE             8
#define NO_SPACE             9
#define TYPE_INUSE          10
#define BAD_COMMAND         11
#define CANT_SEND           12
#define CANT_SET            13
#define BAD_ADDRESS         14
#define CANT_RESET          15

#define CARRY_FLAG 0x1
 
static char error_msg[16][15] = {"NO_ERROR",
                                 "BAD_HANDLE",
                                 "NO_CLASS",             
                                 "NO_TYPE",              
                                 "NO_NUMBER",            
                                 "BAD_TYPE",             
                                 "NO_MULTICAST",         
                                 "CANT_TERMINATE",       
                                 "BAD_MODE",             
                                 "NO_SPACE",             
                                 "TYPE_INUSE",          
                                 "BAD_COMMAND",         
                                 "CANT_SEND",           
                                 "CANT_SET",            
                                 "BAD_ADDRESS",         
                                 "CANT_RESET"};           