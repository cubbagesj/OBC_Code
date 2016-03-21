/*
*********************************************************************************************************
*                                          PC SUPPORT FUNCTIONS
*
*                        (c) Copyright 1992-1998, Jean J. Labrosse, Plantation, FL
*                                           All Rights Reserved
*
* File : PC.C
* By   : Jean J. Labrosse
*********************************************************************************************************
*/

//#include "includes.h"
#include "pc.h"
#include <dos.h>

/*
*********************************************************************************************************
*                                               CONSTANTS
*********************************************************************************************************
*/
#define  DISP_BASE                  0xB800       /* Base segment of display (0xB800=VGA, 0xB000=Mono)  */
#define  DISP_MAX_X                     80       /* Maximum number of columns                          */
#define  DISP_MAX_Y                     25       /* Maximum number of rows                             */

#if 0

#define  TICK_T0_8254_CWR             0x43       /* 8254 PIT Control Word Register address.            */
#define  TICK_T0_8254_CTR0            0x40       /* 8254 PIT Timer 0 Register address.                 */
#define  TICK_T0_8254_CTR1            0x41       /* 8254 PIT Timer 1 Register address.                 */
#define  TICK_T0_8254_CTR2            0x42       /* 8254 PIT Timer 2 Register address.                 */

#define  TICK_T0_8254_CTR0_MODE3      0x36       /* 8254 PIT Binary Mode 3 for Counter 0 control word. */
#define  TICK_T0_8254_CTR2_MODE0      0xB0       /* 8254 PIT Binary Mode 0 for Counter 2 control word. */
#define  TICK_T0_8254_CTR2_LATCH      0x80       /* 8254 PIT Latch command control word                */

#define  VECT_TICK                    0x08       /* Vector number for 82C54 timer tick                 */
#define  VECT_DOS_CHAIN               0x81       /* Vector number used to chain DOS                    */

#endif

/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/
             
//static INT16U    PC_ElapsedOverhead;
//static jmp_buf   PC_JumpBuf;
//static BOOLEAN   PC_ExitFlag;
//void           (*PC_TickISR)(void);

/*$PAGE*/
/*
*********************************************************************************************************
*                           DISPLAY A SINGLE CHARACTER AT 'X' & 'Y' COORDINATE
*
* Description : This function writes a single character anywhere on the PC's screen.  This function
*               writes directly to video RAM instead of using the BIOS for speed reasons.  It assumed 
*               that the video adapter is VGA compatible.  Video RAM starts at absolute address 
*               0x000B8000.  Each character on the screen is composed of two bytes: the ASCII character 
*               to appear on the screen followed by a video attribute.  An attribute of 0x07 displays 
*               the character in WHITE with a black background.
*
* Arguments   : x      corresponds to the desired column on the screen.  Valid columns numbers are from
*                      0 to 79.  Column 0 corresponds to the leftmost column.
*               y      corresponds to the desired row on the screen.  Valid row numbers are from 0 to 24.
*                      Line 0 corresponds to the topmost row.
*               c      Is the ASCII character to display.  You can also specify a character with a 
*                      numeric value higher than 128.  In this case, special character based graphics
*                      will be displayed.
*               color  specifies the foreground/background color to use (see PC.H for available choices)
*                      and whether the character will blink or not.
*
* Returns     : None
*********************************************************************************************************
*/
void PC_DispChar (INT8U x, INT8U y, INT8U c, INT8U color)
{
    INT8U  far *pscr;
    INT16U      offset;


    offset  = (INT16U)y * DISP_MAX_X * 2 + (INT16U)x * 2;  /* Calculate position on the screen         */
    pscr    = (INT8U far *)MK_FP(DISP_BASE, offset);
    *pscr++ = c;                                           /* Put character in video RAM               */
    *pscr   = color;                                       /* Put video attribute in video RAM         */
}
/*$PAGE*/
/*
*********************************************************************************************************
*                                            CLEAR A COLUMN
*
* Description : This function clears one of the 80 columns on the PC's screen by directly accessing video 
*               RAM instead of using the BIOS.  It assumed that the video adapter is VGA compatible.  
*               Video RAM starts at absolute address 0x000B8000.  Each character on the screen is 
*               composed of two bytes: the ASCII character to appear on the screen followed by a video 
*               attribute.  An attribute of 0x07 displays the character in WHITE with a black background.
*
* Arguments   : x            corresponds to the desired column to clear.  Valid column numbers are from 
*                            0 to 79.  Column 0 corresponds to the leftmost column.
*
*               color        specifies the foreground/background color combination to use 
*                            (see PC.H for available choices)
*
* Returns     : None
*********************************************************************************************************
*/
void  __far __cdecl PC_DispClrCol (INT8U x, INT8U color)
{
    INT8U far *pscr;
    INT8U      i;


    pscr = (INT8U far *)MK_FP(DISP_BASE, (INT16U)x * 2);
    for (i = 0; i < DISP_MAX_Y; i++) {
        *pscr++ = ' ';                           /* Put ' ' character in video RAM                     */
        *pscr   = color;                         /* Put video attribute in video RAM                   */
        pscr    = pscr + DISP_MAX_X * 2;         /* Position on next row                               */
    }
}
/*$PAGE*/
/*
*********************************************************************************************************
*                                             CLEAR A ROW
*
* Description : This function clears one of the 25 lines on the PC's screen by directly accessing video 
*               RAM instead of using the BIOS.  It assumed that the video adapter is VGA compatible.  
*               Video RAM starts at absolute address 0x000B8000.  Each character on the screen is 
*               composed of two bytes: the ASCII character to appear on the screen followed by a video 
*               attribute.  An attribute of 0x07 displays the character in WHITE with a black background.
*
* Arguments   : y            corresponds to the desired row to clear.  Valid row numbers are from 
*                            0 to 24.  Row 0 corresponds to the topmost line.
*
*               color        specifies the foreground/background color combination to use 
*                            (see PC.H for available choices)
*
* Returns     : None
*********************************************************************************************************
*/
void  __far __cdecl PC_DispClrRow (INT8U y, INT8U color)
{
    INT8U far *pscr;
    INT8U      i;


    pscr = (INT8U far *)MK_FP(DISP_BASE, (INT16U)y * DISP_MAX_X * 2);
    for (i = 0; i < DISP_MAX_X; i++) {
        *pscr++ = ' ';                           /* Put ' ' character in video RAM                     */
        *pscr++ = color;                         /* Put video attribute in video RAM                   */
    }
}
/*$PAGE*/
/*
*********************************************************************************************************
*                                              CLEAR SCREEN
*
* Description : This function clears the PC's screen by directly accessing video RAM instead of using
*               the BIOS.  It assumed that the video adapter is VGA compatible.  Video RAM starts at
*               absolute address 0x000B8000.  Each character on the screen is composed of two bytes:
*               the ASCII character to appear on the screen followed by a video attribute.  An attribute
*               of 0x07 displays the character in WHITE with a black background.
*
* Arguments   : color   specifies the foreground/background color combination to use 
*                       (see PC.H for available choices)
*
* Returns     : None
*********************************************************************************************************
*/
void  __far __cdecl PC_DispClrScr (INT8U color)
{
    INT8U  far *pscr;
    INT16U      i;


    pscr = (INT8U far *)MK_FP(DISP_BASE, 0x0000);
    for (i = 0; i < (DISP_MAX_X * DISP_MAX_Y); i++) { /* PC display has 80 columns and 25 lines        */
        *pscr++ = ' ';                                /* Put ' ' character in video RAM                */
        *pscr++ = color;                              /* Put video attribute in video RAM              */
    }
}
/*$PAGE*/
/*
*********************************************************************************************************
*                                 DISPLAY A STRING  AT 'X' & 'Y' COORDINATE
*
* Description : This function writes an ASCII string anywhere on the PC's screen.  This function writes
*               directly to video RAM instead of using the BIOS for speed reasons.  It assumed that the 
*               video adapter is VGA compatible.  Video RAM starts at absolute address 0x000B8000.  Each 
*               character on the screen is composed of two bytes: the ASCII character to appear on the 
*               screen followed by a video attribute.  An attribute of 0x07 displays the character in 
*               WHITE with a black background.
*
* Arguments   : x      corresponds to the desired column on the screen.  Valid columns numbers are from
*                      0 to 79.  Column 0 corresponds to the leftmost column.
*               y      corresponds to the desired row on the screen.  Valid row numbers are from 0 to 24.
*                      Line 0 corresponds to the topmost row.
*               s      Is the ASCII string to display.  You can also specify a string containing 
*                      characters with numeric values higher than 128.  In this case, special character 
*                      based graphics will be displayed.
*               color  specifies the foreground/background color to use (see PC.H for available choices)
*                      and whether the characters will blink or not.
*
* Returns     : None
*********************************************************************************************************
*/
void  __far __cdecl PC_DispStr (INT8U x, INT8U y, INT8U *s, INT8U color)
{
    INT8U  far *pscr;
    INT16U      offset;


    offset  = (INT16U)y * DISP_MAX_X * 2 + (INT16U)x * 2;   /* Calculate position of 1st character     */
    pscr    = (INT8U far *)MK_FP(DISP_BASE, offset);
    while (*s) {
        *pscr++ = *s++;                                     /* Put character in video RAM              */
        *pscr++ = color;                                    /* Put video attribute in video RAM        */
    }
}

