/* UIO48.C 

  Copyright 1996 by WinSystems Inc.

  Permission is hereby granted to the purchaser of the WinSystems
  UIO cards and CPU products incorporating the UIO device, to distribute
  any binary file or files compiled using this source code directly or
  in any work derived by the user from this file. In no case may the
  source code, original or derived from this file, be distributed to any
  third party except by explicit permission of WinSystems. This file is
  distributed on an "As-is" basis and no warranty as to performance,
  fitness of purposes, or any other warranty is expressed or implied.
  In no case shall WinSystems be liable for any direct or indirect loss
  or damage, real or consequential resulting from the usage of this
  source code. It is the user's sole responsibility to determine
  fitness for any considered purpose.
*/
/**************************************************************************
* Name  : uio48.c
*
* Project : PCM-UIO48 Software Samples/Examples
*
* Date  : October 30, 1996
*
* Revision: 1.00
*
* Author  : Steve Mottin
*
****************************************************************************
*
* Changes :
*
* Date    Revision  Description
* ________  ________  ______________________________________________
* 10/30/96  1.00    Created
*
*****************************************************************************
*/

#include <dos.h>
#include <conio.h>
#include "uio48.h"

#define outportb _outp
#define inportb  _inp

/* This global holds the base address of the UIO chip */

unsigned base_port;

/* This global array holds the image values of the last write to each I/O
   ports. This allows bit manipulation routines to work without having to
   actually do a read-modify-write to the I/O port.
*/

unsigned port_images[6];

/*===========================================================================
*                              INIT_IO
*
*  This function take a single argument :
*
*
*  io_address  :    This is the base I/O address of the 16C48 UIO Chip
*         on the board.
*
*
*  This function initializes all I/O pins for input, disables all interrupt
*  sensing, and sets the image values.
*
*===========================================================================*/

void init_io(unsigned io_address)
{
int x;

  /* Save the specified address for later use */

  base_port = io_address;

  /* Clear all of the I/O ports. This also makes them inputs */

  for(x=0; x < 7; x++)
    outportb(base_port+x, 0);

  /* Clear our image values as well */

  for(x=0; x < 6; x++)
    port_images[x] = 0;

  /* Set page 2 access, for interrupt enables */

  outportb(base_port+7,0x80);

  /* Clear all interrupt enables */
  
  outportb(base_port+8,0);
  outportb(base_port+9,0);
  outportb(base_port+0x0a,0);

  /* Restore normal page 0 register access */
  outportb(base_port+7,0);

}

/*===========================================================================
*
*                                READ_BIT
*
*
*  This function takes a single argument :
*
*
*  bit_number    :  The integer argument specifies the bit number to read.
*                   Valid arguments are from 1 to 48.
*
*  return value : The current state of the specified bit, 1 or 0.
*
*  This function returns the state of the current I/O pin specified by
*  the argument bit_number.
*
*===========================================================================*/

int read_bit(int bit_number)
{
unsigned port;
int val;

  /* Adjust the bit_number to 0 to 47 numbering */

  --bit_number;

  /* Calculate the I/O port address based on the updated bit_number */

  port = (bit_number / 8) + base_port;

  /* Get the current contents of the port */

  val = inportb(port);

  /* Get just the bit we specified */

  val = val & (1 << (bit_number % 8));
  
  /* Adjust the return for a 0 or 1 value */

  if(val)
    return 1;

  return 0;
}

/*===========================================================================
*
*                                WRITE_BIT
*
* This function takes two arguments :
*
*
*  bit_number :  The I/O pin to access is specified by bit_number 1 to 48.
*
*  val  :   The setting for the specified bit, either 1 or 0.
*
*  This function sets the specified I/O pin to either high or low as dictated
*  by the val argument. A non zero value for val sets the bit.
*
*===========================================================================*/
 
void write_bit(int bit_number, int val)
{
unsigned port;
unsigned temp;
unsigned mask;
 

  /* Adjust bit_number for 0 based numbering */

  --bit_number;

  /* Calculate the I/O address of the port based on the bit number */

  port = (bit_number / 8) + base_port;

  /* Use the image value to avoid having to read the port first. */

  temp = port_images[bit_number / 8]; /* Get current value */

  /* Calculate a bit mask for the specified bit */

  mask = (1 << (bit_number % 8));

  /* Check whether the request was to set or clear and mask accordingly */

  if(val)   /* If the bit is to be set */
    temp = temp | mask;
  else
    temp = temp & ~mask;
  
  /* Update the image value with the value we're about to write */

  port_images[bit_number / 8] = temp;

  /* Now actually update the port. Only the specified bit is affected */

  outportb(port,temp);
}

/*===========================================================================
*                                  SET_BIT
*
*
*  This function takes a single argument :
*
*  bit_number :  The bit number to set.
*
*  This function sets the specified bit.
*
*===========================================================================*/     

void set_bit(int bit_number)
{

  write_bit(bit_number,1);
}

/*===========================================================================
*                                  CLR_BIT
*
*
*  This function takes a single argument :
*
*  bit_number :  The bit number to clear.
*
*  This function clears the specified bit.
*
*===========================================================================*/     

void clr_bit(int bit_number)
{
  write_bit(bit_number,0);
}

/*===========================================================================
*
*                               ENAB_INT
*
*  This function takes two arguments :
*
*  bit_number  : The bit number to enable intterups for. Range from 1 to 48.
*
*  polarity   : This specifies the polarity of the interrupt. A non-zero
*               argument enables rising-edge interrupt. A zero argument
*               enables the interrupt on the flling edge.
*
*  This function enables within the 16C48 an interrupt for the specified bit
*  at the specified polarity. This function does not setup the interrupt 
*  controller, nor does it supply an interrupr handler.
*
*============================================================================*/

void enab_int(int bit_number, int polarity)
{
unsigned port;
unsigned temp;
unsigned mask;

  /* Adjust for 0 based numbering */ 
  
  --bit_number;

  /* Calculate the I/O address based uppon the bit number */

  port = (bit_number / 8) + base_port + 8;

  /* Calculate a bit mask based on the specified bit number */

  mask = (1 << (bit_number % 8));

  /* Turn on page 2 access */

  outportb(base_port+7,0x80);

  /* Get the current state of the interrupt enable register */

  temp = inportb(port);

  /* Set the enable bit for our bit number */

  temp = temp | mask;

  /* Now update the interrupt enable register */

  outportb(port,temp);

  /* Turn on access to page 1 for polarity control */

  outportb(base_port+7,0x40);

  /* Get the current state of the polarity register */

  temp = inportb(port);   /* Get current polarity settings */

  /* Set the polarity according to the argument in the image value */

  if(polarity)    /* If the bit is to be set */
    temp = temp | mask;
  else
    temp = temp & ~mask;
  
  /* Write out the new polarity value */

  outportb(port,temp);

  /* Set access back to Page 0 */

  outportb(base_port+7,0x0);

}

/*===========================================================================
*
*                               DISAB_INT
*
* This function takes a single argument :
*
* bit_number : Specifies the bit number to act upon. Range is from 1 to 48.
*
* This function shuts off the interrupt enabled for the specified bit.
*
*===========================================================================*/
      
void disab_int(int bit_number)
{
unsigned port;
unsigned temp;
unsigned mask;

  /* Adjust the bit_number for 0 based numbering */
 
  --bit_number;

  /* Calculate the I/O Address for the enable port */

  port = (bit_number / 8) + base_port + 8;

  /* Calculate the proper bit mask for this bit number */

  mask = (1 << (bit_number % 8));

  /* Turn on access to page 2 registers */

  outportb(base_port+7,0x80);

  /* Get the current state of the enable register */

  temp = inportb(port);

  /* Clear the enable bit int the image for our bit number */

  temp = temp & ~mask;
  
  /* Update the enable register with the new information */

  outportb(port,temp);

  /* Set access back to page 0 */

  outportb(base_port+7,0x0);

}

/*==========================================================================
*
*                                CLR_INT
*
*  This function takes a single argument :
*
*  bit_number : This argument specifies the bit interrupt to clear. Range
*               is 1 to 24.
*
*
*  This function is use to clear a bit interrupt once it has been recognized.
*  The interrupt left enabled.
*
*===========================================================================*/

void clr_int(int bit_number)
{
unsigned port;
unsigned temp;
unsigned mask;
 

  /* Adjust for 0 based numbering */

  --bit_number;

  /* Calculate the correct I/O address for our enable register */

  port = (bit_number / 8) + base_port + 8;

  /* Calculate a bit mask for this bit number */

  mask = (1 << (bit_number % 8));

  /* Set access to page 2 for the enable register */

  outportb(base_port+7,0x80);

  /* Get current state of the enable register */

  temp = inportb(port);

  /* Temporarily clear only OUR enable. This clears the interrupt */

  temp = temp & ~mask;    /* clear the enable for this bit */
  
  /* Write out the temporary value */ 

  outportb(port,temp);

  /* Re-enable our interrupt bit */

  temp = temp | mask;

  /* Write it out */

  outportb(port,temp);

  /* Set access back to page 0 */

  outportb(base_port+7,0x0);

}


/*==========================================================================
*
*                                GET_INT
*
*  This function take no arguments.
*
*  return value : The value returned is the highest level bit interrupt
*                 currently pending. Range is 1 to 24.
*
*  This function returns the highest level interrupt pending. If no interrupt
*  is pending, a zero is returned. This function does NOT clear the interrupt.
*
*===========================================================================*/
 
int get_int(void)
{
int temp;
int x;

  /* read the master interrupt pending register, mask off undefined bits */

  temp = inportb(base_port+6) & 0x07;

  /* If there are no interrupts pending, return a 0 */

  if((temp & 7) == 0)
    return(0);

  /* There is something pending, now we need to identify what it is */

  
  /* Set access to page 3 for interrupt id registers */

  outportb(base_port+7,0xc0);

  /* Read interrupt ID register for port 0 */

  temp = inportb(base_port+8);


  /* See if any bit set, if so return the bit number */

  if(temp !=0)
  {
    for(x=0; x <=7; x++)
    {
      if(temp & (1 << x))
      {
        outportb(base_port+7,0);  /* Turn off access */
        return(x+1);    /* Return bitnumber with active int */
      }
    }
  }

  /* None in Port 0, read port 1 interrupt ID register */

  temp = inportb(base_port+9);

  /* See if any bit set, if so return the bit number */

  if(temp !=0)
  {
    for(x=0; x <=7; x++)
    {
      if(temp & (1 << x))
      {
        outportb(base_port+7,0);  /* Turn off access */
        return(x+9);    /* Return bitnumber with active int */
      }
    }
  }

  /* Lastly, read status of port 2 int id */
  
  temp = inportb(base_port+0x0a);    /* Read port 2 status */

  /* If any pending, return the appropriate bit number */

  if(temp !=0)
  {
    for(x=0; x <=7; x++)
    {
      if(temp & (1 << x))
      {
        outportb(base_port+7,0);  /* Turn off access */
        return(x+17);   /* Return bitnumber with active int */
      }
    }
  }

  /* We should never get here unless the hardware is misbehaving but just
      to be sure. We'll turn the page access back to 0 and return a 0 for
      no interrupt found.
  */


  outportb(base_port+7,0);
  return 0;
}


unsigned read_port(int port_number)
  {
  return inportb(port_number + base_port) & 0x00ff;
  }
