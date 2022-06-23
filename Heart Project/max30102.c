/** \file max30102.cpp ******************************************************
*
* Project: MAXREFDES117#
* Filename: max30102.cpp
* Description: This module is an embedded controller driver for the MAX30102
*
* Revision History:
*\n 1-18-2016 Rev 01.00 GL Initial release.
*\n
*
* --------------------------------------------------------------------
*
* This code follows the following naming conventions:
*
* char              ch_pmod_value
* char (array)      s_pmod_s_string[16]
* float             f_pmod_value
* int32_t           n_pmod_value
* int32_t (array)   an_pmod_value[16]
* int16_t           w_pmod_value
* int16_t (array)   aw_pmod_value[16]
* uint16_t          uw_pmod_value
* uint16_t (array)  auw_pmod_value[16]
* uint8_t           uch_pmod_value
* uint8_t (array)   auch_pmod_buffer[16]
* uint32_t          un_pmod_value
* int32_t *         pn_pmod_value
*
* ------------------------------------------------------------------------- */
/*******************************************************************************
* Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*
* Except as contained in this notice, the name of Maxim Integrated
* Products, Inc. shall not be used except as stated in the Maxim Integrated
* Products, Inc. Branding Policy.
*
* The mere transfer of this software does not imply any licenses
* of trade secrets, proprietary technology, copyrights, patents,
* trademarks, maskwork rights, or any other form of intellectual
* property whatsoever. Maxim Integrated Products, Inc. retains all
* ownership rights.
*******************************************************************************
*/
#include "max30102.h"
//#include "algorithm.h"
#include "i2c_if.h"
#include <stdlib.h>
#include <string.h>
//#include "softI2C.h"
//#include "stdbool.h"

//Just write to specific register on device
//uch_addr    - register address
//uch_data    - register data
int maxim_max30102_write_reg(uint8_t uch_addr, uint8_t uch_data)
{
    unsigned char aucDataBuf[256];
    memset(aucDataBuf, '\0', 256);
    //got device address
    unsigned char ucDevAddr = DEV_ADDR;

    //get offset address
    unsigned char ucRegOffset = uch_addr;

    aucDataBuf[0] = ucRegOffset;
    aucDataBuf[1] = (unsigned char) uch_data;

    //Assuming Device address is #define I2C_WRITE_ADDR 0xAE
    if(I2C_IF_Write(ucDevAddr, &aucDataBuf[0], 2, 1) != 0)
        return -1;
    else
        return 0;
}

//Just read from specified register
//uch_addr    -register to read
//puch_data   -buffer to hold data
int maxim_max30102_read_reg(uint8_t uch_addr, uint8_t *puch_data)
{
    //got device address
    unsigned char ucDevAddr = DEV_ADDR;

    //get offset address
    unsigned char ucRegOffset = uch_addr;

    //len of data to be read (1 byte)
    unsigned char ucRdLen = 1;

    if(I2C_IF_Write(ucDevAddr,&ucRegOffset,1,0) != 0)
        return -1;

    if(I2C_IF_Read(ucDevAddr, puch_data, ucRdLen) != 0)
        return -1;

    return 0;
}

int maxim_max30102_init()
{
  if(maxim_max30102_write_reg(REG_INTR_ENABLE_1,0xc0) != 0) // INTR setting
    return -1;
  if(maxim_max30102_write_reg(REG_INTR_ENABLE_2,0x00) != 0)
    return -1;
  if(maxim_max30102_write_reg(REG_FIFO_WR_PTR,0x00) != 0)  //FIFO_WR_PTR[4:0]
    return -1;
  if(maxim_max30102_write_reg(REG_OVF_COUNTER,0x00) != 0)  //OVF_COUNTER[4:0]
    return -1;
  if(maxim_max30102_write_reg(REG_FIFO_RD_PTR,0x00) != 0)  //FIFO_RD_PTR[4:0]
    return -1;
  if(maxim_max30102_write_reg(REG_FIFO_CONFIG,0x4f) != 0)  //sample avg = 4, fifo rollover=false, fifo almost full = 17
    return -1;
  if(maxim_max30102_write_reg(REG_MODE_CONFIG,0x02) != 0)   //0x02 for Red only, 0x03 for SpO2 mode 0x07 multimode LED
    return -1;
  if(maxim_max30102_write_reg(REG_SPO2_CONFIG,0x27) != 0)  // SPO2_ADC range = 4096nA, SPO2 sample rate (100 Hz), LED pulseWidth (411uS)
    return -1;

  if(maxim_max30102_write_reg(REG_LED1_PA,0x24) != 0)   //Choose value for ~ 7mA for LED1
    return -1;
  if(maxim_max30102_write_reg(REG_LED2_PA,0x24) != 0)   // Choose value for ~ 7mA for LED2
    return -1;
  if(maxim_max30102_write_reg(REG_PILOT_PA,0x7f) != 0)   // Choose value for ~ 25mA for Pilot LED
    return -1;
  
  return 0;
}


int maxim_max30102_read_fifo(uint32_t *pun_red_led, uint32_t *pun_ir_led)
{
//  uint32_t un_temp;
//  uint8_t uch_temp;
//
//  unsigned char tempByte;
//
//  *pun_ir_led=0;
//  *pun_red_led=0;
//
//
//  if(maxim_max30102_read_reg(REG_INTR_STATUS_1, &uch_temp) != 0)
//      Message("Error Read INT1");
//  if(maxim_max30102_read_reg(REG_INTR_STATUS_2, &uch_temp) != 0)
//      Message("Error Read INT2");
//
//  if(maxim_max30102_read_reg(REG_FIFO_DATA , (uint8_t*)&un_temp) != 0)
//      Message("Error Read FIFO");
//  un_temp<<=16;
//  *pun_red_led+=un_temp;
//
//
//  if(maxim_max30102_read_reg(REG_FIFO_DATA , (uint8_t*)&un_temp) != 0)
//      Message("Error Read FIFO");
//  un_temp<<=8;
//  *pun_red_led+=un_temp;
//
//
//  if(maxim_max30102_read_reg(REG_FIFO_DATA , (uint8_t*)&un_temp) != 0)
//      Message("Error Read FIFO");
//  *pun_red_led+=un_temp;
//
//
//  if(maxim_max30102_read_reg(REG_FIFO_DATA , (uint8_t*)&un_temp) != 0)
//      Message("Error Read FIFO");
//  un_temp<<=16;
//  *pun_ir_led+=un_temp;
//
//
//  if(maxim_max30102_read_reg(REG_FIFO_DATA , (uint8_t*)&un_temp) != 0)
//      Message("Error Read FIFO");
//  un_temp<<=8;
//  *pun_ir_led+=un_temp;
//
//
//  if(maxim_max30102_read_reg(REG_FIFO_DATA , (uint8_t*)&un_temp) != 0)
//      Message("Error Read FIFO");
//  *pun_ir_led+=un_temp;
//
//
//  *pun_red_led&=0x03FFFF;  //Mask MSB [23:18]
//  *pun_ir_led&=0x03FFFF;  //Mask MSB [23:18]
//  return 0;
    unsigned char ucRegOffset = REG_FIFO_DATA;
    unsigned char ucDevAddr = DEV_ADDR;
    uint8_t tempReading[6] = {0};

    if(I2C_IF_Write(ucDevAddr,&ucRegOffset,1,0) != 0)
        return -1;

    if(I2C_IF_Read(ucDevAddr, &tempReading[0],6) != 0)
        return -1;


    *pun_ir_led  =  (((long)tempReading[0] & 0x03)<<16) | (long)tempReading[1]<<8 | (long)tempReading[2];    // Combine values to get the actual number
    *pun_red_led = (((long)tempReading[3] & 0x03)<<16) | (long)tempReading[4]<<8 | (long)tempReading[5];   // Combine values to get the actual number
//      *pun_red_led&=0x03FFFF;  //Mask MSB [23:18]
//      *pun_ir_led&=0x03FFFF;  //Mask MSB [23:18]
}

int maxim_max30102_reset()
{
    if(maxim_max30102_write_reg(REG_MODE_CONFIG,0x40) ==  -1)
        return -1;
    else
        return 0;
}
