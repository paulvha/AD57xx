/*
 * AD57.cpp
 *
 *  Library for handling AD5722 / AD5732 / AD5752
 *  
 *  First version created on 1-Jan 2018
 * 		Paul van Haastrecht
 *  
 * Copyright (c) 2017 Paul van Haastrecht.
 * 
 * 
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 * 
 * 
 */

#include "AD57.h"
#include "Arduino.h"
#include "SPI.h"

char DAC_SYNC_pin = slave_pin_DAC;

/****************** SPI ROUTINES ************************************/
// initialize SPI
void init_spi()
{
   
  SPI.begin(); 			        // initialize SPI
  SPI.setBitOrder(MSBFIRST);  	// data is clocked in MSB first
  SPI.setDataMode(SPI_MODE0);  	// SCLK idle low (CPOL=0), MOSI read on rising edge (CPHI=0)
  SPI.setClockDivider(SPI_CLOCK_DIV32);  // system clock = 16 MHz, chip max = 1 MHz
  
  pinMode (DAC_SYNC_pin, OUTPUT);
  digitalWrite(DAC_SYNC_pin, HIGH); // Sync/CS is active low

#if ldac_pin_DAC                  // LDAC is NOT connected to ground (remark 2, AD57.h)
  pinMode (ldac_pin_DAC, OUTPUT);
  digitalWrite(ldac_pin_DAC, LOW); // is holding on HIGH
#endif
  delay(10);   
}

// write register to AD 
void send_ad(uint8_t *data)
{
  // chip select and give time to settle
  digitalWrite(DAC_SYNC_pin, LOW);
  delay(1);
  
  // sent data
  SPI.transfer(data[0]);
  SPI.transfer(data[1]);
  SPI.transfer(data[2]);
  delay(1);  // maybe not needed
  
  // deselect and give time to settle
  digitalWrite(DAC_SYNC_pin, HIGH);
  delay(1);
}

// read register from AD 
void read_ad(uint8_t *data)
{
  // chip select and give time to settle
  digitalWrite(DAC_SYNC_pin, LOW);
  delay(1);
  
  // transfer register / DAC to read
  SPI.transfer(data[0] | 0x80);
  
  // read first register byte and neglect (we have that already)
  data[1] = SPI.transfer(0x00);
  delay(1);  // maybe not needed

  // read next 2 bytes
  data[1] = SPI.transfer(0x00);
  delay(1);  // maybe not needed
  
  data[2] = SPI.transfer(0x00);
  delay(1);  // maybe not needed

  digitalWrite(DAC_SYNC_pin, HIGH);
  delay(1);
}


/***************************** DAC routine **************************/

/* initialize the SPI interface and set range for the first DAC */
void dac_begin(uint8_t dac, uint8_t range)
{
  // set MOSI channel
  init_spi();

  // set power range on DAC
  setRange(dac, range);
}

/* get version number of driver
 * 2 nibbles
 * xxxx----   Major version number (enhancement/ rework /added features)
 * ----xxxx   Minor version number (bug fixes etc.)
 */
uint8_t getVersion()
{
    uint8_t ret;
    
    ret = VERSION_MAJOR << 4;
    ret |= VERSION_MINOR;
    
    return(ret);
}

/* set control register 
 *  
 * Opcode : OP_NOP, OP_INSTR, OP_CLR, OP_LOAD
 * 
 * instruct: only needed with OP_INSTR else 0x0;
 *      
 */
void setControl(char opcode, uint16_t instruct)
{
    uint8_t data[3];

    /* 0------- Write
     * -0------ zero (always
     * --011--- control register
     * -----xxx operation code
     */
    data[0] = 0x0 | REG_CNTRL << 6; 
    data[0] |= (opcode & 0x7);

    data[1] = data[2] = 0x0;

    // only with opcode OP_INSTR these bits are used
    if (opcode == OP_INSTR)
    {
        // read register
        read_ad(data);
       
        if (instruct & SET_TSD_ENABLE) data[2]    |= SET_TSD_ENABLE;
        if (instruct & SET_CLAMP_ENABLE) data[2]  |= SET_CLAMP_ENABLE;
        if (instruct & SET_CLR_SET) data[2]       |= SET_CLR_SET;
        if (instruct & SET_SDO_DISABLE) data[2]   |= SET_SDO_DISABLE;
        
        if (instruct & STOP_TSD_ENABLE) data[2]    &= ~SET_TSD_ENABLE;
        if (instruct & STOP_CLAMP_ENABLE) data[2]  &= ~SET_CLAMP_ENABLE;
        if (instruct & STOP_CLR_SET) data[2]       &= ~SET_CLR_SET;
        if (instruct & SET_SDO_DISABLE) data[2]    &= ~SET_SDO_DISABLE;
    }
    
    // write register
    send_ad(data);
}

/* read control register
 * 
 * return in control register instructions set
 */
uint8_t getControl()
{
    uint8_t data[3];

    /* 1------- Read
     * -0------ zero (always)
     * --011--- control register
     * -----000 NOP Operation (only read)
     */
    data[0] = 0x80 | REG_CNTRL << 6; 
    
    read_ad(data);
    
    return(data[2] & 0xf);
}

/* set power range
 * dac 	 : DAC_A, DAC_B or DAC_AB
 * range : p5V, p10V, p108V, pn5V, pn10V, pn108V
 */
void setRange(uint8_t dac, uint8_t value)
{
    uint8_t data[3];

    /* 0------- Write
     * -0------ zero (always)
     * --001--- output register
     * -----xxx channel DACA/ DACB or both
     */
    data[0] = 0x0 | REG_OUTPUT << 6; 
    data[0] |= (dac & 0x7);
    
    
    data[1] = 0x0;		          // don't care
    data[2] = 0x0 | (value & 0x7);// add range
    
    send_ad(data);
}

/* get range for dac
 * return in value
 */
uint8_t getRange(uint8_t dac)
{
    uint8_t data[3];

    /* 1------- read
     * -0------ zero (always)
     * --001--- output register
     * -----xxx channel DACA/ DACB or both
     */
    data[0] = 0x80 | REG_OUTPUT << 6; 
    data[0] |= (dac & 0x7);
    
    //read data
    read_ad(data);
 
    return(data[2] & 0xff);
}

/* set a DAC value, enable the channel and load the new value 
 * 
 * dac   : DAC_A or DAC_B or DAC_AB
 * value : either 16, 14 or 12 bit value depending on AD57xx
 * 
 */ 
void setDac(uint8_t dac, uint16_t value)
{
    uint8_t data[3];
    uint8_t setting = 0x0;

    /* 0------- Write
     * -0------ zero (always)
     * --000--- DAC register
     * -----xxx channel DACA / DACB or both
     */
    data[0] = 0x0 | REG_DAC << 6; 
    data[0] |= (dac & 0x7);

#ifdef AD5722
    // add 12 bits value
    data[1] = (value & 0xff0) >> 4);
    data[2] = (value & 0xff) << 4;

#elseif  AD5732
    // add 14 bits value
    data[1] = (value & 0x3ff0) >> 6);
    data[2] = (value & 0xff) << 2;

#elseif AD5752
    // add 16 bits value
    data[1] = (value & 0xff00) >> 8);
    data[2] = value & 0xff;
#endif
    
    // sent new values to the AD57xx
    send_ad(data);
    
    // now this new value has been set, 
    // make sure the output channel is enabled
    if (dac == DAC_A || dac == DAC_AB) setting |= SET_PUA;
    if (dac == DAC_B || dac == DAC_AB) setting |= SET_PUB;
    setPower(setting);
    
    // load the new value into the DAC 
    setControl(OP_LOAD, 0x0);
}

/* read the value in the DAC register */
uint16_t getDac(uint8_t dac)
{
    uint8_t data[3];
    uint16_t value;
    /* 1------- read
     * -0------ zero (always)
     * --000--- DAC register
     * -----xxx channel DAC-A / DACB or both
     */
    data[0] = 0x80 | REG_DAC << 6; 
    data[0] = 0x0 | (dac & 0x7);

    read_ad(data);
    
#ifdef AD5722
    // get 12 bits value
    value  = (data[1] << 4);
    value |= (data[2] & 0xf0) >> 4;

#elseifdef AD5732
    // get 14 bits value
    value  = (data[1] << 6);
    value |= (data[2] & 0xfc) >> 2;

#elseifdef AD5752
    // get 16 bits value
    value  = (data[1] << 8);
    value |= (data[2] & 0xff);
#endif

    return(value);
} 

/* setting channel output on/off 
 * only (re)setting P_PUA or P_PUB are allowed
 */
void setPower(uint8_t setting)
{
    uint8_t data[3];

    /* 0------- Write
     * -0------ zero (always)
     * --010--- power register
     * -----000 zero (always)
     */
    data[0] = 0x0 | REG_POWER << 6; 
    
    // read register
    read_ad(data);
    
    if (setting & SET_PUA == SET_PUA) data[2]   |= PUA_2;
    if (setting & SET_PUB == SET_PUB) data[2]   |= PUB_2;

    if (setting & STOP_PUA == STOP_PUA) data[2] &= ~PUA_2;
    if (setting & STOP_PUB == STOP_PUB) data[2] &= ~PUB_2;
   
    // write updated setting
    send_ad(data);
}

/* get power status 
 * return
 *  0 = DAC channel A and channel B are powered down 
 *  1 = DAC channel A powered-up
 *  2 = DAC channel B powered-up
 *  3 = DAC channel A and B are powered-up
 */
uint8_t getPower()
{
    uint8_t data[3];
    uint8_t ret=0;
    
    /* 1------- read
     * -0------ zero (always)
     * --010--- power register
     * -----000 zero (always)
     */
    data[0] = 0x80 | REG_POWER << 6;
    
    // read register
    read_ad(data);
    
    // check power status bits
    if (data[2] & PUA_2 == PUA_2 ) ret |= 0x1;
    if (data[2] & PUB_2 == PUB_2 ) ret |= 0x2;
    
    return(ret);   
}

/* read status 
 *  
 * return values
 *  0    			// no error / nothing set
 *  stat_err_TO  	// ERROR Therminal overheat shutdown
 *  stat_err_CA   	// ERROR Overcurrent DAC-A
 *  stat_err_CB   	// ERROR Overcurrent DAC-B
 *  stat_TS 	  	// Therminal shutdown is enabled in control
 *  stat_CLR     	// Clr is set in control register in control
 *  stat_CLAMP   	// clamp / Overcurrent protection is enabled in control
 *  stat_SDO  	 	// Serial data out is disabled  (how did you get the output ??)
 * 
 * values are logically "or-ed" as multiple problems 
 * may exist at the same time.
 * 
 */  
uint16_t getStatus()
{
    uint16_t value = getPower();
    uint16_t ret = 0x0;
    
    // extract potential errors
    if(value & E_TSD == E_TSD ) ret |= stat_err_TO;
    if(value & E_OCA == E_OCA ) ret |= stat_err_CA;
    if(value & E_OCA == E_OCB ) ret |= stat_err_CB;
    
    value = getControl();
    
    // extract status settings
    if(value & SET_TSD_ENABLE == SET_TSD_ENABLE )     ret |= stat_TS;
    if(value & SET_CLR_SET == SET_CLR_SET )           ret |= stat_CLR;
    if(value & SET_CLAMP_ENABLE == SET_CLAMP_ENABLE ) ret |= stat_CLAMP;
    if(value & SET_SDO_DISABLE == SET_SDO_DISABLE )   ret |= stat_SDO;
    
    return(ret);
}

/* This is only necessary if you want to change the output for both DAC
 * channels at the same time. (remark 2 in AD57.h). If NOT needed
 * tie LDAC to GND.
 * 
 * call this routine BEFORE setting new values: setOutput(HOLD)
 * set the new values for DAC_A and DAC_B
 * after updating call with setOutput(RELEASE)
 */
#ifdef ldac_pin_DAC

#define HOLD    1
#define RELEASE 2

void setOutput(uint8_t act)
{
    if (act == HOLD)
        digitalWrite(ldac_pin_DAC, HIGH); // is holding on HIGH
    else
        digitalWrite(ldac_pin_DAC, LOW);  // release 
        
    delay(1);
}

#endif
