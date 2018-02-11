/*
 * AD57.cpp
 *
 * Library for handling AD5722 / AD5732 / AD5752 from Analog Devices
 * ©2008–2017 Analog Devices, Inc. All rights reserved. Trademarks and
 * registered trademarks are the property of their respective owners.
 * D06467-0-2/17(F)
 *
 * First version created on February 2018
 *      Paul van Haastrecht
 *
 * Copyright (c) 2018 Paul van Haastrecht.
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
 * This code is released under the MIT license
 */

#include "AD57.h"
#include "printf.h"

/* slave_pin_DAC and DacType must be defined in sketch */
char DAC_SYNC_pin;
uint8_t DAC_TYPE;

// to use with SPI : 16000000 will make it 1mS clock
SPISettings settings_new(16000000, MSBFIRST, SPI_MODE1);

// set debug level
bool DEBUG = 0;

AD57Class::AD57Class()
{
    // nothing
}

/*
 * enable (1) / disable (0) debug information from the library
 *
 * Major version number (enhancement/ rework /added features)
 * Minor version number (bug fixes etc.)
 */

void AD57Class::debug(bool act)
{
    DEBUG = act;

    if (DEBUG) printf("\tD: Debug is set. Library version %x.%x\n\n", VERSION_MAJOR, VERSION_MINOR);
}

/****************** SPI ROUTINES ************************************/
// initialize IO pins
bool AD57Class::init_IO(bool comp2)
{
  // initialize SPI
  SPI.begin();

  // set sync pin
  if (DEBUG) printf("\tD: set DAC_SYNC_pin: %d\n", DAC_SYNC_pin);

  pinMode(DAC_SYNC_pin, OUTPUT);
  digitalWrite(DAC_SYNC_pin, HIGH);    // Sync/CS is active low

#ifdef comp2_pin_DAC
  // see remark 1 in AD57.h. Set for bipolar 2 complement instead of binary

  if (DEBUG) printf("\tD: set comp2_pin_DAC: %d\n", comp2_pin_DAC);

  pinMode (comp2_pin_DAC, OUTPUT);

  if (comp2)
    digitalWrite(comp2_pin_DAC, LOW);  // set to 2scomp for bipolar
  else
    digitalWrite(comp2_pin_DAC, HIGH); // set to Binary for bipolar
#else
  if (comp2 == COMP2)
  {
      if (DEBUG) printf("\tD: Can not set for twos complement as pin is not set\n");
      return false;
  }
#endif


#ifdef ldac_pin_DAC
  // if LDAC is NOT connected to Vcc (remark 2, AD57.h)

  if (DEBUG) printf("\tD: set ldac_pin_DAC: %d\n", ldac_pin_DAC);

  pinMode (ldac_pin_DAC, OUTPUT);
  digitalWrite(ldac_pin_DAC, LOW);     // Enable as it is holding on HIGH
#endif

  return true;
}

/* write register to AD57xx
 *
 * data[0] = register and AD to write
 * data[1], data[2] = register content to write
 */
void AD57Class::write_ad(uint8_t *data)
{
  uint8_t w_data[3];

  // reset write bit (if it was set)
  w_data[0] = data[0] & 0x7f;
  w_data[1] = data[1];
  w_data[2] = data[2];

  if (DEBUG)
     printf("\tD: writing: %02x %02x, %02x\n\n", w_data[0], w_data[1], w_data[2]);

  SPI.beginTransaction(settings_new);

  // chip select
  digitalWrite(DAC_SYNC_pin, LOW);

  // sent data
  SPI.transfer(w_data,3);

  // deselect
  digitalWrite(DAC_SYNC_pin, HIGH);

  SPI.endTransaction();
}

/* read register from AD57xx
 *
 * input :
 *      data[0] = register & selection to read
 *      data[1], data[2] = don't care
 *
 * output
 *     data[0] = register & selection to read
 *     data[1], data[2] = data received from reading
 *
 * see data-sheet page 19 : readback operation
 */
void AD57Class::read_ad(uint8_t *data)
{
  uint8_t r_data[3];

  // set read bit (if it was not set)
  r_data[0] = data[0] | 0x80;
  r_data[1] = r_data[2] = 0;

  // first write the request
  SPI.beginTransaction(settings_new);

  // chip select
  digitalWrite(DAC_SYNC_pin, LOW);

  // transfer request
  SPI.transfer(r_data, 3);

  // deselect & latch request
  digitalWrite(DAC_SYNC_pin, HIGH);

  // set to control register nop function (for reading)
  r_data[0] = 0x18;

  // select for receiving
  digitalWrite(DAC_SYNC_pin, LOW);

  // obtain requested register content from DAC
  SPI.transfer(r_data, 3);

  // deselect
  digitalWrite(DAC_SYNC_pin, HIGH);

  // release
  SPI.endTransaction();

  // store results to caller function
  data[1] = r_data[1];
  data[2] = r_data[2];

  if (DEBUG)
    printf("\tD: Read request: %02x, received: %02x, %02x\n\n", data[0], data[1], data[2]);
}

/***************************** DAC routine **************************/

/* initialize the SPI interface and set range for the first DAC */
bool AD57Class::begin(uint8_t dac, uint8_t range, bool comp2)
{
  bool ret = true;

  DAC_SYNC_pin = slave_pin_DAC;
  DAC_TYPE = DacType;

  // set MOSI channel and output pins
  ret = init_IO(comp2);

  // set power range on DAC (the first action to be performed)
  setRange(dac, range);

  return ret;
}

/* Normally after applying power the DAC will perform a power-up reset
 * setting all values to zero.
 * Reset will perform a soft reset by setting all to zero as well
 */
void AD57Class::reset()
{
    if (DEBUG) printf("\tD: reset\n");

    // set DAC values to zero and load
    setDac(DAC_AB, 0x0, 1);

    // Turn power off
    setPower(STOP_PUA | STOP_PUB);

    // Turn off any instructions
    setControl(OP_INSTR, STOP_CLAMP_ENABLE| STOP_CLR_SET | STOP_TSD_ENABLE | STOP_SDO_DISABLE);

    // reset range to default
    setRange(DAC_AB, p5V);
}

/* set control register
 *
 * Opcode : OP_NOP, OP_INSTR, OP_CLR, OP_LOAD
 *
 * instruct: only needed with OP_INSTR else optional / default 0x0;
 */
void AD57Class::setControl(char opcode, uint16_t instruct)
{
    uint8_t data[3];

    if (DEBUG) printf("\tD: set control: Opcode %02x\n", opcode);

    /* 0------- Write
     * -0------ zero (always
     * --011--- control register
     * -----xxx operation code
     */
    data[0] = REG_CNTRL << 3;
    data[0] |= (opcode & 0x7);

    // only with opcode OP_INSTR these bits are used
    if (opcode == OP_INSTR)
    {
        // obtain current settings
        read_ad(data);

        if (instruct & SET_TSD_ENABLE)   data[2] |= SET_TSD_ENABLE;
        if (instruct & SET_CLAMP_ENABLE) data[2] |= SET_CLAMP_ENABLE;
        if (instruct & SET_CLR_SET)      data[2] |= SET_CLR_SET;
        if (instruct & SET_SDO_DISABLE)  data[2] |= SET_SDO_DISABLE;

        if (instruct & STOP_TSD_ENABLE)  data[2] &= ~SET_TSD_ENABLE;
        if (instruct & STOP_CLAMP_ENABLE)data[2] &= ~SET_CLAMP_ENABLE;
        if (instruct & STOP_CLR_SET)     data[2] &= ~SET_CLR_SET;
        if (instruct & STOP_SDO_DISABLE) data[2] &= ~SET_SDO_DISABLE;
    }

    // write register
    write_ad(data);
}

/* read control register
 *
 * return in control register instruction entry only
 *
 * When reading the instructions, you MUST use OP_INSTR else
 * it will return 0 (as that is don't care)
 *
 */
uint8_t AD57Class::getControl()
{
    uint8_t data[3];

    if (DEBUG)   printf("\tD: getControl\n");

    /* 1------- Read
     * -0------ zero (always)
     * --011--- control register
     * -----xxx Opcode
     */
    data[0] = REG_CNTRL << 3;
    data[0] |= (OP_INSTR & 0x7);

    read_ad(data);

    return(data[2] & 0xf);
}

/* set power range
 *
 * dac   : DAC_A, DAC_B or DAC_AB
 * range : p5V, p10V, p108V, pn5V, pn10V, pn108V,
 */
void AD57Class::setRange(uint8_t dac, uint8_t value)
{
    uint8_t data[3];

    if (DEBUG) printf("\tD: setRange: dac %02x, value %02x\n", dac, value);

    /* 0------- Write
     * -0------ zero (always)
     * --001--- output register
     * -----xxx channel DAC_A / DAC_B or both
     */
    data[0] = REG_OUTPUT << 3;
    data[0] |= (dac & 0x7);

    data[1] = 0x0;              // don't care
    data[2] = value & 0x7;      // add range

    write_ad(data);
}

/* get range for dac
 * return in value
 *
 * If dac is DAC_AB, we can not obtain both DAC-A and DAC-B at the
 * same time. It will obtain DAC_A only in that case
 */
uint8_t AD57Class::getRange(uint8_t dac)
{
    uint8_t data[3];

    if (DEBUG) {
       printf("\tD: getRange\n");
       if (dac == DAC_AB) printf("\tD: Using DAC_A instead of DAC_AB \n");
    }

    /* 1------- read
     * -0------ zero (always)
     * --001--- output register
     * -----xxx channel DAC_A or DAC_B
     */
    data[0] = REG_OUTPUT << 3;

    if (dac == DAC_AB) data[0] |= (DAC_A & 0x7);
    else  data[0] |= (dac & 0x7);

    //read data
    read_ad(data);

    return(data[2] & 0xff);
}

/* set a DAC value, enable the channel and load the new value
 *
 * dac   : DAC_A or DAC_B or DAC_AB
 * value : Value to set
 * load  : 1 will load to amplifier after setting / 0: will not load
 *
 * From the value either 16, 14 or 12 bit will be used depending on AD57xx
 *
 */
void AD57Class::setDac(uint8_t dac, uint16_t value, bool load)
{
    uint8_t data[3];
    uint8_t setting = 0x0;

    if (DEBUG) printf("\tD: setDac: Dac %02x, value %04x, load %x\n", dac, value, load);


    /* 0------- Write
     * -0------ zero (always)
     * --000--- DAC register
     * -----xxx channel DACA / DACB or both
     */
    data[0] = REG_DAC << 3;
    data[0] |= (dac & 0x7);

    if (DAC_TYPE == AD5722)
    {
        // add 12 bits value
        data[1] = (value & 0xff0) >> 4;
        data[2] = (value & 0xf) << 4;
    }
    else if (DAC_TYPE ==  AD5732)
    {
        // add 14 bits value
        data[1] = (value & 0x3ff0) >> 6;
        data[2] = (value & 0xff) << 2;
    }
    else if (DAC_TYPE ==  AD5752)
    {
        // add 16 bits value
        data[1] = (value & 0xff00) >> 8;
        data[2] = value & 0xff;
    }

    // sent new values to the AD57xx
    write_ad(data);

    // According to data-sheet pag 20 first set output
    // make sure the output channel is enabled
    if (dac == DAC_A || dac == DAC_AB) setting |= SET_PUA;
    if (dac == DAC_B || dac == DAC_AB) setting |= SET_PUB;

    setPower(setting);

    // wait atleast 10us to allow power up (see data-sheet page 26)
    delay(1);

    // if requested (default) load the new value into the DAC
    if (load) loadDac();
}

/* this will load the stored / updated DAC values */
void AD57Class::loadDac()
{
    if (DEBUG) printf("\tD: loadDac\n");

    // load the new value into the DAC
    setControl(OP_LOAD);
}

/* Read the value in the DAC register
 *
 * If dac is DAC_AB, we can not obtain both DAC-A and DAC-B at the
 * same time. It will obtain DAC_A only in that case
 */
uint16_t AD57Class::getDac(uint8_t dac)
{
    uint8_t data[3];
    uint16_t value;

    if (DEBUG){
        printf("\tD: getDac %02x\n", dac);
        if (dac == DAC_AB) printf("\tD: Using DAC_A instead of DAC_AB \n");
    }

    /* 1------- read
     * -0------ zero (always)
     * --000--- DAC register
     * -----xxx channel DAC-A / DACB or both
     */
    data[0] =  REG_DAC << 3;

    if (dac == DAC_AB) data[0] |= (DAC_A & 0x7);
    else  data[0] |= (dac & 0x7);

    read_ad(data);

    if (DAC_TYPE == AD5722)
    {
        // get 12 bits value
        value  = (data[1] << 4);
        value |= (data[2] & 0xf0) >> 4;
    }
    else if (DAC_TYPE == AD5732)
    {
        // get 14 bits value
        value  = (data[1] << 6);
        value |= (data[2] & 0xfc) >> 2;
    }
    else if (DAC_TYPE == AD5752)
    {
        // get 16 bits value
        value  = (data[1] << 8);
        value |= (data[2] & 0xff);
    }

    return(value);
}

/* setting channel output on/off
 * only (re)setting P_PUA or P_PUB are allowed
 */
void AD57Class::setPower(uint8_t setting)
{
    uint8_t data[3];

    if (DEBUG) printf("\tD: setPower: setting %02x\n", setting);

    /* 0------- Write
     * -0------ zero (always)
     * --010--- power register
     * -----000 zero (always)
     */
    data[0] = REG_POWER << 3;

    // read register
    read_ad(data);

    if (setting & SET_PUA)   data[2] |= PUA_2;
    if (setting & SET_PUB)   data[2] |= PUB_2;

    if (setting & STOP_PUA)  data[2] &= ~PUA_2;
    if (setting & STOP_PUB)  data[2] &= ~PUB_2;

    // write updated setting
    write_ad(data);
}

/* get power status
 * return
 *  0 = DAC channel A and channel B are powered down
 *  1 = DAC channel A powered-up
 *  2 = DAC channel B powered-up
 *  3 = DAC channel A and B are both powered-up
 */
uint8_t AD57Class::getPower()
{
    uint8_t data[3];
    uint8_t ret=0;

    if (DEBUG) printf("\tD: getPower\n");

    /* 1------- read
     * -0------ zero (always)
     * --010--- power register
     * -----000 zero (always)
     */
    data[0] = REG_POWER << 3;

    // read register
    read_ad(data);

    // check power status bits
    if (data[2] & PUA_2) ret |= 0x1;
    if (data[2] & PUB_2) ret |= 0x2;

    return(ret);
}

/* get Error status
 *
 * return
 * 0                  NO error
 * 0x1 stat_err_TO    ERROR Therminal overheat shutdown
 * 0x2 stat_err_CA    ERROR Overcurrent DAC-A
 * 0x4 stat_err_CB    ERROR Overcurrent DAC-B
 */
uint16_t AD57Class::getError()
{
    uint8_t data[3];
    uint16_t ret=0;

    if (DEBUG)   printf("\tD: getError\n");

    /* 1------- read
     * -0------ zero (always)
     * --010--- power register
     * -----000 zero (always)
     */
    data[0] = REG_POWER << 3;

    // read register
    read_ad(data);

    // extract potential errors
    if(data[2] & E_TSD_2) ret |= stat_err_TO;
    if(data[2] & E_OCA_2) ret |= stat_err_CA;
    if(data[1] & E_OCB_1) ret |= stat_err_CB;

    return(ret);
}

/* read status
 *
 * return value
 *  0               // no error / nothing set
 *  stat_err_TO     // ERROR Therminal overheat shutdown
 *  stat_err_CA     // ERROR Overcurrent DAC-A
 *  stat_err_CB     // ERROR Overcurrent DAC-B
 *  stat_TS         // Therminal shutdown is enabled in control
 *  stat_CLR        // Clr is set in control register in control
 *  stat_CLAMP      // clamp / Overcurrent protection is enabled in control
 *  stat_SDO        // Serial data out is disabled  (how did you get the output ??)
 *
 * values are logically "or-ed" as multiple situations
 * may exist at the same time.
 *
 */
uint16_t AD57Class::getStatus()
{
    if (DEBUG)   printf("\tD: getStatus\n");

    uint16_t value = getControl();
    uint16_t ret = getError();

    // extract status settings
    if(value & SET_TSD_ENABLE)   ret |= stat_TS;
    if(value & SET_CLR_SET)      ret |= stat_CLR;
    if(value & SET_CLAMP_ENABLE) ret |= stat_CLAMP;
    if(value & SET_SDO_DISABLE)  ret |= stat_SDO;

    return(ret);
}

/* This is only necessary if you want to change the output for both DAC
 * channels at the same time with hardware load control. (remark 2 in AD57.h).
 * If NOT needed tie LDAC (pin 10) to Vcc.
 *
 * call this routine BEFORE setting new values: setOutput(HOLD)
 * set the new values for DAC_A and DAC_B
 * after updating call with setOutput(RELEASE)
 */
#ifdef ldac_pin_DAC

void AD57Class::setOutput(uint8_t act)
{
    if (DEBUG) printf("\tD: setOutput: act %02x\n\n", act);

    if (act == HOLD)
        digitalWrite(ldac_pin_DAC, HIGH); // is holding on HIGH
    else
        digitalWrite(ldac_pin_DAC, LOW);  // release

    delay(1);
}

#endif

AD57Class AD57;
