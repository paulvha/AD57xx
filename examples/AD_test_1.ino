/*****************************************************
 * AD57xx demo
 * 
 * Paul van Haastrecht
 * Original Creation date: January 1 2018
 * 
 * This example demonstrates the basic setting 0V, mid-level 
 * and 5V output on DAC-A. 
 * 
 * For the hardware pin connection see the AD57.h or READ.ME 
 * 
 * This  code is released under the MIT license 
 * 
 * Distributed as-is: no warranty is given
 ******************************************************/
 
#include <stdint.h>
#include <AD57.h> 

/******************************************************
 * user variables
 *****************************************************/

// select the DAC channel to use (DAC_A, DAC_B or DAC_AB)
#define DAC_CHANNEL DAC_A

// MUST HAVE : DEFINES THE SYNC PIN CONNECTION FOR AD57XX
char slave_pin_DAC = 9;


/****************************************************
 * RUN once
 ***************************************************/
void setup() {
  
  Serial.begin(9600);
  Serial.println(F("Starting DAC"));
  
  // set range for DAC_A to max 5V
  dac_begin(DAC_CHANNEL, p5V);

  // enable therminal shutdown and overcurrent protection
  // also set make sure that CLR is NOT set, starting from 0v after OP_CLEAR)
  setControl(OP_INSTR,(SET_TSD_ENABLE |SET_CLAMP_ENABLE|STOP_CLR_SET));

}

/**************************************************
 * continued loop
 **************************************************/
void loop() {
  
  // clear the DAC outputs
  setControl(OP_CLEAR);

  // write the highest value to the DAC register
  // with a AD5752 the full 16 bits will be used
  // with a AD5732 lowest 14 bits are used
  // with a AD5722 lowest 12 bits are used.
  setDac(DAC_CHANNEL, 0xffff);
  Serial.print(F("should be HIGHEST: read from DAC register : "));
  Serial.println(getDac(DAC_CHANNEL));
  delay(1000);
  
  // check on errors. 1 = check errors + display status
  if(check_status(1) == 1)  errorLoop();

  // set a value lower (around mid scale)
  #if defined AD5752 
    setDac(DAC_CHANNEL, 0x8000);
  #elif defined AD5732 
    setDac(DAC_CHANNEL, 0x2000);
  #elif defined AD5722 
    setDac(DAC_CHANNEL, 0x7d0);
  #endif
  
  Serial.print(F("should be around mid-range: read from DAC register : "));
  Serial.println(getDac(DAC_CHANNEL));
  delay(1000);
  
  // check on errors. 0 = only check errors
  if(check_status(0) == 1)  errorLoop();
  
  // set lowest value
  setDac(DAC_CHANNEL, 0x0);
  Serial.print(F("should be LOWEST : read from DAC register : "));
  Serial.println(getDac(DAC_CHANNEL));
  delay(1000);
}


/*  
 *   return values from getStatus()
 *   
 *   0              no error / nothing set
 *  stat_err_TO    ERROR Therminal overheat shutdown
 *  stat_err_CA    ERROR Overcurrent DAC-A
 *  stat_err_CB    ERROR Overcurrent DAC-B
 *  stat_TS        Therminal shutdown is enabled in control
 *  stat_CLR       Clr is set in control register in control
 *  stat_CLAMP     clamp / Overcurrent protection is enabled in control
 *  stat_SDO       Serial data out is disabled  (how did you get the output ??)
 *  stat_UPOL
 *  
 *  disp = 1 status setting will be shown, else only error check.
 *  
 *  return 0 is NO errors detected, else 1 is errors detected
 */

bool check_status(bool disp)
{
    uint16_t stat = getStatus();
    bool ret = 0;
    
    if (stat & stat_err_TO == stat_err_TO)
    {
      Serial.println(F("Therminal overheat shutdown"));
      ret = 1;
    }
    
    if (stat & stat_err_CA == stat_err_CA)
    {
      Serial.println(F("DAC - A overcurrent detected"));
      ret = 1;
    }

    if (stat & stat_err_CB == stat_err_CB)
    {
      Serial.println(F("DAC - B overcurrent detected"));
      ret = 1;
    }

    if (disp == 1)
    {
      if (stat & stat_TS == stat_TS)
         Serial.println(F("Therminal shutdown protection enabled"));
      else
         Serial.println(F("Therminal shutdown protection disabled"));

      if (stat & stat_CLR == stat_CLR)
        Serial.println(F("DAC output set to midscale or fully negative with OP_CLEAR command"));
      else
        Serial.println(F("DAC output set to 0V with OP_CLEAR command"));
      
      if (stat & stat_CLAMP == stat_CLAMP)
         Serial.println(F("Overcurrent protection enabled"));
      else
         Serial.println(F("Overcurrent protection disabled"));

      if (stat & stat_SDO == stat_SDO)
         Serial.println(F("Serial data out enabled"));
      else          // you will never see this one :-)
         Serial.println(F("Serial data out disabled"));
    }

    return(ret);
}

// errorLoop prints an message loops forever.
void errorLoop()
{
  // power-off both channels 
  setPower(STOP_PUA|STOP_PUB);
  
  Serial.print(F("Critical error, looping forever"));
  for (;;);
}


