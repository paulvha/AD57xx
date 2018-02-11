/*****************************************************
 * AD57xx demo  +5V to -5V  Bi-polar constant loop output range mode
 * 
 * Paul van Haastrecht
 * Original Creation date: February 2018 / version 1.0
 * 
 * This example demonstrates the basic setting of a DAC output in Bipolar output
 * range mode.
 * 
 * A 9v battery is connected with the +pole or the battery connected to GND and
 * the -pole to pin 1. A such we have a Vdd on pin 24 of +5V and a Vdd on pin 1 
 * of -9V.
 * 
 * We can NOT use the ADC input from the Arduino to measure the DAC Output. 
 * The output voltage will go negative (bipolar) and the Arduino is NOT able 
 * to read negative analog input. In order to measure the output connect 
 * a voltmeter or scope to the DAC-output.
 * 
 * The output is in a loop based on the DAC values.
 * 
 * The hardware pin connection on AD57xx
 * 
 * Pin 15, 18, 19, 20 21       GND
 * Pin 2, 4,  6,  12, 13, 22   not connected
 * Pin 5, 10, 11, 14, 24       +5V
 * Pin 17                      RFIN
 * 
 *                   --------         --------
 *      +5V ---------| 1K   |-----.---| 1K    |----- GND
 *                   --------     |    --------
 *                                |
 *                               REFIN
 * 
 * Battery connection
 * Pin 1                      -pole 9V
 * Pin 15                     +pole 9V
 * 
 * default setting
 * AD57xx   Arduino Uno
 * Pin 7     9          Sync
 * Pin 8    13          clock
 * Pin 9    11          SDIN
 * Pin 16   12          SDO
 * 
 * Measure the result with a voltmeter or scope on :
 * Pin 3           DAC-A out
 * pin 23          DAC-B out
 * 
 * This code is released under the MIT license 
 * 
 * Distributed as-is: no warranty is given
 ******************************************************/
 
#include <stdint.h>
#include <AD57.h> 

/******************************************************
 * user variables
 *****************************************************/
// MUST : select the kind of AD57xx you have:
// AD5722 or AD5732 or AD5752 
uint8_t DacType = AD5722;

// MUST : select the DAC channel to use (DAC_A, DAC_B or DAC_AB)
#define DAC_CHANNEL DAC_A

/* MUST : define the output voltage range (for Bipolar use pnxxx)
 * p5V          +5V
 * p10V         +10V
 * p108V        +10.8V
 * pn5V         +/-5V
 * pn10V        +/-10V
 * pn108V       +/-10.8V
 */
#define output_range pn10V

// MUST : DEFINE THE SYNC PIN CONNECTION FOR AD57xx
/* The initialisation and pulling high and low is handled by the 
 * library. The sketch only needs to define the right pin.
 */
char slave_pin_DAC = 9;

int16_t initial_value;
/****************************************************
 * RUN once
 ***************************************************/
void setup() {
  
  Serial.begin(9600);
  serialTrigger(F("Press any key to begin +5V to -5V  Bi-polar constant loop output range mode"));
  
  /* enable or disable debug information from the library
   *  0 = disable (default.. no need to call)
   *  1 = enable
   */
  AD57.debug(0);
  
  Serial.println(F("Starting DAC"));

  // initialise the AD57xx and connections to and
  // from the AD57xx
  init_AD57xx();

  // set max count depending on AD57xx : 12, 14 or 16 bits
  if (DacType == AD5752)    initial_value = 0xffff;
  else if (DacType == AD5732) initial_value = 0x3fff;
  else if (DacType == AD5722) initial_value = 0x0fff;
}

/**************************************************
 * continued loop
 **************************************************/
void loop() {
  
  static bool dir = 0;                  // 0 = counting down, 1= counting up
  static int16_t value = initial_value;
  
  Serial.print(F("Setting DAC register value (hex) "));
  Serial.println(value & 0xffff, HEX); 

  // the value is written to the registers
  AD57.setDac(DAC_CHANNEL, value);
    
  // check on errors. 1 = check errors + display status
  if(check_status(0) == 1)  errorLoop();

  // set direction for counting or decrementing
  if (value == 0) dir =  1;
  else if (value == initial_value) dir = 0;

  // add or subtract according to direction
  if (dir == 0)  value--;
  else value++;
}

/* initialize the connection with the board  and set the default values */
void init_AD57xx()
{
  // set output voltage range in binary
  // if you want to test 2scomp, see remark 1 in AD57.h, use
  // AD57.begin(DAC_CHANNEL, output_range, COMP2); 
  AD57.begin(DAC_CHANNEL, output_range);

  Serial.print("get range ");
  Serial.println(AD57.getRange(DAC_CHANNEL));

  // enable therminal shutdown and overcurrent protection
  // also stop CLR as, starting from 0v after OP_CLEAR op_code)
  // this can be changed for full negative after clear with
  // AD57.setControl(OP_INSTR,(SET_TSD_ENABLE |SET_CLAMP_ENABLE|SET_CLR_SET));
  AD57.setControl(OP_INSTR,(SET_TSD_ENABLE |SET_CLAMP_ENABLE|STOP_CLR_SET));
   
  // clear the DAC outputs
  AD57.setControl(OP_CLEAR);
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
    uint16_t stat = AD57.getStatus();
    bool ret = 0;
    
    if (stat & stat_err_TO)
    {
      Serial.println(F("Therminal overheat shutdown"));
      ret = 1;
    }
    
    if (stat & stat_err_CA)
    {
      Serial.println(F("DAC - A overcurrent detected"));
      ret = 1;
    }

    if (stat & stat_err_CB)
    {
      Serial.println(F("DAC - B overcurrent detected"));
      ret = 1;
    }

    if (disp == 1)
    {
      Serial.println(F("Display settings\n"));
      
      if (stat & stat_TS)
         Serial.println(F("Therminal shutdown protection enabled"));
      else
         Serial.println(F("Therminal shutdown protection disabled"));

      if (stat & stat_CLR)
        Serial.println(F("DAC output set to midscale or fully negative with OP_CLEAR command"));
      else
        Serial.println(F("DAC output set to 0V with OP_CLEAR command"));
      
      if (stat & stat_CLAMP)
         Serial.println(F("Overcurrent protection enabled"));
      else
         Serial.println(F("Overcurrent protection disabled"));

      if (stat & stat_SDO) // you will never see this one :-)
         Serial.println(F("Serial data out disabled"));
      else          
         Serial.println(F("Serial data out enabled"));

      Serial.println();
    }

    return(ret);
}

/* serialTrigger prints a message, then waits for something
 * to come in from the serial port.
 */
void serialTrigger(String message)
{
  Serial.println();
  Serial.println(message);
  Serial.println();

  while (!Serial.available());

  while (Serial.available())
    Serial.read();
}

/* errorLoop loops forever.*/
void errorLoop()
{
  // power-off both channels 
  AD57.setPower(STOP_PUA|STOP_PUB);
  
  Serial.print(F("Critical error, looping forever"));
  for (;;);
}
