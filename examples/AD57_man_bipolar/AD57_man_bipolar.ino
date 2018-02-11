/*****************************************************
 * AD57xx demo +5V to -9V  Bi-polar output range mode
 *
 * Paul van Haastrecht
 * Original Creation date: February 2018 / version 1.0
 *
 * This example demonstrates the basic setting of a DAC output in Bipolar output
 * range mode.
 *
 * A 9v battery is connected with the +pole or the battery connected to GND and
 * the -pole to pin 1. A such we have a Vdd on pin 24 of +5V and a Vdd on pin 1
 * of -9v.
 *
 * We can NOT use the ADC input from the Arduino to measure the DAC Output.
 * The output voltage will go negative (bipolar) and the Arduino is NOT able
 * to read negative analog input. So in order to measure the output connect
 * a voltmeter or scope to the DAC-output.
 *
 * The starting level is asked and this written and read back from the
 * AD57xx.
 *
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
 * Measure the result with voltmeter or scope on :
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
#define output_range p5V

// MUST : DEFINE THE SYNC PIN CONNECTION FOR AD57xx
/* The initialisation and pulling high and low is handled by the
 * library. The sketch only needs to define the right pin.
 */
char slave_pin_DAC = 9;

/****************************************************
 * RUN once
 ***************************************************/
void setup() {

  Serial.begin(9600);
  serialTrigger(F("Press any key to begin  +5V to -9V  Bi-polar output range mode."));

  /* enable or disable debug information from the library
   *  0 = disable (default.. no need to call)
   *  1 = enable
   */
  AD57.debug(1);

  Serial.println(F("Starting DAC"));

  // initialise the AD57xx and connections to and
  // from the AD57xx
  init_AD57xx();
}

/**************************************************
 * continued loop
 **************************************************/
void loop() {

  // clear the DAC outputs
  AD57.setControl(OP_CLEAR);

  // get value to set on the  DAC output
  uint16_t value = getValue();

  Serial.print(F("Trying to set DAC register value: "));
  Serial.println(value , HEX);

  // the values are written to the registers,
  AD57.setDac(DAC_CHANNEL, value);

  // write the highest value to the DAC register
  // with a AD5752 the full 16 bits will be used
  // with a AD5732 lowest 14 bits are used
  // with a AD5722 lowest 12 bits are used.

  if (DacType == AD5752)
  {
    Serial.print(F("AD5752: should read "));
  }
  else if (DacType == AD5732)
  {
    Serial.print(F("AD5732: should read "));
    value = value & 0x3fff;
  }
  else if (DacType == AD5722)
  {
    Serial.print(F("AD5722: should read "));
    value = value & 0x0fff;
  }

  Serial.println(value, HEX);
  Serial.print(F("Read from DAC register: "));
  Serial.println(AD57.getDac(DAC_CHANNEL), HEX);

  // check on errors. 1 = check errors + display status
  if(check_status(1) == 1)  errorLoop();

  while (Serial.available()) Serial.read();
  serialTrigger(F("Press to continue and try half the value (and load)."));

  value = value / 2;

  // Now load the values directly.
  AD57.setDac(DAC_CHANNEL, value);
  Serial.print(F("should be half of entry: read from DAC register : "));

  // get DAC value
  Serial.println(AD57.getDac(DAC_CHANNEL), HEX);

  // check on errors. 0 = check errors NO display status
  if(check_status(0) == 1)  errorLoop();

  while (Serial.available()) Serial.read();
  serialTrigger(F("Press to continue to set DAC value to 0."));

  // set lowest value.
  AD57.setDac(DAC_CHANNEL, 0x0);
  Serial.print(F("should be LOWEST : read from DAC register : "));

  // get DAC value
  Serial.println(AD57.getDac(DAC_CHANNEL), HEX);

  while (Serial.available()) Serial.read();
  serialTrigger(F("Press to continue to closedown output."));

  // close down power
  AD57.setPower(STOP_PUA | STOP_PUB);
}

/* initialize the connection with the board  and set the default values */
void init_AD57xx()
{
  // set output voltage range in binary
  // if you want to test 2sComp, see remark 1 in AD57.h, use
  //AD57.begin(DAC_CHANNEL, output_range, COMP2);
  AD57.begin(DAC_CHANNEL, output_range);

  Serial.print("get range ");
  Serial.println(AD57.getRange(DAC_CHANNEL));

  // enable therminal shutdown and overcurrent protection
  // also stop CLR as, starting from 0v after OP_CLEAR op_code)
  // this can be changed for full negative after clear with
  //AD57.setControl(OP_INSTR,(SET_TSD_ENABLE |SET_CLAMP_ENABLE|SET_CLR_SET));
  AD57.setControl(OP_INSTR,(SET_TSD_ENABLE |SET_CLAMP_ENABLE|STOP_CLR_SET));
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

/* get value to write as input */
uint16_t getValue()
{
  char input[5];
  int i =0;

  // clear any pending input
  while (Serial.available()) Serial.read();

  Serial.println(F("Enter value in HEX to set in DAC (max 4 digits)"));

  /* this is bit tricky as for input from serial it will only set available once you
   * press enter. NO matter how much you input before <enter>, only maximum the first
   * 4 characters are used.
   */
  while(1)
  {
    if (Serial.available())
    {
        input[i] = Serial.read();

        if (input[i] == '\n' || i > 3)
        {
          if (i > 0)
          {
            input[i] = 0x0;
            return (Hto16(input));;
          }
          else i--;
        }

        i++;
    }
  }
}

/* translate ascii-HEX input to uin16_t */
uint16_t Hto16(char *p)
{
 uint16_t x = 0;
 for(;;) {
   char c = *p;
   if (c >= '0' && c <= '9') {
      x *= 16;
      x += c - '0';
   }
   else if (c >= 'A' && c <= 'F') {
      x *= 16;
      x += (c - 'A') + 10;
   }
   else if (c >= 'a' && c <= 'f') {
      x *= 16;
      x += (c - 'a') + 10;
   }
   else break;
   p++;
 }
 return x;
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

