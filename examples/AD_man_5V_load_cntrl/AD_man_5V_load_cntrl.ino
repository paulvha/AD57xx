/*****************************************************
 * AD57xx demo 0 -5 volt with Hardware load control
 *
 * Paul van Haastrecht
 * Original Creation date: February 2018 / version 1.0
 *
 * This example demonstrates the basic setting of a DAC output.
 * The starting level is asked and this written and read back from the
 * AD57xx. If DAC output is connected to the defined analog input the
 * expected voltage level is shown.
 *
 * Additional in this example we will use hardware control to load the values in the
 * AD57xx, instead of the simpler software control, just to prove it can be done.
 *
 * This is described in detail in the AD57.h file. What we need to
 * do is the connect Pin 10 of the AD57xx to the selected pin on the Arduino. (7).
 *
 * To enable this we need to uncomment in the AD57.h file the line 129
 * //#define ldac_pin_DAC 7  to  #define ldac_pin_DAC 7. No other changes
 * needed to the driver.
 *
 *
 * The hardware pin connection on AD57xx
 *
 * Pin 1, 15, 18, 19, 20 21       GND
 * Pin 2, 4,  6,  12, 13, 22      not connected
 * Pin 5, 11, 14, 24              +5V
 * Pin 17                         RFIN
 *
 *                   --------          --------
 *      +5V ---------| 1K   |-----.---| 1K    |----- GND
 *                   --------     |    --------
 *                                |
 *                               REFIN
 *
 * default setting
 * AD57xx   Arduino Uno
 * Pin 3    A0          DAC-A out
 * Pin 7     9          Sync
 * Pin 8    13          clock
 * Pin 9    11          SDIN
 * Pin 16   12          SDO
 * pin 23   A1          DAC-B out
 * pin 10   7           LDAC control
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

/* MUST : define the output voltage range
 * p5V          +5V
 * p10V         +10V
 * p108V        +10.8V
 * pn5V         +/-5V     pin 1 need to be connected to minus
 * pn10V        +/-10V    pin 1 need to be connected to minus
 * pn108V       +/-10.8V  pin 1 need to be connected to minus
 */
#define output_range p5V

// MUST : DEFINE THE SYNC PIN CONNECTION FOR AD57XX
/* The initialisation and pulling high and low is handled by the
 * library. The sketch only needs to define the right pin.
 */
char slave_pin_DAC = 9;

// OPTIONAL : check DAC output with analog input
#define DAC_A_INPUT A0
#define DAC_B_INPUT A1

/****************************************************
 * RUN once
 ***************************************************/
void setup() {

  Serial.begin(9600);
  serialTrigger(F("Press any key to begin 0 -5 volt with Hardware load control."));

  /* enable or disable debug information from the library
   *  0 = disable (default.. no need to call
   *  1 = enable
   */
  AD57.debug(0);

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

  Serial.print(F("Trying to set DAC register value BUT not load (hex) "));
  Serial.println(value , HEX);

  // hold the loading new values to the register of the DAC.
  AD57.setOutput(HOLD);

  // the values are written to the  input registers, BUT with
  // the '0' option they are not loaded
  AD57.setDac(DAC_CHANNEL, value, 0);

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

  check_values();

  while (Serial.available()) Serial.read();
  serialTrigger(F("Press to continue and load and see the difference in analog output."));

  // we now release the hardware LDAC pin
  AD57.setOutput(RELEASE);

  check_values();

  // check on errors. 1 = check errors + display status
  if(check_status(1) == 1)  errorLoop();

  while (Serial.available()) Serial.read();
  serialTrigger(F("Press to continue to set DAC value to 0."));

  AD57.setOutput(HOLD);

  // set lowest value.again NOT loading
  AD57.setDac(DAC_CHANNEL, 0x0,0);
  Serial.print(F("should be LOWEST : read from DAC register : "));

  // get DAC value
  Serial.println(AD57.getDac(DAC_CHANNEL), HEX);

  // get analog value if requested
  check_values();

  while (Serial.available()) Serial.read();
  serialTrigger(F("Press to continue and load and see the difference in analog output."));

  // instead of releasing the hardware LDAC pin with  AD57.setOutput(RELEASE) we can also use the
  // software load command. The LDAC pin stays high in that case
  AD57.loadDac();

  check_values();

  while (Serial.available()) Serial.read();
  serialTrigger(F("Press to continue and close down power"));

  // close down power
  AD57.setPower(STOP_PUA | STOP_PUB);

  check_values();
}

/* initialize the connection with the board  and set the default values */
void init_AD57xx()
{
   // set output voltage range
  AD57.begin(DAC_CHANNEL, output_range);

  // setup (optional) requested analog input
  #ifdef DAC_A_INPUT
    pinMode(DAC_A_INPUT, INPUT);
  #endif

  #ifdef DAC_B_INPUT
    pinMode(DAC_B_INPUT, INPUT);
  #endif

  // enable therminal shutdown and overcurrent protection
  // also set make sure that CLR is NOT set, starting from 0v after OP_CLEAR op_code)
  AD57.setControl(OP_INSTR,(SET_TSD_ENABLE |SET_CLAMP_ENABLE|STOP_CLR_SET));
}

/* calculate estimated voltage level (only positive)
 *
 * p5V          +5V
 * p10V         +10V
 * p108V        +10.8V
 */

float estimate_voltage(int value)
{
  if ( output_range == p5V)    return (float) (5 * value / 1023);
  if ( output_range == p10V)   return (float) (10 * value / 1023);
  if ( output_range == p108V)  return (float) (10.8 *value / 1023);
}

/* read Dac values and analog input (if requested) and display estimated voltage level*/
void check_values()
{
  int value;

  #ifdef DAC_A_INPUT

    if (DAC_CHANNEL == DAC_A || DAC_CHANNEL == DAC_AB)
    {
      value = analogRead(DAC_A_INPUT);
      Serial.print(F("Reading DAC-A analog as : "));
      Serial.print(value);
      Serial.print(F(" : estimated voltage : "));
      Serial.println(estimate_voltage(value),2);
  }
  #endif

  #ifdef DAC_B_INPUT

    if (DAC_CHANNEL == DAC_B || DAC_CHANNEL == DAC_AB)
    {
      value = analogRead(DAC_B_INPUT);
      Serial.print(F("Reading DAC-B analog as : "));
      Serial.print(value);
      Serial.print(F(" : estimated voltage : "));
      Serial.println(estimate_voltage(value),2);
    }
  #endif

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

