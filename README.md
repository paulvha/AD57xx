# AD57xx
Arduino library for DAC AD5752 / AD5732 / AD5722

The current version is untested and written based on the specifications / data sheet
Testing will be performed as soon as an AD5722 arrives.. 

For pin layout

 * pin usage	action
 *  1	AVss	  Analog negative voltage Bipolor mode : -4 to -16V, otherwise GND
 *  2   		  Not connected
 *  3	outA	  Analog voltage out of DAC_A channel
 *  4			    Not connected
 *  5   BIN 	Connect to VCC unless you got a clue (see remark 1)
 *  6			    Not connected
 *  7	sync	  connect to the slave_pin_DAC (defined in DEFINITION section below)
 *  8   clock	serial clock : connect to clk (UNO : pin 13)
 *  9   sdin	data in : connect to Mosi     (UNO : pin 11)
 * 10   LDAC	connect to ground or pin      (see remark 2)
 * 11   CLR		connect Vcc (pin 14)          (Is software controlled with OP_CLEAR)
 * 12 			  Not connected
 * 
 * 13			    Not Connected
 * 14	VCC		  connect to 2.7V to 5.5V 
 * 15	GND		  connect GND
 * 16	SDO		  serial data out : connect to MISO  (UNO: pin 12)
 * 17	REFIN	  connect to 2 to 3V, typically 2.5V (remark 3)
 * 18	DAC-gnd connect GND
 * 19	DAC-GND connect GND
 * 20	SIG-GND connect GND
 * 21	SIG-GND connect GND
 * 22			    Not Connected
 * 23	outB	  Analog voltage out of DAC_B channel
 * 24	AVdd	  Analog positive voltage: +4V to +16V
 
 More information to follow
 
 Paul
