Library for handling AD5722 / AD5732 / AD5752 from Analog Devices
=============================================
About Analog Devices:
©2008–2017 Analog Devices, Inc. All rights reserved. Trademarks and
registered trademarks are the property of their respective owners.
D06467-0-2/17(F)

This library and examples demonstrate the usage of the DAC AD57xx.
On top in sketch is an explanation and hardware pin setup. It has been
tested with an Arduino Uno and an AD5722.

##  Examples provided.

    AD57_manual_5V
        Allows to set the DAC values between 0 and 5V

    AD_man_5V_load_cntrl
        Allows to set the DAC values between 0 and 5V and delay the
        loading into the DAC registers to start the output based on
        using a seperate PIN and/or software.

    AD57_loop_bipolar:
        will provide a continues loop between +5V and -5V.

    AD57_man_bin_2comp_bipolar
        Allows to set the DAC in either Binary or Two complement
        output, demonstrate the 2 modes for CLEAR command pre-set and
        set a DAC value manually to a value between -5V and +5V.

    AD57_man_bipolar
        Allows to set the DAC value between -5V and +5V in binary
        mode and demonstrate how to set different values and power down
        the DAC.

##  calls and subroutines

Available in the extras folder is AD57 library commands.odt. This
contains an overview of all the library calls, constants and remarks.

##  documentation

The datasheet AD5722/5732/5752 is available in the extras folder.
At the top of AD57.h, the AD57 library overview and in the different
sketches there are additional remarks with respect to LDAC,
BIN/2sComp, RFIN and library usage.

## License Information

This product is _**open source**_!

This code is released under the MIT license

Please use, reuse, and modify these files as you see fit. Please
maintain attribution to this library and release anything derivative
under the same license.

Distributed as-is; no warranty is given.

Paul van Haastrecht
Fabruary 2018
