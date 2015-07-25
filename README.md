XmegaBride
==============

XmegaBridge is a port of the XPLAINBrige code in the LUFA library to work on ATxmega devices.
The original code can be found here: https://github.com/abcminiuser/lufa/tree/master/Projects/XPLAINBridge
More information about the original XPLAINED board can be found here: http://www.fourwalledcubicle.com/XPLAIN.php

Goal
--------------
The XmegaBrige project allows for the xmega device to act as a PDI programmer, or as a USB to Serial converter using the USB CDC class.
The main goal of this port/project is to port the PDI functionality and get the USB to Serial to work on the xmega (still using the LUFA stack of course).

Additional features/changes
--------------
As the code will be used on a custom board and some extra features are needed, some changes and additions shall be made to the code. (meaning it will end up as a bit more than a port)