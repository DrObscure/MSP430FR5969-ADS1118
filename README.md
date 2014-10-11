MSP430FR5969-ADS1118
====================

MSP430FR5969 Launchpad with ADS1118 Booster pack and a Type K thermocouple.  
This is a rewrite of the original example code supplied by TI written for the MSP430G2553.

The code no longer uses the Grace graphical tools, but instead uses the DriverLib library.

The code is written for the Code Composer Studio 6.0.1

This version of the code follows the original example closely with the following exceptions/additions:  

The pinouts from the original launchpad were changed on the FR5969 board

SW1 on the launchpad is used to switch between CH0 and CH1 inputs to the ADS1118
The internal RTC in the FR5969 is used for the current time (24 hour clock)
A temp/time feed is output through the UART to the PC at 9600/8/1, every .5 sec
     
Otherwise, the operation is as follows:\

    To reset threshold temp: (Note this in Centigrade)
       Press SW1 on the boost pack to start at first digit, using SW2 on the launchpad to increment values
       Press SW1 to move through the digits and then commit the change

    To reset clock:
       Press SW2 on the boost pack to start at the hours, using SW2 on the launchpad to increment values.
       Press SW2 on the pack to advance to minutes, using SW2 on teh lanuchpad to increment as before.
       Finally press SW2 on teh boost pack to commit the changes

    To toggle between Fahrenheit and Centigrade readout press SW2 on the launchpad

    To toggle between CH0 and CH1 ADS inputs press SW1 on the launchpad
       
