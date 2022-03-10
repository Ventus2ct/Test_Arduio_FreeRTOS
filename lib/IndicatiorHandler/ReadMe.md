Usage:
 // Create a sub-directory called "IndicatiorHandler" where the .ino file resides
 // Copy the .h and .cpp file into a sub-directory called "IndicatiorHandler" 
 // Include the header file in your ino file:
 #include "IndicatiorHandler.h"  

 // Global init:
 IndicatiorHandler indicator;

 // In setup()
 indicator.Enable();

 // In loop()
 indicator.IndicatiorUpdate();