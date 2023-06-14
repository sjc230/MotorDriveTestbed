/* =================================================================================
File name:  HVACI_Sensored-Settings.H
====================================================================================  */


#ifndef PROJ_SETTINGS_H

/*------------------------------------------------------------------------------
Following is the list of the Build Level choices.
------------------------------------------------------------------------------*/
#define LEVEL1  1           // Module check out, duty cycle waveforms and PWM update
#define LEVEL2  2           // Verify ADC, park/clarke, calibrate the offset
#define LEVEL3  3           // Two current PI regulator and speed measurement test
#define LEVEL4  4           // Sensored closed-loop FOC

/*------------------------------------------------------------------------------
This line sets the BUILDLEVEL to one of the available choices.
------------------------------------------------------------------------------*/
#define   BUILDLEVEL LEVEL4


#ifndef BUILDLEVEL
#error  Critical: BUILDLEVEL must be defined !!
#endif  // BUILDLEVEL
//------------------------------------------------------------------------------


#ifndef TRUE
#define FALSE 0
#define TRUE  1
#endif

#define PI 3.14159265358979

// Define the system frequency (MHz)
#define SYSTEM_FREQUENCY 150

// Define the ISR frequency (kHz)
#define ISR_FREQUENCY 10

// Define the electrical motor parametes (1/4 hp Marathon Motor)
#define RS      1.85                       // Stator resistance (ohm)
#define RR      1.98                        // Rotor resistance (ohm)
#define LS      0.10832                 // Stator inductance (H)
#define LR      0.11156                 // Rotor inductance (H)
#define LM      0.10185                   // Magnatizing inductance (H)
#define POLES   6                           // Number of poles

// Define the base quantites for PU system conversion
#define BASE_VOLTAGE    220       // Base peak phase voltage (volt)
#define BASE_CURRENT    3.4            // Base peak phase current (amp)
#define BASE_TORQUE                   // Base torque (N.m)
#define BASE_FLUX                     // Base flux linkage (volt.sec/rad)
#define BASE_FREQ       150           // Base electrical frequency (Hz)
                                      // Note that 0.5 pu (1800 rpm) is max for this motor
                                      // Above 1800 rpm, field weakening is needed.
#endif

