//==============================================================================
// System Name:    Control_ACIM_F28335
//
// File Name:      Control_ACIM_F28335_Header.h
//
// Description:    Primary system header file for the Real Implementation of
//                 Field Orientation Control for a Three Phase AC Induction
//                 Motor
//==============================================================================
// math blocks
#include "cur_mod.h"            // Include header for the CURMOD object
#include "cur_const.h"          // Include header for the CURCONST object
#include "park.h"               // Include header for the PARK object
#include "ipark.h"              // Include header for the IPARK object
#include "clarke.h"             // Include header for the CLARKE object
#include "svgen.h"              // Include header for the SVGENDQ object
#include "rampgen.h"            // Include header for the RAMPGEN object
#include "rmp_cntl.h"           // Include header for the RMPCNTL object
#include "speed_fr.h"           // Include header for the SPEED_MEAS_QEP object
#include "pi.h"                 // Include header for the PI object

// driver blocks
#include "f2833xpwm.h"
#include "f2833xpwmdac.h"       // Include header for the PWMGEN object
#include "f2833xqep.h"          // Include header for the QEP object
#include "f2833xileg_vdc.h"     // Include header for the ILEG2DCBUSMEAS object
#include "DSP2833x_EPwm_defines.h" // Include header for PWM defines

//==============================================================================
// End of file.
//==============================================================================
