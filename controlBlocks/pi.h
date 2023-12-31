/* =================================================================================
File name:       PI.H 
===================================================================================*/


#ifndef __PI_H__
#define __PI_H__

typedef struct {  _iq  Ref;   			// Input: reference set-point
				  _iq  Fbk;   			// Input: feedback
				  _iq  Out;   			// Output: controller output 
				  _iq  Kp;				// Parameter: proportional loop gain
				  _iq  Ki;			    // Parameter: integral gain
				  _iq  Umax;			// Parameter: upper saturation limit
				  _iq  Umin;			// Parameter: lower saturation limit
				  _iq  up;				// Data: proportional term
				  _iq  ui;				// Data: integral term
				  _iq  v1;				// Data: pre-saturated controller output
				  _iq  i1;				// Data: integrator storage: ui(k-1)
				  _iq  w1;				// Data: saturation record: [u(k-1) - v(k-1)]
				} PI_CONTROLLER;


/*-----------------------------------------------------------------------------
Default initalisation values for the PI_GRANDO objects
-----------------------------------------------------------------------------*/                     

#define PI_CONTROLLER_DEFAULTS {		\
						   0, 			\
                           0, 			\
						   0, 			\
                           _IQ(1.0),	\
                           _IQ(0.0),	\
                           _IQ(1.0),	\
                           _IQ(-1.0), 	\
                           _IQ(0.0),	\
                           _IQ(0.0), 	\
                           _IQ(0.0),	\
                           _IQ(0.0),	\
                           _IQ(1.0) 	\
              			  }


/*------------------------------------------------------------------------------
 	PI_GRANDO Macro Definition
------------------------------------------------------------------------------*/

#define PI_MACRO(v)												\
																\
	/* proportional term */ 									\
	v.up = v.Ref - v.Fbk;										\
																\
	/* integral term */ 										\
	v.ui = (v.Out == v.v1)?(_IQmpy(v.Ki, v.up)+ v.i1) : v.i1;	\
	v.i1 = v.ui;												\
																\
	/* control output */ 										\
	v.v1 = _IQmpy(v.Kp, v.up)  + v.ui;							\
	v.Out= _IQsat(v.v1, v.Umax, v.Umin);						\
	v.w1 = (v.Out == v.v1) ? _IQ(1.0) : _IQ(0.0);				\
	
#endif // __PI_H__

