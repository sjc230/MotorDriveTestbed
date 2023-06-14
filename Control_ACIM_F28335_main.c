//==============================================================================
// Project Name: AC Induction Motor FOC Control with TMS320F28335
//
// File Name: Control_ACIM_F28335_main.c
//
// Description: main file of the project
//==============================================================================

//==============================================================================
// Include header files used in main file & define code sections
#include "IQmathLib.h"
#include <math.h>
#include "PeripheralHeaderIncludes.h"
#include "Control_ACIM_F28335_Settings.h"
#include "Control_ACIM_F28335_Header.h"

#ifdef FLASH
#pragma CODE_SECTION(MainISR,"ramfuncs");
#pragma CODE_SECTION(OffsetISR,"ramfuncs");
#endif
//==============================================================================

//==============================================================================
// Prototype statements for functions found within this file
interrupt void MainISR(void);
interrupt void OffsetISR(void);

void DeviceInit();
void MemCopy();
void InitFlash();
void HVDMC_Protection(void);

// Used for running BackGround in flash, and ISR in RAM
extern Uint16 *RamfuncsLoadStart, *RamfuncsLoadEnd, *RamfuncsRunStart;

// Global variables used in this system
// Default GLOBAL_Q = 24
//==============================================================================

//==============================================================================
// Control Parameters Settings & Initialization
//------------------------------------------------------------------------------
// System parameter

// control sampling frequency
float32 T = 0.001/ISR_FREQUENCY;     // Samping period (sec), see Settings.h

// Ticker initialization
Uint32 IsrTicker = 0;
Uint16 BackTicker = 0;

Uint16 SpeedLoopPrescaler = 1;      // Speed loop prescaler
Uint16 SpeedLoopCount = 1;           // Speed loop counter

//------------------------------------------------------------------------------
// ADC parameters

// ADC offset initialization
_iq offsetA=0;
_iq offsetB=0;
_iq offsetC=0;

// ADC filter settings
_iq K1=_IQ(0.998);      //Offset filter coefficient K1: 0.05/(T+0.05);
_iq K2=_IQ(0.001999);   //Offset filter coefficient K2: T/(T+0.05);
//// ADC filter coefficients (1st order butterworth low pass filter)
//_iq K1_adc=_IQ(0.8508);        //ADC filter K1
//_iq K2_adc=_IQ(0.0746);        //ADC filter K2
//_iq K3_adc=_IQ(0.0746);        //ADC filter K3
//// ADC filter temporary memory variable
//_iq tempY_A=0;
//_iq tempY_B=0;
//_iq tempY_C=0;
//_iq tempX_A=0;
//_iq tempX_B=0;
//_iq tempX_C=0;
//_iq tempXX_A=0;
//_iq tempXX_B=0;
//_iq tempXX_C=0;

// Sensor gain settings
_iq SensorGain=_IQ(3*20/BASE_CURRENT); // current sensor: TMCS1107A1B 50mV/A

// Default ADC initialization
int ChSel[16]   = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int TrigSel[16] = {5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5};
int ACQPS[16]   = {8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8};

//------------------------------------------------------------------------------
// feed-forward variables
_iq Ffd_d=0;
_iq Ffd_q=0;
// voltage commands before saturation
_iq vds=0;
_iq vqs=0;

//------------------------------------------------------------------------------
// sin & cos tables initialization
extern _iq IQsinTable[];
extern _iq IQcosTable[];

//------------------------------------------------------------------------------
// Sensor protection thresholds
#if (BUILDLEVEL<=LEVEL2)
Uint16 limCurrentMax = 3800;  // maximum protection threshold for open-loop
Uint16 limCurrentMin = 200;     // minimum protection threshold for open-loop
#else
Uint16 limCurrentMax = 2457;  // 6A
Uint16 limCurrentMin = 1639;  // -6A
#endif

//------------------------------------------------------------------------------
// Reference settings
volatile _iq VdTesting = _IQ(0.1);           // Vd reference (pu)
volatile _iq VqTesting = _IQ(0.1);          // Vq reference (pu)

volatile _iq IdRef = _IQ(0.5);               // Id reference (pu)
volatile _iq IqRef = _IQ(0.5);               // Iq reference (pu)

#if (BUILDLEVEL<LEVEL3)                      // Speed reference (pu)
volatile _iq  SpeedRef = _IQ(0.2);           // For Open Loop tests
#else
volatile _iq  SpeedRef = _IQ(0.333333);          // For Closed Loop tests
#endif

//------------------------------------------------------------------------------
// Flags initialization
// PWM trip status flag
Uint16 TripFlagDMC=0;          //PWM trip status

// external enable flag
volatile Uint16 EnableFlag = FALSE;
volatile Uint16 DisableFlag = FALSE;
volatile Uint16 HardstopFlag = FALSE;

//------------------------------------------------------------------------------
// monitoring data array initialization
#define monitorSize 100
Uint16 monitorTicker = 0;
Uint16 monitorDivider = 2;
Uint16 monitorDivTicker = 0;
_iq dataLogA[monitorSize] = { [0 ... monitorSize-1] = 0 };
_iq dataLogB[monitorSize] = { [0 ... monitorSize-1] = 0 };
_iq dataA[monitorSize] = { [0 ... monitorSize-1] = 0 };
_iq dataB[monitorSize] = { [0 ... monitorSize-1] = 0 };
//==============================================================================

//==============================================================================
// Instance of controller modules

// Instance a current model object
CURMOD cm1 = CURMOD_DEFAULTS;

// Instance a current model constant object
CURMOD_CONST cm1_const = CURMOD_CONST_DEFAULTS;

// Instance a QEP interface driver
QEP qep1 = QEP_DEFAULTS;

// Instance a few transform objects (ICLARKE is added into SVGEN module)
CLARKE clarke1 = CLARKE_DEFAULTS;
PARK park1 = PARK_DEFAULTS;
IPARK ipark1 = IPARK_DEFAULTS;

// Instance PI regulators to regulate the d and q  axis currents, and speed
PI_CONTROLLER pi_spd = PI_CONTROLLER_DEFAULTS;
PI_CONTROLLER pi_id  = PI_CONTROLLER_DEFAULTS;
PI_CONTROLLER pi_iq  = PI_CONTROLLER_DEFAULTS;

// Instance a PWM driver instance
PWMGEN pwm1 = PWMGEN_DEFAULTS;

// Instance a PWM DAC driver instance
PWMDAC pwmdac1 = PWMDAC_DEFAULTS;

// Instance a Space Vector PWM modulator. This modulator generates a, b and c
// phases based on the d and q stationery reference frame inputs
SVGEN svgen1 = SVGEN_DEFAULTS;

// Instance a ramp controller to smoothly ramp the frequency
RMPCNTL rc1 = RMPCNTL_DEFAULTS;

//  Instance a ramp(sawtooth) generator to simulate an Anglele
RAMPGEN rg1 = RAMPGEN_DEFAULTS;

// Instance a speed calculator based on QEP
SPEED_MEAS_QEP speed1 = SPEED_MEAS_QEP_DEFAULTS;

//==============================================================================
// MAIN CODE
void main(void)
{
    DeviceInit();   // Device Life support & GPIO

    // Only used if running from FLASH
    // Note that the variable FLASH is defined by the compiler
    #ifdef FLASH
    // Copy time critical code and Flash setup code to RAM
    // The  RamfuncsLoadStart, RamfuncsLoadEnd, and RamfuncsRunStart
    // symbols are created by the linker. Refer to the linker files.
        MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);

    // Call Flash Initialization to setup flash waitstates
    // This function must reside in RAM
        InitFlash();    // Call the flash wrapper init function
    #endif //(FLASH)

    //--------------------------------------------------------------------------
    // Waiting for enable flag set
    while (EnableFlag==FALSE)
    {
        BackTicker++;
    }

    //--------------------------------------------------------------------------
    // Initialization

    // Initialize GPIO for ePWM (GPIO 0 to GPIO 5)
    EALLOW;
    // Enable internal pull-up for the selected pins
    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 0;  // Enable pull-up on GPIO0 (ePWM 1A)
    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 0;  // Enable pull-up on GPIO1 (ePWM 1B)
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 0;  // Enable pull-up on GPIO2 (ePWM 2A)
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 0;  // Enable pull-up on GPIO3 (ePWM 2B)
    GpioCtrlRegs.GPAPUD.bit.GPIO4 = 0;  // Enable pull-up on GPIO4 (ePWM 3A)
    GpioCtrlRegs.GPAPUD.bit.GPIO5 = 0;  // Enable pull-up on GPIO5 (ePWM 3B)
    GpioCtrlRegs.GPAPUD.bit.GPIO10 = 0;  // Enable pull-up on GPIO6 (ePWM 6A)
    GpioCtrlRegs.GPAPUD.bit.GPIO11 = 0;  // Enable pull-up on GPIO7 (ePWM 6B)
    EDIS;

    // Initialize PWM module
    pwm1.PeriodMax = SYSTEM_FREQUENCY*1000000*T/2;
        // PWM half-period in CPU cycles -> 5 kHz
    pwm1.HalfPerMax = pwm1.PeriodMax/2;
//    pwm1.Deadband = SYSTEM_FREQUENCY;
    pwm1.Deadband = 60;
        // PWM deadband in CPU cycles -> 150 cycles -> 1 us
    PWM_INIT_MACRO(1,2,3,pwm1)
        // initialize PWM channel 1,2,3

    // Initialize PWMDAC module
    pwmdac1.PeriodMax=500;
        // @60khz, 1500->20kHz, 1000-> 30kHz, 500->60kHz
    pwmdac1.HalfPerMax=pwmdac1.PeriodMax/2;
    PWMDAC_INIT_MACRO(6,pwmdac1)
        // PWM 6A,6B

    // Initialize ADC for PMSM motor control board (1.5kW)
    ChSel[0]=0;     // Dummy meas. avoid 1st sample issue Rev0 Picollo*/
    ChSel[1]=2;     // ChSelect: ADC A2-> Phase U Current
    ChSel[2]=3;     // ChSelect: ADC A3-> Phase V Current
    ChSel[3]=4;     // ChSelect: ADC A4-> Phase W Current
    ChSel[4]=5;     // ChSelect: ADC A5-> Sim. Mech. Pos. (OPAL-RT Mode)
    ChSel[5]=0;     // ChSelect:
    ChSel[6]=0;     // ChSelect:
    ChSel[7]=0;     // ChSelect:

    // Initialize ADC module
    ADC_MACRO_INIT(ChSel,TrigSel,ACQPS);

    // Initialize QEP module
    qep1.LineEncoder = 1000;
    qep1.MechScaler = _IQ30(0.25/qep1.LineEncoder);
    qep1.PolePairs = (int)(POLES/2);
    qep1.CalibratedAngle = 0;
    QEP_INIT_MACRO(2,qep1);

    // Initialize GPIO for eQEP (eQEP2 - GPIO24 & GPIO25)
    EALLOW;
    // Enable internal pull-up for the selected pins
    GpioCtrlRegs.GPAPUD.bit.GPIO24 = 0;  // Enable pull-up on GPIO24 (EQEP2A)
    GpioCtrlRegs.GPAPUD.bit.GPIO25 = 0;  // Enable pull-up on GPIO25 (EQEP2B)
    // Sync to SYSCLKOUT
    GpioCtrlRegs.GPAQSEL2.bit.GPIO24 = 0;  // Sync to SYSCLKOUT GPIO24(EQEP2A)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO25 = 0;  // Sync to SYSCLKOUT GPIO25(EQEP2B)
    // Configure eQEP-2 pins using GPIO
    GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 2;  // Configure GPIO24 as EQEP2A
    GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 2;  // Configure GPIO25 as EQEP2B
    EDIS;

    //============================================================================== !!!!!! Update Parameters !!!!!!
    // Initialize the Speed module for QEP based speed calculation
    speed1.K1 = _IQ21(1/(BASE_FREQ*T));
    speed1.K2 = _IQ(1/(1+T*2*PI*5));      // Low-pass cut-off frequency
    speed1.K3 = _IQ(1)-speed1.K2;
    speed1.BaseRpm = 120*(BASE_FREQ/POLES);

    // Initialize the RAMPGEN module
    rg1.StepAngleMax = _IQ(0.3*BASE_FREQ*T);

    // Initialize the CUR_MOD constant module
    cm1_const.Rr = RR;
    cm1_const.Lr = LR;
    cm1_const.fb = BASE_FREQ;
    cm1_const.Ts = T;
    CUR_CONST_MACRO(cm1_const)

    // Initialize the CUR_MOD module
    cm1.Kr = _IQ(cm1_const.Kr);
    cm1.Kt = _IQ(cm1_const.Kt);
    cm1.K  = _IQ(cm1_const.K);

    // Initialize the PI module for speed
    pi_spd.Kp=_IQ(2.0);
    pi_spd.Ki=_IQ(T*SpeedLoopPrescaler/0.5);
    pi_spd.Umax =_IQ(0.95);
    pi_spd.Umin =_IQ(-0.95);

    // Initialize the PI module for id
    pi_id.Kp=_IQ(1.0);
    pi_id.Ki=_IQ(T/0.004);
    pi_id.Umax =_IQ(0.5);
    pi_id.Umin =_IQ(-0.5);

    // Initialize the PI module for iq
    pi_iq.Kp=_IQ(1.0);
    pi_iq.Ki=_IQ(T/0.004);
    pi_iq.Umax =_IQ(0.8);
    pi_iq.Umin =_IQ(-0.8);

    // Initialize the push button 2 (GPIO 48)
    EALLOW;
    GpioCtrlRegs.GPBPUD.bit.GPIO48 = 0;  // Enable pull-up
    GpioCtrlRegs.GPBDIR.bit.GPIO48 = 0;     // 1=OUTput,  0=INput
    EDIS;

//    // Enable voltage level translator
//    EALLOW;
//    // Enable PWM (effective low)
//    GpioCtrlRegs.GPBPUD.bit.GPIO60 = 0;
//    GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;
//    // Enable Encoder (effective high)
//    GpioCtrlRegs.GPBPUD.bit.GPIO61 = 0;
//    GpioDataRegs.GPBSET.bit.GPIO61 = 1;
//    EDIS;

    //============================================================================== !!!!!! Update Parameters !!!!!!

    // Initialization Complete
    //--------------------------------------------------------------------------
    // Call HVDMC Protection function
    HVDMC_Protection();

    //--------------------------------------------------------------------------
    // Enable ISRs

    // Reassign ISRs
    EALLOW;
    PieVectTable.EPWM1_INT = &OffsetISR;
    EDIS;

    // Enable PIE group 3 interrupt 1 for EPWM1_INT
    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;

    // Enable CNT_zero interrupt using EPWM1 Time-base
    EPwm1Regs.ETSEL.bit.INTEN = 1;   // Enable EPWM1INT generation
    EPwm1Regs.ETSEL.bit.INTSEL = 1;  // Enable interrupt CNT_zero event
    EPwm1Regs.ETPS.bit.INTPRD = 1;   // Generate interrupt on the 1st event
    EPwm1Regs.ETCLR.bit.INT = 1;     // Enable more interrupts

    // Enable CPU INT3 for EPWM1_INT:
    IER |= M_INT3;

    // Enable global Interrupts and higher priority real-time debug events:
    EINT;   // Enable Global interrupt INTM
    ERTM;   // Enable Global real-time interrupt DBGM

    // ISR Enabled
    //--------------------------------------------------------------------------
    // IDLE loop. Just sit and loop forever:
    for(;;)  //infinite loop
    {

    }
}
// END MAIN CODE
//==============================================================================

//==============================================================================
// MainISR
interrupt void MainISR(void)
{
    // Verifying the ISR
    IsrTicker++;

    // software shut-down the system
    // soft shut-down
    if (DisableFlag == TRUE)
    {
        SpeedRef = _IQ(0.0);            // Speed reference (pu)
        IdRef = _IQ(0.1);               // Id reference (pu)
        IqRef = _IQ(0.0);               // Iq reference (pu)
    }

    // hardware shut-down (force PWM to low)
    EALLOW;
        if ((GpioDataRegs.GPBDAT.bit.GPIO48 == 0) || (HardstopFlag == TRUE))
        {
           EPwm1Regs.TZFRC.bit.OST   = 1;  // force a trip-zone protection
           EPwm2Regs.TZFRC.bit.OST   = 1;  // force a trip-zone protection
           EPwm3Regs.TZFRC.bit.OST   = 1;  // force a trip-zone protection
        }
    EDIS;

    // Software Protection
    // Current exceeds +-10A will trigger the trip-zone signal
    EALLOW;
        if ((AdcMirror.ADCRESULT1 > limCurrentMax) || \
                (AdcMirror.ADCRESULT1 < limCurrentMin))
        {
           EPwm1Regs.TZFRC.bit.OST   = 1;  // force a trip-zone protection
           EPwm2Regs.TZFRC.bit.OST   = 1;  // force a trip-zone protection
           EPwm3Regs.TZFRC.bit.OST   = 1;  // force a trip-zone protection
        }
        if ((AdcMirror.ADCRESULT2 > limCurrentMax) || \
                (AdcMirror.ADCRESULT2 < limCurrentMin))
        {
           EPwm1Regs.TZFRC.bit.OST   = 1;  // force a trip-zone protection
           EPwm2Regs.TZFRC.bit.OST   = 1;  // force a trip-zone protection
           EPwm3Regs.TZFRC.bit.OST   = 1;  // force a trip-zone protection
        }
        if ((AdcMirror.ADCRESULT3 > limCurrentMax) || \
                (AdcMirror.ADCRESULT3 < limCurrentMin))
        {
           EPwm1Regs.TZFRC.bit.OST   = 1;  // force a trip-zone protection
           EPwm2Regs.TZFRC.bit.OST   = 1;  // force a trip-zone protection
           EPwm3Regs.TZFRC.bit.OST   = 1;  // force a trip-zone protection
        }
    EDIS;

    // LEVEL 1 =================================================================
    // Checks target PWM generation and IGBT gate driver functionalities
    // Checks the eQep readings as well
    // Keep the motor disconnected at this level
    //==========================================================================

    #if (BUILDLEVEL==LEVEL1)
        //----------------------------------------------------------------------
        // Connect inputs of the RMP module
        // Call the ramp control macro
        // *gradually increase the speed set point to the SpeedRef
        //----------------------------------------------------------------------
        rc1.TargetValue = SpeedRef;
        RC_MACRO(rc1);

        //----------------------------------------------------------------------
        // Connect inputs of the RAMP GEN module
        // Call the ramp generator macro
        // *generate ramp signal with frequency = SpeedRef * BaseFreq
        //----------------------------------------------------------------------
        rg1.Freq = rc1.SetpointValue;
        RG_MACRO(rg1);

        //----------------------------------------------------------------------
        // Call the QEP calculation module
        //----------------------------------------------------------------------
        QEP_MACRO(2,qep1);

        //----------------------------------------------------------------------
        // Connect inputs of the INV_PARK module
        // Call the inverse park trans. macro
        //----------------------------------------------------------------------
        ipark1.Ds = VdTesting;
        ipark1.Qs = VqTesting;

        ipark1.Sine=_IQsinPU(rg1.Out);
        ipark1.Cosine=_IQcosPU(rg1.Out);
        IPARK_MACRO(ipark1);

        //----------------------------------------------------------------------
        // Connect inputs of the SVGEN_DQ module
        // Call the space-vector gen. macro
        //----------------------------------------------------------------------
        svgen1.Ualpha = ipark1.Alpha;
        svgen1.Ubeta = ipark1.Beta;
        SVGENDQ_MACRO(svgen1);

        //----------------------------------------------------------------------
        // Connect inputs of the PWM_DRV module
        // Call the PWM signal generation macro
        //----------------------------------------------------------------------
        pwm1.MfuncC1 = svgen1.Ta;
        pwm1.MfuncC2 = svgen1.Tb;
        pwm1.MfuncC3 = svgen1.Tc;
        PWM_MACRO(1,2,3,pwm1);      // Calculate the new PWM compare values

        //----------------------------------------------------------------------
        // Update monitoring data log
        //----------------------------------------------------------------------
        if (monitorTicker < monitorSize)
        {
            if (monitorDivTicker < monitorDivider)
            {
                monitorDivTicker++;
            }
            else
            {
               dataLogA[monitorTicker] = qep1.ElecTheta;
               dataLogB[monitorTicker] = qep1.MechTheta;
               monitorTicker++;
               monitorDivTicker = 0;
            }
        }
        else
        {
            memcpy(dataA, dataLogA, sizeof dataA);
            memcpy(dataB, dataLogB, sizeof dataB);
            monitorTicker = 0;
        }

    #endif // (BUILDLEVEL==LEVEL1)

    // LEVEL 2 =================================================================
    // Motor Open-Loop Testing
    // Checks ADC calculation, eQEP, speed calculation
    //==========================================================================

    #if (BUILDLEVEL==LEVEL2)
        //----------------------------------------------------------------------
        // Connect inputs of the RMP module
        // Call the ramp control macro
        //----------------------------------------------------------------------
        rc1.TargetValue = SpeedRef;
        RC_MACRO(rc1);

        //----------------------------------------------------------------------
        // Connect inputs of the RAMP GEN module
        // Call the ramp generator macro
        //----------------------------------------------------------------------
        rg1.Freq = rc1.SetpointValue;
        RG_MACRO(rg1);

        //----------------------------------------------------------------------
        // Connect inputs of the CLARKE module
        // Call the clarke transformation macro
        // *measure phase currents,
        // subtract the offset and normalize from (-0.5,+0.5) to (-1,+1).
        //----------------------------------------------------------------------
        // current sensor output (-3V - 3V) = ((ADCmeas(q12)/2^12)*3)*2-3
        clarke1.As = \
                _IQmpy(SensorGain, (_IQ12toIQ(AdcMirror.ADCRESULT1)-offsetA));
        clarke1.Bs = \
                _IQmpy(SensorGain, (_IQ12toIQ(AdcMirror.ADCRESULT2)-offsetB));
        clarke1.Cs = \
                _IQmpy(SensorGain, (_IQ12toIQ(AdcMirror.ADCRESULT3)-offsetC));

        CLARKE_MACRO(clarke1);

        //----------------------------------------------------------------------
        // Connect inputs of the PARK module
        // Call the park trans. macro
        //----------------------------------------------------------------------
        park1.Alpha = clarke1.Alpha;
        park1.Beta = clarke1.Beta;
        park1.Angle = rg1.Out;
        park1.Sine = _IQsinPU(park1.Angle);
        park1.Cosine = _IQcosPU(park1.Angle);
        PARK_MACRO(park1);

        //----------------------------------------------------------------------
        // Connect inputs of the INV_PARK module
        // Call the inverse park trans. macro
        //----------------------------------------------------------------------
        ipark1.Ds = VdTesting;
        ipark1.Qs = VqTesting;
        ipark1.Sine=park1.Sine;
        ipark1.Cosine=park1.Cosine;
        IPARK_MACRO(ipark1);

        //----------------------------------------------------------------------
        // Call the QEP calculation module
        //----------------------------------------------------------------------
        QEP_MACRO(2,qep1);

        //----------------------------------------------------------------------
        // Connect inputs of the SPEED_FR module
        // Call the speed calculation macro
        //----------------------------------------------------------------------
        speed1.ElecTheta = qep1.ElecTheta;
        speed1.DirectionQep = (int32)(qep1.DirectionQep);
        SPEED_FR_MACRO(speed1);

        //----------------------------------------------------------------------
        // Connect inputs of the SVGEN_DQ module
        // Call the space-vector gen. macro
        //----------------------------------------------------------------------
        svgen1.Ualpha = ipark1.Alpha;
        svgen1.Ubeta = ipark1.Beta;
        SVGENDQ_MACRO(svgen1);

        //----------------------------------------------------------------------
        // Connect inputs of the PWM_DRV module
        // Call the PWM signal generation macro
        //----------------------------------------------------------------------
        pwm1.MfuncC1 = svgen1.Ta;
        pwm1.MfuncC2 = svgen1.Tb;
        pwm1.MfuncC3 = svgen1.Tc;
        PWM_MACRO(1,2,3,pwm1);       // Calculate the new PWM compare values

        //----------------------------------------------------------------------
        // Connect inputs of the PWMDAC module
        //----------------------------------------------------------------------
        pwmdac1.MfuncC1 = qep1.ElecTheta;
        pwmdac1.MfuncC2 = qep1.MechTheta;
        PWMDAC_MACRO(6,pwmdac1)                         // PWMDAC 6A, 6B

        //----------------------------------------------------------------------
        // Update monitoring data log
        //----------------------------------------------------------------------
        if (monitorTicker < monitorSize)
        {
            if (monitorDivTicker < monitorDivider)
            {
                monitorDivTicker += 1;
            }
            else
            {
               dataLogA[monitorTicker] = clarke1.As;
               dataLogB[monitorTicker] = clarke1.Bs;
               monitorTicker += 1;
               monitorDivTicker = 0;
            }
        }
        else
        {
            memcpy(dataA, dataLogA, sizeof dataA);
            memcpy(dataB, dataLogB, sizeof dataB);
            monitorTicker = 0;
        }

    #endif // (BUILDLEVEL==LEVEL2)
    // LEVEL 3 =================================================================
    // Motor Current Closed-Loop Testing
    // Checks the current loop PI controller
    //==========================================================================
    #if (BUILDLEVEL==LEVEL3)
        //----------------------------------------------------------------------
        // Connect inputs of the RMP module
        // Call the ramp control macro
        //----------------------------------------------------------------------
        rc1.TargetValue = SpeedRef;
        RC_MACRO(rc1);

        //----------------------------------------------------------------------
        // Connect inputs of the RAMP GEN module
        // Call the ramp generator macro
        //----------------------------------------------------------------------
        rg1.Freq = rc1.SetpointValue;
        RG_MACRO(rg1);

        //----------------------------------------------------------------------
        // Call the QEP calculation module
        //----------------------------------------------------------------------
        QEP_MACRO(2,qep1);

        //----------------------------------------------------------------------
        // Measure phase currents, subtract the offset and normalize.
        // Connect inputs of the CLARKE module
        // Call the clarke transformation macro
        //----------------------------------------------------------------------
        // current sensor output (current sensor ratio 100mV/A)
        clarke1.As = \
                _IQmpy(SensorGain, (_IQ12toIQ(AdcMirror.ADCRESULT1)-offsetA));
        clarke1.Bs = \
                _IQmpy(SensorGain, (_IQ12toIQ(AdcMirror.ADCRESULT2)-offsetB));
        clarke1.Cs = \
                _IQmpy(SensorGain, (_IQ12toIQ(AdcMirror.ADCRESULT3)-offsetC));

        CLARKE_MACRO(clarke1);
        //----------------------------------------------------------------------
       // Connect inputs of the PARK module
       // Call the park trans. macro
       //----------------------------------------------------------------------
       park1.Alpha = clarke1.Alpha;
       park1.Beta = clarke1.Beta;

       // choose position inputs (internal, OPAL-RT, or encoder)
//       park1.Angle = rg1.Out;
//       park1.Angle = qep1.ElecTheta;
//       park1.Angle = _IQ12toIQ(AdcMirror.ADCRESULT4);
       park1.Angle = cm1.Theta;

       park1.Sine = _IQsinPU(park1.Angle);
       park1.Cosine = _IQcosPU(park1.Angle);

       PARK_MACRO(park1);

       //----------------------------------------------------------------------
       // Connect inputs of the PI module
       // Call the PI IQ controller macro
       //----------------------------------------------------------------------
       pi_iq.Ref = IqRef;
       pi_iq.Fbk = park1.Qs;
       PI_MACRO(pi_iq);

       //----------------------------------------------------------------------
       // Connect inputs of the PI module
       // Call the PI ID controller macro
       //----------------------------------------------------------------------
       pi_id.Ref = IdRef;
       pi_id.Fbk = park1.Ds;
       PI_MACRO(pi_id);

       //----------------------------------------------------------------------
       // Connect inputs of the INV_PARK module
       // Call the inverse park trans. macro
       //----------------------------------------------------------------------
       ipark1.Ds = pi_id.Out;
       ipark1.Qs = pi_iq.Out;
       ipark1.Sine   = park1.Sine;
       ipark1.Cosine = park1.Cosine;
       IPARK_MACRO(ipark1);

       //----------------------------------------------------------------------
       // Connect inputs of the SPEED_FR module
       // Call the speed calculation macro
       //----------------------------------------------------------------------
       speed1.ElecTheta = qep1.ElecTheta;
       speed1.DirectionQep = (int32)(qep1.DirectionQep);
       SPEED_FR_MACRO(speed1);

       // ------------------------------------------------------------------------------
       //    Connect inputs of the CUR_MOD module and call the current model
       //    calculation function.
       // ------------------------------------------------------------------------------
       cm1.IDs = park1.Ds;
       cm1.IQs = park1.Qs;
       cm1.Wr = speed1.Speed;
       CUR_MOD_MACRO(cm1)

       //----------------------------------------------------------------------
       // Connect inputs of the SVGEN_DQ module
       // Call the space-vector gen. macro
       //----------------------------------------------------------------------
       svgen1.Ualpha = ipark1.Alpha;
       svgen1.Ubeta = ipark1.Beta;
       SVGENDQ_MACRO(svgen1);

       //----------------------------------------------------------------------
       // Connect inputs of the PWM_DRV module
       // Call the PWM signal generation macro
       //----------------------------------------------------------------------
       pwm1.MfuncC1 = svgen1.Ta;
       pwm1.MfuncC2 = svgen1.Tb;
       pwm1.MfuncC3 = svgen1.Tc;

       PWM_MACRO(1,2,3,pwm1);     // Calculate the new PWM compare values

       //----------------------------------------------------------------------
       //  Connect inputs of the PWMDAC module
       //----------------------------------------------------------------------
       pwmdac1.MfuncC1 = qep1.ElecTheta;
       pwmdac1.MfuncC2 = qep1.MechTheta;
//        pwmdac1.MfuncC1 = clarke1.As;
//        pwmdac1.MfuncC2 = clarke1.Bs;

       PWMDAC_MACRO(6,pwmdac1)    // PWMDAC 6A, 6B

       //----------------------------------------------------------------------
       //  Update monitoring data log
       //----------------------------------------------------------------------
       if (monitorTicker < monitorSize)
       {
           if (monitorDivTicker < monitorDivider)
           {
               monitorDivTicker++;
           }
           else
           {
              dataLogA[monitorTicker] = pi_id.Fbk;
              dataLogB[monitorTicker] = pi_iq.Fbk;
              monitorTicker++;
              monitorDivTicker = 0;
           }
       }
       else
       {
           memcpy(dataA, dataLogA, sizeof dataA);
           memcpy(dataB, dataLogB, sizeof dataB);
           monitorTicker = 0;
       }

   #endif // (BUILDLEVEL==LEVEL3)

    // LEVEL 4 =================================================================
    // Motor Closed-Loop Testing
    // Checks the speed close loop control
    //==========================================================================
    #if (BUILDLEVEL==LEVEL4)
        //----------------------------------------------------------------------
        // Connect inputs of the RMP module
        // Call the ramp control macro
        //----------------------------------------------------------------------
        rc1.TargetValue = SpeedRef;
        RC_MACRO(rc1);

        //----------------------------------------------------------------------
        // Connect inputs of the RAMP GEN module
        // Call the ramp generator macro
        //----------------------------------------------------------------------
        rg1.Freq = rc1.SetpointValue;
        RG_MACRO(rg1);

        //----------------------------------------------------------------------
        // Call the QEP calculation module
        //----------------------------------------------------------------------
        QEP_MACRO(2,qep1);

        //----------------------------------------------------------------------
        // Connect inputs of the SPEED_FR module
        // Call the speed calculation macro
        //----------------------------------------------------------------------
        speed1.ElecTheta = qep1.ElecTheta;
        speed1.DirectionQep = (int32)(qep1.DirectionQep);
        SPEED_FR_MACRO(speed1)

        //----------------------------------------------------------------------
        // Measure phase currents, subtract the offset and normalize.
        // Connect inputs of the CLARKE module
        // Call the clarke transformation macro
        //----------------------------------------------------------------------
        // current sensor output (current sensor ratio 100mV/A)
        clarke1.As = \
                _IQmpy(SensorGain, (_IQ12toIQ(AdcMirror.ADCRESULT1)-offsetA));
        clarke1.Bs = \
                _IQmpy(SensorGain, (_IQ12toIQ(AdcMirror.ADCRESULT2)-offsetB));
        clarke1.Cs = \
                _IQmpy(SensorGain, (_IQ12toIQ(AdcMirror.ADCRESULT3)-offsetC));

        CLARKE_MACRO(clarke1);

        //----------------------------------------------------------------------
        // Connect inputs of the PARK module
        // Call the park trans. macro
        //----------------------------------------------------------------------
        park1.Alpha = clarke1.Alpha;
        park1.Beta = clarke1.Beta;

        // choose position inputs (internal, OPAL-RT, encoder, or RRF est.)
//       park1.Angle = rg1.Out;
//       park1.Angle = qep1.ElecTheta;
//       park1.Angle = _IQ12toIQ(AdcMirror.ADCRESULT4);
        park1.Angle = cm1.Theta;

        park1.Sine = _IQsinPU(park1.Angle);
        park1.Cosine = _IQcosPU(park1.Angle);

        PARK_MACRO(park1);

        //----------------------------------------------------------------------
        // Connect inputs of the PI module
        // Call the PI speed controller macro
        //----------------------------------------------------------------------
        if (SpeedLoopCount==SpeedLoopPrescaler)
         {
          pi_spd.Ref = rc1.SetpointValue;
          pi_spd.Fbk = speed1.Speed;
          PI_MACRO(pi_spd);
          SpeedLoopCount=1;
         }
        else SpeedLoopCount++;

        //----------------------------------------------------------------------
        // Connect inputs of the PI module
        // Call the PI IQ controller macro
        //----------------------------------------------------------------------
        pi_iq.Ref = pi_spd.Out;
        pi_iq.Fbk = park1.Qs;
        PI_MACRO(pi_iq);

        //----------------------------------------------------------------------
        // Connect inputs of the PI module
        // Call the PI ID controller macro
        //----------------------------------------------------------------------
        pi_id.Ref = IdRef;
        pi_id.Fbk = park1.Ds;
        PI_MACRO(pi_id);

        //----------------------------------------------------------------------
        // Connect inputs of the INV_PARK module
        // Call the inverse park trans. macro
        //----------------------------------------------------------------------
        ipark1.Ds = pi_id.Out;
        ipark1.Qs = pi_iq.Out;
        ipark1.Sine   = park1.Sine;
        ipark1.Cosine = park1.Cosine;
        IPARK_MACRO(ipark1);

        // ---------------------------------------------------------------------
        // Connect inputs of the CUR_MOD module and call the current model
        // calculation function.
        // ------------------------------------------------------------------------------
        cm1.IDs = park1.Ds;
        cm1.IQs = park1.Qs;
        cm1.Wr = speed1.Speed;
        CUR_MOD_MACRO(cm1)

        //----------------------------------------------------------------------
        // Connect inputs of the SVGEN_DQ module
        // Call the space-vector gen. macro
        //----------------------------------------------------------------------
        svgen1.Ualpha = ipark1.Alpha;
        svgen1.Ubeta = ipark1.Beta;
        SVGENDQ_MACRO(svgen1);

        //----------------------------------------------------------------------
        // Connect inputs of the PWM_DRV module
        // Call the PWM signal generation macro
        //----------------------------------------------------------------------
        pwm1.MfuncC1 = svgen1.Ta;
        pwm1.MfuncC2 = svgen1.Tb;
        pwm1.MfuncC3 = svgen1.Tc;

        PWM_MACRO(1,2,3,pwm1);     // Calculate the new PWM compare values

        //----------------------------------------------------------------------
        //  Connect inputs of the PWMDAC module
        //----------------------------------------------------------------------
        pwmdac1.MfuncC1 = qep1.ElecTheta;
        pwmdac1.MfuncC2 = qep1.MechTheta;
        //        pwmdac1.MfuncC1 = clarke1.As;
        //        pwmdac1.MfuncC2 = clarke1.Bs;

        PWMDAC_MACRO(6,pwmdac1)    // PWMDAC 6A, 6B

        //----------------------------------------------------------------------
        //  Update monitoring data log
        //----------------------------------------------------------------------
        if (monitorTicker < monitorSize)
        {
           if (monitorDivTicker < monitorDivider)
           {
               monitorDivTicker++;
           }
           else
           {
              dataLogA[monitorTicker] = pi_id.Fbk;
              dataLogB[monitorTicker] = pi_iq.Fbk;
              monitorTicker++;
              monitorDivTicker = 0;
           }
        }
        else
        {
           memcpy(dataA, dataLogA, sizeof dataA);
           memcpy(dataB, dataLogB, sizeof dataB);
           monitorTicker = 0;
        }

    #endif // (BUILDLEVEL==LEVEL4)
    //==========================================================================

    // Enable more interrupts from this timer
    EPwm1Regs.ETCLR.bit.INT = 1;

    // Acknowledge interrupt to receive more interrupts from PIE group 3
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;

}
// END MAIN ISR CODE
//==============================================================================

//==============================================================================
// Offset Compensation
interrupt void OffsetISR(void)
{
    // Verifying the ISR
    IsrTicker++;

    // Software Protection
    // Current exceeds +-10A will trigger the trip-zone signal
    EALLOW;
        if ((AdcMirror.ADCRESULT1 > limCurrentMax) || \
                (AdcMirror.ADCRESULT1 < limCurrentMin))
        {
           EPwm1Regs.TZFRC.bit.OST   = 1;  // force a trip-zone protection
           EPwm2Regs.TZFRC.bit.OST   = 1;  // force a trip-zone protection
           EPwm3Regs.TZFRC.bit.OST   = 1;  // force a trip-zone protection
        }
        if ((AdcMirror.ADCRESULT2 > limCurrentMax) || \
                (AdcMirror.ADCRESULT2 < limCurrentMin))
        {
           EPwm1Regs.TZFRC.bit.OST   = 1;  // force a trip-zone protection
           EPwm2Regs.TZFRC.bit.OST   = 1;  // force a trip-zone protection
           EPwm3Regs.TZFRC.bit.OST   = 1;  // force a trip-zone protection
        }
        if ((AdcMirror.ADCRESULT3 > limCurrentMax) || \
                (AdcMirror.ADCRESULT3 < limCurrentMin))
        {
           EPwm1Regs.TZFRC.bit.OST   = 1;  // force a trip-zone protection
           EPwm2Regs.TZFRC.bit.OST   = 1;  // force a trip-zone protection
           EPwm3Regs.TZFRC.bit.OST   = 1;  // force a trip-zone protection
        }
        EDIS;
    //--------------------------------------------------------------------------
    // ADC offset (1s - 2s)
    if (IsrTicker>=10000 && IsrTicker<20000)
        {
            // Calculate the offset of ADC channels
            offsetA= _IQmpy(K1,offsetA) + \
                    _IQmpy(K2,_IQ12toIQ(AdcMirror.ADCRESULT1)); //Phase A offset
            offsetB= _IQmpy(K1,offsetB) + \
                    _IQmpy(K2,_IQ12toIQ(AdcMirror.ADCRESULT2)); //Phase B offset
            offsetC= _IQmpy(K1,offsetC) + \
                    _IQmpy(K2,_IQ12toIQ(AdcMirror.ADCRESULT3)); //Phase C offset
        }

    //--------------------------------------------------------------------------
    // point to mainISR (2s)
    if (IsrTicker > 20000)
    {
        EALLOW;
        PieVectTable.EPWM1_INT = &MainISR;
        EDIS;
    }

    // Enable more interrupts from this timer
    EPwm1Regs.ETCLR.bit.INT = 1;

    // Acknowledge interrupt to receive more interrupts from PIE group 3
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;

}
//End of Offset Compensation.
//==============================================================================

//==============================================================================
// Protection Configuration
void HVDMC_Protection(void)
{

    EALLOW;

    // Configure Trip Mechanism for the Motor control software
    // -Cycle by cycle trip on CPU halt
    // -One shot IPM trip zone trip
    // These trips need to be repeated for EPWM1 ,2 & 3

    // Configure GPIO-17 as trip-zone source
    GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 3;
        // 0=GPIO,  1=SPISOMI-A,  2=CANRX-B,  3=TZ6
    GpioCtrlRegs.GPADIR.bit.GPIO17 = 0;
        // 1=OUTput,  0=INput
    GpioDataRegs.GPASET.bit.GPIO17 = 1;
        // if --> Set High initially

    // Set TZ6 as one-shot trip source
    EPwm1Regs.TZSEL.bit.OSHT6   = 1;  //enable TZ1 for OSHT
    EPwm2Regs.TZSEL.bit.OSHT6   = 1;  //enable TZ1 for OSHT
    EPwm3Regs.TZSEL.bit.OSHT6   = 1;  //enable TZ1 for OSHT

    // What do we want the OST/CBC events to do?
    // TZA events can force EPWMxA
    // TZB events can force EPWMxB

    EPwm1Regs.TZCTL.bit.TZA = TZ_FORCE_LO; // EPWMxA will go low
    EPwm1Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // EPWMxB will go low

    EPwm2Regs.TZCTL.bit.TZA = TZ_FORCE_LO; // EPWMxA will go low
    EPwm2Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // EPWMxB will go low

    EPwm3Regs.TZCTL.bit.TZA = TZ_FORCE_LO; // EPWMxA will go low
    EPwm3Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // EPWMxB will go low

    EDIS;

    // Clear any spurious OV trip
    EPwm1Regs.TZCLR.bit.OST = 1;
    EPwm2Regs.TZCLR.bit.OST = 1;
    EPwm3Regs.TZCLR.bit.OST = 1;

}
// End of Protection Configuration.
//==============================================================================

//==============================================================================
// End of the Main File.
//==============================================================================






























