/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: controller_autogen.c
 *
 * Code generated for Simulink model 'controller_autogen'.
 *
 * Model version                  : 1.39
 * Simulink Coder version         : 9.8 (R2022b) 13-May-2022
 * C/C++ source code generated on : Wed Aug  2 21:15:06 2023
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "controller_autogen.h"
#include "rtwtypes.h"
#include <math.h>

/* Named constants for Chart: '<S4>/LEFT_MOTOR' */
#define IN_AMK_errorDetected           ((uint8_T)1U)
#define IN_AMK_errorReset              ((uint8_T)2U)
#define IN_AMK_ready                   ((uint8_T)3U)
#define IN_AMK_running                 ((uint8_T)4U)
#define IN_AMK_startup                 ((uint8_T)5U)
#define IN_NO_ACTIVE_CHILD             ((uint8_T)0U)
#define IN_commandOn                   ((uint8_T)1U)
#define IN_enforceSetpointsZero        ((uint8_T)2U)
#define IN_enforceSetpointsZero_p      ((uint8_T)1U)
#define IN_motorOff_waitingForGov      ((uint8_T)6U)
#define IN_sendReset                   ((uint8_T)2U)
#define IN_toggleDCon                  ((uint8_T)3U)
#define IN_toggleEnable                ((uint8_T)3U)
#define IN_toggleReset                 ((uint8_T)4U)
#define IN_waiting_sysReady            ((uint8_T)4U)

/* Named constants for Chart: '<S1>/Chart' */
#define IN_ErrorStartupState           ((uint8_T)1U)
#define IN_InitialState                ((uint8_T)2U)
#define IN_InitializePrechargeState    ((uint8_T)3U)
#define IN_NO_ACTIVE_CHILD_p           ((uint8_T)0U)
#define IN_PrechargeState              ((uint8_T)4U)
#define IN_RunningState                ((uint8_T)5U)
#define IN_StartupState                ((uint8_T)6U)
#define IN_StartupState1               ((uint8_T)7U)

/* Named constants for Chart: '<S1>/Chart1' */
#define IN_Close_HVneg1                ((uint8_T)1U)
#define IN_Close_HVpos                 ((uint8_T)2U)
#define IN_Close_precharge             ((uint8_T)3U)
#define IN_Open_precharge              ((uint8_T)4U)
#define IN_StartupCMD                  ((uint8_T)5U)

/* Named constants for Chart: '<S2>/Chart1' */
#define IN_DI_error                    ((uint8_T)1U)
#define IN_DI_running                  ((uint8_T)2U)
#define IN_Driver_requested_start      ((uint8_T)1U)
#define IN_INIT                        ((uint8_T)3U)
#define IN_Motor_ready                 ((uint8_T)2U)
#define IN_Ready_to_drive              ((uint8_T)3U)
#define IN_SpeakerOff                  ((uint8_T)1U)
#define IN_SpeakerOn_                  ((uint8_T)2U)
#define IN_Vehicle_coasting            ((uint8_T)4U)
#define IN_Waiting_for_driver          ((uint8_T)4U)

/* Named constants for Chart: '<S3>/Chart' */
#define IN_Command_motor_startup       ((uint8_T)1U)
#define IN_DriverInterface_Error       ((uint8_T)1U)
#define IN_Err_reset                   ((uint8_T)2U)
#define IN_HV_run_error                ((uint8_T)1U)
#define IN_HV_startup                  ((uint8_T)2U)
#define IN_HV_startup_error            ((uint8_T)3U)
#define IN_Initialize_outputs          ((uint8_T)1U)
#define IN_Motor_faulted               ((uint8_T)4U)
#define IN_Motor_run_error             ((uint8_T)2U)
#define IN_RUNNING                     ((uint8_T)2U)
#define IN_RUNNING_ERROR               ((uint8_T)3U)
#define IN_STARTUP                     ((uint8_T)4U)
#define IN_STARTUP_ERROR               ((uint8_T)5U)
#define IN_Send_ReadyToDrive           ((uint8_T)3U)
#define IN_bmOff                       ((uint8_T)6U)

/* Named constants for Chart: '<S5>/Chart' */
#define IN_Run                         ((uint8_T)1U)

/* Block signals and states (default storage) */
DW rtDW;

/* External inputs (root inport signals with default storage) */
ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
ExtY rtY;

/* Real-time model */
static RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;
static real32_T look1_iflf_binlc(real32_T u0, const real32_T bp0[], const
  real32_T table[], uint32_T maxIndex);
static real32_T look1_iflf_binlx(real32_T u0, const real32_T bp0[], const
  real32_T table[], uint32_T maxIndex);
static void LEFT_MOTOR_Init(MI_STATUSES *rty_MI_motorStatus, uint8_T
  *rty_AMK_bInverterOn_tx, uint8_T *rty_AMK_bDcOn_tx, uint8_T *rty_AMK_bEnable,
  uint8_T *rty_AMK_bErrorReset, real32_T *rty_AMK_TargetVelocity, real32_T
  *rty_AMK_TorqueLimitPositiv, real32_T *rty_AMK_TorqueLimitNegativ);
static void LEFT_MOTOR(MI_CMD rtu_GOV_e_miCmd, boolean_T rtu_AMK_bSystemReady,
  boolean_T rtu_AMK_bError, boolean_T rtu_AMK_bQuitDcOn, boolean_T
  rtu_AMK_bDcOn_rx, boolean_T rtu_AMK_bQuitInverterOn, boolean_T
  rtu_AMK_bInverterOn_rx, boolean_T rtu_AMK_bDerating, real32_T
  rtu_VD_T_motorSpeedRequest, real32_T rtu_VD_T_motorTorqueLimitPositi, real32_T
  rtu_VD_T_motorTorqueLimitNegati, MI_STATUSES *rty_MI_motorStatus, uint8_T
  *rty_AMK_bInverterOn_tx, uint8_T *rty_AMK_bDcOn_tx, uint8_T *rty_AMK_bEnable,
  uint8_T *rty_AMK_bErrorReset, real32_T *rty_AMK_TargetVelocity, real32_T
  *rty_AMK_TorqueLimitPositiv, real32_T *rty_AMK_TorqueLimitNegativ,
  DW_LEFT_MOTOR *localDW);
static real32_T look1_iflf_binlc(real32_T u0, const real32_T bp0[], const
  real32_T table[], uint32_T maxIndex)
{
  real32_T frac;
  real32_T yL_0d0;
  uint32_T iLeft;

  /* Column-major Lookup 1-D
     Search method: 'binary'
     Use previous index: 'off'
     Interpolation method: 'Linear point-slope'
     Extrapolation method: 'Clip'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  /* Prelookup - Index and Fraction
     Index Search method: 'binary'
     Extrapolation method: 'Clip'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  if (u0 <= bp0[0U]) {
    iLeft = 0U;
    frac = 0.0F;
  } else if (u0 < bp0[maxIndex]) {
    uint32_T bpIdx;
    uint32_T iRght;

    /* Binary Search */
    bpIdx = maxIndex >> 1U;
    iLeft = 0U;
    iRght = maxIndex;
    while (iRght - iLeft > 1U) {
      if (u0 < bp0[bpIdx]) {
        iRght = bpIdx;
      } else {
        iLeft = bpIdx;
      }

      bpIdx = (iRght + iLeft) >> 1U;
    }

    frac = (u0 - bp0[iLeft]) / (bp0[iLeft + 1U] - bp0[iLeft]);
  } else {
    iLeft = maxIndex - 1U;
    frac = 1.0F;
  }

  /* Column-major Interpolation 1-D
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'off'
     Overflow mode: 'wrapping'
   */
  yL_0d0 = table[iLeft];
  return (table[iLeft + 1U] - yL_0d0) * frac + yL_0d0;
}

static real32_T look1_iflf_binlx(real32_T u0, const real32_T bp0[], const
  real32_T table[], uint32_T maxIndex)
{
  real32_T frac;
  real32_T yL_0d0;
  uint32_T iLeft;

  /* Column-major Lookup 1-D
     Search method: 'binary'
     Use previous index: 'off'
     Interpolation method: 'Linear point-slope'
     Extrapolation method: 'Linear'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  /* Prelookup - Index and Fraction
     Index Search method: 'binary'
     Extrapolation method: 'Linear'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  if (u0 <= bp0[0U]) {
    iLeft = 0U;
    frac = (u0 - bp0[0U]) / (bp0[1U] - bp0[0U]);
  } else if (u0 < bp0[maxIndex]) {
    uint32_T bpIdx;
    uint32_T iRght;

    /* Binary Search */
    bpIdx = maxIndex >> 1U;
    iLeft = 0U;
    iRght = maxIndex;
    while (iRght - iLeft > 1U) {
      if (u0 < bp0[bpIdx]) {
        iRght = bpIdx;
      } else {
        iLeft = bpIdx;
      }

      bpIdx = (iRght + iLeft) >> 1U;
    }

    frac = (u0 - bp0[iLeft]) / (bp0[iLeft + 1U] - bp0[iLeft]);
  } else {
    iLeft = maxIndex - 1U;
    frac = (u0 - bp0[maxIndex - 1U]) / (bp0[maxIndex] - bp0[maxIndex - 1U]);
  }

  /* Column-major Interpolation 1-D
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'off'
     Overflow mode: 'wrapping'
   */
  yL_0d0 = table[iLeft];
  return (table[iLeft + 1U] - yL_0d0) * frac + yL_0d0;
}

/*
 * System initialize for atomic system:
 *    '<S4>/LEFT_MOTOR'
 *    '<S4>/RIGHT_MOTOR'
 */
static void LEFT_MOTOR_Init(MI_STATUSES *rty_MI_motorStatus, uint8_T
  *rty_AMK_bInverterOn_tx, uint8_T *rty_AMK_bDcOn_tx, uint8_T *rty_AMK_bEnable,
  uint8_T *rty_AMK_bErrorReset, real32_T *rty_AMK_TargetVelocity, real32_T
  *rty_AMK_TorqueLimitPositiv, real32_T *rty_AMK_TorqueLimitNegativ)
{
  *rty_MI_motorStatus = UNKNOWN;
  *rty_AMK_bInverterOn_tx = 0U;
  *rty_AMK_bDcOn_tx = 0U;
  *rty_AMK_bEnable = 0U;
  *rty_AMK_bErrorReset = 0U;
  *rty_AMK_TargetVelocity = 0.0F;
  *rty_AMK_TorqueLimitPositiv = 0.0F;
  *rty_AMK_TorqueLimitNegativ = 0.0F;
}

/*
 * Output and update for atomic system:
 *    '<S4>/LEFT_MOTOR'
 *    '<S4>/RIGHT_MOTOR'
 */
static void LEFT_MOTOR(MI_CMD rtu_GOV_e_miCmd, boolean_T rtu_AMK_bSystemReady,
  boolean_T rtu_AMK_bError, boolean_T rtu_AMK_bQuitDcOn, boolean_T
  rtu_AMK_bDcOn_rx, boolean_T rtu_AMK_bQuitInverterOn, boolean_T
  rtu_AMK_bInverterOn_rx, boolean_T rtu_AMK_bDerating, real32_T
  rtu_VD_T_motorSpeedRequest, real32_T rtu_VD_T_motorTorqueLimitPositi, real32_T
  rtu_VD_T_motorTorqueLimitNegati, MI_STATUSES *rty_MI_motorStatus, uint8_T
  *rty_AMK_bInverterOn_tx, uint8_T *rty_AMK_bDcOn_tx, uint8_T *rty_AMK_bEnable,
  uint8_T *rty_AMK_bErrorReset, real32_T *rty_AMK_TargetVelocity, real32_T
  *rty_AMK_TorqueLimitPositiv, real32_T *rty_AMK_TorqueLimitNegativ,
  DW_LEFT_MOTOR *localDW)
{
  if (localDW->temporalCounter_i1 < 127U) {
    localDW->temporalCounter_i1++;
  }

  /* Chart: '<S4>/LEFT_MOTOR' */
  if (localDW->is_active_c1_motor_interface_li == 0U) {
    localDW->is_active_c1_motor_interface_li = 1U;
    localDW->is_c1_motor_interface_lib = IN_motorOff_waitingForGov;
    *rty_MI_motorStatus = OFF;
    *rty_AMK_bInverterOn_tx = 0U;
    *rty_AMK_bDcOn_tx = 0U;
    *rty_AMK_bEnable = 0U;
    *rty_AMK_bErrorReset = 0U;
    *rty_AMK_TargetVelocity = 0.0F;
    *rty_AMK_TorqueLimitPositiv = 0.0F;
    *rty_AMK_TorqueLimitNegativ = 0.0F;
  } else {
    switch (localDW->is_c1_motor_interface_lib) {
     case IN_AMK_errorDetected:
      if (rtu_GOV_e_miCmd == ERR_RESET) {
        localDW->is_c1_motor_interface_lib = IN_AMK_errorReset;
        localDW->is_AMK_errorReset = IN_enforceSetpointsZero_p;
        *rty_AMK_TargetVelocity = 0.0F;
        *rty_AMK_TorqueLimitPositiv = 0.0F;
        *rty_AMK_TorqueLimitNegativ = 0.0F;
        *rty_AMK_bInverterOn_tx = 0U;
      }
      break;

     case IN_AMK_errorReset:
      switch (localDW->is_AMK_errorReset) {
       case IN_enforceSetpointsZero_p:
        *rty_AMK_bInverterOn_tx = 0U;
        if (!rtu_AMK_bInverterOn_rx) {
          localDW->is_AMK_errorReset = IN_toggleEnable;
          localDW->temporalCounter_i1 = 0U;
          *rty_AMK_bEnable = 0U;
        }
        break;

       case IN_sendReset:
        *rty_AMK_bErrorReset = 1U;
        if (localDW->temporalCounter_i1 >= 100U) {
          localDW->is_AMK_errorReset = IN_toggleReset;
          *rty_AMK_bErrorReset = 0U;
        }
        break;

       case IN_toggleEnable:
        *rty_AMK_bEnable = 0U;
        if (localDW->temporalCounter_i1 >= 100U) {
          localDW->is_AMK_errorReset = IN_sendReset;
          localDW->temporalCounter_i1 = 0U;
          *rty_AMK_bErrorReset = 1U;
        }
        break;

       default:
        /* case IN_toggleReset: */
        *rty_AMK_bErrorReset = 0U;
        if (rtu_AMK_bSystemReady) {
          localDW->is_AMK_errorReset = IN_NO_ACTIVE_CHILD;
          localDW->is_c1_motor_interface_lib = IN_motorOff_waitingForGov;
          *rty_MI_motorStatus = OFF;
          *rty_AMK_bInverterOn_tx = 0U;
          *rty_AMK_bDcOn_tx = 0U;
          *rty_AMK_bEnable = 0U;
          *rty_AMK_bErrorReset = 0U;
          *rty_AMK_TargetVelocity = 0.0F;
          *rty_AMK_TorqueLimitPositiv = 0.0F;
          *rty_AMK_TorqueLimitNegativ = 0.0F;
        }
        break;
      }
      break;

     case IN_AMK_ready:
      if (rtu_AMK_bQuitInverterOn && (!rtu_AMK_bDerating)) {
        localDW->is_c1_motor_interface_lib = IN_AMK_running;
      } else {
        *rty_MI_motorStatus = READY;
      }
      break;

     case IN_AMK_running:
      if (rtu_AMK_bError) {
        localDW->is_c1_motor_interface_lib = IN_AMK_errorDetected;
        *rty_MI_motorStatus = MI_STS_ERROR;
      } else if (rtu_AMK_bDerating || (rtu_GOV_e_miCmd == CMD_SHUTDOWN)) {
        localDW->is_c1_motor_interface_lib = IN_motorOff_waitingForGov;
        *rty_MI_motorStatus = OFF;
        *rty_AMK_bInverterOn_tx = 0U;
        *rty_AMK_bDcOn_tx = 0U;
        *rty_AMK_bEnable = 0U;
        *rty_AMK_bErrorReset = 0U;
        *rty_AMK_TargetVelocity = 0.0F;
        *rty_AMK_TorqueLimitPositiv = 0.0F;
        *rty_AMK_TorqueLimitNegativ = 0.0F;
      } else {
        *rty_MI_motorStatus = RUNNING;
        *rty_AMK_TargetVelocity = rtu_VD_T_motorSpeedRequest;
        *rty_AMK_TorqueLimitPositiv = rtu_VD_T_motorTorqueLimitPositi;
        *rty_AMK_TorqueLimitNegativ = rtu_VD_T_motorTorqueLimitNegati;
      }
      break;

     case IN_AMK_startup:
      switch (localDW->is_AMK_startup) {
       case IN_commandOn:
        *rty_AMK_bEnable = 1U;
        *rty_AMK_bInverterOn_tx = 1U;
        if (rtu_AMK_bInverterOn_rx && (!rtu_AMK_bDerating)) {
          localDW->is_AMK_startup = IN_NO_ACTIVE_CHILD;
          localDW->is_c1_motor_interface_lib = IN_AMK_ready;
          *rty_MI_motorStatus = READY;
        } else if (rtu_AMK_bError) {
          localDW->is_AMK_startup = IN_NO_ACTIVE_CHILD;
          localDW->is_c1_motor_interface_lib = IN_AMK_errorDetected;
          *rty_MI_motorStatus = MI_STS_ERROR;
        }
        break;

       case IN_enforceSetpointsZero:
        if (localDW->temporalCounter_i1 >= 20U) {
          localDW->is_AMK_startup = IN_commandOn;
          *rty_AMK_bEnable = 1U;
          *rty_AMK_bInverterOn_tx = 1U;
        } else if (rtu_AMK_bError) {
          localDW->is_AMK_startup = IN_NO_ACTIVE_CHILD;
          localDW->is_c1_motor_interface_lib = IN_AMK_errorDetected;
          *rty_MI_motorStatus = MI_STS_ERROR;
        }
        break;

       case IN_toggleDCon:
        *rty_AMK_bDcOn_tx = 1U;
        if (rtu_AMK_bDcOn_rx && rtu_AMK_bQuitDcOn) {
          localDW->is_AMK_startup = IN_enforceSetpointsZero;
          localDW->temporalCounter_i1 = 0U;
          *rty_AMK_TargetVelocity = 0.0F;
          *rty_AMK_TorqueLimitPositiv = 0.0F;
          *rty_AMK_TorqueLimitNegativ = 0.0F;
        } else if (rtu_AMK_bError) {
          localDW->is_AMK_startup = IN_NO_ACTIVE_CHILD;
          localDW->is_c1_motor_interface_lib = IN_AMK_errorDetected;
          *rty_MI_motorStatus = MI_STS_ERROR;
        }
        break;

       default:
        /* case IN_waiting_sysReady: */
        if (rtu_AMK_bSystemReady) {
          localDW->is_AMK_startup = IN_toggleDCon;
          *rty_AMK_bDcOn_tx = 1U;
        } else if (rtu_AMK_bError) {
          localDW->is_AMK_startup = IN_NO_ACTIVE_CHILD;
          localDW->is_c1_motor_interface_lib = IN_AMK_errorDetected;
          *rty_MI_motorStatus = MI_STS_ERROR;
        }
        break;
      }
      break;

     default:
      /* case IN_motorOff_waitingForGov: */
      *rty_AMK_bInverterOn_tx = 0U;
      *rty_AMK_bDcOn_tx = 0U;
      *rty_AMK_bEnable = 0U;
      *rty_AMK_bErrorReset = 0U;
      if (rtu_GOV_e_miCmd == CMD_STARTUP) {
        localDW->is_c1_motor_interface_lib = IN_AMK_startup;
        localDW->is_AMK_startup = IN_waiting_sysReady;
        *rty_MI_motorStatus = STARTUP;
      }
      break;
    }
  }

  /* End of Chart: '<S4>/LEFT_MOTOR' */
}

/* Model step function */
void controller_autogen_step(void)
{
  int32_T i;
  real32_T rtb_VectorConcatenate[11];
  real32_T rtb_Gain_c;
  real32_T rtb_Merge;
  boolean_T rtb_NOT2_i;
  boolean_T rtb_NOT_g;
  boolean_T rtb_b_DriverInterfaceError;
  BM_STATUSES rtb_GOV_BM_STATUS;
  MI_STATUSES rtb_Switch;
  MI_STATUSES rtb_overallMotorState;

  /* Chart: '<S3>/Chart' incorporates:
   *  Delay: '<S3>/Delay'
   *  Delay: '<S3>/Delay1'
   *  Delay: '<S3>/Delay2'
   */
  if (rtDW.is_active_c1_governor_lib == 0U) {
    rtDW.is_active_c1_governor_lib = 1U;
    rtDW.is_c1_governor_lib = IN_Initialize_outputs;
    rtDW.GOV_e_bmCmd = 0U;
    rtDW.GOV_e_miCmd = CMD_INIT;
    rtDW.GOV_e_diCmd = DI_CMD_INIT;
    rtDW.motorStartCount = 0U;
    rtDW.GOV_e_govSts = GOV_INIT;
  } else {
    switch (rtDW.is_c1_governor_lib) {
     case IN_Initialize_outputs:
      rtDW.GOV_e_bmCmd = 0U;
      rtDW.GOV_e_diCmd = DI_CMD_INIT;
      rtDW.GOV_e_govSts = GOV_INIT;
      if (rtDW.Delay2_DSTATE_a == DRV_START_REQ) {
        rtDW.is_c1_governor_lib = IN_STARTUP;
        rtDW.GOV_e_govSts = GOV_STARTUP;
        rtDW.is_STARTUP = IN_HV_startup;
      }
      break;

     case IN_RUNNING:
      rtDW.GOV_e_govSts = GOV_RUNNING;
      if (rtDW.Delay1_DSTATE == OFF) {
        rtDW.is_c1_governor_lib = IN_bmOff;
        rtDW.GOV_e_bmCmd = 1U;
        rtDW.GOV_e_miCmd = CMD_INIT;
      } else if (rtDW.Delay_DSTATE_f == ERR_RUNNING) {
        rtDW.is_c1_governor_lib = IN_RUNNING_ERROR;
        rtDW.is_RUNNING_ERROR = IN_HV_run_error;
        rtDW.GOV_e_diCmd = SYSTEM_ERROR;
        rtDW.GOV_e_govSts = HV_RUN_ERROR;
      } else if (rtDW.Delay1_DSTATE == MI_STS_ERROR) {
        rtDW.is_c1_governor_lib = IN_RUNNING_ERROR;
        rtDW.is_RUNNING_ERROR = IN_Motor_run_error;
        rtDW.GOV_e_diCmd = SYSTEM_ERROR;
        rtDW.GOV_e_govSts = MOTOR_RUN_ERROR;
      }
      break;

     case IN_RUNNING_ERROR:
      if (rtDW.is_RUNNING_ERROR == IN_HV_run_error) {
        rtDW.GOV_e_diCmd = SYSTEM_ERROR;
        rtDW.GOV_e_govSts = HV_RUN_ERROR;
      } else {
        /* case IN_Motor_run_error: */
        rtDW.GOV_e_diCmd = SYSTEM_ERROR;
        rtDW.GOV_e_govSts = MOTOR_RUN_ERROR;
      }
      break;

     case IN_STARTUP:
      {
        rtDW.GOV_e_govSts = GOV_STARTUP;
        if (rtDW.Delay_DSTATE_f == ERR_STARTUP) {
          rtDW.is_STARTUP = IN_NO_ACTIVE_CHILD_p;
          rtDW.is_c1_governor_lib = IN_STARTUP_ERROR;
          rtDW.is_STARTUP_ERROR = IN_HV_startup_error;
          rtDW.GOV_e_govSts = HV_STARTUP_ERROR;
        } else if (rtDW.Delay2_DSTATE_a == DI_ERROR) {
          rtDW.is_STARTUP = IN_NO_ACTIVE_CHILD_p;
          rtDW.is_c1_governor_lib = IN_STARTUP_ERROR;
          rtDW.is_STARTUP_ERROR = IN_DriverInterface_Error;
          rtDW.GOV_e_govSts = GOV_DI_ERROR;
        } else if (rtDW.Delay1_DSTATE == MI_STS_ERROR) {
          if (rtDW.motorStartCount >= 5) {
            rtDW.is_STARTUP = IN_NO_ACTIVE_CHILD_p;
            rtDW.is_c1_governor_lib = IN_STARTUP_ERROR;
            rtDW.is_STARTUP_ERROR = IN_Motor_faulted;
            rtDW.GOV_e_miCmd = CMD_SHUTDOWN;
            rtDW.GOV_e_govSts = MOTOR_STARTUP_ERROR;
          } else {
            rtDW.is_STARTUP = IN_NO_ACTIVE_CHILD_p;
            rtDW.is_c1_governor_lib = IN_STARTUP_ERROR;
            rtDW.is_STARTUP_ERROR = IN_Err_reset;
          }
        } else {
          switch (rtDW.is_STARTUP) {
           case IN_Command_motor_startup:
            if (rtDW.Delay1_DSTATE == RUNNING) {
              rtDW.is_STARTUP = IN_Send_ReadyToDrive;
              rtDW.GOV_e_diCmd = READY_TO_DRIVE;
            }
            break;

           case IN_HV_startup:
            {
              if (rtDW.Delay_DSTATE_f == BM_RUNNING) {
                uint32_T tmp;
                rtDW.is_STARTUP = IN_Command_motor_startup;
                rtDW.GOV_e_miCmd = CMD_STARTUP;
                tmp = rtDW.motorStartCount + 1U;
                if (rtDW.motorStartCount + 1U > 65535U) {
                  tmp = 65535U;
                }

                rtDW.motorStartCount = (uint16_T)tmp;
              }
            }
            break;

           default:
            /* case IN_Send_ReadyToDrive: */
            rtDW.GOV_e_diCmd = READY_TO_DRIVE;
            if (rtDW.Delay2_DSTATE_a == DI_RUNNING) {
              rtDW.is_STARTUP = IN_NO_ACTIVE_CHILD_p;
              rtDW.is_c1_governor_lib = IN_RUNNING;
              rtDW.GOV_e_govSts = GOV_RUNNING;
            }
            break;
          }
        }
      }
      break;

     case IN_STARTUP_ERROR:
      switch (rtDW.is_STARTUP_ERROR) {
       case IN_DriverInterface_Error:
        rtDW.GOV_e_govSts = GOV_DI_ERROR;
        break;

       case IN_Err_reset:
        if (rtDW.Delay1_DSTATE == OFF) {
          rtDW.is_STARTUP_ERROR = IN_NO_ACTIVE_CHILD_p;
          rtDW.is_c1_governor_lib = IN_STARTUP;
          rtDW.GOV_e_govSts = GOV_STARTUP;
          rtDW.is_STARTUP = IN_HV_startup;
        } else {
          rtDW.GOV_e_miCmd = ERR_RESET;
        }
        break;

       case IN_HV_startup_error:
        rtDW.GOV_e_govSts = HV_STARTUP_ERROR;
        break;

       default:
        /* case IN_Motor_faulted: */
        rtDW.GOV_e_govSts = MOTOR_STARTUP_ERROR;
        break;
      }
      break;

     default:
      /* case IN_bmOff: */
      rtDW.is_c1_governor_lib = IN_Initialize_outputs;
      rtDW.GOV_e_bmCmd = 0U;
      rtDW.GOV_e_miCmd = CMD_INIT;
      rtDW.GOV_e_diCmd = DI_CMD_INIT;
      rtDW.motorStartCount = 0U;
      rtDW.GOV_e_govSts = GOV_INIT;
      break;
    }
  }

  /* End of Chart: '<S3>/Chart' */

  /* Logic: '<S13>/NOT' incorporates:
   *  Constant: '<S13>/LowerPotentiometerLimit1'
   *  Constant: '<S13>/UpperPotentiometerLimit1'
   *  Inport: '<Root>/DI_V_AccelPedalPos1'
   *  Logic: '<S17>/FixPt Logical Operator'
   *  RelationalOperator: '<S17>/Lower Test'
   *  RelationalOperator: '<S17>/Upper Test'
   */
  rtb_NOT_g = ((!(rtU.DI_V_AccelPedalPos1 >= 0.0)) || (!(rtU.DI_V_AccelPedalPos1
    <= 4095.0)));

  /* Gain: '<S18>/Gain' incorporates:
   *  DataTypeConversion: '<S13>/Data Type Conversion1'
   *  Inport: '<Root>/DI_V_AccelPedalPos1'
   *  Product: '<S18>/Divide'
   */
  rtb_Gain_c = (real32_T)rtU.DI_V_AccelPedalPos1 / 4095.0F * 100.0F;

  /* Logic: '<S2>/b_DriverInterfaceError' incorporates:
   *  Abs: '<S8>/Abs'
   *  Constant: '<S14>/LowerPotentiometerLimit1'
   *  Constant: '<S14>/UpperPotentiometerLimit1'
   *  Constant: '<S8>/Constant'
   *  DataTypeConversion: '<S14>/Data Type Conversion1'
   *  Gain: '<S20>/Gain'
   *  Inport: '<Root>/DI_V_AccelPedalPos2'
   *  Logic: '<S14>/NOT'
   *  Logic: '<S19>/FixPt Logical Operator'
   *  Logic: '<S2>/AND'
   *  Product: '<S20>/Divide'
   *  RelationalOperator: '<S19>/Lower Test'
   *  RelationalOperator: '<S19>/Upper Test'
   *  RelationalOperator: '<S8>/GreaterThan'
   *  Sum: '<S8>/Subtract'
   */
  rtb_b_DriverInterfaceError = (rtb_NOT_g || ((!(rtU.DI_V_AccelPedalPos2 >= 0.0))
    || (!(rtU.DI_V_AccelPedalPos2 <= 4095.0))) || (fabsf(rtb_Gain_c - (real32_T)
    rtU.DI_V_AccelPedalPos2 / 4095.0F * 100.0F) > 10.0F));

  /* Chart: '<S2>/Chart1' incorporates:
   *  Delay: '<S2>/Delay2'
   *  Delay: '<S2>/Delay3'
   *  Inport: '<Root>/DI_b_DriverButton'
   *  Logic: '<S2>/NOT2'
   */
  if (rtDW.temporalCounter_i1 < 2047U) {
    rtDW.temporalCounter_i1++;
  }

  if (rtDW.is_active_c1_driver_interface_l == 0U) {
    rtDW.is_active_c1_driver_interface_l = 1U;
    rtDW.is_c1_driver_interface_lib = IN_INIT;
    rtDW.temporalCounter_i1 = 0U;
    rtDW.GOV_e_diSts = DI_STS_INIT;
    rtDW.b_ReadyToDrive = false;

    /* Outport: '<Root>/DI_b_driverSpeaker' */
    rtY.DI_b_driverSpeaker = false;

    /* Outport: '<Root>/DI_p_PWMstatusLightCycle' */
    rtY.DI_p_PWMstatusLightCycle = 0.0;

    /* Outport: '<Root>/DI_p_PWMstatusLightFreq' */
    rtY.DI_p_PWMstatusLightFreq = 1.0;
  } else {
    switch (rtDW.is_c1_driver_interface_lib) {
     case IN_DI_error:
      rtDW.GOV_e_diSts = DI_ERROR;
      break;

     case IN_DI_running:
      if (rtb_b_DriverInterfaceError) {
        rtDW.is_Ready_to_drive = IN_NO_ACTIVE_CHILD_p;
        rtDW.is_DI_running = IN_NO_ACTIVE_CHILD_p;
        rtDW.is_c1_driver_interface_lib = IN_DI_error;
        rtDW.GOV_e_diSts = DI_ERROR;
      } else {
        switch (rtDW.is_DI_running) {
         case IN_Driver_requested_start:
          rtDW.GOV_e_diSts = DRV_START_REQ;

          /* Outport: '<Root>/DI_p_PWMstatusLightCycle' */
          rtY.DI_p_PWMstatusLightCycle = 50.0;

          /* Outport: '<Root>/DI_p_PWMstatusLightFreq' */
          rtY.DI_p_PWMstatusLightFreq = 10.0;
          if (rtDW.Delay2_DSTATE == READY) {
            rtDW.is_DI_running = IN_Motor_ready;

            /* Outport: '<Root>/DI_p_PWMstatusLightCycle' */
            rtY.DI_p_PWMstatusLightCycle = 100.0;
          }
          break;

         case IN_Motor_ready:
          /* Outport: '<Root>/DI_p_PWMstatusLightCycle' */
          rtY.DI_p_PWMstatusLightCycle = 100.0;

          /* Outport: '<Root>/DI_p_PWMstatusLightFreq' */
          rtY.DI_p_PWMstatusLightFreq = 10.0;
          if ((rtDW.GOV_e_diCmd == READY_TO_DRIVE) && (rtDW.Delay3_DSTATE ==
               0.0F)) {
            rtDW.is_DI_running = IN_Ready_to_drive;
            rtDW.b_ReadyToDrive = true;
            rtDW.GOV_e_diSts = DI_RUNNING;
            rtDW.is_Ready_to_drive = IN_SpeakerOn_;
            rtDW.temporalCounter_i1 = 0U;

            /* Outport: '<Root>/DI_b_driverSpeaker' */
            rtY.DI_b_driverSpeaker = true;
          }
          break;

         case IN_Ready_to_drive:
          rtDW.b_ReadyToDrive = true;
          rtDW.GOV_e_diSts = DI_RUNNING;

          /* Outport: '<Root>/DI_p_PWMstatusLightCycle' */
          rtY.DI_p_PWMstatusLightCycle = 100.0;

          /* Outport: '<Root>/DI_p_PWMstatusLightFreq' */
          rtY.DI_p_PWMstatusLightFreq = 10.0;
          if (rtDW.GOV_e_diCmd == SYSTEM_ERROR) {
            rtDW.is_Ready_to_drive = IN_NO_ACTIVE_CHILD_p;
            rtDW.is_DI_running = IN_NO_ACTIVE_CHILD_p;
            rtDW.is_c1_driver_interface_lib = IN_Vehicle_coasting;
            rtDW.b_ReadyToDrive = false;
            rtDW.GOV_e_diSts = DI_IDLE;
          } else if (rtDW.Delay2_DSTATE == OFF) {
            rtDW.is_Ready_to_drive = IN_NO_ACTIVE_CHILD_p;
            rtDW.is_DI_running = IN_NO_ACTIVE_CHILD_p;
            rtDW.is_c1_driver_interface_lib = IN_INIT;
            rtDW.temporalCounter_i1 = 0U;
            rtDW.GOV_e_diSts = DI_STS_INIT;
            rtDW.b_ReadyToDrive = false;

            /* Outport: '<Root>/DI_b_driverSpeaker' */
            rtY.DI_b_driverSpeaker = false;

            /* Outport: '<Root>/DI_p_PWMstatusLightCycle' */
            rtY.DI_p_PWMstatusLightCycle = 0.0;

            /* Outport: '<Root>/DI_p_PWMstatusLightFreq' */
            rtY.DI_p_PWMstatusLightFreq = 1.0;
          } else if (rtDW.is_Ready_to_drive == IN_SpeakerOff) {
            /* Outport: '<Root>/DI_b_driverSpeaker' */
            rtY.DI_b_driverSpeaker = false;
          } else {
            /* Outport: '<Root>/DI_b_driverSpeaker' */
            /* case IN_SpeakerOn_: */
            rtY.DI_b_driverSpeaker = true;
            if (rtDW.temporalCounter_i1 >= 400U) {
              rtDW.is_Ready_to_drive = IN_SpeakerOff;

              /* Outport: '<Root>/DI_b_driverSpeaker' */
              rtY.DI_b_driverSpeaker = false;
            }
          }
          break;

         default:
          /* case IN_Waiting_for_driver: */
          rtDW.GOV_e_diSts = WAITING_FOR_DRVR;

          /* Outport: '<Root>/DI_p_PWMstatusLightCycle' */
          rtY.DI_p_PWMstatusLightCycle = 50.0;

          /* Outport: '<Root>/DI_p_PWMstatusLightFreq' */
          rtY.DI_p_PWMstatusLightFreq = 1.0;
          if (!rtU.DI_b_DriverButton) {
            rtDW.is_DI_running = IN_Driver_requested_start;
            rtDW.GOV_e_diSts = DRV_START_REQ;

            /* Outport: '<Root>/DI_p_PWMstatusLightFreq' */
            rtY.DI_p_PWMstatusLightFreq = 10.0;
          }
          break;
        }
      }
      break;

     case IN_INIT:
      rtDW.GOV_e_diSts = DI_STS_INIT;
      rtDW.b_ReadyToDrive = false;

      /* Outport: '<Root>/DI_b_driverSpeaker' */
      rtY.DI_b_driverSpeaker = false;

      /* Outport: '<Root>/DI_p_PWMstatusLightCycle' */
      rtY.DI_p_PWMstatusLightCycle = 0.0;

      /* Outport: '<Root>/DI_p_PWMstatusLightFreq' */
      rtY.DI_p_PWMstatusLightFreq = 1.0;
      if (rtDW.temporalCounter_i1 >= 2000U) {
        rtDW.is_c1_driver_interface_lib = IN_DI_running;
        rtDW.is_DI_running = IN_Waiting_for_driver;
        rtDW.GOV_e_diSts = WAITING_FOR_DRVR;

        /* Outport: '<Root>/DI_p_PWMstatusLightCycle' */
        rtY.DI_p_PWMstatusLightCycle = 50.0;
      }
      break;

     default:
      /* case IN_Vehicle_coasting: */
      rtDW.b_ReadyToDrive = false;
      rtDW.GOV_e_diSts = DI_IDLE;
      break;
    }
  }

  /* End of Chart: '<S2>/Chart1' */

  /* Lookup_n-D: '<S2>/AccelPedalPos1 LUT' incorporates:
   *  Gain: '<S18>/Gain'
   */
  rtb_Gain_c = look1_iflf_binlc(rtb_Gain_c, rtConstP.AccelPedalPos1LUT_bp01Data,
    rtConstP.AccelPedalPos1LUT_tableData, 99U);

  /* If: '<S2>/If' incorporates:
   *  Logic: '<S2>/NOT'
   *  Logic: '<S2>/OR1'
   */
  if ((!rtDW.b_ReadyToDrive) || rtb_b_DriverInterfaceError) {
    /* Outputs for IfAction SubSystem: '<S2>/If Action Subsystem' incorporates:
     *  ActionPort: '<S10>/Action Port'
     */
    /* SignalConversion generated from: '<S10>/In1' incorporates:
     *  Constant: '<S2>/Constant'
     */
    rtb_Merge = 0.0F;

    /* End of Outputs for SubSystem: '<S2>/If Action Subsystem' */
  } else if (rtb_NOT_g) {
    /* Outputs for IfAction SubSystem: '<S2>/If Action Subsystem2' incorporates:
     *  ActionPort: '<S12>/Action Port'
     */
    /* SignalConversion generated from: '<S12>/In1' incorporates:
     *  Constant: '<S2>/Constant'
     */
    rtb_Merge = 0.0F;

    /* End of Outputs for SubSystem: '<S2>/If Action Subsystem2' */
  } else {
    /* Outputs for IfAction SubSystem: '<S2>/If Action Subsystem1' incorporates:
     *  ActionPort: '<S11>/Action Port'
     */
    /* SignalConversion generated from: '<S11>/In1' */
    rtb_Merge = rtb_Gain_c;

    /* End of Outputs for SubSystem: '<S2>/If Action Subsystem1' */
  }

  /* End of If: '<S2>/If' */

  /* Chart: '<S5>/Chart' */
  if (rtDW.is_active_c3_simp_vd_lib == 0U) {
    rtDW.is_active_c3_simp_vd_lib = 1U;
    rtDW.is_c3_simp_vd_lib = IN_Run;
  } else if (rtDW.is_c3_simp_vd_lib != IN_Run) {
    /* case IN_Stop: */
    if (rtb_Merge < 5.0F) {
      rtDW.is_c3_simp_vd_lib = IN_Run;
    } else {
      rtb_Merge = 0.0F;
    }
  }

  /* End of Chart: '<S5>/Chart' */

  /* Lookup_n-D: '<S5>/%pedal to %torque' */
  rtDW.pedaltotorque = look1_iflf_binlx(rtb_Merge,
    rtConstP.pedaltotorque_bp01Data, rtConstP.pedaltotorque_tableData, 100U);

  /* SignalConversion generated from: '<S29>/Vector Concatenate' */
  rtb_VectorConcatenate[0] = rtDW.pedaltotorque;

  /* S-Function (sfix_udelay): '<S29>/Tapped Delay' */
  for (i = 0; i < 10; i++) {
    rtb_VectorConcatenate[i + 1] = rtDW.TappedDelay_X[i];
  }

  /* End of S-Function (sfix_udelay): '<S29>/Tapped Delay' */

  /* Sum: '<S29>/Sum of Elements' */
  rtb_Merge = -0.0F;
  for (i = 0; i < 11; i++) {
    rtb_Merge += rtb_VectorConcatenate[i];
  }

  real_T rtb_Divide;

  /* End of Sum: '<S29>/Sum of Elements' */

  /* Product: '<S29>/Divide' */
  rtb_Divide = rtb_Merge / 11.0;

  /* Chart: '<S4>/RIGHT_MOTOR' incorporates:
   *  DataTypeConversion: '<S4>/Cast To Single1'
   *  Inport: '<Root>/AMK_bDcOn_R'
   *  Inport: '<Root>/AMK_bDerating_R'
   *  Inport: '<Root>/AMK_bError_R'
   *  Inport: '<Root>/AMK_bInverterOn_R'
   *  Inport: '<Root>/AMK_bQuitDcOn_R'
   *  Inport: '<Root>/AMK_bQuitInverterOn_R'
   *  Inport: '<Root>/AMK_bSystemReady_R'
   *  Outport: '<Root>/AMK_TargetVelocity_R'
   *  Outport: '<Root>/AMK_TorqueLimitNegativ_R'
   *  Outport: '<Root>/AMK_TorqueLimitPositiv_R'
   *  Outport: '<Root>/AMK_bDcOn_tx_R'
   *  Outport: '<Root>/AMK_bEnable_R'
   *  Outport: '<Root>/AMK_bErrorReset_R'
   *  Outport: '<Root>/AMK_bInverterOn_tx_R'
   */
  LEFT_MOTOR(rtDW.GOV_e_miCmd, rtU.AMK_bSystemReady_R, rtU.AMK_bError_R,
             rtU.AMK_bQuitDcOn_R, rtU.AMK_bDcOn_R, rtU.AMK_bQuitInverterOn_R,
             rtU.AMK_bInverterOn_R, rtU.AMK_bDerating_R, rtConstB.CastToSingle,
             (real32_T)rtb_Divide, rtConstB.CastToSingle2, &rtDW.MI_motorStatus,
             &rtY.AMK_bInverterOn_tx_R, &rtY.AMK_bDcOn_tx_R, &rtY.AMK_bEnable_R,
             &rtY.AMK_bErrorReset_R, &rtY.AMK_TargetVelocity_R,
             &rtY.AMK_TorqueLimitPositiv_R, &rtY.AMK_TorqueLimitNegativ_R,
             &rtDW.sf_RIGHT_MOTOR);

  /* Chart: '<S4>/LEFT_MOTOR' incorporates:
   *  DataTypeConversion: '<S4>/Cast To Single4'
   *  Inport: '<Root>/AMK_bDcOn_L'
   *  Inport: '<Root>/AMK_bDerating_L'
   *  Inport: '<Root>/AMK_bError_L'
   *  Inport: '<Root>/AMK_bInverterOn_L'
   *  Inport: '<Root>/AMK_bQuitDcOn_L'
   *  Inport: '<Root>/AMK_bQuitInverterOn_L'
   *  Inport: '<Root>/AMK_bSystemReady_L'
   *  Outport: '<Root>/AMK_TargetVelocity_L'
   *  Outport: '<Root>/AMK_TorqueLimitNegativ_L'
   *  Outport: '<Root>/AMK_TorqueLimitPositiv_L'
   *  Outport: '<Root>/AMK_bDcOn_tx_L'
   *  Outport: '<Root>/AMK_bEnable_L'
   *  Outport: '<Root>/AMK_bErrorReset_L'
   *  Outport: '<Root>/AMK_bInverterOn_tx_L'
   */
  LEFT_MOTOR(rtDW.GOV_e_miCmd, rtU.AMK_bSystemReady_L, rtU.AMK_bError_L,
             rtU.AMK_bQuitDcOn_L, rtU.AMK_bDcOn_L, rtU.AMK_bQuitInverterOn_L,
             rtU.AMK_bInverterOn_L, rtU.AMK_bDerating_L, rtConstB.CastToSingle3,
             (real32_T)rtb_Divide, rtConstB.CastToSingle5,
             &rtDW.MI_motorStatus_b, &rtY.AMK_bInverterOn_tx_L,
             &rtY.AMK_bDcOn_tx_L, &rtY.AMK_bEnable_L, &rtY.AMK_bErrorReset_L,
             &rtY.AMK_TargetVelocity_L, &rtY.AMK_TorqueLimitPositiv_L,
             &rtY.AMK_TorqueLimitNegativ_L, &rtDW.sf_LEFT_MOTOR);

  /* Logic: '<S1>/NOT' incorporates:
   *  Inport: '<Root>/BM_b_prechrgContactorSts'
   */
  rtb_NOT_g = !rtU.BM_b_prechrgContactorSts;

  /* Logic: '<S1>/NOT1' incorporates:
   *  Inport: '<Root>/BM_b_HVposContactorSts'
   */
  rtb_b_DriverInterfaceError = !rtU.BM_b_HVposContactorSts;

  /* Logic: '<S1>/NOT2' incorporates:
   *  Inport: '<Root>/BM_b_HVnegContactorSts'
   */
  rtb_NOT2_i = !rtU.BM_b_HVnegContactorSts;

  /* Chart: '<S1>/Chart' */
  if (rtDW.temporalCounter_i1_m < 32767U) {
    rtDW.temporalCounter_i1_m++;
  }

  if (rtDW.is_active_c4_battery_monitor_li == 0U) {
    rtDW.is_active_c4_battery_monitor_li = 1U;
    rtDW.is_c4_battery_monitor_lib = IN_InitialState;
    rtb_GOV_BM_STATUS = BM_INIT;
  } else {
    boolean_T guard1 = false;
    guard1 = false;
    switch (rtDW.is_c4_battery_monitor_lib) {
     case IN_ErrorStartupState:
      rtb_GOV_BM_STATUS = ERR_STARTUP;
      break;

     case IN_InitialState:
      {
        boolean_T tmp_0;
        boolean_T tmp_1;
        rtb_GOV_BM_STATUS = BM_INIT;
        tmp_0 = !rtb_NOT2_i;
        tmp_1 = !rtb_b_DriverInterfaceError;
        if ((rtb_NOT_g && tmp_0 && tmp_1) || (rtb_NOT_g && rtb_NOT2_i &&
             rtb_b_DriverInterfaceError)) {
          guard1 = true;
        } else {
          tmp_0 = ((!rtb_NOT_g) && tmp_0);
          if (tmp_0 && rtb_b_DriverInterfaceError) {
            guard1 = true;
          } else if (tmp_0 && tmp_1 && (rtDW.GOV_e_diSts == DRV_START_REQ)) {
            rtDW.is_c4_battery_monitor_lib = IN_StartupState1;
            rtb_GOV_BM_STATUS = BM_IDLE;
          }
        }
      }
      break;

     case IN_InitializePrechargeState:
      rtb_GOV_BM_STATUS = INIT_PRECHARGE;
      if (rtb_NOT_g && rtb_NOT2_i && rtb_b_DriverInterfaceError) {
        rtDW.is_c4_battery_monitor_lib = IN_PrechargeState;
        rtDW.temporalCounter_i1_m = 0U;
        rtb_GOV_BM_STATUS = PRECHARGE;
      } else if (rtDW.temporalCounter_i1_m >= 20000U) {
        rtDW.is_c4_battery_monitor_lib = IN_ErrorStartupState;
        rtb_GOV_BM_STATUS = ERR_STARTUP;
      }
      break;

     case IN_PrechargeState:
      rtb_GOV_BM_STATUS = PRECHARGE;
      if ((!rtb_NOT_g) && rtb_NOT2_i && rtb_b_DriverInterfaceError) {
        rtDW.is_c4_battery_monitor_lib = IN_RunningState;
        rtb_GOV_BM_STATUS = BM_RUNNING;
      } else if (rtDW.temporalCounter_i1_m >= 20000U) {
        rtDW.is_c4_battery_monitor_lib = IN_ErrorStartupState;
        rtb_GOV_BM_STATUS = ERR_STARTUP;
      }
      break;

     case IN_RunningState:
      rtb_GOV_BM_STATUS = BM_RUNNING;
      if (rtDW.GOV_e_bmCmd == 1) {
        rtDW.is_c4_battery_monitor_lib = IN_InitialState;
        rtb_GOV_BM_STATUS = BM_INIT;
      } else if ((!rtb_NOT_g) && rtb_NOT2_i && rtb_b_DriverInterfaceError) {
        rtDW.is_c4_battery_monitor_lib = IN_RunningState;
        rtb_GOV_BM_STATUS = BM_RUNNING;
      }
      break;

     case IN_StartupState:
      rtb_GOV_BM_STATUS = BM_STARTUP;
      if (rtb_NOT_g && rtb_NOT2_i && (!rtb_b_DriverInterfaceError)) {
        rtDW.is_c4_battery_monitor_lib = IN_InitializePrechargeState;
        rtDW.temporalCounter_i1_m = 0U;
        rtb_GOV_BM_STATUS = INIT_PRECHARGE;
      } else if (rtDW.temporalCounter_i1_m >= 20000U) {
        rtDW.is_c4_battery_monitor_lib = IN_ErrorStartupState;
        rtb_GOV_BM_STATUS = ERR_STARTUP;
      }
      break;

     default:
      /* case IN_StartupState1: */
      rtb_GOV_BM_STATUS = BM_IDLE;
      if ((!rtb_NOT_g) && rtb_NOT2_i && (!rtb_b_DriverInterfaceError)) {
        rtDW.is_c4_battery_monitor_lib = IN_StartupState;
        rtDW.temporalCounter_i1_m = 0U;
        rtb_GOV_BM_STATUS = BM_STARTUP;
      }
      break;
    }

    if (guard1) {
      rtDW.is_c4_battery_monitor_lib = IN_ErrorStartupState;
      rtb_GOV_BM_STATUS = ERR_STARTUP;
    }
  }

  /* End of Chart: '<S1>/Chart' */

  /* Chart: '<S1>/Chart1' */
  if (rtDW.temporalCounter_i1_a < 2047U) {
    rtDW.temporalCounter_i1_a++;
  }

  if (rtDW.is_active_c5_battery_monitor_li == 0U) {
    rtDW.is_active_c5_battery_monitor_li = 1U;
    rtDW.is_c5_battery_monitor_lib = IN_StartupCMD;

    /* Outport: '<Root>/BM_b_prechargeContactorCMD' */
    rtY.BM_b_prechargeContactorCMD = 0.0;

    /* Outport: '<Root>/BM_b_HVnegContactorCMD' */
    rtY.BM_b_HVnegContactorCMD = 0.0;

    /* Outport: '<Root>/BM_b_HVposContactorCMD' */
    rtY.BM_b_HVposContactorCMD = 0.0;
  } else {
    switch (rtDW.is_c5_battery_monitor_lib) {
     case IN_Close_HVneg1:
      if ((rtDW.temporalCounter_i1_a >= 20U) && (rtb_GOV_BM_STATUS == BM_STARTUP))
      {
        rtDW.is_c5_battery_monitor_lib = IN_Close_precharge;
        rtDW.temporalCounter_i1_a = 0U;

        /* Outport: '<Root>/BM_b_prechargeContactorCMD' */
        rtY.BM_b_prechargeContactorCMD = 1.0;

        /* Outport: '<Root>/BM_b_HVnegContactorCMD' */
        rtY.BM_b_HVnegContactorCMD = 1.0;

        /* Outport: '<Root>/BM_b_HVposContactorCMD' */
        rtY.BM_b_HVposContactorCMD = 0.0;
      }
      break;

     case IN_Close_HVpos:
      if ((rtDW.temporalCounter_i1_a >= 20U) && (rtb_GOV_BM_STATUS == PRECHARGE))
      {
        rtDW.is_c5_battery_monitor_lib = IN_Open_precharge;

        /* Outport: '<Root>/BM_b_prechargeContactorCMD' */
        rtY.BM_b_prechargeContactorCMD = 0.0;

        /* Outport: '<Root>/BM_b_HVnegContactorCMD' */
        rtY.BM_b_HVnegContactorCMD = 1.0;

        /* Outport: '<Root>/BM_b_HVposContactorCMD' */
        rtY.BM_b_HVposContactorCMD = 1.0;
      }
      break;

     case IN_Close_precharge:
      if ((rtDW.temporalCounter_i1_a >= 1300U) && (rtb_GOV_BM_STATUS ==
           INIT_PRECHARGE)) {
        rtDW.is_c5_battery_monitor_lib = IN_Close_HVpos;
        rtDW.temporalCounter_i1_a = 0U;

        /* Outport: '<Root>/BM_b_prechargeContactorCMD' */
        rtY.BM_b_prechargeContactorCMD = 1.0;

        /* Outport: '<Root>/BM_b_HVnegContactorCMD' */
        rtY.BM_b_HVnegContactorCMD = 1.0;

        /* Outport: '<Root>/BM_b_HVposContactorCMD' */
        rtY.BM_b_HVposContactorCMD = 1.0;
      }
      break;

     case IN_Open_precharge:
      if (rtb_GOV_BM_STATUS == BM_RUNNING) {
        rtDW.is_c5_battery_monitor_lib = IN_Open_precharge;

        /* Outport: '<Root>/BM_b_prechargeContactorCMD' */
        rtY.BM_b_prechargeContactorCMD = 0.0;

        /* Outport: '<Root>/BM_b_HVnegContactorCMD' */
        rtY.BM_b_HVnegContactorCMD = 1.0;

        /* Outport: '<Root>/BM_b_HVposContactorCMD' */
        rtY.BM_b_HVposContactorCMD = 1.0;
      } else if (rtDW.GOV_e_bmCmd == 1) {
        rtDW.is_c5_battery_monitor_lib = IN_StartupCMD;

        /* Outport: '<Root>/BM_b_prechargeContactorCMD' */
        rtY.BM_b_prechargeContactorCMD = 0.0;

        /* Outport: '<Root>/BM_b_HVnegContactorCMD' */
        rtY.BM_b_HVnegContactorCMD = 0.0;

        /* Outport: '<Root>/BM_b_HVposContactorCMD' */
        rtY.BM_b_HVposContactorCMD = 0.0;
      } else {
        /* Outport: '<Root>/BM_b_prechargeContactorCMD' */
        rtY.BM_b_prechargeContactorCMD = 0.0;

        /* Outport: '<Root>/BM_b_HVnegContactorCMD' */
        rtY.BM_b_HVnegContactorCMD = 1.0;

        /* Outport: '<Root>/BM_b_HVposContactorCMD' */
        rtY.BM_b_HVposContactorCMD = 1.0;
      }
      break;

     default:
      /* case IN_StartupCMD: */
      if (rtb_GOV_BM_STATUS == BM_IDLE) {
        rtDW.is_c5_battery_monitor_lib = IN_Close_HVneg1;
        rtDW.temporalCounter_i1_a = 0U;

        /* Outport: '<Root>/BM_b_prechargeContactorCMD' */
        rtY.BM_b_prechargeContactorCMD = 0.0;

        /* Outport: '<Root>/BM_b_HVnegContactorCMD' */
        rtY.BM_b_HVnegContactorCMD = 1.0;

        /* Outport: '<Root>/BM_b_HVposContactorCMD' */
        rtY.BM_b_HVposContactorCMD = 0.0;
      }
      break;
    }
  }

  /* End of Chart: '<S1>/Chart1' */

  /* Outport: '<Root>/GOV_e_govSts' incorporates:
   *  DataTypeConversion: '<Root>/Cast To Single'
   */
  rtY.GOV_e_govSts = (real32_T)rtDW.GOV_e_govSts;

  /* Switch: '<S4>/overallMotorState' incorporates:
   *  Delay: '<S4>/Delay'
   *  RelationalOperator: '<S4>/GreaterThan'
   */
  if (rtDW.MI_motorStatus_b == rtDW.MI_motorStatus) {
    rtb_overallMotorState = rtDW.MI_motorStatus_b;
  } else {
    rtb_overallMotorState = rtDW.Delay_DSTATE;
  }

  /* End of Switch: '<S4>/overallMotorState' */

  /* Switch: '<S4>/Switch' incorporates:
   *  Constant: '<S4>/Constant'
   *  Constant: '<S4>/Constant1'
   *  Logic: '<S4>/OR'
   *  RelationalOperator: '<S4>/motorErrorActive'
   *  RelationalOperator: '<S4>/motorErrorActive1'
   */
  if ((rtDW.MI_motorStatus == MI_STS_ERROR) || (rtDW.MI_motorStatus_b ==
       MI_STS_ERROR)) {
    rtb_Switch = MI_STS_ERROR;
  } else {
    rtb_Switch = rtb_overallMotorState;
  }

  /* End of Switch: '<S4>/Switch' */

  /* Update for Delay: '<S3>/Delay' */
  rtDW.Delay_DSTATE_f = rtb_GOV_BM_STATUS;

  /* Update for Delay: '<S3>/Delay1' */
  rtDW.Delay1_DSTATE = rtb_Switch;

  /* Update for Delay: '<S3>/Delay2' */
  rtDW.Delay2_DSTATE_a = rtDW.GOV_e_diSts;

  /* Update for Delay: '<S2>/Delay2' */
  rtDW.Delay2_DSTATE = rtb_Switch;

  /* Update for Delay: '<S2>/Delay3' */
  rtDW.Delay3_DSTATE = rtb_Gain_c;

  /* Update for S-Function (sfix_udelay): '<S29>/Tapped Delay' */
  for (i = 0; i < 9; i++) {
    rtDW.TappedDelay_X[i] = rtDW.TappedDelay_X[i + 1];
  }

  rtDW.TappedDelay_X[9] = rtDW.pedaltotorque;

  /* End of Update for S-Function (sfix_udelay): '<S29>/Tapped Delay' */

  /* Update for Delay: '<S4>/Delay' */
  rtDW.Delay_DSTATE = rtb_overallMotorState;
}

/* Model initialize function */
void controller_autogen_initialize(void)
{
  /* InitializeConditions for Delay: '<S4>/Delay' */
  rtDW.Delay_DSTATE = OFF;

  /* SystemInitialize for Chart: '<S4>/RIGHT_MOTOR' incorporates:
   *  Outport: '<Root>/AMK_TargetVelocity_R'
   *  Outport: '<Root>/AMK_TorqueLimitNegativ_R'
   *  Outport: '<Root>/AMK_TorqueLimitPositiv_R'
   *  Outport: '<Root>/AMK_bDcOn_tx_R'
   *  Outport: '<Root>/AMK_bEnable_R'
   *  Outport: '<Root>/AMK_bErrorReset_R'
   *  Outport: '<Root>/AMK_bInverterOn_tx_R'
   */
  LEFT_MOTOR_Init(&rtDW.MI_motorStatus, &rtY.AMK_bInverterOn_tx_R,
                  &rtY.AMK_bDcOn_tx_R, &rtY.AMK_bEnable_R,
                  &rtY.AMK_bErrorReset_R, &rtY.AMK_TargetVelocity_R,
                  &rtY.AMK_TorqueLimitPositiv_R, &rtY.AMK_TorqueLimitNegativ_R);

  /* SystemInitialize for Chart: '<S4>/LEFT_MOTOR' incorporates:
   *  Outport: '<Root>/AMK_TargetVelocity_L'
   *  Outport: '<Root>/AMK_TorqueLimitNegativ_L'
   *  Outport: '<Root>/AMK_TorqueLimitPositiv_L'
   *  Outport: '<Root>/AMK_bDcOn_tx_L'
   *  Outport: '<Root>/AMK_bEnable_L'
   *  Outport: '<Root>/AMK_bErrorReset_L'
   *  Outport: '<Root>/AMK_bInverterOn_tx_L'
   */
  LEFT_MOTOR_Init(&rtDW.MI_motorStatus_b, &rtY.AMK_bInverterOn_tx_L,
                  &rtY.AMK_bDcOn_tx_L, &rtY.AMK_bEnable_L,
                  &rtY.AMK_bErrorReset_L, &rtY.AMK_TargetVelocity_L,
                  &rtY.AMK_TorqueLimitPositiv_L, &rtY.AMK_TorqueLimitNegativ_L);
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
