/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: controller_autogen.c
 *
 * Code generated for Simulink model 'controller_autogen'.
 *
 * Model version                  : 1.31
 * Simulink Coder version         : 9.8 (R2022b) 13-May-2022
 * C/C++ source code generated on : Sat Jun  3 19:31:01 2023
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "controller_autogen.h"
#include "controller_autogen_types.h"
#include "rtwtypes.h"
#include "controller_autogen_private.h"

/* Named constants for Chart: '<S4>/LEFT_MOTOR' */
#define contr_IN_enforceSetpointsZero_p ((uint8_T)1U)
#define contr_IN_motorOff_waitingForGov ((uint8_T)6U)
#define control_IN_enforceSetpointsZero ((uint8_T)2U)
#define controller_IN_AMK_errorDetected ((uint8_T)1U)
#define controller__IN_waiting_sysReady ((uint8_T)4U)
#define controller_a_IN_NO_ACTIVE_CHILD ((uint8_T)0U)
#define controller_au_IN_AMK_errorReset ((uint8_T)2U)
#define controller_auto_IN_toggleEnable ((uint8_T)3U)
#define controller_autog_IN_AMK_running ((uint8_T)4U)
#define controller_autog_IN_AMK_startup ((uint8_T)5U)
#define controller_autog_IN_toggleReset ((uint8_T)4U)
#define controller_autoge_IN_toggleDCon ((uint8_T)3U)
#define controller_autogen_IN_AMK_ready ((uint8_T)3U)
#define controller_autogen_IN_commandOn ((uint8_T)1U)
#define controller_autogen_IN_sendReset ((uint8_T)2U)

/* Named constants for Chart: '<S1>/Chart' */
#define IN_ErrorInitializePrechargeStat ((uint8_T)3U)
#define co_IN_ErrorPrechargeClosedState ((uint8_T)4U)
#define con_IN_InitializePrechargeState ((uint8_T)8U)
#define controll_IN_ErrorAllClosedState ((uint8_T)1U)
#define controll_IN_ErrorPrechargeState ((uint8_T)5U)
#define controller_IN_ErrorStartupState ((uint8_T)6U)
#define controller_IN_NO_ACTIVE_CHILD_o ((uint8_T)0U)
#define controller_a_IN_ErrorHVPositive ((uint8_T)2U)
#define controller_au_IN_PrechargeState ((uint8_T)9U)
#define controller_aut_IN_StartupState1 ((uint8_T)12U)
#define controller_auto_IN_InitialState ((uint8_T)7U)
#define controller_auto_IN_RunningState ((uint8_T)10U)
#define controller_auto_IN_StartupState ((uint8_T)11U)

/* Named constants for Chart: '<S1>/Chart1' */
#define controller_a_IN_Close_precharge ((uint8_T)3U)
#define controller_au_IN_Open_precharge ((uint8_T)4U)
#define controller_autog_IN_Close_HVneg ((uint8_T)1U)
#define controller_autog_IN_Close_HVpos ((uint8_T)2U)
#define controller_autoge_IN_StartupCMD ((uint8_T)5U)

/* Named constants for Chart: '<S2>/Chart' */
#define contr_IN_Driver_requested_start ((uint8_T)1U)
#define controlle_IN_Waiting_for_driver ((uint8_T)4U)
#define controller__IN_Vehicle_coasting ((uint8_T)4U)
#define controller_au_IN_Ready_to_drive ((uint8_T)3U)
#define controller_autog_IN_Motor_ready ((uint8_T)2U)
#define controller_autoge_IN_DI_running ((uint8_T)2U)
#define controller_autoge_IN_SpeakerOff ((uint8_T)1U)
#define controller_autoge_IN_SpeakerOn_ ((uint8_T)2U)
#define controller_autogen_IN_DI_error ((uint8_T)1U)
#define controller_autogen_IN_INIT     ((uint8_T)3U)

/* Named constants for Chart: '<S3>/Chart' */
#define contro_IN_Command_motor_startup ((uint8_T)1U)
#define contro_IN_DriverInterface_Error ((uint8_T)1U)
#define controlle_IN_Initialize_outputs ((uint8_T)1U)
#define controller_IN_Send_ReadyToDrive ((uint8_T)3U)
#define controller__IN_HV_startup_error ((uint8_T)3U)
#define controller_a_IN_Motor_run_error ((uint8_T)2U)
#define controller_aut_IN_Motor_faulted ((uint8_T)4U)
#define controller_aut_IN_RUNNING_ERROR ((uint8_T)3U)
#define controller_aut_IN_STARTUP_ERROR ((uint8_T)5U)
#define controller_auto_IN_HV_run_error ((uint8_T)1U)
#define controller_autoge_IN_HV_startup ((uint8_T)2U)
#define controller_autogen_IN_Err_reset ((uint8_T)2U)
#define controller_autogen_IN_RUNNING  ((uint8_T)2U)
#define controller_autogen_IN_STARTUP  ((uint8_T)4U)

/* Named constants for Chart: '<S5>/Chart' */
#define controller_autogen_IN_Run      ((uint8_T)1U)
#define controller_autogen_IN_Stop     ((uint8_T)2U)

/* Block signals (default storage) */
B_controller_autogen_T controller_autogen_B;

/* Block states (default storage) */
DW_controller_autogen_T controller_autogen_DW;

/* External inputs (root inport signals with default storage) */
ExtU_controller_autogen_T controller_autogen_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_controller_autogen_T controller_autogen_Y;

/* Real-time model */
static RT_MODEL_controller_autogen_T controller_autogen_M_;
RT_MODEL_controller_autogen_T *const controller_autogen_M =
  &controller_autogen_M_;
real32_T look1_iflf_binlcpw(real32_T u0, const real32_T bp0[], const real32_T
  table[], uint32_T maxIndex)
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
     Overflow mode: 'portable wrapping'
   */
  yL_0d0 = table[iLeft];
  return (table[iLeft + 1U] - yL_0d0) * frac + yL_0d0;
}

/*
 * System initialize for atomic system:
 *    '<S4>/LEFT_MOTOR'
 *    '<S4>/RIGHT_MOTOR'
 */
void controller_auto_LEFT_MOTOR_Init(MI_STATUSES *rty_MI_motorStatus, uint8_T
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
void controller_autogen_LEFT_MOTOR(MI_CMD rtu_GOV_e_miCmd, boolean_T
  rtu_AMK_bSystemReady, boolean_T rtu_AMK_bError, boolean_T rtu_AMK_bQuitDcOn,
  boolean_T rtu_AMK_bDcOn_rx, boolean_T rtu_AMK_bQuitInverterOn, boolean_T
  rtu_AMK_bInverterOn_rx, boolean_T rtu_AMK_bDerating, real32_T
  rtu_VD_T_motorSpeedRequest, real32_T rtu_VD_T_motorTorqueLimitPositi, real32_T
  rtu_VD_T_motorTorqueLimitNegati, MI_STATUSES *rty_MI_motorStatus, uint8_T
  *rty_AMK_bInverterOn_tx, uint8_T *rty_AMK_bDcOn_tx, uint8_T *rty_AMK_bEnable,
  uint8_T *rty_AMK_bErrorReset, real32_T *rty_AMK_TargetVelocity, real32_T
  *rty_AMK_TorqueLimitPositiv, real32_T *rty_AMK_TorqueLimitNegativ,
  DW_LEFT_MOTOR_controller_auto_T *localDW)
{
  if (localDW->temporalCounter_i1 < 127U) {
    localDW->temporalCounter_i1++;
  }

  /* Chart: '<S4>/LEFT_MOTOR' */
  if (localDW->is_active_c1_motor_interface_li == 0U) {
    localDW->is_active_c1_motor_interface_li = 1U;
    localDW->is_c1_motor_interface_lib = contr_IN_motorOff_waitingForGov;
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
     case controller_IN_AMK_errorDetected:
      if (rtu_GOV_e_miCmd == ERR_RESET) {
        localDW->is_c1_motor_interface_lib = controller_au_IN_AMK_errorReset;
        localDW->is_AMK_errorReset = contr_IN_enforceSetpointsZero_p;
        *rty_AMK_TargetVelocity = 0.0F;
        *rty_AMK_TorqueLimitPositiv = 0.0F;
        *rty_AMK_TorqueLimitNegativ = 0.0F;
        *rty_AMK_bInverterOn_tx = 0U;
      }
      break;

     case controller_au_IN_AMK_errorReset:
      switch (localDW->is_AMK_errorReset) {
       case contr_IN_enforceSetpointsZero_p:
        *rty_AMK_bInverterOn_tx = 0U;
        if (!rtu_AMK_bInverterOn_rx) {
          localDW->is_AMK_errorReset = controller_auto_IN_toggleEnable;
          localDW->temporalCounter_i1 = 0U;
          *rty_AMK_bEnable = 0U;
        }
        break;

       case controller_autogen_IN_sendReset:
        *rty_AMK_bErrorReset = 1U;
        if (localDW->temporalCounter_i1 >= 100U) {
          localDW->is_AMK_errorReset = controller_autog_IN_toggleReset;
          *rty_AMK_bErrorReset = 0U;
        }
        break;

       case controller_auto_IN_toggleEnable:
        *rty_AMK_bEnable = 0U;
        if (localDW->temporalCounter_i1 >= 100U) {
          localDW->is_AMK_errorReset = controller_autogen_IN_sendReset;
          localDW->temporalCounter_i1 = 0U;
          *rty_AMK_bErrorReset = 1U;
        }
        break;

       default:
        /* case IN_toggleReset: */
        *rty_AMK_bErrorReset = 0U;
        if (rtu_AMK_bSystemReady) {
          localDW->is_AMK_errorReset = controller_a_IN_NO_ACTIVE_CHILD;
          localDW->is_c1_motor_interface_lib = contr_IN_motorOff_waitingForGov;
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

     case controller_autogen_IN_AMK_ready:
      if (rtu_AMK_bQuitInverterOn && (!rtu_AMK_bDerating)) {
        localDW->is_c1_motor_interface_lib = controller_autog_IN_AMK_running;
      } else {
        *rty_MI_motorStatus = READY;
      }
      break;

     case controller_autog_IN_AMK_running:
      if (rtu_AMK_bError) {
        localDW->is_c1_motor_interface_lib = controller_IN_AMK_errorDetected;
        *rty_MI_motorStatus = MI_STS_ERROR;
      } else if (rtu_AMK_bDerating || (rtu_GOV_e_miCmd == CMD_SHUTDOWN)) {
        localDW->is_c1_motor_interface_lib = contr_IN_motorOff_waitingForGov;
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

     case controller_autog_IN_AMK_startup:
      switch (localDW->is_AMK_startup) {
       case controller_autogen_IN_commandOn:
        *rty_AMK_bEnable = 1U;
        *rty_AMK_bInverterOn_tx = 1U;
        if (rtu_AMK_bInverterOn_rx && (!rtu_AMK_bDerating)) {
          localDW->is_AMK_startup = controller_a_IN_NO_ACTIVE_CHILD;
          localDW->is_c1_motor_interface_lib = controller_autogen_IN_AMK_ready;
          *rty_MI_motorStatus = READY;
        } else if (rtu_AMK_bError) {
          localDW->is_AMK_startup = controller_a_IN_NO_ACTIVE_CHILD;
          localDW->is_c1_motor_interface_lib = controller_IN_AMK_errorDetected;
          *rty_MI_motorStatus = MI_STS_ERROR;
        }
        break;

       case control_IN_enforceSetpointsZero:
        if (localDW->temporalCounter_i1 >= 20U) {
          localDW->is_AMK_startup = controller_autogen_IN_commandOn;
          *rty_AMK_bEnable = 1U;
          *rty_AMK_bInverterOn_tx = 1U;
        } else if (rtu_AMK_bError) {
          localDW->is_AMK_startup = controller_a_IN_NO_ACTIVE_CHILD;
          localDW->is_c1_motor_interface_lib = controller_IN_AMK_errorDetected;
          *rty_MI_motorStatus = MI_STS_ERROR;
        }
        break;

       case controller_autoge_IN_toggleDCon:
        *rty_AMK_bDcOn_tx = 1U;
        if (rtu_AMK_bDcOn_rx && rtu_AMK_bQuitDcOn) {
          localDW->is_AMK_startup = control_IN_enforceSetpointsZero;
          localDW->temporalCounter_i1 = 0U;
          *rty_AMK_TargetVelocity = 0.0F;
          *rty_AMK_TorqueLimitPositiv = 0.0F;
          *rty_AMK_TorqueLimitNegativ = 0.0F;
        } else if (rtu_AMK_bError) {
          localDW->is_AMK_startup = controller_a_IN_NO_ACTIVE_CHILD;
          localDW->is_c1_motor_interface_lib = controller_IN_AMK_errorDetected;
          *rty_MI_motorStatus = MI_STS_ERROR;
        }
        break;

       default:
        /* case IN_waiting_sysReady: */
        if (rtu_AMK_bSystemReady) {
          localDW->is_AMK_startup = controller_autoge_IN_toggleDCon;
          *rty_AMK_bDcOn_tx = 1U;
        } else if (rtu_AMK_bError) {
          localDW->is_AMK_startup = controller_a_IN_NO_ACTIVE_CHILD;
          localDW->is_c1_motor_interface_lib = controller_IN_AMK_errorDetected;
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
        localDW->is_c1_motor_interface_lib = controller_autog_IN_AMK_startup;
        localDW->is_AMK_startup = controller__IN_waiting_sysReady;
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
  real_T rtb_Gain;
  int32_T rtb_Switch;
  real32_T rtb_AccelPedalPos1LUT;
  real32_T rtb_Gain_o;
  boolean_T rtb_NOT_g;
  boolean_T rtb_NOT_j;
  boolean_T rtb_b_DriverInterfaceError;

  /* Chart: '<S3>/Chart' incorporates:
   *  Delay: '<S3>/Delay'
   *  Delay: '<S3>/Delay1'
   *  Delay: '<S3>/Delay2'
   */
  if (controller_autogen_DW.is_active_c1_governor_lib == 0U) {
    controller_autogen_DW.is_active_c1_governor_lib = 1U;
    controller_autogen_DW.is_c1_governor_lib = controlle_IN_Initialize_outputs;
    controller_autogen_B.GOV_e_miCmd = CMD_INIT;
    controller_autogen_B.GOV_e_diCmd = DI_CMD_INIT;
    controller_autogen_DW.motorStartCount = 0U;
  } else {
    switch (controller_autogen_DW.is_c1_governor_lib) {
     case controlle_IN_Initialize_outputs:
      controller_autogen_B.GOV_e_diCmd = DI_CMD_INIT;
      controller_autogen_DW.is_c1_governor_lib = controller_autogen_IN_STARTUP;
      controller_autogen_DW.is_STARTUP = controller_autoge_IN_HV_startup;
      break;

     case controller_autogen_IN_RUNNING:
      if (controller_autogen_DW.Delay_DSTATE_f == ERR_RUNNING) {
        controller_autogen_DW.is_c1_governor_lib =
          controller_aut_IN_RUNNING_ERROR;
        controller_autogen_DW.is_RUNNING_ERROR = controller_auto_IN_HV_run_error;
        controller_autogen_B.GOV_e_diCmd = SYSTEM_ERROR;
      } else {
        switch (controller_autogen_DW.Delay1_DSTATE) {
         case MI_STS_ERROR:
          controller_autogen_DW.is_c1_governor_lib =
            controller_aut_IN_RUNNING_ERROR;
          controller_autogen_DW.is_RUNNING_ERROR =
            controller_a_IN_Motor_run_error;
          controller_autogen_B.GOV_e_diCmd = SYSTEM_ERROR;
          break;

         case OFF:
          controller_autogen_DW.is_c1_governor_lib =
            controlle_IN_Initialize_outputs;
          controller_autogen_B.GOV_e_miCmd = CMD_INIT;
          controller_autogen_B.GOV_e_diCmd = DI_CMD_INIT;
          controller_autogen_DW.motorStartCount = 0U;
          break;
        }
      }
      break;

     case controller_aut_IN_RUNNING_ERROR:
      if (controller_autogen_DW.is_RUNNING_ERROR ==
          controller_auto_IN_HV_run_error) {
        controller_autogen_B.GOV_e_diCmd = SYSTEM_ERROR;
      } else {
        /* case IN_Motor_run_error: */
        controller_autogen_B.GOV_e_diCmd = SYSTEM_ERROR;
      }
      break;

     case controller_autogen_IN_STARTUP:
      {
        if (controller_autogen_DW.Delay_DSTATE_f == ERR_STARTUP) {
          controller_autogen_DW.is_STARTUP = controller_IN_NO_ACTIVE_CHILD_o;
          controller_autogen_DW.is_c1_governor_lib =
            controller_aut_IN_STARTUP_ERROR;
          controller_autogen_DW.is_STARTUP_ERROR =
            controller__IN_HV_startup_error;
        } else if (controller_autogen_DW.Delay2_DSTATE == DI_ERROR) {
          controller_autogen_DW.is_STARTUP = controller_IN_NO_ACTIVE_CHILD_o;
          controller_autogen_DW.is_c1_governor_lib =
            controller_aut_IN_STARTUP_ERROR;
          controller_autogen_DW.is_STARTUP_ERROR =
            contro_IN_DriverInterface_Error;
        } else if (controller_autogen_DW.Delay1_DSTATE == MI_STS_ERROR) {
          if (controller_autogen_DW.motorStartCount >= 5) {
            controller_autogen_DW.is_STARTUP = controller_IN_NO_ACTIVE_CHILD_o;
            controller_autogen_DW.is_c1_governor_lib =
              controller_aut_IN_STARTUP_ERROR;
            controller_autogen_DW.is_STARTUP_ERROR =
              controller_aut_IN_Motor_faulted;
            controller_autogen_B.GOV_e_miCmd = CMD_SHUTDOWN;
          } else {
            controller_autogen_DW.is_STARTUP = controller_IN_NO_ACTIVE_CHILD_o;
            controller_autogen_DW.is_c1_governor_lib =
              controller_aut_IN_STARTUP_ERROR;
            controller_autogen_DW.is_STARTUP_ERROR =
              controller_autogen_IN_Err_reset;
          }
        } else {
          switch (controller_autogen_DW.is_STARTUP) {
           case contro_IN_Command_motor_startup:
            if ((controller_autogen_DW.Delay1_DSTATE == RUNNING) &&
                (controller_autogen_DW.Delay2_DSTATE == DRV_START_REQ)) {
              controller_autogen_DW.is_STARTUP = controller_IN_Send_ReadyToDrive;
              controller_autogen_B.GOV_e_diCmd = READY_TO_DRIVE;
            }
            break;

           case controller_autoge_IN_HV_startup:
            {
              if (controller_autogen_DW.Delay_DSTATE_f == BM_RUNNING) {
                uint32_T tmp;
                controller_autogen_DW.is_STARTUP =
                  contro_IN_Command_motor_startup;
                controller_autogen_B.GOV_e_miCmd = CMD_STARTUP;
                tmp = controller_autogen_DW.motorStartCount + 1U;
                if (controller_autogen_DW.motorStartCount + 1U > 65535U) {
                  tmp = 65535U;
                }

                controller_autogen_DW.motorStartCount = (uint16_T)tmp;
              }
            }
            break;

           default:
            /* case IN_Send_ReadyToDrive: */
            controller_autogen_B.GOV_e_diCmd = READY_TO_DRIVE;
            if (controller_autogen_DW.Delay2_DSTATE == DI_RUNNING) {
              controller_autogen_DW.is_STARTUP = controller_IN_NO_ACTIVE_CHILD_o;
              controller_autogen_DW.is_c1_governor_lib =
                controller_autogen_IN_RUNNING;
            }
            break;
          }
        }
      }
      break;

     default:
      /* case IN_STARTUP_ERROR: */
      switch (controller_autogen_DW.is_STARTUP_ERROR) {
       case contro_IN_DriverInterface_Error:
       case controller__IN_HV_startup_error:
        break;

       case controller_autogen_IN_Err_reset:
        if (controller_autogen_DW.Delay1_DSTATE == OFF) {
          controller_autogen_DW.is_STARTUP_ERROR =
            controller_IN_NO_ACTIVE_CHILD_o;
          controller_autogen_DW.is_c1_governor_lib =
            controller_autogen_IN_STARTUP;
          controller_autogen_DW.is_STARTUP = controller_autoge_IN_HV_startup;
        } else {
          controller_autogen_B.GOV_e_miCmd = ERR_RESET;
        }
        break;

       default:
        /* case IN_Motor_faulted: */
        break;
      }
      break;
    }
  }

  /* End of Chart: '<S3>/Chart' */

  /* Logic: '<S12>/NOT' incorporates:
   *  Constant: '<S12>/LowerPotentiometerLimit1'
   *  Constant: '<S12>/UpperPotentiometerLimit1'
   *  Inport: '<Root>/DI_V_AccelPedalPos1'
   *  Logic: '<S16>/FixPt Logical Operator'
   *  RelationalOperator: '<S16>/Lower Test'
   *  RelationalOperator: '<S16>/Upper Test'
   */
  rtb_NOT_g = ((!(controller_autogen_U.DI_V_AccelPedalPos1 >= 0.0)) ||
               (!(controller_autogen_U.DI_V_AccelPedalPos1 <= 4095.0)));

  /* Logic: '<S14>/NOT' incorporates:
   *  Constant: '<S14>/LowerPotentiometerLimit1'
   *  Constant: '<S14>/UpperPotentiometerLimit1'
   *  Inport: '<Root>/DI_V_BrakePedalPos'
   *  Logic: '<S20>/FixPt Logical Operator'
   *  RelationalOperator: '<S20>/Lower Test'
   *  RelationalOperator: '<S20>/Upper Test'
   */
  rtb_NOT_j = ((!(controller_autogen_U.DI_V_BrakePedalPos >= 0.0)) ||
               (!(controller_autogen_U.DI_V_BrakePedalPos <= 4095.0)));

  /* Logic: '<S2>/b_DriverInterfaceError' incorporates:
   *  Constant: '<S13>/LowerPotentiometerLimit1'
   *  Constant: '<S13>/UpperPotentiometerLimit1'
   *  Constant: '<S15>/LowerPotentiometerLimit1'
   *  Constant: '<S15>/UpperPotentiometerLimit1'
   *  Inport: '<Root>/DI_V_AccelPedalPos2'
   *  Inport: '<Root>/DI_V_SteeringAngle'
   *  Logic: '<S13>/NOT'
   *  Logic: '<S15>/NOT'
   *  Logic: '<S18>/FixPt Logical Operator'
   *  Logic: '<S22>/FixPt Logical Operator'
   *  Logic: '<S2>/AND'
   *  RelationalOperator: '<S18>/Lower Test'
   *  RelationalOperator: '<S18>/Upper Test'
   *  RelationalOperator: '<S22>/Lower Test'
   *  RelationalOperator: '<S22>/Upper Test'
   */
  rtb_b_DriverInterfaceError = ((rtb_NOT_g &&
    ((!(controller_autogen_U.DI_V_AccelPedalPos2 >= 0.0)) ||
     (!(controller_autogen_U.DI_V_AccelPedalPos2 <= 4095.0)))) || rtb_NOT_j ||
    ((!(controller_autogen_U.DI_V_SteeringAngle >= 0.0)) ||
     (!(controller_autogen_U.DI_V_SteeringAngle <= 4095.0))));

  /* Gain: '<S21>/Gain' incorporates:
   *  DataTypeConversion: '<S14>/Data Type Conversion1'
   *  Inport: '<Root>/DI_V_BrakePedalPos'
   *  Product: '<S21>/Divide'
   *  Sum: '<S21>/Subtract1'
   */
  rtb_Gain_o = ((real32_T)controller_autogen_U.DI_V_BrakePedalPos -
                controller_autogen_ConstB.DataTypeConversion2_b) /
    controller_autogen_ConstB.range_i * 100.0F;

  /* Chart: '<S2>/Chart' incorporates:
   *  Delay: '<S2>/Delay1'
   *  Inport: '<Root>/DI_b_DriverButton'
   */
  if (controller_autogen_DW.temporalCounter_i1 < 511U) {
    controller_autogen_DW.temporalCounter_i1++;
  }

  if (controller_autogen_DW.is_active_c1_driver_interface_l == 0U) {
    controller_autogen_DW.is_active_c1_driver_interface_l = 1U;
    controller_autogen_DW.is_c1_driver_interface_lib =
      controller_autogen_IN_INIT;
    controller_autogen_B.GOV_e_diSts = DI_STS_INIT;
    controller_autogen_B.b_ReadyToDrive = false;

    /* Outport: '<Root>/DI_b_driverSpeaker' */
    controller_autogen_Y.DI_b_driverSpeaker = false;

    /* Outport: '<Root>/DI_p_PWMstatusLightCycle' */
    controller_autogen_Y.DI_p_PWMstatusLightCycle = 0.0;

    /* Outport: '<Root>/DI_p_PWMstatusLightFreq' */
    controller_autogen_Y.DI_p_PWMstatusLightFreq = 1.0;
  } else {
    switch (controller_autogen_DW.is_c1_driver_interface_lib) {
     case controller_autogen_IN_DI_error:
      controller_autogen_B.GOV_e_diSts = DI_ERROR;
      break;

     case controller_autoge_IN_DI_running:
      if (rtb_b_DriverInterfaceError) {
        controller_autogen_DW.is_Ready_to_drive =
          controller_IN_NO_ACTIVE_CHILD_o;
        controller_autogen_DW.is_DI_running = controller_IN_NO_ACTIVE_CHILD_o;
        controller_autogen_DW.is_c1_driver_interface_lib =
          controller_autogen_IN_DI_error;
        controller_autogen_B.GOV_e_diSts = DI_ERROR;
      } else {
        switch (controller_autogen_DW.is_DI_running) {
         case contr_IN_Driver_requested_start:
          controller_autogen_B.GOV_e_diSts = DRV_START_REQ;

          /* Outport: '<Root>/DI_p_PWMstatusLightCycle' */
          controller_autogen_Y.DI_p_PWMstatusLightCycle = 50.0;

          /* Outport: '<Root>/DI_p_PWMstatusLightFreq' */
          controller_autogen_Y.DI_p_PWMstatusLightFreq = 10.0;
          if (controller_autogen_DW.Delay1_DSTATE_l == READY) {
            controller_autogen_DW.is_DI_running =
              controller_autog_IN_Motor_ready;

            /* Outport: '<Root>/DI_p_PWMstatusLightCycle' */
            controller_autogen_Y.DI_p_PWMstatusLightCycle = 100.0;
          }
          break;

         case controller_autog_IN_Motor_ready:
          /* Outport: '<Root>/DI_p_PWMstatusLightCycle' */
          controller_autogen_Y.DI_p_PWMstatusLightCycle = 100.0;

          /* Outport: '<Root>/DI_p_PWMstatusLightFreq' */
          controller_autogen_Y.DI_p_PWMstatusLightFreq = 10.0;
          if ((controller_autogen_B.GOV_e_diCmd == READY_TO_DRIVE) &&
              (rtb_Gain_o > 50.0F)) {
            controller_autogen_DW.is_DI_running =
              controller_au_IN_Ready_to_drive;
            controller_autogen_B.b_ReadyToDrive = true;
            controller_autogen_B.GOV_e_diSts = DI_RUNNING;
            controller_autogen_DW.is_Ready_to_drive =
              controller_autoge_IN_SpeakerOn_;
            controller_autogen_DW.temporalCounter_i1 = 0U;

            /* Outport: '<Root>/DI_b_driverSpeaker' */
            controller_autogen_Y.DI_b_driverSpeaker = true;
          }
          break;

         case controller_au_IN_Ready_to_drive:
          controller_autogen_B.b_ReadyToDrive = true;
          controller_autogen_B.GOV_e_diSts = DI_RUNNING;

          /* Outport: '<Root>/DI_p_PWMstatusLightCycle' */
          controller_autogen_Y.DI_p_PWMstatusLightCycle = 100.0;

          /* Outport: '<Root>/DI_p_PWMstatusLightFreq' */
          controller_autogen_Y.DI_p_PWMstatusLightFreq = 10.0;
          if (controller_autogen_B.GOV_e_diCmd == SYSTEM_ERROR) {
            controller_autogen_DW.is_Ready_to_drive =
              controller_IN_NO_ACTIVE_CHILD_o;
            controller_autogen_DW.is_DI_running =
              controller_IN_NO_ACTIVE_CHILD_o;
            controller_autogen_DW.is_c1_driver_interface_lib =
              controller__IN_Vehicle_coasting;
            controller_autogen_B.b_ReadyToDrive = false;
            controller_autogen_B.GOV_e_diSts = DI_IDLE;
          } else if (controller_autogen_DW.Delay1_DSTATE_l == OFF) {
            controller_autogen_DW.is_Ready_to_drive =
              controller_IN_NO_ACTIVE_CHILD_o;
            controller_autogen_DW.is_DI_running =
              controller_IN_NO_ACTIVE_CHILD_o;
            controller_autogen_DW.is_c1_driver_interface_lib =
              controller_autogen_IN_INIT;
            controller_autogen_B.GOV_e_diSts = DI_STS_INIT;
            controller_autogen_B.b_ReadyToDrive = false;

            /* Outport: '<Root>/DI_b_driverSpeaker' */
            controller_autogen_Y.DI_b_driverSpeaker = false;

            /* Outport: '<Root>/DI_p_PWMstatusLightCycle' */
            controller_autogen_Y.DI_p_PWMstatusLightCycle = 0.0;

            /* Outport: '<Root>/DI_p_PWMstatusLightFreq' */
            controller_autogen_Y.DI_p_PWMstatusLightFreq = 1.0;
          } else if (controller_autogen_DW.is_Ready_to_drive ==
                     controller_autoge_IN_SpeakerOff) {
            /* Outport: '<Root>/DI_b_driverSpeaker' */
            controller_autogen_Y.DI_b_driverSpeaker = false;
          } else {
            /* Outport: '<Root>/DI_b_driverSpeaker' */
            /* case IN_SpeakerOn_: */
            controller_autogen_Y.DI_b_driverSpeaker = true;
            if (controller_autogen_DW.temporalCounter_i1 >= 400U) {
              controller_autogen_DW.is_Ready_to_drive =
                controller_autoge_IN_SpeakerOff;

              /* Outport: '<Root>/DI_b_driverSpeaker' */
              controller_autogen_Y.DI_b_driverSpeaker = false;
            }
          }
          break;

         default:
          /* case IN_Waiting_for_driver: */
          controller_autogen_B.GOV_e_diSts = WAITING_FOR_DRVR;

          /* Outport: '<Root>/DI_p_PWMstatusLightCycle' */
          controller_autogen_Y.DI_p_PWMstatusLightCycle = 50.0;

          /* Outport: '<Root>/DI_p_PWMstatusLightFreq' */
          controller_autogen_Y.DI_p_PWMstatusLightFreq = 1.0;
          if (controller_autogen_U.DI_b_DriverButton == 1.0) {
            controller_autogen_DW.is_DI_running =
              contr_IN_Driver_requested_start;
            controller_autogen_B.GOV_e_diSts = DRV_START_REQ;

            /* Outport: '<Root>/DI_p_PWMstatusLightFreq' */
            controller_autogen_Y.DI_p_PWMstatusLightFreq = 10.0;
          }
          break;
        }
      }
      break;

     case controller_autogen_IN_INIT:
      controller_autogen_B.b_ReadyToDrive = false;

      /* Outport: '<Root>/DI_b_driverSpeaker' */
      controller_autogen_Y.DI_b_driverSpeaker = false;
      controller_autogen_DW.is_c1_driver_interface_lib =
        controller_autoge_IN_DI_running;
      controller_autogen_DW.is_DI_running = controlle_IN_Waiting_for_driver;
      controller_autogen_B.GOV_e_diSts = WAITING_FOR_DRVR;

      /* Outport: '<Root>/DI_p_PWMstatusLightCycle' */
      controller_autogen_Y.DI_p_PWMstatusLightCycle = 50.0;

      /* Outport: '<Root>/DI_p_PWMstatusLightFreq' */
      controller_autogen_Y.DI_p_PWMstatusLightFreq = 1.0;
      break;

     default:
      /* case IN_Vehicle_coasting: */
      controller_autogen_B.b_ReadyToDrive = false;
      controller_autogen_B.GOV_e_diSts = DI_IDLE;
      break;
    }
  }

  /* End of Chart: '<S2>/Chart' */

  /* If: '<S2>/If' incorporates:
   *  Constant: '<S2>/Constant'
   *  DataTypeConversion: '<S12>/Data Type Conversion1'
   *  DataTypeConversion: '<S13>/Data Type Conversion1'
   *  Gain: '<S17>/Gain'
   *  Gain: '<S19>/Gain'
   *  Inport: '<Root>/DI_V_AccelPedalPos1'
   *  Inport: '<Root>/DI_V_AccelPedalPos2'
   *  Logic: '<S2>/NOT'
   *  Logic: '<S2>/OR1'
   *  Lookup_n-D: '<S2>/AccelPedalPos1 LUT'
   *  Product: '<S17>/Divide'
   *  Product: '<S19>/Divide'
   *  SignalConversion generated from: '<S9>/In1'
   *  Sum: '<S17>/Subtract1'
   *  Sum: '<S19>/Subtract1'
   */
  if ((!controller_autogen_B.b_ReadyToDrive) || rtb_b_DriverInterfaceError) {
    /* Outputs for IfAction SubSystem: '<S2>/If Action Subsystem' incorporates:
     *  ActionPort: '<S9>/Action Port'
     */
    rtb_AccelPedalPos1LUT = 0.0F;

    /* End of Outputs for SubSystem: '<S2>/If Action Subsystem' */
  } else if (rtb_NOT_g) {
    rtb_AccelPedalPos1LUT = ((real32_T)controller_autogen_U.DI_V_AccelPedalPos2
      - controller_autogen_ConstB.DataTypeConversion2_e) /
      controller_autogen_ConstB.range_l * 100.0F;
  } else {
    rtb_AccelPedalPos1LUT = ((real32_T)controller_autogen_U.DI_V_AccelPedalPos1
      - controller_autogen_ConstB.DataTypeConversion2) /
      controller_autogen_ConstB.range * 100.0F;
  }

  rtb_AccelPedalPos1LUT = look1_iflf_binlcpw(rtb_AccelPedalPos1LUT,
    controller_autogen_ConstP.pooled8, controller_autogen_ConstP.pooled8, 20U);

  /* End of If: '<S2>/If' */

  /* Switch: '<S2>/Switch2' incorporates:
   *  Constant: '<S2>/Constant1'
   */
  if (rtb_NOT_j) {
    rtb_Gain_o = 0.0F;
  }

  /* Lookup_n-D: '<S2>/BrakePedalPos1 LUT1' incorporates:
   *  Switch: '<S2>/Switch2'
   */
  rtb_Gain_o = look1_iflf_binlcpw(rtb_Gain_o, controller_autogen_ConstP.pooled8,
    controller_autogen_ConstP.pooled8, 20U);

  /* Switch: '<S5>/Switch' */
  rtb_Switch = (rtb_Gain_o > 10.0F);

  /* Chart: '<S5>/Chart' */
  if (controller_autogen_DW.is_active_c3_controller_autogen == 0U) {
    controller_autogen_DW.is_active_c3_controller_autogen = 1U;
    controller_autogen_DW.is_c3_controller_autogen = controller_autogen_IN_Run;
  } else if (controller_autogen_DW.is_c3_controller_autogen ==
             controller_autogen_IN_Run) {
    if ((rtb_AccelPedalPos1LUT >= 20.0F) && (rtb_Switch != 0)) {
      controller_autogen_DW.is_c3_controller_autogen =
        controller_autogen_IN_Stop;
      rtb_AccelPedalPos1LUT = 0.0F;
    }

    /* case IN_Stop: */
  } else if ((rtb_AccelPedalPos1LUT < 5.0F) && (rtb_Switch == 0)) {
    controller_autogen_DW.is_c3_controller_autogen = controller_autogen_IN_Run;
  } else {
    rtb_AccelPedalPos1LUT = 0.0F;
  }

  /* End of Chart: '<S5>/Chart' */

  /* Gain: '<S5>/Gain' incorporates:
   *  Constant: '<S5>/Constant6'
   *  Product: '<S5>/Divide'
   */
  rtb_Gain = rtb_AccelPedalPos1LUT / 100.0 * 500.0;

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
  controller_autogen_LEFT_MOTOR(controller_autogen_B.GOV_e_miCmd,
    controller_autogen_U.AMK_bSystemReady_R, controller_autogen_U.AMK_bError_R,
    controller_autogen_U.AMK_bQuitDcOn_R, controller_autogen_U.AMK_bDcOn_R,
    controller_autogen_U.AMK_bQuitInverterOn_R,
    controller_autogen_U.AMK_bInverterOn_R, controller_autogen_U.AMK_bDerating_R,
    controller_autogen_ConstB.CastToSingle, (real32_T)rtb_Gain,
    controller_autogen_ConstB.CastToSingle2,
    &controller_autogen_B.MI_motorStatus,
    &controller_autogen_Y.AMK_bInverterOn_tx_R,
    &controller_autogen_Y.AMK_bDcOn_tx_R, &controller_autogen_Y.AMK_bEnable_R,
    &controller_autogen_Y.AMK_bErrorReset_R,
    &controller_autogen_Y.AMK_TargetVelocity_R,
    &controller_autogen_Y.AMK_TorqueLimitPositiv_R,
    &controller_autogen_Y.AMK_TorqueLimitNegativ_R,
    &controller_autogen_DW.sf_RIGHT_MOTOR);

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
  controller_autogen_LEFT_MOTOR(controller_autogen_B.GOV_e_miCmd,
    controller_autogen_U.AMK_bSystemReady_L, controller_autogen_U.AMK_bError_L,
    controller_autogen_U.AMK_bQuitDcOn_L, controller_autogen_U.AMK_bDcOn_L,
    controller_autogen_U.AMK_bQuitInverterOn_L,
    controller_autogen_U.AMK_bInverterOn_L, controller_autogen_U.AMK_bDerating_L,
    controller_autogen_ConstB.CastToSingle3, (real32_T)rtb_Gain,
    controller_autogen_ConstB.CastToSingle5,
    &controller_autogen_B.MI_motorStatus_b,
    &controller_autogen_Y.AMK_bInverterOn_tx_L,
    &controller_autogen_Y.AMK_bDcOn_tx_L, &controller_autogen_Y.AMK_bEnable_L,
    &controller_autogen_Y.AMK_bErrorReset_L,
    &controller_autogen_Y.AMK_TargetVelocity_L,
    &controller_autogen_Y.AMK_TorqueLimitPositiv_L,
    &controller_autogen_Y.AMK_TorqueLimitNegativ_L,
    &controller_autogen_DW.sf_LEFT_MOTOR);

  /* Chart: '<S1>/Chart' incorporates:
   *  Delay: '<S3>/Delay'
   *  Inport: '<Root>/BM_b_HVnegContactorSts'
   *  Inport: '<Root>/BM_b_HVposContactorSts'
   *  Inport: '<Root>/BM_b_prechrgContactorSts'
   */
  if (controller_autogen_DW.temporalCounter_i1_g < 32767U) {
    controller_autogen_DW.temporalCounter_i1_g++;
  }

  if (controller_autogen_DW.is_active_c2_battery_monitor_li == 0U) {
    controller_autogen_DW.is_active_c2_battery_monitor_li = 1U;
    controller_autogen_DW.is_c2_battery_monitor_lib =
      controller_auto_IN_InitialState;
    controller_autogen_DW.Delay_DSTATE_f = BM_INIT;
  } else {
    switch (controller_autogen_DW.is_c2_battery_monitor_lib) {
     case controll_IN_ErrorAllClosedState:
      controller_autogen_DW.Delay_DSTATE_f = ERR_ALL_CLOSED;
      break;

     case controller_a_IN_ErrorHVPositive:
      controller_autogen_DW.Delay_DSTATE_f = ERR_HV_POSITIVE;
      break;

     case IN_ErrorInitializePrechargeStat:
      controller_autogen_DW.Delay_DSTATE_f = ERR_INIT_PRECHARGE;
      break;

     case co_IN_ErrorPrechargeClosedState:
      controller_autogen_DW.Delay_DSTATE_f = ERR_PRECHARGE_CLOSED;
      break;

     case controll_IN_ErrorPrechargeState:
      controller_autogen_DW.Delay_DSTATE_f = ERR_PRECHARGE;
      break;

     case controller_IN_ErrorStartupState:
      controller_autogen_DW.Delay_DSTATE_f = ERR_STARTUP;
      break;

     case controller_auto_IN_InitialState:
      controller_autogen_DW.Delay_DSTATE_f = BM_INIT;
      if ((controller_autogen_U.BM_b_prechrgContactorSts == 0.0) &&
          (controller_autogen_U.BM_b_HVnegContactorSts == 0.0) &&
          (controller_autogen_U.BM_b_HVposContactorSts == 0.0) &&
          (controller_autogen_B.GOV_e_diSts == DRV_START_REQ)) {
        controller_autogen_DW.is_c2_battery_monitor_lib =
          controller_aut_IN_StartupState1;
        controller_autogen_DW.Delay_DSTATE_f = BM_IDLE;
      } else if ((controller_autogen_U.BM_b_prechrgContactorSts == 1.0) &&
                 (controller_autogen_U.BM_b_HVnegContactorSts == 0.0) &&
                 (controller_autogen_U.BM_b_HVposContactorSts == 0.0)) {
        controller_autogen_DW.is_c2_battery_monitor_lib =
          co_IN_ErrorPrechargeClosedState;
        controller_autogen_DW.Delay_DSTATE_f = ERR_PRECHARGE_CLOSED;
      } else if ((controller_autogen_U.BM_b_prechrgContactorSts == 0.0) &&
                 (controller_autogen_U.BM_b_HVnegContactorSts == 0.0) &&
                 (controller_autogen_U.BM_b_HVposContactorSts == 1.0)) {
        controller_autogen_DW.is_c2_battery_monitor_lib =
          controller_a_IN_ErrorHVPositive;
        controller_autogen_DW.Delay_DSTATE_f = ERR_HV_POSITIVE;
      }
      break;

     case con_IN_InitializePrechargeState:
      controller_autogen_DW.Delay_DSTATE_f = INIT_PRECHARGE;
      if ((controller_autogen_U.BM_b_prechrgContactorSts == 1.0) &&
          (controller_autogen_U.BM_b_HVnegContactorSts == 1.0) &&
          (controller_autogen_U.BM_b_HVposContactorSts == 1.0)) {
        controller_autogen_DW.is_c2_battery_monitor_lib =
          controller_au_IN_PrechargeState;
        controller_autogen_DW.temporalCounter_i1_g = 0U;
        controller_autogen_DW.Delay_DSTATE_f = PRECHARGE;
      } else if (controller_autogen_DW.temporalCounter_i1_g >= 20000U) {
        controller_autogen_DW.is_c2_battery_monitor_lib =
          IN_ErrorInitializePrechargeStat;
        controller_autogen_DW.Delay_DSTATE_f = ERR_INIT_PRECHARGE;
      }
      break;

     case controller_au_IN_PrechargeState:
      controller_autogen_DW.Delay_DSTATE_f = PRECHARGE;
      if ((controller_autogen_U.BM_b_prechrgContactorSts == 0.0) &&
          (controller_autogen_U.BM_b_HVnegContactorSts == 1.0) &&
          (controller_autogen_U.BM_b_HVposContactorSts == 1.0)) {
        controller_autogen_DW.is_c2_battery_monitor_lib =
          controller_auto_IN_RunningState;
        controller_autogen_DW.temporalCounter_i1_g = 0U;
        controller_autogen_DW.Delay_DSTATE_f = BM_RUNNING;
      } else if (controller_autogen_DW.temporalCounter_i1_g >= 20000U) {
        controller_autogen_DW.is_c2_battery_monitor_lib =
          controll_IN_ErrorPrechargeState;
        controller_autogen_DW.Delay_DSTATE_f = ERR_PRECHARGE;
      }
      break;

     case controller_auto_IN_RunningState:
      controller_autogen_DW.Delay_DSTATE_f = BM_RUNNING;
      if ((controller_autogen_U.BM_b_prechrgContactorSts == 0.0) &&
          (controller_autogen_U.BM_b_HVnegContactorSts == 1.0) &&
          (controller_autogen_U.BM_b_HVposContactorSts == 1.0)) {
        controller_autogen_DW.is_c2_battery_monitor_lib =
          controller_auto_IN_RunningState;
        controller_autogen_DW.temporalCounter_i1_g = 0U;
        controller_autogen_DW.Delay_DSTATE_f = BM_RUNNING;
      } else if (controller_autogen_DW.temporalCounter_i1_g >= 100U) {
        controller_autogen_DW.is_c2_battery_monitor_lib =
          controller_auto_IN_InitialState;
        controller_autogen_DW.Delay_DSTATE_f = BM_INIT;
      }
      break;

     case controller_auto_IN_StartupState:
      controller_autogen_DW.Delay_DSTATE_f = BM_STARTUP;
      if ((controller_autogen_U.BM_b_prechrgContactorSts == 1.0) &&
          (controller_autogen_U.BM_b_HVnegContactorSts == 1.0) &&
          (controller_autogen_U.BM_b_HVposContactorSts == 0.0)) {
        controller_autogen_DW.is_c2_battery_monitor_lib =
          con_IN_InitializePrechargeState;
        controller_autogen_DW.temporalCounter_i1_g = 0U;
        controller_autogen_DW.Delay_DSTATE_f = INIT_PRECHARGE;
      } else if (controller_autogen_DW.temporalCounter_i1_g >= 20000U) {
        controller_autogen_DW.is_c2_battery_monitor_lib =
          controller_IN_ErrorStartupState;
        controller_autogen_DW.Delay_DSTATE_f = ERR_STARTUP;
      }
      break;

     default:
      /* case IN_StartupState1: */
      controller_autogen_DW.Delay_DSTATE_f = BM_IDLE;
      if ((controller_autogen_U.BM_b_prechrgContactorSts == 0.0) &&
          (controller_autogen_U.BM_b_HVnegContactorSts == 1.0) &&
          (controller_autogen_U.BM_b_HVposContactorSts == 0.0)) {
        controller_autogen_DW.is_c2_battery_monitor_lib =
          controller_auto_IN_StartupState;
        controller_autogen_DW.temporalCounter_i1_g = 0U;
        controller_autogen_DW.Delay_DSTATE_f = BM_STARTUP;
      }
      break;
    }
  }

  /* End of Chart: '<S1>/Chart' */

  /* Chart: '<S1>/Chart1' incorporates:
   *  Delay: '<S3>/Delay'
   */
  if (controller_autogen_DW.temporalCounter_i1_b < 2047U) {
    controller_autogen_DW.temporalCounter_i1_b++;
  }

  if (controller_autogen_DW.is_active_c4_battery_monitor_li == 0U) {
    controller_autogen_DW.is_active_c4_battery_monitor_li = 1U;
    controller_autogen_DW.is_c4_battery_monitor_lib =
      controller_autoge_IN_StartupCMD;

    /* Outport: '<Root>/BM_b_prechargeContactorCMD' */
    controller_autogen_Y.BM_b_prechargeContactorCMD = 0.0;

    /* Outport: '<Root>/BM_b_HVnegContactorCMD' */
    controller_autogen_Y.BM_b_HVnegContactorCMD = 0.0;

    /* Outport: '<Root>/BM_b_HVposContactorCMD' */
    controller_autogen_Y.BM_b_HVposContactorCMD = 0.0;
  } else {
    switch (controller_autogen_DW.is_c4_battery_monitor_lib) {
     case controller_autog_IN_Close_HVneg:
      /* Outport: '<Root>/BM_b_prechargeContactorCMD' */
      controller_autogen_Y.BM_b_prechargeContactorCMD = 1.0;

      /* Outport: '<Root>/BM_b_HVnegContactorCMD' */
      controller_autogen_Y.BM_b_HVnegContactorCMD = 1.0;

      /* Outport: '<Root>/BM_b_HVposContactorCMD' */
      controller_autogen_Y.BM_b_HVposContactorCMD = 0.0;
      if ((controller_autogen_DW.temporalCounter_i1_b >= 1300U) &&
          (controller_autogen_DW.Delay_DSTATE_f == INIT_PRECHARGE)) {
        controller_autogen_DW.is_c4_battery_monitor_lib =
          controller_autog_IN_Close_HVpos;
        controller_autogen_DW.temporalCounter_i1_b = 0U;

        /* Outport: '<Root>/BM_b_HVposContactorCMD' */
        controller_autogen_Y.BM_b_HVposContactorCMD = 1.0;
      }
      break;

     case controller_autog_IN_Close_HVpos:
      /* Outport: '<Root>/BM_b_prechargeContactorCMD' */
      controller_autogen_Y.BM_b_prechargeContactorCMD = 1.0;

      /* Outport: '<Root>/BM_b_HVnegContactorCMD' */
      controller_autogen_Y.BM_b_HVnegContactorCMD = 1.0;

      /* Outport: '<Root>/BM_b_HVposContactorCMD' */
      controller_autogen_Y.BM_b_HVposContactorCMD = 1.0;
      if ((controller_autogen_DW.temporalCounter_i1_b >= 20U) &&
          (controller_autogen_DW.Delay_DSTATE_f == PRECHARGE)) {
        controller_autogen_DW.is_c4_battery_monitor_lib =
          controller_au_IN_Open_precharge;
        controller_autogen_DW.temporalCounter_i1_b = 0U;

        /* Outport: '<Root>/BM_b_prechargeContactorCMD' */
        controller_autogen_Y.BM_b_prechargeContactorCMD = 0.0;
      }
      break;

     case controller_a_IN_Close_precharge:
      /* Outport: '<Root>/BM_b_prechargeContactorCMD' */
      controller_autogen_Y.BM_b_prechargeContactorCMD = 0.0;

      /* Outport: '<Root>/BM_b_HVnegContactorCMD' */
      controller_autogen_Y.BM_b_HVnegContactorCMD = 1.0;

      /* Outport: '<Root>/BM_b_HVposContactorCMD' */
      controller_autogen_Y.BM_b_HVposContactorCMD = 0.0;
      if ((controller_autogen_DW.temporalCounter_i1_b >= 20U) &&
          (controller_autogen_DW.Delay_DSTATE_f == BM_STARTUP)) {
        controller_autogen_DW.is_c4_battery_monitor_lib =
          controller_autog_IN_Close_HVneg;
        controller_autogen_DW.temporalCounter_i1_b = 0U;

        /* Outport: '<Root>/BM_b_prechargeContactorCMD' */
        controller_autogen_Y.BM_b_prechargeContactorCMD = 1.0;
      }
      break;

     case controller_au_IN_Open_precharge:
      /* Outport: '<Root>/BM_b_prechargeContactorCMD' */
      controller_autogen_Y.BM_b_prechargeContactorCMD = 0.0;

      /* Outport: '<Root>/BM_b_HVnegContactorCMD' */
      controller_autogen_Y.BM_b_HVnegContactorCMD = 1.0;

      /* Outport: '<Root>/BM_b_HVposContactorCMD' */
      controller_autogen_Y.BM_b_HVposContactorCMD = 1.0;
      if (controller_autogen_DW.Delay_DSTATE_f == BM_RUNNING) {
        controller_autogen_DW.is_c4_battery_monitor_lib =
          controller_au_IN_Open_precharge;
        controller_autogen_DW.temporalCounter_i1_b = 0U;
      } else if (controller_autogen_DW.temporalCounter_i1_b >= 100U) {
        controller_autogen_DW.is_c4_battery_monitor_lib =
          controller_autoge_IN_StartupCMD;

        /* Outport: '<Root>/BM_b_HVnegContactorCMD' */
        controller_autogen_Y.BM_b_HVnegContactorCMD = 0.0;

        /* Outport: '<Root>/BM_b_HVposContactorCMD' */
        controller_autogen_Y.BM_b_HVposContactorCMD = 0.0;
      }
      break;

     default:
      /* Outport: '<Root>/BM_b_prechargeContactorCMD' */
      /* case IN_StartupCMD: */
      controller_autogen_Y.BM_b_prechargeContactorCMD = 0.0;

      /* Outport: '<Root>/BM_b_HVnegContactorCMD' */
      controller_autogen_Y.BM_b_HVnegContactorCMD = 0.0;

      /* Outport: '<Root>/BM_b_HVposContactorCMD' */
      controller_autogen_Y.BM_b_HVposContactorCMD = 0.0;
      if (controller_autogen_DW.Delay_DSTATE_f == BM_IDLE) {
        controller_autogen_DW.is_c4_battery_monitor_lib =
          controller_a_IN_Close_precharge;
        controller_autogen_DW.temporalCounter_i1_b = 0U;

        /* Outport: '<Root>/BM_b_HVnegContactorCMD' */
        controller_autogen_Y.BM_b_HVnegContactorCMD = 1.0;
      }
      break;
    }
  }

  /* End of Chart: '<S1>/Chart1' */

  /* Outport: '<Root>/DI_b_brakeLightEn' incorporates:
   *  Switch: '<S2>/Switch'
   */
  controller_autogen_Y.DI_b_brakeLightEn = (real32_T)(rtb_Gain_o > 10.0F);

  /* Switch: '<S4>/overallMotorState' incorporates:
   *  Delay: '<S4>/Delay'
   *  RelationalOperator: '<S4>/GreaterThan'
   */
  if (controller_autogen_B.MI_motorStatus_b ==
      controller_autogen_B.MI_motorStatus) {
    controller_autogen_DW.Delay_DSTATE = controller_autogen_B.MI_motorStatus_b;
  }

  /* End of Switch: '<S4>/overallMotorState' */

  /* Switch: '<S4>/Switch' incorporates:
   *  Constant: '<S4>/Constant'
   *  Constant: '<S4>/Constant1'
   *  Delay: '<S3>/Delay1'
   *  Delay: '<S4>/Delay'
   *  Logic: '<S4>/OR'
   *  RelationalOperator: '<S4>/motorErrorActive'
   *  RelationalOperator: '<S4>/motorErrorActive1'
   */
  if ((controller_autogen_B.MI_motorStatus == MI_STS_ERROR) ||
      (controller_autogen_B.MI_motorStatus_b == MI_STS_ERROR)) {
    controller_autogen_DW.Delay1_DSTATE = MI_STS_ERROR;
  } else {
    controller_autogen_DW.Delay1_DSTATE = controller_autogen_DW.Delay_DSTATE;
  }

  /* End of Switch: '<S4>/Switch' */

  /* Update for Delay: '<S3>/Delay2' */
  controller_autogen_DW.Delay2_DSTATE = controller_autogen_B.GOV_e_diSts;

  /* Update for Delay: '<S2>/Delay1' incorporates:
   *  Delay: '<S3>/Delay1'
   */
  controller_autogen_DW.Delay1_DSTATE_l = controller_autogen_DW.Delay1_DSTATE;
}

/* Model initialize function */
void controller_autogen_initialize(void)
{
  /* InitializeConditions for Delay: '<S4>/Delay' */
  controller_autogen_DW.Delay_DSTATE = OFF;

  /* SystemInitialize for Chart: '<S4>/RIGHT_MOTOR' incorporates:
   *  Outport: '<Root>/AMK_TargetVelocity_R'
   *  Outport: '<Root>/AMK_TorqueLimitNegativ_R'
   *  Outport: '<Root>/AMK_TorqueLimitPositiv_R'
   *  Outport: '<Root>/AMK_bDcOn_tx_R'
   *  Outport: '<Root>/AMK_bEnable_R'
   *  Outport: '<Root>/AMK_bErrorReset_R'
   *  Outport: '<Root>/AMK_bInverterOn_tx_R'
   */
  controller_auto_LEFT_MOTOR_Init(&controller_autogen_B.MI_motorStatus,
    &controller_autogen_Y.AMK_bInverterOn_tx_R,
    &controller_autogen_Y.AMK_bDcOn_tx_R, &controller_autogen_Y.AMK_bEnable_R,
    &controller_autogen_Y.AMK_bErrorReset_R,
    &controller_autogen_Y.AMK_TargetVelocity_R,
    &controller_autogen_Y.AMK_TorqueLimitPositiv_R,
    &controller_autogen_Y.AMK_TorqueLimitNegativ_R);

  /* SystemInitialize for Chart: '<S4>/LEFT_MOTOR' incorporates:
   *  Outport: '<Root>/AMK_TargetVelocity_L'
   *  Outport: '<Root>/AMK_TorqueLimitNegativ_L'
   *  Outport: '<Root>/AMK_TorqueLimitPositiv_L'
   *  Outport: '<Root>/AMK_bDcOn_tx_L'
   *  Outport: '<Root>/AMK_bEnable_L'
   *  Outport: '<Root>/AMK_bErrorReset_L'
   *  Outport: '<Root>/AMK_bInverterOn_tx_L'
   */
  controller_auto_LEFT_MOTOR_Init(&controller_autogen_B.MI_motorStatus_b,
    &controller_autogen_Y.AMK_bInverterOn_tx_L,
    &controller_autogen_Y.AMK_bDcOn_tx_L, &controller_autogen_Y.AMK_bEnable_L,
    &controller_autogen_Y.AMK_bErrorReset_L,
    &controller_autogen_Y.AMK_TargetVelocity_L,
    &controller_autogen_Y.AMK_TorqueLimitPositiv_L,
    &controller_autogen_Y.AMK_TorqueLimitNegativ_L);
}

/* Model terminate function */
void controller_autogen_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
