/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: controller_autogen_private.h
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

#ifndef RTW_HEADER_controller_autogen_private_h_
#define RTW_HEADER_controller_autogen_private_h_
#include "rtwtypes.h"
#include "controller_autogen_types.h"
#include "controller_autogen.h"

extern real32_T look1_iflf_binlcpw(real32_T u0, const real32_T bp0[], const
  real32_T table[], uint32_T maxIndex);
extern void controller_auto_LEFT_MOTOR_Init(MI_STATUSES *rty_MI_motorStatus,
  uint8_T *rty_AMK_bInverterOn_tx, uint8_T *rty_AMK_bDcOn_tx, uint8_T
  *rty_AMK_bEnable, uint8_T *rty_AMK_bErrorReset, real32_T
  *rty_AMK_TargetVelocity, real32_T *rty_AMK_TorqueLimitPositiv, real32_T
  *rty_AMK_TorqueLimitNegativ);
extern void controller_autogen_LEFT_MOTOR(MI_CMD rtu_GOV_e_miCmd, boolean_T
  rtu_AMK_bSystemReady, boolean_T rtu_AMK_bError, boolean_T rtu_AMK_bQuitDcOn,
  boolean_T rtu_AMK_bDcOn_rx, boolean_T rtu_AMK_bQuitInverterOn, boolean_T
  rtu_AMK_bInverterOn_rx, boolean_T rtu_AMK_bDerating, real32_T
  rtu_VD_T_motorSpeedRequest, real32_T rtu_VD_T_motorTorqueLimitPositi, real32_T
  rtu_VD_T_motorTorqueLimitNegati, MI_STATUSES *rty_MI_motorStatus, uint8_T
  *rty_AMK_bInverterOn_tx, uint8_T *rty_AMK_bDcOn_tx, uint8_T *rty_AMK_bEnable,
  uint8_T *rty_AMK_bErrorReset, real32_T *rty_AMK_TargetVelocity, real32_T
  *rty_AMK_TorqueLimitPositiv, real32_T *rty_AMK_TorqueLimitNegativ,
  DW_LEFT_MOTOR_controller_auto_T *localDW);

#endif                            /* RTW_HEADER_controller_autogen_private_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
