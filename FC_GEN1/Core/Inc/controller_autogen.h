/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: controller_autogen.h
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

#ifndef RTW_HEADER_controller_autogen_h_
#define RTW_HEADER_controller_autogen_h_
#ifndef controller_autogen_COMMON_INCLUDES_
#define controller_autogen_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* controller_autogen_COMMON_INCLUDES_ */

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#define controller_autogen_M           (rtM)

/* Forward declaration for rtModel */
typedef struct tag_RTM RT_MODEL;

#ifndef DEFINED_TYPEDEF_FOR_MI_STATUSES_
#define DEFINED_TYPEDEF_FOR_MI_STATUSES_

typedef enum {
  UNKNOWN = 0,                         /* Default value */
  STS_INIT,
  STARTUP,
  READY,
  RUNNING,
  SHUTDOWN,
  MI_STS_ERROR,
  OFF
} MI_STATUSES;

#endif

#ifndef DEFINED_TYPEDEF_FOR_BM_STATUSES_
#define DEFINED_TYPEDEF_FOR_BM_STATUSES_

typedef enum {
  BM_UNKNOWN = 0,                      /* Default value */
  BM_INIT,
  BM_IDLE,
  BM_STARTUP,
  INIT_PRECHARGE,
  PRECHARGE,
  BM_RUNNING,
  ERR_PRECHARGE_CLOSED,
  ERR_STARTUP,
  ERR_INIT_PRECHARGE,
  ERR_PRECHARGE,
  ERR_RUNNING,
  ERR_ALL_CLOSED,
  ERR_HV_POSITIVE
} BM_STATUSES;

#endif

#ifndef DEFINED_TYPEDEF_FOR_DI_CMD_
#define DEFINED_TYPEDEF_FOR_DI_CMD_

typedef enum {
  DI_CMD_INIT = 0,                     /* Default value */
  READY_TO_DRIVE,
  SYSTEM_ERROR,
  DI_ERR_RESET
} DI_CMD;

#endif

#ifndef DEFINED_TYPEDEF_FOR_DI_STATUSES_
#define DEFINED_TYPEDEF_FOR_DI_STATUSES_

typedef enum {
  DI_UNKNOWN = 0,                      /* Default value */
  DI_STS_INIT,
  DI_IDLE,
  DI_STARTUP,
  WAITING_FOR_DRVR,
  DRV_START_REQ,
  DI_RUNNING,
  DI_ERROR
} DI_STATUSES;

#endif

#ifndef DEFINED_TYPEDEF_FOR_MI_CMD_
#define DEFINED_TYPEDEF_FOR_MI_CMD_

typedef enum {
  CMD_INIT = 0,                        /* Default value */
  IDLE,
  CMD_STARTUP,
  CMD_SHUTDOWN,
  ERR_RESET
} MI_CMD;

#endif

#ifndef DEFINED_TYPEDEF_FOR_GOV_STATUSES_
#define DEFINED_TYPEDEF_FOR_GOV_STATUSES_

typedef enum {
  GOV_INIT = 0,                        /* Default value */
  GOV_STARTUP,
  GOV_RUNNING,
  HV_STARTUP_ERROR,
  MOTOR_STARTUP_ERROR,
  GOV_DI_ERROR,
  HV_RUN_ERROR,
  MOTOR_RUN_ERROR
} GOV_STATUSES;

#endif

/* Block signals and states (default storage) for system '<S4>/LEFT_MOTOR' */
typedef struct {
  uint8_T is_c1_motor_interface_lib;   /* '<S4>/LEFT_MOTOR' */
  uint8_T is_AMK_startup;              /* '<S4>/LEFT_MOTOR' */
  uint8_T is_AMK_errorReset;           /* '<S4>/LEFT_MOTOR' */
  uint8_T is_active_c1_motor_interface_li;/* '<S4>/LEFT_MOTOR' */
  uint8_T temporalCounter_i1;          /* '<S4>/LEFT_MOTOR' */
} DW_LEFT_MOTOR;

/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  DW_LEFT_MOTOR sf_RIGHT_MOTOR;        /* '<S4>/RIGHT_MOTOR' */
  DW_LEFT_MOTOR sf_LEFT_MOTOR;         /* '<S4>/LEFT_MOTOR' */
  real32_T TappedDelay_X[10];          /* '<S29>/Tapped Delay' */
  real32_T pedaltotorque;              /* '<S5>/%pedal to %torque' */
  real32_T Delay3_DSTATE;              /* '<S2>/Delay3' */
  MI_STATUSES MI_motorStatus;          /* '<S4>/RIGHT_MOTOR' */
  MI_STATUSES MI_motorStatus_b;        /* '<S4>/LEFT_MOTOR' */
  MI_STATUSES Delay1_DSTATE;           /* '<S3>/Delay1' */
  MI_STATUSES Delay2_DSTATE;           /* '<S2>/Delay2' */
  MI_STATUSES Delay_DSTATE;            /* '<S4>/Delay' */
  BM_STATUSES Delay_DSTATE_f;          /* '<S3>/Delay' */
  DI_CMD GOV_e_diCmd;                  /* '<S3>/Chart' */
  DI_STATUSES GOV_e_diSts;             /* '<S2>/Chart1' */
  DI_STATUSES Delay2_DSTATE_a;         /* '<S3>/Delay2' */
  MI_CMD GOV_e_miCmd;                  /* '<S3>/Chart' */
  GOV_STATUSES GOV_e_govSts;           /* '<S3>/Chart' */
  uint16_T motorStartCount;            /* '<S3>/Chart' */
  uint16_T temporalCounter_i1;         /* '<S2>/Chart1' */
  uint16_T temporalCounter_i1_a;       /* '<S1>/Chart1' */
  uint16_T temporalCounter_i1_m;       /* '<S1>/Chart' */
  uint8_T GOV_e_bmCmd;                 /* '<S3>/Chart' */
  uint8_T is_c3_simp_vd_lib;           /* '<S5>/Chart' */
  uint8_T is_active_c3_simp_vd_lib;    /* '<S5>/Chart' */
  uint8_T is_c1_governor_lib;          /* '<S3>/Chart' */
  uint8_T is_STARTUP;                  /* '<S3>/Chart' */
  uint8_T is_STARTUP_ERROR;            /* '<S3>/Chart' */
  uint8_T is_RUNNING_ERROR;            /* '<S3>/Chart' */
  uint8_T is_active_c1_governor_lib;   /* '<S3>/Chart' */
  uint8_T is_c1_driver_interface_lib;  /* '<S2>/Chart1' */
  uint8_T is_DI_running;               /* '<S2>/Chart1' */
  uint8_T is_Ready_to_drive;           /* '<S2>/Chart1' */
  uint8_T is_active_c1_driver_interface_l;/* '<S2>/Chart1' */
  uint8_T is_c5_battery_monitor_lib;   /* '<S1>/Chart1' */
  uint8_T is_active_c5_battery_monitor_li;/* '<S1>/Chart1' */
  uint8_T is_c4_battery_monitor_lib;   /* '<S1>/Chart' */
  uint8_T is_active_c4_battery_monitor_li;/* '<S1>/Chart' */
  boolean_T b_ReadyToDrive;            /* '<S2>/Chart1' */
} DW;

/* Invariant block signals (default storage) */
typedef struct {
  const real32_T CastToSingle;         /* '<S4>/Cast To Single' */
  const real32_T CastToSingle2;        /* '<S4>/Cast To Single2' */
  const real32_T CastToSingle3;        /* '<S4>/Cast To Single3' */
  const real32_T CastToSingle5;        /* '<S4>/Cast To Single5' */
} ConstB;

/* Constant parameters (default storage) */
typedef struct {
  /* Computed Parameter: AccelPedalPos1LUT_tableData
   * Referenced by: '<S2>/AccelPedalPos1 LUT'
   */
  real32_T AccelPedalPos1LUT_tableData[100];

  /* Computed Parameter: AccelPedalPos1LUT_bp01Data
   * Referenced by: '<S2>/AccelPedalPos1 LUT'
   */
  real32_T AccelPedalPos1LUT_bp01Data[100];

  /* Computed Parameter: pedaltotorque_tableData
   * Referenced by: '<S5>/%pedal to %torque'
   */
  real32_T pedaltotorque_tableData[101];

  /* Computed Parameter: pedaltotorque_bp01Data
   * Referenced by: '<S5>/%pedal to %torque'
   */
  real32_T pedaltotorque_bp01Data[101];
} ConstP;

/* External inputs (root inport signals with default storage) */
typedef struct {
  boolean_T AMK_bReserve_R;            /* '<Root>/AMK_bReserve_R' */
  real_T DI_V_SteeringAngle;           /* '<Root>/DI_V_SteeringAngle' */
  real_T DI_V_BrakePedalPos;           /* '<Root>/DI_V_BrakePedalPos' */
  boolean_T DI_b_DriverButton;         /* '<Root>/DI_b_DriverButton' */
  real_T DI_V_AccelPedalPos1;          /* '<Root>/DI_V_AccelPedalPos1' */
  real_T DI_V_AccelPedalPos2;          /* '<Root>/DI_V_AccelPedalPos2' */
  boolean_T BM_b_prechrgContactorSts;  /* '<Root>/BM_b_prechrgContactorSts' */
  boolean_T BM_b_HVposContactorSts;    /* '<Root>/BM_b_HVposContactorSts' */
  boolean_T BM_b_HVnegContactorSts;    /* '<Root>/BM_b_HVnegContactorSts' */
  boolean_T AMK_bSystemReady_R;        /* '<Root>/AMK_bSystemReady_R' */
  boolean_T AMK_bError_R;              /* '<Root>/AMK_bError_R' */
  boolean_T AMK_bWarn_R;               /* '<Root>/AMK_bWarn_R' */
  boolean_T AMK_bQuitDcOn_R;           /* '<Root>/AMK_bQuitDcOn_R' */
  boolean_T AMK_bDcOn_R;               /* '<Root>/AMK_bDcOn_R' */
  boolean_T AMK_bQuitInverterOn_R;     /* '<Root>/AMK_bQuitInverterOn_R' */
  boolean_T AMK_bInverterOn_R;         /* '<Root>/AMK_bInverterOn_R' */
  boolean_T AMK_bDerating_R;           /* '<Root>/AMK_bDerating_R' */
  int16_T AMK_TempMotor_R;             /* '<Root>/AMK_TempMotor_R' */
  int16_T AMK_TempInverter_R;          /* '<Root>/AMK_TempInverter_R' */
  uint16_T AMK_ErrorInfo_R;            /* '<Root>/AMK_ErrorInfo_R' */
  int16_T AMK_TempIGBT_R;              /* '<Root>/AMK_TempIGBT_R' */
  int16_T AMK_ActualVelocity_R;        /* '<Root>/AMK_ActualVelocity_R' */
  int16_T AMK_TorqueCurrent_R;         /* '<Root>/AMK_TorqueCurrent_R' */
  int16_T AMK_MagnetizingCurrent_R;    /* '<Root>/AMK_MagnetizingCurrent_R' */
  boolean_T AMK_bReserve_L;            /* '<Root>/AMK_bReserve_L' */
  boolean_T AMK_bSystemReady_L;        /* '<Root>/AMK_bSystemReady_L' */
  boolean_T AMK_bError_L;              /* '<Root>/AMK_bError_L' */
  boolean_T AMK_bWarn_L;               /* '<Root>/AMK_bWarn_L' */
  boolean_T AMK_bQuitDcOn_L;           /* '<Root>/AMK_bQuitDcOn_L' */
  boolean_T AMK_bDcOn_L;               /* '<Root>/AMK_bDcOn_L' */
  boolean_T AMK_bQuitInverterOn_L;     /* '<Root>/AMK_bQuitInverterOn_L' */
  boolean_T AMK_bInverterOn_L;         /* '<Root>/AMK_bInverterOn_L' */
  boolean_T AMK_bDerating_L;           /* '<Root>/AMK_bDerating_L' */
  int16_T AMK_TempMotor_L;             /* '<Root>/AMK_TempMotor_L' */
  int16_T AMK_TempInverter_L;          /* '<Root>/AMK_TempInverter_L' */
  uint16_T AMK_ErrorInfo_L;            /* '<Root>/AMK_ErrorInfo_L' */
  int16_T AMK_TempIGBT_L;              /* '<Root>/AMK_TempIGBT_L' */
  int16_T AMK_ActualVelocity_L;        /* '<Root>/AMK_ActualVelocity_L' */
  int16_T AMK_TorqueCurrent_L;         /* '<Root>/AMK_TorqueCurrent_L' */
  int16_T AMK_MagnetizingCurrent_L;    /* '<Root>/AMK_MagnetizingCurrent_L' */
} ExtU;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  uint8_T AMK_bInverterOn_tx_R;        /* '<Root>/AMK_bInverterOn_tx_R' */
  uint8_T AMK_bDcOn_tx_R;              /* '<Root>/AMK_bDcOn_tx_R' */
  uint8_T AMK_bEnable_R;               /* '<Root>/AMK_bEnable_R' */
  uint8_T AMK_bErrorReset_R;           /* '<Root>/AMK_bErrorReset_R' */
  real32_T AMK_TargetVelocity_R;       /* '<Root>/AMK_TargetVelocity_R' */
  real32_T AMK_TorqueLimitPositiv_R;   /* '<Root>/AMK_TorqueLimitPositiv_R' */
  real32_T AMK_TorqueLimitNegativ_R;   /* '<Root>/AMK_TorqueLimitNegativ_R' */
  uint8_T AMK_bInverterOn_tx_L;        /* '<Root>/AMK_bInverterOn_tx_L' */
  uint8_T AMK_bDcOn_tx_L;              /* '<Root>/AMK_bDcOn_tx_L' */
  uint8_T AMK_bEnable_L;               /* '<Root>/AMK_bEnable_L' */
  uint8_T AMK_bErrorReset_L;           /* '<Root>/AMK_bErrorReset_L' */
  real32_T AMK_TargetVelocity_L;       /* '<Root>/AMK_TargetVelocity_L' */
  real32_T AMK_TorqueLimitPositiv_L;   /* '<Root>/AMK_TorqueLimitPositiv_L' */
  real32_T AMK_TorqueLimitNegativ_L;   /* '<Root>/AMK_TorqueLimitNegativ_L' */
  real_T BM_b_prechargeContactorCMD;   /* '<Root>/BM_b_prechargeContactorCMD' */
  real_T BM_b_HVposContactorCMD;       /* '<Root>/BM_b_HVposContactorCMD' */
  real_T BM_b_HVnegContactorCMD;       /* '<Root>/BM_b_HVnegContactorCMD' */
  boolean_T DI_b_driverSpeaker;        /* '<Root>/DI_b_driverSpeaker' */
  real32_T DI_b_brakeLightEn;          /* '<Root>/DI_b_brakeLightEn' */
  real_T DI_p_PWMstatusLightCycle;     /* '<Root>/DI_p_PWMstatusLightCycle' */
  real_T DI_p_PWMstatusLightFreq;      /* '<Root>/DI_p_PWMstatusLightFreq' */
  real32_T GOV_e_govSts;               /* '<Root>/GOV_e_govSts' */
} ExtY;

/* Real-time Model Data Structure */
struct tag_RTM {
  const char_T * volatile errorStatus;
};

/* Block signals and states (default storage) */
extern DW rtDW;

/* External inputs (root inport signals with default storage) */
extern ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY rtY;
extern const ConstB rtConstB;          /* constant block i/o */

/* Constant parameters (default storage) */
extern const ConstP rtConstP;

/* Model entry point functions */
extern void controller_autogen_initialize(void);
extern void controller_autogen_step(void);

/* Real-time Model object */
extern RT_MODEL *const rtM;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S2>/Constant2' : Unused code path elimination
 * Block '<S2>/Switch1' : Unused code path elimination
 * Block '<S17>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S19>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S21>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S16>/Data Type Conversion' : Unused code path elimination
 * Block '<S16>/Data Type Conversion1' : Unused code path elimination
 * Block '<S16>/Data Type Conversion2' : Unused code path elimination
 * Block '<S23>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S24>/Divide' : Unused code path elimination
 * Block '<S24>/Gain' : Unused code path elimination
 * Block '<S24>/Subtract' : Unused code path elimination
 * Block '<S24>/Subtract1' : Unused code path elimination
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'controller_autogen'
 * '<S1>'   : 'controller_autogen/battery_monitor'
 * '<S2>'   : 'controller_autogen/driver_interface'
 * '<S3>'   : 'controller_autogen/governor'
 * '<S4>'   : 'controller_autogen/motor_interface'
 * '<S5>'   : 'controller_autogen/simp_vd_interface'
 * '<S6>'   : 'controller_autogen/battery_monitor/Chart'
 * '<S7>'   : 'controller_autogen/battery_monitor/Chart1'
 * '<S8>'   : 'controller_autogen/driver_interface/Accelerator plausibility check'
 * '<S9>'   : 'controller_autogen/driver_interface/Chart1'
 * '<S10>'  : 'controller_autogen/driver_interface/If Action Subsystem'
 * '<S11>'  : 'controller_autogen/driver_interface/If Action Subsystem1'
 * '<S12>'  : 'controller_autogen/driver_interface/If Action Subsystem2'
 * '<S13>'  : 'controller_autogen/driver_interface/potentiometerProcessing'
 * '<S14>'  : 'controller_autogen/driver_interface/potentiometerProcessing1'
 * '<S15>'  : 'controller_autogen/driver_interface/potentiometerProcessing2'
 * '<S16>'  : 'controller_autogen/driver_interface/potentiometerProcessing3'
 * '<S17>'  : 'controller_autogen/driver_interface/potentiometerProcessing/Interval Test Dynamic'
 * '<S18>'  : 'controller_autogen/driver_interface/potentiometerProcessing/convertToPercent'
 * '<S19>'  : 'controller_autogen/driver_interface/potentiometerProcessing1/Interval Test Dynamic'
 * '<S20>'  : 'controller_autogen/driver_interface/potentiometerProcessing1/convertToPercent'
 * '<S21>'  : 'controller_autogen/driver_interface/potentiometerProcessing2/Interval Test Dynamic'
 * '<S22>'  : 'controller_autogen/driver_interface/potentiometerProcessing2/convertToPercent'
 * '<S23>'  : 'controller_autogen/driver_interface/potentiometerProcessing3/Interval Test Dynamic'
 * '<S24>'  : 'controller_autogen/driver_interface/potentiometerProcessing3/convertToPercent'
 * '<S25>'  : 'controller_autogen/governor/Chart'
 * '<S26>'  : 'controller_autogen/motor_interface/LEFT_MOTOR'
 * '<S27>'  : 'controller_autogen/motor_interface/RIGHT_MOTOR'
 * '<S28>'  : 'controller_autogen/simp_vd_interface/Chart'
 * '<S29>'  : 'controller_autogen/simp_vd_interface/Running Avg'
 */
#endif                                 /* RTW_HEADER_controller_autogen_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
