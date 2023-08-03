/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: controller_autogen_data.c
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

/* Invariant block signals (default storage) */
const ConstB rtConstB = {
  1000.0F,                             /* '<S4>/Cast To Single' */
  0.0F,                                /* '<S4>/Cast To Single2' */
  1000.0F,                             /* '<S4>/Cast To Single3' */
  0.0F                                 /* '<S4>/Cast To Single5' */
};

/* Constant parameters (default storage) */
const ConstP rtConstP = {
  /* Computed Parameter: AccelPedalPos1LUT_tableData
   * Referenced by: '<S2>/AccelPedalPos1 LUT'
   */
  { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.85185182F,
    3.70370364F, 5.55555534F, 7.40740728F, 9.25925922F, 11.1111107F, 12.9629631F,
    14.8148146F, 16.666666F, 18.5185184F, 20.3703709F, 22.2222214F, 24.0740738F,
    25.9259262F, 27.7777786F, 29.6296291F, 31.4814816F, 33.3333321F, 35.1851845F,
    37.0370369F, 38.8888893F, 40.7407417F, 42.5925941F, 44.4444427F, 46.2962952F,
    48.1481476F, 50.0F, 51.8518524F, 53.7037048F, 55.5555573F, 57.4074059F,
    59.2592583F, 61.1111107F, 62.9629631F, 64.8148117F, 66.6666641F, 68.5185165F,
    70.370369F, 72.2222214F, 74.0740738F, 75.9259262F, 77.7777786F, 79.629631F,
    81.4814835F, 83.3333359F, 85.1851883F, 87.0370407F, 88.8888855F, 90.7407379F,
    92.5925903F, 94.4444427F, 96.2962952F, 98.1481476F, 100.0F, 100.0F, 100.0F,
    100.0F, 100.0F, 100.0F, 100.0F, 100.0F, 100.0F, 100.0F, 100.0F, 100.0F,
    100.0F, 100.0F, 100.0F, 100.0F, 100.0F, 100.0F, 100.0F, 100.0F, 100.0F,
    100.0F, 100.0F, 100.0F, 100.0F },

  /* Computed Parameter: AccelPedalPos1LUT_bp01Data
   * Referenced by: '<S2>/AccelPedalPos1 LUT'
   */
  { 0.0F, 1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F, 7.0F, 8.0F, 9.0F, 10.0F, 11.0F,
    12.0F, 13.0F, 14.0F, 15.0F, 16.0F, 17.0F, 18.0F, 19.0F, 20.0F, 21.0F, 22.0F,
    23.0F, 24.0F, 25.0F, 26.0F, 27.0F, 28.0F, 29.0F, 30.0F, 31.0F, 32.0F, 33.0F,
    34.0F, 35.0F, 36.0F, 37.0F, 38.0F, 39.0F, 40.0F, 41.0F, 42.0F, 43.0F, 44.0F,
    45.0F, 46.0F, 47.0F, 48.0F, 49.0F, 50.0F, 51.0F, 52.0F, 53.0F, 54.0F, 55.0F,
    56.0F, 57.0F, 58.0F, 59.0F, 60.0F, 61.0F, 62.0F, 63.0F, 64.0F, 65.0F, 66.0F,
    67.0F, 68.0F, 69.0F, 70.0F, 71.0F, 72.0F, 73.0F, 74.0F, 75.0F, 76.0F, 77.0F,
    78.0F, 79.0F, 80.0F, 81.0F, 82.0F, 83.0F, 84.0F, 85.0F, 86.0F, 87.0F, 88.0F,
    89.0F, 90.0F, 91.0F, 92.0F, 93.0F, 94.0F, 95.0F, 96.0F, 97.0F, 98.0F, 99.0F
  },

  /* Computed Parameter: pedaltotorque_tableData
   * Referenced by: '<S5>/%pedal to %torque'
   */
  { 0.0F, 10.0F, 20.0F, 30.0F, 40.0F, 50.0F, 60.0F, 70.0F, 80.0F, 90.0F, 100.0F,
    110.0F, 120.0F, 130.0F, 140.0F, 150.0F, 160.0F, 170.0F, 180.0F, 190.0F,
    200.0F, 210.0F, 220.0F, 230.0F, 240.0F, 250.0F, 260.0F, 270.0F, 280.0F,
    290.0F, 300.0F, 310.0F, 320.0F, 330.0F, 340.0F, 350.0F, 360.0F, 370.0F,
    380.0F, 390.0F, 400.0F, 410.0F, 420.0F, 430.0F, 440.0F, 450.0F, 460.0F,
    470.0F, 480.0F, 490.0F, 500.0F, 510.0F, 520.0F, 530.0F, 540.0F, 550.0F,
    560.0F, 570.0F, 580.0F, 590.0F, 600.0F, 610.0F, 620.0F, 630.0F, 640.0F,
    650.0F, 660.0F, 670.0F, 680.0F, 690.0F, 700.0F, 710.0F, 720.0F, 730.0F,
    740.0F, 750.0F, 760.0F, 770.0F, 780.0F, 790.0F, 800.0F, 810.0F, 820.0F,
    830.0F, 840.0F, 850.0F, 860.0F, 870.0F, 880.0F, 890.0F, 900.0F, 910.0F,
    920.0F, 930.0F, 940.0F, 950.0F, 960.0F, 970.0F, 980.0F, 990.0F, 1000.0F },

  /* Computed Parameter: pedaltotorque_bp01Data
   * Referenced by: '<S5>/%pedal to %torque'
   */
  { 0.0F, 1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F, 7.0F, 8.0F, 9.0F, 10.0F, 11.0F,
    12.0F, 13.0F, 14.0F, 15.0F, 16.0F, 17.0F, 18.0F, 19.0F, 20.0F, 21.0F, 22.0F,
    23.0F, 24.0F, 25.0F, 26.0F, 27.0F, 28.0F, 29.0F, 30.0F, 31.0F, 32.0F, 33.0F,
    34.0F, 35.0F, 36.0F, 37.0F, 38.0F, 39.0F, 40.0F, 41.0F, 42.0F, 43.0F, 44.0F,
    45.0F, 46.0F, 47.0F, 48.0F, 49.0F, 50.0F, 51.0F, 52.0F, 53.0F, 54.0F, 55.0F,
    56.0F, 57.0F, 58.0F, 59.0F, 60.0F, 61.0F, 62.0F, 63.0F, 64.0F, 65.0F, 66.0F,
    67.0F, 68.0F, 69.0F, 70.0F, 71.0F, 72.0F, 73.0F, 74.0F, 75.0F, 76.0F, 77.0F,
    78.0F, 79.0F, 80.0F, 81.0F, 82.0F, 83.0F, 84.0F, 85.0F, 86.0F, 87.0F, 88.0F,
    89.0F, 90.0F, 91.0F, 92.0F, 93.0F, 94.0F, 95.0F, 96.0F, 97.0F, 98.0F, 99.0F,
    100.0F }
};

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
