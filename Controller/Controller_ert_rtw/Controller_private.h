//
// File: Controller_private.h
//
// Code generated for Simulink model 'Controller'.
//
// Model version                  : 1.87
// Simulink Coder version         : 9.2 (R2019b) 18-Jul-2019
// C/C++ source code generated on : Fri Jun 19 17:34:39 2026
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Windows64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_Controller_private_h_
#define RTW_HEADER_Controller_private_h_
#include "rtwtypes.h"

extern uint32_T plook_u32ff_lincpa(real32_T u, const real32_T bp[], uint32_T
  maxIndex, real32_T *fraction, uint32_T *prevIndex);
extern uint32_T plook_u32u8f_lincpa(uint8_T u, const uint8_T bp[], uint32_T
  maxIndex, real32_T *fraction, uint32_T *prevIndex);
extern real32_T intrp4d_fu32fla_pw(const uint32_T bpIndex[], const real32_T
  frac[], const real32_T table[], const uint32_T stride[], const uint32_T
  maxIndex[]);
extern uint32_T linsearch_u32f(real32_T u, const real32_T bp[], uint32_T
  startIndex);
extern uint32_T linsearch_u32u8(uint8_T u, const uint8_T bp[], uint32_T
  startIndex);

#endif                                 // RTW_HEADER_Controller_private_h_

//
// File trailer for generated code.
//
// [EOF]
//
