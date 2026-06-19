//
// File: Controller.cpp
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
#include "Controller.h"
#include "Controller_private.h"

// Named constants for Chart: '<Root>/Chart'
const uint8_T Controller_IN_CONTROL_INIT = 1U;
const uint8_T Controller_IN_CONTROL_OFF = 2U;
const uint8_T Controller_IN_CONTROL_ON = 3U;
const uint8_T Controller_IN_CONTROL_WAIT = 4U;
const uint8_T Controller_IN_MODE_CURVE = 1U;
const uint8_T Controller_IN_MODE_ENTRY = 2U;
const uint8_T Controller_IN_MODE_KEEP = 3U;
const uint8_T Controller_IN_NO_ACTIVE_CHILD = 0U;
const Controler_Bus Controller_rtZControler_Bus = {
  false,                               // Flag_Controlador_Ready
  false,                               // Flag_Delta_Saturation
  false,                               // Flag_Enable_Servo
  CONTROLLER_STATE_NONE,               // Operation_Mode
  0.0F,                                // Angular_Speed_Target
  false,                               // Toggle_Disengage
  false,                               // Pulse_Disengage
  false,                               // Pulse_Enable_Servo_Changed
  false                                // Flag_Enable_Control
} ;                                    // Controler_Bus ground

uint32_T plook_u32ff_lincpa(real32_T u, const real32_T bp[], uint32_T maxIndex,
  real32_T *fraction, uint32_T *prevIndex)
{
  uint32_T bpIndex;

  // Prelookup - Index and Fraction
  // Index Search method: 'linear'
  // Extrapolation method: 'Clip'
  // Use previous index: 'on'
  // Use last breakpoint for index at or above upper limit: 'on'
  // Remove protection against out-of-range input in generated code: 'off'

  if (u <= bp[0U]) {
    bpIndex = 0U;
    *fraction = 0.0F;
  } else if (u < bp[maxIndex]) {
    bpIndex = linsearch_u32f(u, bp, *prevIndex);
    *fraction = (u - bp[bpIndex]) / (bp[bpIndex + 1U] - bp[bpIndex]);
  } else {
    bpIndex = maxIndex;
    *fraction = 0.0F;
  }

  *prevIndex = bpIndex;
  return bpIndex;
}

uint32_T plook_u32u8f_lincpa(uint8_T u, const uint8_T bp[], uint32_T maxIndex,
  real32_T *fraction, uint32_T *prevIndex)
{
  uint32_T bpIndex;

  // Prelookup - Index and Fraction
  // Index Search method: 'linear'
  // Extrapolation method: 'Clip'
  // Use previous index: 'on'
  // Use last breakpoint for index at or above upper limit: 'on'
  // Remove protection against out-of-range input in generated code: 'off'

  if (u <= bp[0U]) {
    bpIndex = 0U;
    *fraction = 0.0F;
  } else if (u < bp[maxIndex]) {
    bpIndex = linsearch_u32u8(u, bp, *prevIndex);
    *fraction = static_cast<real32_T>(static_cast<uint8_T>((static_cast<uint32_T>
      (u) - bp[bpIndex]))) / static_cast<real32_T>(static_cast<uint8_T>((
      static_cast<uint32_T>(bp[bpIndex + 1U]) - bp[bpIndex])));
  } else {
    bpIndex = maxIndex;
    *fraction = 0.0F;
  }

  *prevIndex = bpIndex;
  return bpIndex;
}

real32_T intrp4d_fu32fla_pw(const uint32_T bpIndex[], const real32_T frac[],
  const real32_T table[], const uint32_T stride[], const uint32_T maxIndex[])
{
  real32_T y;
  uint32_T offset_3d;
  real32_T yR_1d;
  uint32_T offset_0d;
  real32_T yL_1d;
  uint32_T offset_1d;
  real32_T yL_1d_0;

  // Column-major Interpolation 4-D
  // Interpolation method: 'Linear point-slope'
  // Use last breakpoint for index at or above upper limit: 'on'
  // Overflow mode: 'portable wrapping'

  offset_3d = ((bpIndex[3U] * stride[3U] + bpIndex[2U] * stride[2U]) + bpIndex
               [1U] * stride[1U]) + bpIndex[0U];
  if (bpIndex[0U] == maxIndex[0U]) {
    y = table[offset_3d];
  } else {
    y = (table[offset_3d + 1U] - table[offset_3d]) * frac[0U] + table[offset_3d];
  }

  if (bpIndex[1U] == maxIndex[1U]) {
  } else {
    offset_0d = offset_3d + stride[1U];
    if (bpIndex[0U] == maxIndex[0U]) {
      yR_1d = table[offset_0d];
    } else {
      yR_1d = (table[offset_0d + 1U] - table[offset_0d]) * frac[0U] +
        table[offset_0d];
    }

    y += (yR_1d - y) * frac[1U];
  }

  if (bpIndex[2U] == maxIndex[2U]) {
  } else {
    offset_1d = offset_3d + stride[2U];
    if (bpIndex[0U] == maxIndex[0U]) {
      yL_1d = table[offset_1d];
    } else {
      yL_1d = (table[offset_1d + 1U] - table[offset_1d]) * frac[0U] +
        table[offset_1d];
    }

    if (bpIndex[1U] == maxIndex[1U]) {
    } else {
      offset_0d = offset_1d + stride[1U];
      if (bpIndex[0U] == maxIndex[0U]) {
        yR_1d = table[offset_0d];
      } else {
        yR_1d = (table[offset_0d + 1U] - table[offset_0d]) * frac[0U] +
          table[offset_0d];
      }

      yL_1d += (yR_1d - yL_1d) * frac[1U];
    }

    y += (yL_1d - y) * frac[2U];
  }

  if (bpIndex[3U] == maxIndex[3U]) {
  } else {
    offset_1d = offset_3d + stride[3U];
    if (bpIndex[0U] == maxIndex[0U]) {
      yL_1d = table[offset_1d];
    } else {
      yL_1d = (table[offset_1d + 1U] - table[offset_1d]) * frac[0U] +
        table[offset_1d];
    }

    if (bpIndex[1U] == maxIndex[1U]) {
    } else {
      offset_0d = offset_1d + stride[1U];
      if (bpIndex[0U] == maxIndex[0U]) {
        yR_1d = table[offset_0d];
      } else {
        yR_1d = (table[offset_0d + 1U] - table[offset_0d]) * frac[0U] +
          table[offset_0d];
      }

      yL_1d += (yR_1d - yL_1d) * frac[1U];
    }

    if (bpIndex[2U] == maxIndex[2U]) {
    } else {
      offset_1d += stride[2U];
      if (bpIndex[0U] == maxIndex[0U]) {
        yL_1d_0 = table[offset_1d];
      } else {
        yL_1d_0 = (table[offset_1d + 1U] - table[offset_1d]) * frac[0U] +
          table[offset_1d];
      }

      if (bpIndex[1U] == maxIndex[1U]) {
      } else {
        offset_0d = offset_1d + stride[1U];
        if (bpIndex[0U] == maxIndex[0U]) {
          yR_1d = table[offset_0d];
        } else {
          yR_1d = (table[offset_0d + 1U] - table[offset_0d]) * frac[0U] +
            table[offset_0d];
        }

        yL_1d_0 += (yR_1d - yL_1d_0) * frac[1U];
      }

      yL_1d += (yL_1d_0 - yL_1d) * frac[2U];
    }

    y += (yL_1d - y) * frac[3U];
  }

  return y;
}

uint32_T linsearch_u32f(real32_T u, const real32_T bp[], uint32_T startIndex)
{
  uint32_T bpIndex;

  // Linear Search
  for (bpIndex = startIndex; u < bp[bpIndex]; bpIndex--) {
  }

  while (u >= bp[bpIndex + 1U]) {
    bpIndex++;
  }

  return bpIndex;
}

uint32_T linsearch_u32u8(uint8_T u, const uint8_T bp[], uint32_T startIndex)
{
  uint32_T bpIndex;

  // Linear Search
  for (bpIndex = startIndex; u < bp[bpIndex]; bpIndex--) {
  }

  while (u >= bp[bpIndex + 1U]) {
    bpIndex++;
  }

  return bpIndex;
}

// Function for MATLAB Function: '<S9>/MATLAB Function1'
real32_T ControladorModelClass::Controller_mod(real32_T x)
{
  real32_T r;
  boolean_T rEQ0;
  real32_T q;
  if (x == 0.0F) {
    r = 0.0F;
  } else {
    r = std::fmod(x, 6.28318548F);
    rEQ0 = (r == 0.0F);
    if (!rEQ0) {
      q = std::abs(x / 6.28318548F);
      rEQ0 = (std::abs(q - std::floor(q + 0.5F)) <= 1.1920929E-7F * q);
    }

    if (rEQ0) {
      r = 0.0F;
    } else {
      if (x < 0.0F) {
        r += 6.28318548F;
      }
    }
  }

  return r;
}

//
// Output and update for atomic system:
//    '<S9>/MATLAB Function1'
//    '<S12>/MATLAB Function1'
//
void ControladorModelClass::Controller_MATLABFunction1(real32_T rtu_ref,
  real32_T rtu_meas, real32_T *rty_ang_error)
{
  real32_T ref;
  real32_T meas;
  real32_T tmp;
  ref = Controller_mod(rtu_ref);
  meas = Controller_mod(rtu_meas);
  if (meas > ref) {
    tmp = ref - meas;
    ref += 6.28318548F - meas;
    if (std::abs(ref) < std::abs(tmp)) {
      *rty_ang_error = ref;
    } else {
      *rty_ang_error = tmp;
    }
  } else if (meas < ref) {
    if (std::abs(meas - ref) < std::abs((6.28318548F - ref) + meas)) {
      *rty_ang_error = ref - meas;
    } else {
      *rty_ang_error = -((meas + 6.28318548F) - ref);
    }
  } else {
    *rty_ang_error = 0.0F;
  }
}

// Function for MATLAB Function: '<S13>/MATLAB Function'
real32_T ControladorModelClass::Controller_mod_d(real32_T x)
{
  real32_T r;
  boolean_T rEQ0;
  real32_T q;
  if (x == 0.0F) {
    r = 0.0F;
  } else {
    r = std::fmod(x, 6.28318548F);
    rEQ0 = (r == 0.0F);
    if (!rEQ0) {
      q = std::abs(x / 6.28318548F);
      rEQ0 = (std::abs(q - std::floor(q + 0.5F)) <= 1.1920929E-7F * q);
    }

    if (rEQ0) {
      r = 0.0F;
    } else {
      if (x < 0.0F) {
        r += 6.28318548F;
      }
    }
  }

  return r;
}

// Model step function
void ControladorModelClass::step()
{
  real32_T psi_ref_;
  uint32_T bpIndices[4];
  real32_T fractions[4];
  uint32_T bpIndices_0[4];
  real32_T fractions_0[4];
  ControllerState_t rtb_controllerState;
  real32_T rtb_Switch1;
  boolean_T Modo_Curva_prev;
  real32_T rtb_uDLookupTable1_idx_0;
  real32_T rtb_uDLookupTable1_idx_1;
  real32_T rtb_uDLookupTable_idx_2;
  real_T rtb_Switch1_0;
  real32_T tmp;
  boolean_T guard1 = false;

  // Chart: '<Root>/Chart' incorporates:
  //   Constant: '<Root>/Constant1'
  //   Inport: '<Root>/Enable'
  //   Inport: '<Root>/vehicleMode'
  //   Lookup_n-D: '<S12>/1-D Lookup Table1'

  if (Controller_DW.temporalCounter_i1 < 3U) {
    Controller_DW.temporalCounter_i1 = static_cast<uint8_T>
      ((Controller_DW.temporalCounter_i1 + 1U));
  }

  Modo_Curva_prev = Controller_DW.Modo_Curva_start;
  Controller_DW.Modo_Curva_start = Controller_P.userParameters.isCurveMode;
  if (Controller_DW.is_active_c1_Controller == 0U) {
    Controller_DW.is_active_c1_Controller = 1U;
    Controller_DW.is_c1_Controller = Controller_IN_CONTROL_OFF;
    Controller_B.enable_servo = false;
    Controller_B.enable_control = false;
    Controller_B.ready = false;
    rtb_controllerState = CONTROLLER_STATE_IDLE;
  } else {
    guard1 = false;
    switch (Controller_DW.is_c1_Controller) {
     case Controller_IN_CONTROL_INIT:
      if (Controller_DW.temporalCounter_i1 >= 2U) {
        Controller_DW.is_c1_Controller = Controller_IN_CONTROL_WAIT;
        Controller_B.enable_control = false;
        Controller_B.ready = true;
        rtb_controllerState = CONTROLLER_STATE_WAIT;
      } else {
        Controller_B.enable_servo = true;
        rtb_controllerState = CONTROLLER_STATE_IDLE;
      }
      break;

     case Controller_IN_CONTROL_OFF:
      if ((Controller_U.vehicleMode == VEHICLE_MODE_REVERSE) ||
          (!Controller_U.Enable)) {
        Controller_DW.is_c1_Controller = Controller_IN_CONTROL_OFF;
        Controller_B.enable_servo = false;
        Controller_B.enable_control = false;
        Controller_B.ready = false;
        rtb_controllerState = CONTROLLER_STATE_IDLE;
      } else {
        Controller_DW.is_c1_Controller = Controller_IN_CONTROL_INIT;
        Controller_DW.temporalCounter_i1 = 0U;
        Controller_B.enable_servo = true;
        rtb_controllerState = CONTROLLER_STATE_IDLE;
      }
      break;

     case Controller_IN_CONTROL_ON:
      if (!Controller_U.Enable) {
        Controller_B.flag_disengage = !Controller_B.flag_disengage;
        Controller_DW.is_CONTROL_ON = Controller_IN_NO_ACTIVE_CHILD;
        Controller_DW.is_c1_Controller = Controller_IN_CONTROL_OFF;
        Controller_B.enable_servo = false;
        Controller_B.enable_control = false;
        Controller_B.ready = false;
        rtb_controllerState = CONTROLLER_STATE_IDLE;
      } else if ((Controller_U.vehicleMode != VEHICLE_MODE_FORWARD) ||
                 (Modo_Curva_prev != Controller_DW.Modo_Curva_start)) {
        Controller_DW.is_CONTROL_ON = Controller_IN_NO_ACTIVE_CHILD;
        Controller_DW.is_c1_Controller = Controller_IN_CONTROL_WAIT;
        Controller_B.enable_control = false;
        Controller_B.ready = true;
        rtb_controllerState = CONTROLLER_STATE_WAIT;
      } else {
        Controller_B.enable_control = true;
        Controller_B.ready = true;
        switch (Controller_DW.is_CONTROL_ON) {
         case Controller_IN_MODE_CURVE:
          rtb_controllerState = CONTROLLER_STATE_CURVE;
          break;

         case Controller_IN_MODE_ENTRY:
          if ((std::abs(Controller_U.Guidance.e) <= 0.15) && (std::abs
               (Controller_U.Guidance.psiError) <= 0.1)) {
            Controller_DW.is_CONTROL_ON = Controller_IN_MODE_KEEP;
            rtb_controllerState = CONTROLLER_STATE_KEEP;
          } else {
            rtb_controllerState = CONTROLLER_STATE_ENTRY;
            guard1 = true;
          }
          break;

         default:
          // case IN_MODE_KEEP:
          rtb_controllerState = CONTROLLER_STATE_KEEP;
          break;
        }
      }
      break;

     default:
      // case IN_CONTROL_WAIT:
      if ((!Controller_U.Enable) || (Controller_U.vehicleMode ==
           VEHICLE_MODE_REVERSE)) {
        Controller_B.flag_disengage = !Controller_B.flag_disengage;
        Controller_DW.is_c1_Controller = Controller_IN_CONTROL_OFF;
        Controller_B.enable_servo = false;
        Controller_B.enable_control = false;
        Controller_B.ready = false;
        rtb_controllerState = CONTROLLER_STATE_IDLE;
      } else if (Controller_U.vehicleMode == VEHICLE_MODE_STOPPED) {
        Controller_DW.is_c1_Controller = Controller_IN_CONTROL_WAIT;
        Controller_B.enable_control = false;
        Controller_B.ready = true;
        rtb_controllerState = CONTROLLER_STATE_WAIT;
      } else if (Controller_P.userParameters.isCurveMode) {
        Controller_DW.is_c1_Controller = Controller_IN_CONTROL_ON;
        Controller_B.enable_control = true;
        Controller_B.ready = true;
        Controller_DW.is_CONTROL_ON = Controller_IN_MODE_CURVE;
        rtb_controllerState = CONTROLLER_STATE_CURVE;
      } else {
        Controller_DW.is_c1_Controller = Controller_IN_CONTROL_ON;
        Controller_B.enable_control = true;
        Controller_B.ready = true;
        Controller_DW.is_CONTROL_ON = Controller_IN_MODE_ENTRY;
        rtb_controllerState = CONTROLLER_STATE_ENTRY;
        guard1 = true;
      }
      break;
    }

    if (guard1) {
      // Outputs for Enabled SubSystem: '<Root>/LQR - Entry Straight' incorporates:
      //   EnablePort: '<S12>/Enable'

      // Lookup_n-D: '<S12>/1-D Lookup Table1' incorporates:
      //   Constant: '<S12>/Constant4'
      //   Constant: '<S12>/Constant6'
      //   Inport: '<Root>/Measurements'
      //   SignalConversion generated from: '<Root>/Measurements'

      bpIndices[0U] = plook_u32ff_lincpa(Controller_U.Measurements.vx,
        Controller_ConstP.pooled5, 7U, &rtb_Switch1, &Controller_DW.m_Cache01_i);
      fractions[0U] = rtb_Switch1;
      bpIndices[1U] = plook_u32u8f_lincpa
        (Controller_P.userParameters.curveAggressiveness,
         Controller_ConstP.pooled8, 6U, &rtb_Switch1, &Controller_DW.m_Cache02_j);
      fractions[1U] = rtb_Switch1;
      bpIndices[3U] = plook_u32u8f_lincpa(Controller_P.userParameters.modelPlant,
        Controller_ConstP.pooled9, 1U, &rtb_Switch1, &Controller_DW.m_Cache04_o);
      fractions[3U] = rtb_Switch1;
      bpIndices[2U] = plook_u32u8f_lincpa
        (Controller_ConstB.DataTypeConversion6_n[0], Controller_ConstP.pooled9,
         1U, &rtb_Switch1, &Controller_DW.m_Cache03_e[0]);

      // End of Outputs for SubSystem: '<Root>/LQR - Entry Straight'
      fractions[2U] = rtb_Switch1;

      // Outputs for Enabled SubSystem: '<Root>/LQR - Entry Straight' incorporates:
      //   EnablePort: '<S12>/Enable'

      // Lookup_n-D: '<S12>/1-D Lookup Table1'
      rtb_uDLookupTable1_idx_0 = intrp4d_fu32fla_pw(bpIndices, fractions,
        Controller_ConstP.uDLookupTable1_tableData,
        Controller_ConstP.uDLookupTable1_dimSizes,
        Controller_ConstP.uDLookupTable1_maxIndex);
      bpIndices[2U] = plook_u32u8f_lincpa
        (Controller_ConstB.DataTypeConversion6_n[1], Controller_ConstP.pooled9,
         1U, &rtb_Switch1, &Controller_DW.m_Cache03_e[1]);

      // End of Outputs for SubSystem: '<Root>/LQR - Entry Straight'
      fractions[2U] = rtb_Switch1;

      // Outputs for Enabled SubSystem: '<Root>/LQR - Entry Straight' incorporates:
      //   EnablePort: '<S12>/Enable'

      // Lookup_n-D: '<S12>/1-D Lookup Table1'
      rtb_uDLookupTable1_idx_1 = intrp4d_fu32fla_pw(bpIndices, fractions,
        Controller_ConstP.uDLookupTable1_tableData,
        Controller_ConstP.uDLookupTable1_dimSizes,
        Controller_ConstP.uDLookupTable1_maxIndex);

      // MATLAB Function: '<S12>/MATLAB Function1' incorporates:
      //   Constant: '<S12>/Constant'
      //   Inport: '<Root>/Guidance'
      //   Inport: '<Root>/Measurements'
      //   MATLAB Function: '<S12>/MATLAB Function'
      //   SignalConversion generated from: '<Root>/Measurements'

      Controller_MATLABFunction1(Controller_U.Guidance.alpha + std::atan
        (-Controller_U.Guidance.e / 6.0F), Controller_U.Measurements.Psi,
        &rtb_Switch1);

      // DotProduct: '<S12>/Dot Product1' incorporates:
      //   Gain: '<S12>/Gain2'
      //   Inport: '<Root>/Measurements'
      //   SignalConversion generated from: '<Root>/Measurements'
      //   SignalConversion generated from: '<S12>/Dot Product1'

      rtb_Switch1 = rtb_uDLookupTable1_idx_0 * rtb_Switch1 +
        rtb_uDLookupTable1_idx_1 * -Controller_U.Measurements.r;

      // Saturate: '<S12>/Saturation1' incorporates:
      //   DotProduct: '<S12>/Dot Product1'

      if (rtb_Switch1 > 10.0F) {
        // DataTypeConversion: '<S12>/Data Type Conversion7'
        Controller_B.u_control = 10.0F;
      } else if (rtb_Switch1 < -10.0F) {
        // DataTypeConversion: '<S12>/Data Type Conversion7'
        Controller_B.u_control = -10.0F;
      } else {
        // DataTypeConversion: '<S12>/Data Type Conversion7'
        Controller_B.u_control = rtb_Switch1;
      }

      // End of Saturate: '<S12>/Saturation1'
      // End of Outputs for SubSystem: '<Root>/LQR - Entry Straight'
    }
  }

  // End of Chart: '<Root>/Chart'

  // Outputs for Enabled SubSystem: '<Root>/LQR - Keep Straight' incorporates:
  //   EnablePort: '<S13>/Enable'

  // RelationalOperator: '<S2>/Compare' incorporates:
  //   Constant: '<S2>/Constant'
  //   Lookup_n-D: '<S13>/1-D Lookup Table'

  if (rtb_controllerState == CONTROLLER_STATE_KEEP) {
    // Lookup_n-D: '<S13>/1-D Lookup Table' incorporates:
    //   Constant: '<S13>/Constant1'
    //   Constant: '<S13>/Constant4'
    //   Inport: '<Root>/Measurements'
    //   SignalConversion generated from: '<Root>/Measurements'

    bpIndices_0[0U] = plook_u32ff_lincpa(Controller_U.Measurements.vx,
      Controller_ConstP.pooled5, 7U, &rtb_Switch1, &Controller_DW.m_Cache01);
    fractions_0[0U] = rtb_Switch1;
    bpIndices_0[1U] = plook_u32u8f_lincpa
      (Controller_P.userParameters.straightAggressiveness,
       Controller_ConstP.pooled8, 6U, &rtb_Switch1, &Controller_DW.m_Cache02);
    fractions_0[1U] = rtb_Switch1;
    bpIndices_0[3U] = plook_u32u8f_lincpa(Controller_P.userParameters.modelPlant,
      Controller_ConstP.uDLookupTable_bp04Data, 3U, &rtb_Switch1,
      &Controller_DW.m_Cache04);
    fractions_0[3U] = rtb_Switch1;
    bpIndices_0[2U] = plook_u32u8f_lincpa(Controller_ConstB.DataTypeConversion6
      [0], Controller_ConstP.uDLookupTable_bp03Data, 2U, &rtb_Switch1,
      &Controller_DW.m_Cache03[0]);
    fractions_0[2U] = rtb_Switch1;

    // Lookup_n-D: '<S13>/1-D Lookup Table'
    rtb_uDLookupTable1_idx_0 = intrp4d_fu32fla_pw(bpIndices_0, fractions_0,
      Controller_ConstP.uDLookupTable_tableData,
      Controller_ConstP.uDLookupTable_dimSizes,
      Controller_ConstP.uDLookupTable_maxIndex);
    bpIndices_0[2U] = plook_u32u8f_lincpa(Controller_ConstB.DataTypeConversion6
      [1], Controller_ConstP.uDLookupTable_bp03Data, 2U, &rtb_Switch1,
      &Controller_DW.m_Cache03[1]);
    fractions_0[2U] = rtb_Switch1;

    // Lookup_n-D: '<S13>/1-D Lookup Table'
    rtb_uDLookupTable1_idx_1 = intrp4d_fu32fla_pw(bpIndices_0, fractions_0,
      Controller_ConstP.uDLookupTable_tableData,
      Controller_ConstP.uDLookupTable_dimSizes,
      Controller_ConstP.uDLookupTable_maxIndex);
    bpIndices_0[2U] = plook_u32u8f_lincpa(Controller_ConstB.DataTypeConversion6
      [2], Controller_ConstP.uDLookupTable_bp03Data, 2U, &rtb_Switch1,
      &Controller_DW.m_Cache03[2]);
    fractions_0[2U] = rtb_Switch1;

    // Lookup_n-D: '<S13>/1-D Lookup Table'
    rtb_uDLookupTable_idx_2 = intrp4d_fu32fla_pw(bpIndices_0, fractions_0,
      Controller_ConstP.uDLookupTable_tableData,
      Controller_ConstP.uDLookupTable_dimSizes,
      Controller_ConstP.uDLookupTable_maxIndex);

    // MATLAB Function: '<S13>/MATLAB Function' incorporates:
    //   Inport: '<Root>/Guidance'
    //   Inport: '<Root>/Measurements'
    //   Inport: '<S13>/alpha'
    //   SignalConversion generated from: '<Root>/Measurements'

    rtb_Switch1 = Controller_mod_d(Controller_U.Measurements.Psi);
    psi_ref_ = Controller_mod_d(Controller_U.Guidance.alpha);
    if (rtb_Switch1 > psi_ref_) {
      tmp = psi_ref_ - rtb_Switch1;
      rtb_Switch1 = (6.28318548F - rtb_Switch1) + psi_ref_;
      if (std::abs(rtb_Switch1) < std::abs(tmp)) {
        Controller_B.psi_error_keep = rtb_Switch1;
      } else {
        Controller_B.psi_error_keep = tmp;
      }
    } else if (rtb_Switch1 < psi_ref_) {
      if (std::abs(rtb_Switch1 - psi_ref_) < std::abs((6.28318548F - psi_ref_) +
           rtb_Switch1)) {
        Controller_B.psi_error_keep = psi_ref_ - rtb_Switch1;
      } else {
        Controller_B.psi_error_keep = -((rtb_Switch1 + 6.28318548F) - psi_ref_);
      }
    } else {
      Controller_B.psi_error_keep = 0.0F;
    }

    // End of MATLAB Function: '<S13>/MATLAB Function'

    // DotProduct: '<S13>/Dot Product' incorporates:
    //   Gain: '<S13>/Gain2'
    //   Gain: '<S13>/Gain3'
    //   Inport: '<Root>/Guidance'
    //   Inport: '<Root>/Measurements'
    //   SignalConversion generated from: '<Root>/Measurements'
    //   SignalConversion generated from: '<S13>/Dot Product'

    rtb_Switch1 = (rtb_uDLookupTable1_idx_0 * Controller_B.psi_error_keep +
                   rtb_uDLookupTable1_idx_1 * -Controller_U.Measurements.r) +
      rtb_uDLookupTable_idx_2 * -Controller_U.Guidance.e;

    // Saturate: '<S13>/Saturation1' incorporates:
    //   DotProduct: '<S13>/Dot Product'

    if (rtb_Switch1 > 5.0F) {
      // DataTypeConversion: '<S13>/Data Type Conversion7'
      Controller_B.u_control = 5.0F;
    } else if (rtb_Switch1 < -5.0F) {
      // DataTypeConversion: '<S13>/Data Type Conversion7'
      Controller_B.u_control = -5.0F;
    } else {
      // DataTypeConversion: '<S13>/Data Type Conversion7'
      Controller_B.u_control = rtb_Switch1;
    }

    // End of Saturate: '<S13>/Saturation1'
  }

  // End of RelationalOperator: '<S2>/Compare'
  // End of Outputs for SubSystem: '<Root>/LQR - Keep Straight'

  // Outputs for Enabled SubSystem: '<Root>/Controle_Curva' incorporates:
  //   EnablePort: '<S9>/Enable'

  // RelationalOperator: '<S7>/Compare' incorporates:
  //   Constant: '<S7>/Constant'

  if (rtb_controllerState == CONTROLLER_STATE_CURVE) {
    // Product: '<S9>/Product3' incorporates:
    //   Constant: '<S9>/Constant8'
    //   Inport: '<Root>/Measurements'
    //   SignalConversion generated from: '<Root>/Measurements'

    rtb_Switch1 = 2.0F * Controller_U.Measurements.vx;

    // MinMax: '<S9>/Max'
    if (rtb_Switch1 > 2.0F) {
      rtb_Switch1_0 = rtb_Switch1;
    } else {
      rtb_Switch1_0 = 2.0;
    }

    // End of MinMax: '<S9>/Max'

    // MATLAB Function: '<S9>/MATLAB Function' incorporates:
    //   Inport: '<Root>/Guidance'

    Controller_B.psi_ref = std::atan(-Controller_U.Guidance.e /
      static_cast<real32_T>(rtb_Switch1_0)) + Controller_U.Guidance.alpha;

    // MATLAB Function: '<S9>/MATLAB Function1' incorporates:
    //   Inport: '<Root>/Measurements'
    //   SignalConversion generated from: '<Root>/Measurements'
    //   Sum: '<S9>/Add'

    Controller_MATLABFunction1(Controller_B.psi_ref,
      Controller_U.Measurements.sideslip + Controller_U.Measurements.Psi,
      &rtb_Switch1);

    // Product: '<S9>/Product' incorporates:
    //   Inport: '<Root>/Guidance'
    //   Inport: '<Root>/Measurements'
    //   SignalConversion generated from: '<Root>/Measurements'

    Controller_B.r_ref = Controller_U.Guidance.curvature *
      Controller_U.Measurements.vx;

    // Sum: '<S9>/Add1' incorporates:
    //   Constant: '<S9>/Constant4'
    //   Constant: '<S9>/Constant5'
    //   Inport: '<Root>/Measurements'
    //   Product: '<S9>/Product1'
    //   Product: '<S9>/Product2'
    //   SignalConversion generated from: '<Root>/Measurements'
    //   Sum: '<S9>/Add2'

    rtb_Switch1_0 = (Controller_B.r_ref - Controller_U.Measurements.r) * 75.0 +
      rtb_Switch1 * 81.0;

    // Saturate: '<S9>/Saturation'
    if (rtb_Switch1_0 > 6.0) {
      // DataTypeConversion: '<S9>/Data Type Conversion3'
      Controller_B.u_control = 6.0F;
    } else if (rtb_Switch1_0 < -6.0) {
      // DataTypeConversion: '<S9>/Data Type Conversion3'
      Controller_B.u_control = -6.0F;
    } else {
      // DataTypeConversion: '<S9>/Data Type Conversion3'
      Controller_B.u_control = static_cast<real32_T>(rtb_Switch1_0);
    }

    // End of Saturate: '<S9>/Saturation'
  }

  // End of RelationalOperator: '<S7>/Compare'
  // End of Outputs for SubSystem: '<Root>/Controle_Curva'

  // Outputs for Enabled SubSystem: '<Root>/Control Off' incorporates:
  //   EnablePort: '<S8>/Enable'

  // Logic: '<Root>/AND' incorporates:
  //   Constant: '<S4>/Constant'
  //   Constant: '<S5>/Constant'
  //   Constant: '<S6>/Constant'
  //   RelationalOperator: '<S4>/Compare'
  //   RelationalOperator: '<S5>/Compare'
  //   RelationalOperator: '<S6>/Compare'

  if ((rtb_controllerState != CONTROLLER_STATE_ENTRY) && (rtb_controllerState !=
       CONTROLLER_STATE_KEEP) && (rtb_controllerState != CONTROLLER_STATE_CURVE))
  {
    // DataTypeConversion: '<S8>/Data Type Conversion8' incorporates:
    //   Constant: '<S8>/Constant'

    Controller_B.u_control = 0.0F;
  }

  // End of Logic: '<Root>/AND'
  // End of Outputs for SubSystem: '<Root>/Control Off'

  // MATLAB Function: '<Root>/MATLAB Function1' incorporates:
  //   Inport: '<Root>/Measurements'
  //   SignalConversion generated from: '<Root>/Measurements'

  Modo_Curva_prev = false;
  if (Controller_P.steerCalibration.isCalibrated) {
    if (Controller_DW.rigthSaturation) {
      Controller_DW.rigthSaturation = ((Controller_U.Measurements.theta <=
        Controller_P.steerCalibration.rightValue +
        Controller_P.steerCalibration.deadZone) && Controller_DW.rigthSaturation);
    } else {
      Controller_DW.rigthSaturation = ((Controller_U.Measurements.theta <
        Controller_P.steerCalibration.rightValue) ||
        Controller_DW.rigthSaturation);
    }

    if (Controller_DW.leftSaturation) {
      Controller_DW.leftSaturation = ((Controller_U.Measurements.theta >=
        Controller_P.steerCalibration.leftValue -
        Controller_P.steerCalibration.deadZone) && Controller_DW.leftSaturation);
    } else {
      Controller_DW.leftSaturation = ((Controller_U.Measurements.theta >
        Controller_P.steerCalibration.leftValue) || Controller_DW.leftSaturation);
    }

    Modo_Curva_prev = ((Controller_DW.rigthSaturation && (Controller_B.u_control
      < 0.0F)) || (Controller_DW.leftSaturation && (Controller_B.u_control >
      0.0F)));
  }

  // End of MATLAB Function: '<Root>/MATLAB Function1'

  // RelationalOperator: '<S10>/FixPt Relational Operator' incorporates:
  //   UnitDelay: '<S10>/Delay Input1'
  //
  //  Block description for '<S10>/Delay Input1':
  //
  //   Store in Global RAM

  Controller_Y.Controller_State.Pulse_Disengage = (Controller_B.flag_disengage
    != Controller_DW.DelayInput1_DSTATE);

  // RelationalOperator: '<S11>/FixPt Relational Operator' incorporates:
  //   UnitDelay: '<S11>/Delay Input1'
  //
  //  Block description for '<S11>/Delay Input1':
  //
  //   Store in Global RAM

  Controller_Y.Controller_State.Pulse_Enable_Servo_Changed =
    (Controller_B.enable_servo != Controller_DW.DelayInput1_DSTATE_m);

  // BusCreator: '<Root>/Bus Creator' incorporates:
  //   Outport: '<Root>/Controller_State'

  Controller_Y.Controller_State.Flag_Controlador_Ready = Controller_B.ready;
  Controller_Y.Controller_State.Flag_Delta_Saturation = Modo_Curva_prev;
  Controller_Y.Controller_State.Flag_Enable_Servo = Controller_B.enable_servo;
  Controller_Y.Controller_State.Operation_Mode = rtb_controllerState;

  // Switch: '<Root>/Switch1' incorporates:
  //   Logic: '<Root>/Logical Operator1'
  //   Logic: '<Root>/NOT'

  if (Modo_Curva_prev || (!Controller_B.enable_control)) {
    // BusCreator: '<Root>/Bus Creator' incorporates:
    //   Outport: '<Root>/Controller_State'

    Controller_Y.Controller_State.Angular_Speed_Target =
      Controller_ConstB.DataTypeConversion1;
  } else {
    // BusCreator: '<Root>/Bus Creator' incorporates:
    //   Outport: '<Root>/Controller_State'

    Controller_Y.Controller_State.Angular_Speed_Target = Controller_B.u_control;
  }

  // End of Switch: '<Root>/Switch1'

  // BusCreator: '<Root>/Bus Creator' incorporates:
  //   Outport: '<Root>/Controller_State'

  Controller_Y.Controller_State.Toggle_Disengage = Controller_B.flag_disengage;
  Controller_Y.Controller_State.Flag_Enable_Control =
    Controller_B.enable_control;

  // Update for UnitDelay: '<S10>/Delay Input1'
  //
  //  Block description for '<S10>/Delay Input1':
  //
  //   Store in Global RAM

  Controller_DW.DelayInput1_DSTATE = Controller_B.flag_disengage;

  // Update for UnitDelay: '<S11>/Delay Input1'
  //
  //  Block description for '<S11>/Delay Input1':
  //
  //   Store in Global RAM

  Controller_DW.DelayInput1_DSTATE_m = Controller_B.enable_servo;
}

// Model initialize function
void ControladorModelClass::initialize()
{
  // Registration code

  // block I/O
  (void) std::memset((static_cast<void *>(&Controller_B)), 0,
                     sizeof(B_Controller_T));

  // states (dwork)
  (void) std::memset(static_cast<void *>(&Controller_DW), 0,
                     sizeof(DW_Controller_T));

  // external inputs
  (void)std::memset(&Controller_U, 0, sizeof(ExtU_Controller_T));
  Controller_U.vehicleMode = VEHICLE_MODE_NONE;

  // external outputs
  Controller_Y.Controller_State = Controller_rtZControler_Bus;

  // SystemInitialize for Chart: '<Root>/Chart'
  Controller_DW.temporalCounter_i1 = 0U;
  Controller_DW.is_CONTROL_ON = Controller_IN_NO_ACTIVE_CHILD;
  Controller_DW.is_active_c1_Controller = 0U;
  Controller_DW.is_c1_Controller = Controller_IN_NO_ACTIVE_CHILD;
  Controller_B.enable_control = false;
  Controller_B.ready = false;
  Controller_B.flag_disengage = false;

  // SystemInitialize for Merge: '<Root>/Merge'
  Controller_B.u_control = 0.0F;

  // SystemInitialize for MATLAB Function: '<Root>/MATLAB Function1'
  Controller_DW.rigthSaturation = false;
  Controller_DW.leftSaturation = false;
}

// Model terminate function
void ControladorModelClass::terminate()
{
  // (no terminate code required)
}

// Constructor
ControladorModelClass::ControladorModelClass()
{
  // Currently there is no constructor body generated.
}

// Destructor
ControladorModelClass::~ControladorModelClass()
{
  // Currently there is no destructor body generated.
}

//
// File trailer for generated code.
//
// [EOF]
//
