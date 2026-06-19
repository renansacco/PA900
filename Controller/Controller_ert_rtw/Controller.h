//
// File: Controller.h
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
#ifndef RTW_HEADER_Controller_h_
#define RTW_HEADER_Controller_h_
#include <cmath>
#include <cstring>
#ifndef Controller_COMMON_INCLUDES_
# define Controller_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 // Controller_COMMON_INCLUDES_

#include "Controller_types.h"

// Macros for accessing real-time model data structure

// Block signals (default storage)
typedef struct {
  real32_T u_control;                  // '<Root>/Merge'
  real32_T psi_error_keep;             // '<S13>/MATLAB Function'
  real32_T r_ref;                      // '<S9>/Product'
  real32_T psi_ref;                    // '<S9>/MATLAB Function'
  boolean_T enable_servo;              // '<Root>/Chart'
  boolean_T enable_control;            // '<Root>/Chart'
  boolean_T ready;                     // '<Root>/Chart'
  boolean_T flag_disengage;            // '<Root>/Chart'
} B_Controller_T;

// Block states (default storage) for system '<Root>'
typedef struct {
  real32_T m_bpLambda[4];              // '<S13>/1-D Lookup Table'
  real32_T m_bpLambda_n[4];            // '<S12>/1-D Lookup Table1'
  uint32_T m_bpIndex[6];               // '<S13>/1-D Lookup Table'
  uint32_T m_Cache01;                  // '<S13>/1-D Lookup Table'
  uint32_T m_Cache02;                  // '<S13>/1-D Lookup Table'
  uint32_T m_Cache03[3];               // '<S13>/1-D Lookup Table'
  uint32_T m_Cache04;                  // '<S13>/1-D Lookup Table'
  uint32_T m_bpIndex_j[5];             // '<S12>/1-D Lookup Table1'
  uint32_T m_Cache01_i;                // '<S12>/1-D Lookup Table1'
  uint32_T m_Cache02_j;                // '<S12>/1-D Lookup Table1'
  uint32_T m_Cache03_e[2];             // '<S12>/1-D Lookup Table1'
  uint32_T m_Cache04_o;                // '<S12>/1-D Lookup Table1'
  boolean_T DelayInput1_DSTATE;        // '<S10>/Delay Input1'
  boolean_T DelayInput1_DSTATE_m;      // '<S11>/Delay Input1'
  uint8_T is_active_c1_Controller;     // '<Root>/Chart'
  uint8_T is_c1_Controller;            // '<Root>/Chart'
  uint8_T is_CONTROL_ON;               // '<Root>/Chart'
  uint8_T temporalCounter_i1;          // '<Root>/Chart'
  boolean_T rigthSaturation;           // '<Root>/MATLAB Function1'
  boolean_T leftSaturation;            // '<Root>/MATLAB Function1'
  boolean_T Modo_Curva_start;          // '<Root>/Chart'
} DW_Controller_T;

// Invariant block signals (default storage)
typedef const struct tag_ConstB_Controller_T {
  real32_T DataTypeConversion1;        // '<Root>/Data Type Conversion1'
  uint8_T DataTypeConversion6[3];      // '<S13>/Data Type Conversion6'
  uint8_T DataTypeConversion6_n[2];    // '<S12>/Data Type Conversion6'
} ConstB_Controller_T;

// Constant parameters (default storage)
typedef struct {
  // Expression: Controlador.Entry.Gains
  //  Referenced by: '<S12>/1-D Lookup Table1'

  real32_T uDLookupTable1_tableData[224];

  // Pooled Parameter (Mixed Expressions)
  //  Referenced by:
  //    '<S12>/1-D Lookup Table1'
  //    '<S13>/1-D Lookup Table'

  real32_T pooled5[8];

  // Expression: Controlador.Keep.Gains
  //  Referenced by: '<S13>/1-D Lookup Table'

  real32_T uDLookupTable_tableData[672];

  // Computed Parameter: uDLookupTable1_maxIndex
  //  Referenced by: '<S12>/1-D Lookup Table1'

  uint32_T uDLookupTable1_maxIndex[4];

  // Computed Parameter: uDLookupTable1_dimSizes
  //  Referenced by: '<S12>/1-D Lookup Table1'

  uint32_T uDLookupTable1_dimSizes[4];

  // Computed Parameter: uDLookupTable_maxIndex
  //  Referenced by: '<S13>/1-D Lookup Table'

  uint32_T uDLookupTable_maxIndex[4];

  // Computed Parameter: uDLookupTable_dimSizes
  //  Referenced by: '<S13>/1-D Lookup Table'

  uint32_T uDLookupTable_dimSizes[4];

  // Pooled Parameter (Expression: uint8(1:7))
  //  Referenced by:
  //    '<S12>/1-D Lookup Table1'
  //    '<S13>/1-D Lookup Table'

  uint8_T pooled8[7];

  // Pooled Parameter (Expression: uint8(1:2))
  //  Referenced by: '<S12>/1-D Lookup Table1'

  uint8_T pooled9[2];

  // Expression: uint8(1:3)
  //  Referenced by: '<S13>/1-D Lookup Table'

  uint8_T uDLookupTable_bp03Data[3];

  // Expression: uint8(1:4)
  //  Referenced by: '<S13>/1-D Lookup Table'

  uint8_T uDLookupTable_bp04Data[4];
} ConstP_Controller_T;

// External inputs (root inport signals with default storage)
typedef struct {
  controllerMeasurementBus_t Measurements;// '<Root>/Measurements'
  controllerInputGuidanceBus_t Guidance;// '<Root>/Guidance'
  boolean_T Enable;                    // '<Root>/Enable'
  VehicleMode_t vehicleMode;           // '<Root>/vehicleMode'
} ExtU_Controller_T;

// External outputs (root outports fed by signals with default storage)
typedef struct {
  Controler_Bus Controller_State;      // '<Root>/Controller_State'
} ExtY_Controller_T;

// Parameters (default storage)
struct P_Controller_T_ {
  controllerSteerCalibrationBus_t steerCalibration;// Variable: steerCalibration
                                                      //  Referenced by: '<Root>/MATLAB Function1'

  controllerParameterBus_t userParameters;// Variable: userParameters
                                             //  Referenced by:
                                             //    '<Root>/Constant1'
                                             //    '<S12>/Constant4'
                                             //    '<S12>/Constant6'
                                             //    '<S13>/Constant1'
                                             //    '<S13>/Constant4'

};

// External data declarations for dependent source files
extern const Controler_Bus Controller_rtZControler_Bus;// Controler_Bus ground
extern const ConstB_Controller_T Controller_ConstB;// constant block i/o

// Constant parameters (default storage)
extern const ConstP_Controller_T Controller_ConstP;

// Class declaration for model Controller
class ControladorModelClass {
  // public data and function members
 public:
  // Tunable parameters
  static P_Controller_T Controller_P;

  // External inputs
  ExtU_Controller_T Controller_U;

  // External outputs
  ExtY_Controller_T Controller_Y;

  // model initialize function
  void initialize();

  // model step function
  void step();

  // model terminate function
  void terminate();

  // Constructor
  ControladorModelClass();

  // Destructor
  ~ControladorModelClass();

  // private data and function members
 private:
  // Block signals
  B_Controller_T Controller_B;

  // Block states
  DW_Controller_T Controller_DW;

  // private member function(s) for subsystem '<S9>/MATLAB Function1'
  void Controller_MATLABFunction1(real32_T rtu_ref, real32_T rtu_meas, real32_T *
    rty_ang_error);
  real32_T Controller_mod(real32_T x);

  // private member function(s) for subsystem '<Root>'
  real32_T Controller_mod_d(real32_T x);
};

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S9>/1-D Lookup Table2' : Unused code path elimination
//  Block '<S9>/Constant2' : Unused code path elimination
//  Block '<S9>/Display' : Unused code path elimination
//  Block '<S9>/Display1' : Unused code path elimination
//  Block '<S9>/Display2' : Unused code path elimination
//  Block '<S9>/Scope2' : Unused code path elimination
//  Block '<S9>/Scope3' : Unused code path elimination
//  Block '<S9>/Scope8' : Unused code path elimination
//  Block '<Root>/Data Type Conversion' : Eliminate redundant data type conversion
//  Block '<Root>/Data Type Conversion2' : Eliminate redundant data type conversion
//  Block '<Root>/Data Type Conversion3' : Eliminate redundant data type conversion
//  Block '<S12>/Data Type Conversion2' : Eliminate redundant data type conversion
//  Block '<S12>/Data Type Conversion5' : Eliminate redundant data type conversion
//  Block '<S13>/Data Type Conversion1' : Eliminate redundant data type conversion
//  Block '<S13>/Data Type Conversion5' : Eliminate redundant data type conversion
//  Block '<Root>/Zero-Order Hold' : Eliminated since input and output rates are identical


//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'Controller'
//  '<S1>'   : 'Controller/Chart'
//  '<S2>'   : 'Controller/Compare To Constant1'
//  '<S3>'   : 'Controller/Compare To Constant2'
//  '<S4>'   : 'Controller/Compare To Constant3'
//  '<S5>'   : 'Controller/Compare To Constant4'
//  '<S6>'   : 'Controller/Compare To Constant5'
//  '<S7>'   : 'Controller/Compare To Constant6'
//  '<S8>'   : 'Controller/Control Off'
//  '<S9>'   : 'Controller/Controle_Curva'
//  '<S10>'  : 'Controller/Detect Change'
//  '<S11>'  : 'Controller/Detect Change1'
//  '<S12>'  : 'Controller/LQR - Entry Straight'
//  '<S13>'  : 'Controller/LQR - Keep Straight'
//  '<S14>'  : 'Controller/MATLAB Function1'
//  '<S15>'  : 'Controller/Controle_Curva/MATLAB Function'
//  '<S16>'  : 'Controller/Controle_Curva/MATLAB Function1'
//  '<S17>'  : 'Controller/LQR - Entry Straight/MATLAB Function'
//  '<S18>'  : 'Controller/LQR - Entry Straight/MATLAB Function1'
//  '<S19>'  : 'Controller/LQR - Keep Straight/MATLAB Function'

#endif                                 // RTW_HEADER_Controller_h_

//
// File trailer for generated code.
//
// [EOF]
//
