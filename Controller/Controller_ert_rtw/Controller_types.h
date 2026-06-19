//
// File: Controller_types.h
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
#ifndef RTW_HEADER_Controller_types_h_
#define RTW_HEADER_Controller_types_h_
#include "rtwtypes.h"
#include "mg900_model_types.h"
#ifndef DEFINED_TYPEDEF_FOR_controllerMeasurementBus_t_
#define DEFINED_TYPEDEF_FOR_controllerMeasurementBus_t_

// Entrada de medidas do modelo 'Controlador'
typedef struct {
  // Ângulo de guinada do veículo, em rad
  real32_T Psi;

  // Velocidade angular de guinada do veículo, em rad/s
  real32_T r;

  // Ângulo de esterçamento da direção, em rad
  real32_T theta;

  // Velocidade linear frontal do veículo, em m/s
  real32_T vx;

  // Angulo de escorregamento (course=psi+sideslip) em rad
  real32_T sideslip;
} controllerMeasurementBus_t;

#endif

#ifndef DEFINED_TYPEDEF_FOR_controllerInputGuidanceBus_t_
#define DEFINED_TYPEDEF_FOR_controllerInputGuidanceBus_t_

// Entrada de guiagem do modelo 'Controlador'
typedef struct {
  // Ângulo da trajetória no ponto relevante, em rad
  real32_T alpha;

  // Erro lateral do veículo em relação à trajetória no ponto relevante, em metros 
  real32_T e;

  // Curvatura da trajetória no ponto relevante, em 1/m
  real32_T curvature;

  // Erro angular em radianos (psiRef - psi)
  real32_T psiError;
} controllerInputGuidanceBus_t;

#endif

#ifndef DEFINED_TYPEDEF_FOR_Controler_Bus_
#define DEFINED_TYPEDEF_FOR_Controler_Bus_

// Saída do subsistema 'controller'
typedef struct {
  // Indica que o subsistema do controlador está pronto
  boolean_T Flag_Controlador_Ready;

  // Indica que houve saturação no ângulo de esterçamento da roda frontal
  boolean_T Flag_Delta_Saturation;

  // Se 'true', indica que o servo deve ser ligado. Se 'false', indica que o servo deve ser desligado. 
  boolean_T Flag_Enable_Servo;

  // Indica o modo de operação do controlador
  ControllerState_t Operation_Mode;

  // Velocidade angular comandada para o motor
  real32_T Angular_Speed_Target;

  // Quando alternado, indica que o piloto foi desengatado
  boolean_T Toggle_Disengage;

  // Produz um pulso quando o piloto for desengatado. 'true' indica desengate no step anterior. 
  boolean_T Pulse_Disengage;

  // Produz um pulso quando o servo for habilitado ou desabilitado
  boolean_T Pulse_Enable_Servo_Changed;

  // Se true, indica que o controlador está operando em algum dos modos (keep, entry,curve) 
  boolean_T Flag_Enable_Control;
} Controler_Bus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_ModoVeiculo_
#define DEFINED_TYPEDEF_FOR_ModoVeiculo_

// enumeration to track active leaf state of Simulacao/Modos do Veiculo
typedef enum {
  ModoVeiculo_None = 0,                // Default value
  ModoVeiculo_Parado,
  ModoVeiculo_Andando,
  ModoVeiculo_Re,
  ModoVeiculo_Parado_Re
} ModoVeiculo;

#endif

#ifndef DEFINED_TYPEDEF_FOR_controllerParameterBus_t_
#define DEFINED_TYPEDEF_FOR_controllerParameterBus_t_

// Parametros do piloto automático (agressividade, planta etc)
typedef struct {
  // Indica o modelo da planta que o controle deve considerar
  uint8_T modelPlant;

  // Modo do veículo
  ModoVeiculo vehicleMode;

  // Parâmetro de agressividade da reta
  uint8_T straightAggressiveness;

  // Parâmetro de agressividade da entrada na reta
  uint8_T curveAggressiveness;

  // Habilita/desabilita o modo durva
  boolean_T isCurveMode;
} controllerParameterBus_t;

#endif

#ifndef DEFINED_TYPEDEF_FOR_controllerSteerCalibrationBus_t_
#define DEFINED_TYPEDEF_FOR_controllerSteerCalibrationBus_t_

// Parametros de calibração da direção
typedef struct {
  // Ângulo 'theta' de máximo esterçamento para ESQUERDA
  real32_T leftValue;

  // Ângulo 'theta' de máximo esterçamento para DIREITA
  real32_T rightValue;

  // Ângulo 'theta' de esterçamento centralizado
  real32_T centerValue;

  // Indica se os valores foram setados ou não
  boolean_T isCalibrated;

  // Zona morta da saturação, em rad
  real32_T deadZone;
} controllerSteerCalibrationBus_t;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_xm1VixvgQvCXgRr2KQqytC_
#define DEFINED_TYPEDEF_FOR_struct_xm1VixvgQvCXgRr2KQqytC_

typedef struct {
  real32_T Gains[16];
  real32_T v_index[8];
  real32_T T_look;
  real32_T omegam_sat;
} struct_xm1VixvgQvCXgRr2KQqytC;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_aZbHsNpZ2wOKlbf6Q3u2yD_
#define DEFINED_TYPEDEF_FOR_struct_aZbHsNpZ2wOKlbf6Q3u2yD_

typedef struct {
  real32_T Gains[672];
  real32_T R_index[7];
  real32_T v_index[8];
  real_T omegam_sat;
} struct_aZbHsNpZ2wOKlbf6Q3u2yD;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_K7aNpc71bVF4P6UKAeMFnB_
#define DEFINED_TYPEDEF_FOR_struct_K7aNpc71bVF4P6UKAeMFnB_

typedef struct {
  real_T omegam_sat;
  real32_T Delta;
  real32_T Gains[224];
  real32_T R_index[7];
  real32_T v_index[8];
} struct_K7aNpc71bVF4P6UKAeMFnB;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_D1gEF8DYleRuqwK7MuNeeH_
#define DEFINED_TYPEDEF_FOR_struct_D1gEF8DYleRuqwK7MuNeeH_

typedef struct {
  real_T Ts;
  struct_xm1VixvgQvCXgRr2KQqytC Curva;
  struct_aZbHsNpZ2wOKlbf6Q3u2yD Keep;
  struct_K7aNpc71bVF4P6UKAeMFnB Entry;
} struct_D1gEF8DYleRuqwK7MuNeeH;

#endif

// Parameters (default storage)
typedef struct P_Controller_T_ P_Controller_T;

#endif                                 // RTW_HEADER_Controller_types_h_

//
// File trailer for generated code.
//
// [EOF]
//
