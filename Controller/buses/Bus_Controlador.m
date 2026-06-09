controllerElement(1) = Simulink.BusElement;
controllerElement(1).Name = 'Flag_Controlador_Ready';
controllerElement(1).Description = 'Indica que o subsistema do controlador está pronto';
controllerElement(1).DataType = 'boolean';
controllerElement(1).Dimensions = 1;

controllerElement(2) = Simulink.BusElement;
controllerElement(2).Name = 'Flag_Delta_Saturation';
controllerElement(2).Description = 'Indica que houve saturação no ângulo de esterçamento da roda frontal';
controllerElement(2).DataType = 'boolean';
controllerElement(2).Dimensions = 1;

controllerElement(3) = Simulink.BusElement;
controllerElement(3).Name = 'Flag_Enable_Servo';
controllerElement(3).Description = 'Se ''true'', indica que o servo deve ser ligado. Se ''false'', indica que o servo deve ser desligado.';
controllerElement(3).DataType = 'boolean';
controllerElement(3).Dimensions = 1;

controllerElement(4) = Simulink.BusElement;
controllerElement(4).Name = 'Operation_Mode';
controllerElement(4).Description = 'Indica o modo de operação do controlador';
controllerElement(4).DataType = 'Enum: ControllerState_t';
controllerElement(4).Dimensions = 1;

controllerElement(5) = Simulink.BusElement;
controllerElement(5).Name = 'Angular_Speed_Target';
controllerElement(5).Description = 'Velocidade angular comandada para o motor';
controllerElement(5).DataType = 'single';
controllerElement(5).Unit = 'rad/s';
controllerElement(5).Dimensions = 1;

controllerElement(6) = Simulink.BusElement;
controllerElement(6).Name = 'Toggle_Disengage';
controllerElement(6).Description = 'Quando alternado, indica que o piloto foi desengatado';
controllerElement(6).DataType = 'boolean';
controllerElement(6).Dimensions = 1;

controllerElement(7) = Simulink.BusElement;
controllerElement(7).Name = 'Pulse_Disengage';
controllerElement(7).Description = 'Produz um pulso quando o piloto for desengatado. ''true'' indica desengate no step anterior.';
controllerElement(7).DataType = 'boolean';
controllerElement(7).Dimensions = 1;

controllerElement(8) = Simulink.BusElement;
controllerElement(8).Name = 'Pulse_Enable_Servo_Changed';
controllerElement(8).Description = 'Produz um pulso quando o servo for habilitado ou desabilitado';
controllerElement(8).DataType = 'boolean';
controllerElement(8).Dimensions = 1;

controllerElement(9) = Simulink.BusElement;
controllerElement(9).Name = 'Flag_Enable_Control';
controllerElement(9).Description = 'Se true, indica que o controlador está operando em algum dos modos (keep, entry,curve)';
controllerElement(9).DataType = 'boolean';
controllerElement(9).Dimensions = 1;

Controler_Bus = Simulink.Bus;
Controler_Bus.Elements = controllerElement;
Controler_Bus.Description = 'Saída do subsistema ''controller''';