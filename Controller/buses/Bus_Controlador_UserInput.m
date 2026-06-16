clear controllerElement

controllerElement(1) = Simulink.BusElement;
controllerElement(1).Name = 'Enable';
controllerElement(1).Description = 'Habilita o controlador';
controllerElement(1).DataType = 'boolean';
controllerElement(1).Dimensions = 1;

controllerElement(2) = Simulink.BusElement;
controllerElement(2).Name = 'Modo_Veiculo';
controllerElement(2).Description = 'Modo do veículo';
controllerElement(2).DataType = 'Enum: ModoVeiculo';
controllerElement(2).Dimensions = 1;

controllerElement(3) = Simulink.BusElement;
controllerElement(3).Name = 'Agressividade_Reta';
controllerElement(3).Description = 'Parâmetro de agressividade da reta';
controllerElement(3).DataType = 'uint8';
controllerElement(3).Dimensions = 1;

controllerElement(4) = Simulink.BusElement;
controllerElement(4).Name = 'Agressividade_Entrada';
controllerElement(4).Description = 'Parâmetro de agressividade da entrada na reta';
controllerElement(4).DataType = 'uint8';
controllerElement(4).Dimensions = 1;

controllerElement(5) = Simulink.BusElement;
controllerElement(5).Name = 'Enable_Modo_Curva';
controllerElement(5).Description = 'Habilita/desabilita o modo curva';
controllerElement(5).DataType = 'boolean';
controllerElement(5).Dimensions = 1;

controllerElement(6) = Simulink.BusElement;
controllerElement(6).Name = 'Planta';
controllerElement(6).Description = 'Indica o modelo da planta que o controle deve considerar';
controllerElement(6).DataType = 'uint8';
controllerElement(6).Dimensions = 1;

Controlador_UserInput_Bus = Simulink.Bus;
Controlador_UserInput_Bus.Elements = controllerElement;
Controlador_UserInput_Bus.Description = 'Entrada de comandos do usuário do modelo ''Controlador''';
