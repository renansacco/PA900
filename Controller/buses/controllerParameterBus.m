clear controllerElement

controllerElement(1) = Simulink.BusElement;
controllerElement(1).Name = 'modelPlant';
controllerElement(1).Description = 'Indica o modelo da planta que o controle deve considerar';
controllerElement(1).DataType = 'uint8';
controllerElement(1).Dimensions = 1;

controllerElement(2) = Simulink.BusElement;
controllerElement(2).Name = 'vehicleMode';
controllerElement(2).Description = 'Modo do veículo';
controllerElement(2).DataType = 'Enum: ModoVeiculo';
controllerElement(2).Dimensions = 1;

controllerElement(3) = Simulink.BusElement;
controllerElement(3).Name = 'straightAggressiveness';
controllerElement(3).Description = 'Parâmetro de agressividade da reta';
controllerElement(3).DataType = 'uint8';
controllerElement(3).Dimensions = 1;

controllerElement(4) = Simulink.BusElement;
controllerElement(4).Name = 'curveAggressiveness';
controllerElement(4).Description = 'Parâmetro de agressividade da entrada na reta';
controllerElement(4).DataType = 'uint8';
controllerElement(4).Dimensions = 1;

controllerElement(5) = Simulink.BusElement;
controllerElement(5).Name = 'isCurveMode';
controllerElement(5).Description = 'Habilita/desabilita o modo durva';
controllerElement(5).DataType = 'boolean';
controllerElement(5).Dimensions = 1;

controllerParameterBus_t = Simulink.Bus;
controllerParameterBus_t.Elements = controllerElement;
controllerParameterBus_t.Description = 'Parametros do piloto automático (agressividade, planta etc)';