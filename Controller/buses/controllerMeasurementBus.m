clear controllerElement

controllerElement(1) = Simulink.BusElement;
controllerElement(1).Name = 'Psi';
controllerElement(1).Description = 'Ângulo de guinada do veículo, em rad';
controllerElement(1).DataType = 'single';
controllerElement(1).Dimensions = 1;

controllerElement(2) = Simulink.BusElement;
controllerElement(2).Name = 'r';
controllerElement(2).Description = 'Velocidade angular de guinada do veículo, em rad/s';
controllerElement(2).DataType = 'single';
controllerElement(2).Dimensions = 1;

controllerElement(3) = Simulink.BusElement;
controllerElement(3).Name = 'theta';
controllerElement(3).Description = 'Ângulo de esterçamento da direção, em rad';
controllerElement(3).DataType = 'single';
controllerElement(3).Dimensions = 1;

controllerElement(4) = Simulink.BusElement;
controllerElement(4).Name = 'vx';
controllerElement(4).Description = 'Velocidade linear frontal do veículo, em m/s';
controllerElement(4).DataType = 'single';
controllerElement(4).Dimensions = 1;


controllerMeasurementBus_t = Simulink.Bus;
controllerMeasurementBus_t.Elements = controllerElement;
controllerMeasurementBus_t.Description = 'Entrada de medidas do modelo ''Controlador''';