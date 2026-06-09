clear controllerElement

controllerElement(1) = Simulink.BusElement;
controllerElement(1).Name = 'leftValue';
controllerElement(1).Description = 'Ângulo ''theta'' de máximo esterçamento para ESQUERDA';
controllerElement(1).DataType = 'single';
controllerElement(1).Dimensions = 1;

controllerElement(2) = Simulink.BusElement;
controllerElement(2).Name = 'rightValue';
controllerElement(2).Description = 'Ângulo ''theta'' de máximo esterçamento para DIREITA';
controllerElement(2).DataType = 'single';
controllerElement(2).Dimensions = 1;

controllerElement(3) = Simulink.BusElement;
controllerElement(3).Name = 'centerValue';
controllerElement(3).Description = 'Ângulo ''theta'' de esterçamento centralizado';
controllerElement(3).DataType = 'single';
controllerElement(3).Dimensions = 1;

controllerElement(4) = Simulink.BusElement;
controllerElement(4).Name = 'isCalibrated';
controllerElement(4).Description = 'Indica se os valores foram setados ou não';
controllerElement(4).DataType = 'boolean';
controllerElement(4).Dimensions = 1;

controllerElement(5) = Simulink.BusElement;
controllerElement(5).Name = 'deadZone';
controllerElement(5).Description = 'Zona morta da saturação, em rad';
controllerElement(5).DataType = 'single';
controllerElement(5).Dimensions = 1;

controllerSteerCalibrationBus_t = Simulink.Bus;
controllerSteerCalibrationBus_t.Elements = controllerElement;
controllerSteerCalibrationBus_t.Description = 'Parametros de calibração da direção';
