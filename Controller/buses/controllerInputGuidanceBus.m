clear controllerElement
controllerElement(1) = Simulink.BusElement;
controllerElement(1).Name = 'alpha';
controllerElement(1).Description = 'Ângulo da trajetória no ponto relevante, em rad';
controllerElement(1).DataType = 'single';
controllerElement(1).Dimensions = 1;

controllerElement(2) = Simulink.BusElement;
controllerElement(2).Name = 'e';
controllerElement(2).Description = 'Erro lateral do veículo em relação à trajetória no ponto relevante, em metros';
controllerElement(2).DataType = 'single';
controllerElement(2).Dimensions = 1;

controllerElement(3) = Simulink.BusElement;
controllerElement(3).Name = 'curvature';
controllerElement(3).Description = 'Curvatura da trajetória no ponto relevante, em 1/m';
controllerElement(3).DataType = 'single';
controllerElement(3).Dimensions = 1;

controllerElement(4) = Simulink.BusElement;
controllerElement(4).Name = 'psiError';
controllerElement(4).Description = 'Erro angular em radianos (psiRef - psi)';
controllerElement(4).DataType = 'single';
controllerElement(4).Dimensions = 1;


controllerInputGuidanceBus_t = Simulink.Bus;
controllerInputGuidanceBus_t.Elements = controllerElement;
controllerInputGuidanceBus_t.Description = 'Entrada de guiagem do modelo ''Controlador''';