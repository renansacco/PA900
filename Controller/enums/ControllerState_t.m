classdef ControllerState_t < Simulink.IntEnumType
    % MATLAB enumeration class definition generated from template
    
    enumeration
        CONTROLLER_STATE_NONE(0),
		CONTROLLER_STATE_WAIT(1),
		CONTROLLER_STATE_IDLE(2),
		CONTROLLER_STATE_KEEP(3),
		CONTROLLER_STATE_ENTRY(4),
		CONTROLLER_STATE_CURVE(5)
    end

    methods (Static)
        
        function defaultValue = getDefaultValue()
            % GETDEFAULTVALUE  Returns the default enumerated value.
            %   If this method is not defined, the first enumeration is used.
            defaultValue = ControllerState_t.CONTROLLER_STATE_NONE;
        end

        function dScope = getDataScope()
            % GETDATASCOPE  Specifies whether the data type definition should be imported from,
            %               or exported to, a header file during code generation.
            dScope = 'Imported';
        end

        function desc = getDescription()
            % GETDESCRIPTION  Returns a description of the enumeration.
            desc = '';
        end
        
        function headerFile = getHeaderFile()
            % GETHEADERFILE  Specifies the name of a header file. 
            headerFile = 'mg900_model_types.h';
        end
        
        function flag = addClassNameToEnumNames()
            % ADDCLASSNAMETOENUMNAMES  Indicate whether code generator applies the class name as a prefix
            %                          to the enumeration.
            flag = false;
        end

    end

end
