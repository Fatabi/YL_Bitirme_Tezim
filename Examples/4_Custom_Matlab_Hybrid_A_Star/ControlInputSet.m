classdef ControlInputSet<handle
    properties
        set ControlInput
        tSpan
    end
    methods
        function obj  = ControlInputSet()
            obj.set   = ControlInputSet.CreateDefaultControlSet();
            obj.tSpan = 0:0.1:1;
        end
    end
    methods(Static)
        function set = CreateDefaultControlSet()
            turnLeftSet  = ControlInput(1,-30);
            straightSet  = ControlInput(1,0);
            turnRightSet = ControlInput(1,30);

            turnBackLeftSet  =  ControlInput(-1,-30);
            straightBackSet  =  ControlInput(-1,0);
            turnRightBackSet =  ControlInput(-1,30);
            set = [turnLeftSet,straightSet,turnRightSet,turnBackLeftSet,straightBackSet,turnRightBackSet];
        end
    end
end

