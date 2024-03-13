classdef ControlInputSet<handle
    properties
        set ControlInput
        tSpan
    end
    methods
        function obj  = ControlInputSet()
            obj.set   = ControlInputSet.CreateDefaultControlSet();
            obj.tSpan = 0:0.05:0.5;
        end
    end
    methods(Static)
        function set = CreateDefaultControlSet()
            vel_m_s = 5;
            steeringAngle_mag_deg = 15;
            turnLeftSet  = ControlInput(vel_m_s ,-steeringAngle_mag_deg);
            straightSet  = ControlInput(vel_m_s ,0);
            turnRightSet = ControlInput(vel_m_s ,steeringAngle_mag_deg);
            % 
            % turnBackLeftSet  =  ControlInput(-vel_m_s,-steeringAngle_mag_deg);
            % straightBackSet  =  ControlInput(-vel_m_s,0);
            % turnRightBackSet =  ControlInput(-vel_m_s,steeringAngle_mag_deg);
            % set = [turnLeftSet,straightSet,turnRightSet,turnBackLeftSet,straightBackSet,turnRightBackSet];
            set = [turnLeftSet,straightSet,turnRightSet];
        end
    end
end

