classdef ControlInput<handle
    properties
        velocity_m_s
        steeringAngle_deg
    end
    methods
        function obj = ControlInput(velocity_m_s,steeringAngle_deg)
            obj.velocity_m_s      = velocity_m_s;
            obj.steeringAngle_deg = steeringAngle_deg;
        end
    end
end

