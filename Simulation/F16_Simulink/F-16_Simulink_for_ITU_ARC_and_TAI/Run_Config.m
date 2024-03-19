%% Path
addpath(genpath(cd))

%% User Inputs  
deltaTime = 0.01; %Simulation Step Size
calculateTrim = false;
trimDataFileName = "08M_500m_0degGamma_Trim";
load("Trim_Result_Table_Mach"); 
Autopilot_Settings;
Servo_Limits;
UDP_Settings;

%% Trim Data
if calculateTrim 
    trimRoutine
else
    load(trimDataFileName);
end

%% load Terrain data
load('lateral_pd_controllers.mat')
load('optimal_pi_structure_tecs_gains.mat')
statesFinal(5) = pi/2;
statesFinal(end-1) = -2000;
statesFinal(end) = 260;



