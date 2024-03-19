clear all
clc

%% User Inputs
machTrimList = 0.1:0.05:0.8;
selectedAltitude_m = 500;

%% Loading Data
dataBase        = load('f16_AerodynamicData.mat');
dataBase.Engine = load('f16_EngineData.mat');
dataBase.MAC    = 3.45;     % m
dataBase.b      = 9.144;    % m
dataBase.Sw     = 27.87;    % m^2
dataBase.CGref  = 0.35;
dataBase.CG     = 0.35;
trimOption      = 1;

%% Trim Different Mach's
for i = 1:length(machTrimList)
    selectedMach = machTrimList(i);
    states = [0 0 0 0 0 0 0 0 0 500 250 0];
    prompt = {'Mach:','Altitude(m):','Path Angle(deg):','Bank Angle(deg):','Simulation Stop Time(s):'};
    title = 'Trim Configuration';
    states(9)  = selectedMach;
    states(12) = selectedAltitude_m;
    gamma      = 0;
    states(4)  = 0;
    stopTime   = 10;
    n          = 1;

    [controlDeflections, statesFinal, deltaLEF, Velocity,costResults] = trimOptimization_for_Autopilot(states, dataBase, trimOption, gamma, n);

    trimResults{i,1} = selectedMach;
    trimResults{i,2} = costResults;
    if costResults<1e-7
        status = 'Successful';
    else
        status = 'Failure';
    end
    trimResults{i,3} = status;
    trimResults{i,4} = controlDeflections;
    trimResults{i,5} = statesFinal;
    trimResults{i,6} = deltaLEF;
    trimResults{i,7} = Velocity;
end
varNames = {'Mach'
    'costResults'
    'status'
    'controlDeflections'
    'statesFinal'
    'deltaLEF'
    'Velocity'};

trimResultTable = cell2table(trimResults,'VariableNames',varNames);


feedForwardTables.Thrust.machBP        = trimResultTable.Mach; % BP: Break Point
feedForwardTables.Thrust.thrustTable    = trimResultTable.controlDeflections(:,4);
