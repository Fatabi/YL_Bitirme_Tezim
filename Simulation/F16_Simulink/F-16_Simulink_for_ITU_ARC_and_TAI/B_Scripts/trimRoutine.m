clear all
clc

dataBase = load('f16_AerodynamicData.mat');
dataBase.Engine = load('f16_EngineData.mat');

trimOption = menu('F16 Trim Routine Selection',...
                    'Straight Flight',...
                    'Straight Flight (Special Cond.)',...
                    'Steady Turn (Phi Input)',...
                    'Steady Turn (N Input)',...
                    'Pull Up / Push Over',...
                    'Steady Sideslip Flight',...
                    'Steady Roll',...
                    'Rolling Pull Up / Rolling Push Over',...
                    'Exit');
                
switch trimOption
    case 1
        states = [0 0 0 0 0 0 0 0 0 0 0 0];
        prompt = {'Mach:','Altitude(m):','Path Angle(deg):','Bank Angle(deg):','Simulation Stop Time(s):'};
        title = 'Trim Configuration';
        answer = inputdlg(prompt,title);
        states(9)  = str2num(answer{1});
        states(12) = str2num(answer{2});
        gamma      = deg2rad(str2num(answer{3}));
        states(4)  = deg2rad(str2num(answer{4}));
        stopTime   = str2num(answer{5});
        n          = 1;
    
    case 2
        states = [0 0 0 0 0 0 0 0 0 0 0 0];
        prompt = {'Mach:','Altitude(m):','Path Angle(deg):','Bank Angle(deg):','Simulation Stop Time(s):'};
        title = 'Trim Configuration';
        answer = inputdlg(prompt,title);
        states(9)  = str2num(answer{1});
        states(12) = str2num(answer{2});
        gamma      = deg2rad(str2num(answer{3}));
        states(4)  = deg2rad(str2num(answer{4}));
        stopTime   = str2num(answer{5});
        n          = 1;
   
    case 3
        states = [0 0 0 0 0 0 0 0 0 0 0 0];
        prompt = {'Mach:','Altitude(m):','Path Angle(deg):','Beta(deg):','Bank Angle(deg):','Simulation Stop Time(s):'};
        title = 'Trim Configuration';
        answer = inputdlg(prompt,title);
        states(9)  = str2num(answer{1});
        states(12) = str2num(answer{2});
        gamma      = deg2rad(str2num(answer{3}));
        states(2)  = deg2rad(str2num(answer{4}));
        states(4)  = deg2rad(str2num(answer{5})); 
        stopTime   = str2num(answer{6});        
        n          = 1/cos(states(4));
        
    case 4
        states = [0 0 0 0 0 0 0 0 0 0 0 0];
        prompt = {'Mach:','Altitude(m):','Path Angle(deg):','Beta(deg):','Load Factor:','Simulation Stop Time(s):'};
        title = 'Trim Configuration';
        answer = inputdlg(prompt,title);
        states(9)  = str2num(answer{1});
        states(12) = str2num(answer{2});
        gamma      = deg2rad(str2num(answer{3}));
        states(2)  = deg2rad(str2num(answer{4}));       
        n          = str2num(answer{5}); 
        stopTime   = str2num(answer{6});
        states(4)  = acos(1/n);
        
    case 5
        states = [0 0 0 0 0 0 0 0 0 0 0 0];
        prompt = {'Mach:','Altitude(m):','Path Angle(deg):','Bank Angle(deg):','Load Factor:','Simulation Stop Time(s):'};
        title = 'Trim Configuration';
        answer = inputdlg(prompt,title);
        states(9)  = str2num(answer{1});
        states(12) = str2num(answer{2});
        gamma      = deg2rad(str2num(answer{3}));
        states(4)  = deg2rad(str2num(answer{4}));
        n          = str2num(answer{5}); 
        stopTime   = str2num(answer{6});
        
    case 6
        states = [0 0 0 0 0 0 0 0 0 0 0 0];
        prompt = {'Mach:','Altitude(m):','Path Angle(deg):','Bank Angle(deg):','Sideslip Angle(deg)','Simulation Stop Time(s):'};
        title = 'Trim Configuration';
        answer = inputdlg(prompt,title);
        states(9)  = str2num(answer{1});
        states(12) = str2num(answer{2});
        gamma      = deg2rad(str2num(answer{3}));
        states(4)  = deg2rad(str2num(answer{4}));
        states(2)  = deg2rad(str2num(answer{5}));
        stopTime   = str2num(answer{6});
        n          = 1;
        
     case 7
        states = [0 0 0 0 0 0 0 0 0 0 0 0];
        prompt = {'Mach:','Altitude(m):','Path Angle(deg):','Roll Rate(deg/s):','Simulation Stop Time(s):'};
        title = 'Trim Configuration';
        answer = inputdlg(prompt,title);
        states(9)  = str2num(answer{1});
        states(12) = str2num(answer{2});
        gamma      = deg2rad(str2num(answer{3}));
        states(6)  = str2num(answer{4});
        stopTime   = str2num(answer{5});
        n          = 1;
        
     case 8
        states = [0 0 0 0 0 0 0 0 0 0 0 0];
        prompt = {'Mach:','Altitude(m):','Path Angle(deg):','Roll Rate(deg/s):','Load Factor:','Simulation Stop Time(s):'};
        title = 'Trim Configuration';
        answer = inputdlg(prompt,title);
        states(9)  = str2num(answer{1});
        states(12) = str2num(answer{2});
        gamma      = deg2rad(str2num(answer{3}));
        states(6)  = str2num(answer{4});
        stopTime   = str2num(answer{6});
        n          = str2num(answer{5});
        
    case 9
        msgbox('F16 could not be trimmed!')
end

dataBase.MAC = 3.45; % m
dataBase.b = 9.144;  % m
dataBase.Sw = 27.87; % m^2
dataBase.CGref = 0.35;
dataBase.CG = 0.35;

[controlDeflections, statesFinal, deltaLEF, Velocity] = trimOptimization(states, dataBase, trimOption, gamma, n);

runwayElevation = 80; % m
runwayCoordinates = [40.966 28.811]; % deg, deg

%%

% simOut = sim('C:\Users\ege98\OneDrive\Masaüstü\F16 - All\F16 - All\Simulink\F16_Model_v4',stopTime);
% figure
% trajectory3(out.North,-out.East,out.Altitude,out.Theta,out.Phi,-out.Psi,0.025,100,'gripen')
% 
% %%
% 
% figure
% plot(out.tout, out.loadsBody(:,3))
% 
% figure
% plot(out.tout, out.Alpha*57.3)
% 
% figure
% plot(out.tout, out.Velocity)

%%

% dos('runfg.bat &')
% 58.5 derece heading -- 05 runway at İstanbul Atatürk Airport

% İstanbul Atatürk Havaalanı / Pist 05
% runwayHeading = 59; % deg
% runwayElevation = 78; % m
% runwayCoordinates = [41.05 29.03];