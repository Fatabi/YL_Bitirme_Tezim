function [statesUpdated, statesDot, loadFactor, deltaLEF, Velocity] = aircraftModel(states, controlInputs, dataBase, trimOption)

Alpha = states(1);
Beta = states(2);
Theta = states(3);
Phi = states(4);
Psi = states(5);
P = states(6);
Q = states(7);
R = states(8);
Mach = states(9);
North = states(10);
East = states(11);
Altitude = states(12);

%% Atmospheric Model

g = 9.80665*(6371020/(6371020+Altitude))^2;
Temperature = 288.15 - 6.5*Altitude/1000;
airPressure = 101325*(Temperature/288.15)^(g/1.86584);
rho = airPressure/(287.053*Temperature);
k = 1.4;
R_c = 287.06;
Velocity = Mach*sqrt(k*R_c*Temperature);
Qdyn = 0.5*rho*Velocity^2;

%% Control Inputs

deltaHT = controlInputs(1);
deltaAil = controlInputs(2);
deltaRud = controlInputs(3);
Throttle = controlInputs(4);
deltaLEF = 1.38*rad2deg(Alpha) - 9.05*Qdyn/airPressure + 1.45;
deltaSB = 0;

if deltaLEF < 0
    deltaLEF = 0;
elseif deltaLEF > 25
    deltaLEF = 25;
else
    deltaLEF = deltaLEF;
end

%% Geometric Parameters

MAC = dataBase.MAC; % m
b = dataBase.b; % m
Sw = dataBase.Sw; % m^2
CGref = dataBase.CGref;  
CG = dataBase.CG; 
Mass = 91188/9.8065; % kg
Ixx = 12875; % kgm^2
Iyy = 75674; % kgm^2
Izz = 85552; % kgm^2
Ixz = 1331; % kgm^2

%% X-Axis Force Coefficient Interpolation

CX = interpn(dataBase.f16_AerodynamicData.BP.CX.CX.alpha, dataBase.f16_AerodynamicData.BP.CX.CX.beta, ...
            dataBase.f16_AerodynamicData.BP.CX.CX.dHT, dataBase.f16_AerodynamicData.data.CX.CX(1:end,1:end,1:end),...
            rad2deg(Alpha), rad2deg(Beta), deltaHT, 'linear');
        
CX_HT0 = interpn(dataBase.f16_AerodynamicData.BP.CX.CX.alpha, dataBase.f16_AerodynamicData.BP.CX.CX.beta, ...
            dataBase.f16_AerodynamicData.BP.CX.CX.dHT, dataBase.f16_AerodynamicData.data.CX.CX(1:end,1:end,1:end),...
            rad2deg(Alpha), rad2deg(Beta), 0, 'linear');

CX_LEF = interpn(dataBase.f16_AerodynamicData.BP.CX.CXlef.alpha, dataBase.f16_AerodynamicData.BP.CX.CXlef.beta, ...
                dataBase.f16_AerodynamicData.data.CX.CXlef, rad2deg(Alpha), rad2deg(Beta), 'linear');
            
CX_SB = interp1(dataBase.f16_AerodynamicData.BP.CX.deltaCXSB.alpha,dataBase.f16_AerodynamicData.data.CX.deltaCXSB, rad2deg(Alpha), 'linear', 'extrap');

CX_q = interp1(dataBase.f16_AerodynamicData.BP.CX.CXq.alpha,dataBase.f16_AerodynamicData.data.CX.CXq, rad2deg(Alpha), 'linear', 'extrap');

CX_q_LEF = interp1(dataBase.f16_AerodynamicData.BP.CX.deltaCXqlef.alpha,dataBase.f16_AerodynamicData.data.CX.deltaCXqlef, rad2deg(Alpha), 'linear', 'extrap');

coeffs.CX_total = CX + (CX_LEF - CX_HT0)*(1-deltaLEF/25) + CX_SB*deltaSB/60 + Q*MAC*(CX_q + CX_q_LEF*(1-deltaLEF/25))/(2*Velocity);


%% Y-Axis Force Coefficient Interpolation

CY = interpn(dataBase.f16_AerodynamicData.BP.CY.CY.alpha, dataBase.f16_AerodynamicData.BP.CY.CY.beta, dataBase.f16_AerodynamicData.data.CY.CY(1:end,1:end),...
            rad2deg(Alpha), rad2deg(Beta), 'linear');
      
CY_da20lef = interpn(dataBase.f16_AerodynamicData.BP.CY.CYda20lef.alpha, dataBase.f16_AerodynamicData.BP.CY.CYda20lef.beta, ...
           dataBase.f16_AerodynamicData.data.CY.CYda20lef(1:end,1:end),...
            rad2deg(Alpha), rad2deg(Beta), 'linear');
        
CY_da20 = interpn(dataBase.f16_AerodynamicData.BP.CY.CYda20.alpha, dataBase.f16_AerodynamicData.BP.CY.CYda20.beta, ...
           dataBase.f16_AerodynamicData.data.CY.CYda20(1:end,1:end),...
            rad2deg(Alpha), rad2deg(Beta), 'linear');

CY_dr30 = interpn(dataBase.f16_AerodynamicData.BP.CY.CYdr30.alpha, dataBase.f16_AerodynamicData.BP.CY.CYdr30.beta, ...
           dataBase.f16_AerodynamicData.data.CY.CYdr30(1:end,1:end),...
            rad2deg(Alpha), rad2deg(Beta), 'linear');
        
CYr = interp1(dataBase.f16_AerodynamicData.BP.CY.CYr.alpha,dataBase.f16_AerodynamicData.data.CY.CYr(1:end),...
            rad2deg(Alpha), 'linear');
        
CYp = interp1(dataBase.f16_AerodynamicData.BP.CY.CYp.alpha,dataBase.f16_AerodynamicData.data.CY.CYp(1:end),...
            rad2deg(Alpha), 'linear');
        
deltaCYr_LEF = interp1(dataBase.f16_AerodynamicData.BP.CY.deltaCYrlef.alpha,dataBase.f16_AerodynamicData.data.CY.deltaCYrlef(1:end),...
            rad2deg(Alpha), 'linear');
        
deltaCYp_LEF = interp1(dataBase.f16_AerodynamicData.BP.CY.deltaCYplef.alpha,dataBase.f16_AerodynamicData.data.CY.deltaCYplef(1:end),...
            rad2deg(Alpha), 'linear');

CY_LEF = interpn(dataBase.f16_AerodynamicData.BP.CY.CYlef.alpha, dataBase.f16_AerodynamicData.BP.CY.CYlef.beta, ...
                dataBase.f16_AerodynamicData.data.CY.CYlef, rad2deg(Alpha), rad2deg(Beta), 'linear');

coeffs.CY_total = CY + (CY_LEF - CY)*(1- deltaLEF/25) + (CY_da20 - CY + (CY_da20lef - CY_LEF - CY_da20 + CY)*(1- deltaLEF/25))*deltaAil/20 +...
 (CY_dr30 - CY)*(deltaRud/30) + b/(2*Velocity)*...
 ((CYr + deltaCYr_LEF*(1- deltaLEF/25))*R + (CYp + deltaCYp_LEF*(1- deltaLEF/25))*P);


%% Z-Axis Force Coefficient Interpolation

CZ = interpn(dataBase.f16_AerodynamicData.BP.CZ.CZ.alpha, dataBase.f16_AerodynamicData.BP.CZ.CZ.beta, ...
            dataBase.f16_AerodynamicData.BP.CZ.CZ.dHT, dataBase.f16_AerodynamicData.data.CZ.CZ(1:end,1:end,1:end),...
            rad2deg(Alpha), rad2deg(Beta), deltaHT, 'linear');
        
CZ_HT0 = interpn(dataBase.f16_AerodynamicData.BP.CZ.CZ.alpha, dataBase.f16_AerodynamicData.BP.CZ.CZ.beta, ...
            dataBase.f16_AerodynamicData.BP.CZ.CZ.dHT, dataBase.f16_AerodynamicData.data.CZ.CZ(1:end,1:end,1:end),...
            rad2deg(Alpha), rad2deg(Beta), 0, 'linear');

CZ_LEF = interpn(dataBase.f16_AerodynamicData.BP.CZ.CZlef.alpha, dataBase.f16_AerodynamicData.BP.CZ.CZlef.beta, ...
                dataBase.f16_AerodynamicData.data.CZ.CZlef, rad2deg(Alpha), rad2deg(Beta), 'linear');
            
deltaCZ_SB = interp1(dataBase.f16_AerodynamicData.BP.CZ.deltaCZSB.alpha,dataBase.f16_AerodynamicData.data.CZ.deltaCZSB, rad2deg(Alpha), 'linear', 'extrap');

CZ_q = interp1(dataBase.f16_AerodynamicData.BP.CZ.CZq.alpha,dataBase.f16_AerodynamicData.data.CZ.CZq, rad2deg(Alpha), 'linear', 'extrap');

deltaCZ_q_LEF = interp1(dataBase.f16_AerodynamicData.BP.CZ.deltaCZqlef.alpha,dataBase.f16_AerodynamicData.data.CZ.deltaCZqlef, rad2deg(Alpha), 'linear', 'extrap');

coeffs.CZ_total = CZ + (CZ_LEF - CZ_HT0)*(1- deltaLEF/25) + deltaCZ_SB*deltaSB/60 + Q*MAC*(CZ_q + deltaCZ_q_LEF*(1-deltaLEF/25))/(2*Velocity);


%% Pitching Moment Coefficient Interpolation

htEfficiency = interp1(dataBase.f16_AerodynamicData.BP.Cm.n_dHT.dHT,dataBase.f16_AerodynamicData.data.Cm.n_dHT, deltaHT, 'linear');

Cm = interpn(dataBase.f16_AerodynamicData.BP.Cm.Cm.alpha, dataBase.f16_AerodynamicData.BP.Cm.Cm.beta, ...
            dataBase.f16_AerodynamicData.BP.Cm.Cm.dHT, dataBase.f16_AerodynamicData.data.Cm.Cm(1:end,1:end,1:end),...
            rad2deg(Alpha), rad2deg(Beta), deltaHT, 'linear');

Cm_HT0 = interpn(dataBase.f16_AerodynamicData.BP.Cm.Cm.alpha, dataBase.f16_AerodynamicData.BP.Cm.Cm.beta, ...
            dataBase.f16_AerodynamicData.BP.Cm.Cm.dHT, dataBase.f16_AerodynamicData.data.Cm.Cm(1:end,1:end,1:end),...
            rad2deg(Alpha), rad2deg(Beta), 0, 'linear'); 
        
Cm_LEF = interpn(dataBase.f16_AerodynamicData.BP.Cm.Cmlef.alpha, dataBase.f16_AerodynamicData.BP.Cm.Cmlef.beta, ...
                dataBase.f16_AerodynamicData.data.Cm.Cmlef, rad2deg(Alpha), rad2deg(Beta), 'linear');
        
deltaCm_SB = interp1(dataBase.f16_AerodynamicData.BP.Cm.deltaCmSB.alpha,dataBase.f16_AerodynamicData.data.Cm.deltaCmSB, rad2deg(Alpha), 'linear', 'extrap');

Cm_q = interp1(dataBase.f16_AerodynamicData.BP.Cm.Cmq.alpha,dataBase.f16_AerodynamicData.data.Cm.Cmq, rad2deg(Alpha), 'linear', 'extrap');

deltaCm_q_LEF = interp1(dataBase.f16_AerodynamicData.BP.Cm.deltaCmqlef.alpha,dataBase.f16_AerodynamicData.data.Cm.deltaCmqlef, rad2deg(Alpha), 'linear', 'extrap');

deltaCm = interp1(dataBase.f16_AerodynamicData.BP.Cm.deltaCm.alpha,dataBase.f16_AerodynamicData.data.Cm.deltaCm, rad2deg(Alpha), 'linear', 'extrap');

deltaCm_DS = interpn(dataBase.f16_AerodynamicData.BP.Cm.deltaCmds.alpha, dataBase.f16_AerodynamicData.BP.Cm.deltaCmds.dHT, ...
                dataBase.f16_AerodynamicData.data.Cm.deltaCmds, rad2deg(Alpha), deltaHT, 'linear');
            
coeffs.Cm_total = Cm*htEfficiency + coeffs.CZ_total*(CGref-CG) + (Cm_LEF - Cm_HT0)*(1-deltaLEF/25) + deltaCm_SB*deltaSB/60 + ...
    MAC*Q*(Cm_q + deltaCm_q_LEF*(1-deltaLEF/25))/(2*Velocity) + deltaCm + deltaCm_DS;     


%% Yawing Moment Coefficient Interpolation
 
Cn = interpn(dataBase.f16_AerodynamicData.BP.Cn.Cn.alpha, dataBase.f16_AerodynamicData.BP.Cn.Cn.beta, ...
            dataBase.f16_AerodynamicData.BP.Cn.Cn.dHT, dataBase.f16_AerodynamicData.data.Cn.Cn(1:end,1:end,1:end),...
            rad2deg(Alpha), rad2deg(Beta), deltaHT, 'linear');
        
Cn_HT0 = interpn(dataBase.f16_AerodynamicData.BP.Cn.Cn.alpha, dataBase.f16_AerodynamicData.BP.Cn.Cn.beta, ...
            dataBase.f16_AerodynamicData.BP.Cn.Cn.dHT, dataBase.f16_AerodynamicData.data.Cn.Cn(1:end,1:end,1:end),...
            rad2deg(Alpha), rad2deg(Beta), 0, 'linear');
        
Cn_LEF = interpn(dataBase.f16_AerodynamicData.BP.Cn.Cnlef.alpha, dataBase.f16_AerodynamicData.BP.Cn.Cnlef.beta, ...
                dataBase.f16_AerodynamicData.data.Cn.Cnlef, rad2deg(Alpha), rad2deg(Beta), 'linear');
        
Cn_da20 = interpn(dataBase.f16_AerodynamicData.BP.Cn.Cnda20.alpha, dataBase.f16_AerodynamicData.BP.Cn.Cnda20.beta, ...
                dataBase.f16_AerodynamicData.data.Cn.Cnda20, rad2deg(Alpha), rad2deg(Beta), 'linear');

Cn_dr30 = interpn(dataBase.f16_AerodynamicData.BP.Cn.Cndr30.alpha, dataBase.f16_AerodynamicData.BP.Cn.Cndr30.beta, ...
                dataBase.f16_AerodynamicData.data.Cn.Cndr30, rad2deg(Alpha), rad2deg(Beta), 'linear');
            
Cn_da20_LEF = interpn(dataBase.f16_AerodynamicData.BP.Cn.Cnda20lef.alpha, dataBase.f16_AerodynamicData.BP.Cn.Cnda20lef.beta, ...
                dataBase.f16_AerodynamicData.data.Cn.Cnda20lef, rad2deg(Alpha), rad2deg(Beta), 'linear');
            
Cn_r = interp1(dataBase.f16_AerodynamicData.BP.Cn.Cnr.alpha,dataBase.f16_AerodynamicData.data.Cn.Cnr, rad2deg(Alpha), 'linear', 'extrap');

deltaCn_r_LEF = interp1(dataBase.f16_AerodynamicData.BP.Cn.deltaCnrlef.alpha,dataBase.f16_AerodynamicData.data.Cn.deltaCnrlef, rad2deg(Alpha), 'linear', 'extrap');

Cn_p = interp1(dataBase.f16_AerodynamicData.BP.Cn.Cnp.alpha,dataBase.f16_AerodynamicData.data.Cn.Cnp, rad2deg(Alpha), 'linear', 'extrap');

deltaCn_p_LEF = interp1(dataBase.f16_AerodynamicData.BP.Cn.deltaCnplef.alpha,dataBase.f16_AerodynamicData.data.Cn.deltaCnplef, rad2deg(Alpha), 'linear', 'extrap');

deltaCn_beta = interp1(dataBase.f16_AerodynamicData.BP.Cn.deltaCnBeta.alpha,dataBase.f16_AerodynamicData.data.Cn.deltaCnBeta, rad2deg(Alpha), 'linear', 'extrap');

deltaCn_LEF = Cn_LEF - Cn_HT0;

deltaCn_da20 = Cn_da20 - Cn_HT0;

deltaCn_dr30 = Cn_dr30 - Cn_HT0;

deltaCn_da20_LEF = Cn_da20_LEF - Cn_LEF - deltaCn_da20;

coeffs.Cn_total = Cn + (deltaAil/20)*(deltaCn_da20 + deltaCn_da20_LEF*(1-deltaLEF/25)) - coeffs.CY_total*(CGref-CG)*MAC/b ...
    + deltaCn_dr30*deltaRud/30 + deltaCn_LEF*(1-deltaLEF/25) + R*b*(Cn_r + deltaCn_r_LEF*(1-deltaLEF/25))/(2*Velocity)...
    + P*b*(Cn_p + deltaCn_p_LEF*(1-deltaLEF/25))/(2*Velocity) + deltaCn_beta*rad2deg(Beta);


%% Rolling Moment Coefficient Interpolation

Cl = interpn(dataBase.f16_AerodynamicData.BP.Cl.Cl.alpha, dataBase.f16_AerodynamicData.BP.Cl.Cl.beta, ...
            dataBase.f16_AerodynamicData.BP.Cl.Cl.dHT, dataBase.f16_AerodynamicData.data.Cl.Cl(1:end,1:end,1:end),...
            rad2deg(Alpha), rad2deg(Beta), deltaHT, 'linear');

Cl_HT0 = interpn(dataBase.f16_AerodynamicData.BP.Cl.Cl.alpha, dataBase.f16_AerodynamicData.BP.Cl.Cl.beta, ...
            dataBase.f16_AerodynamicData.BP.Cl.Cl.dHT, dataBase.f16_AerodynamicData.data.Cl.Cl(1:end,1:end,1:end),...
            rad2deg(Alpha), rad2deg(Beta), 0, 'linear');
   
Cl_LEF = interpn(dataBase.f16_AerodynamicData.BP.Cl.Cllef.alpha, dataBase.f16_AerodynamicData.BP.Cl.Cllef.beta, ...
                dataBase.f16_AerodynamicData.data.Cl.Cllef, rad2deg(Alpha), rad2deg(Beta), 'linear');
        
Cl_da20 = interpn(dataBase.f16_AerodynamicData.BP.Cl.Clda20.alpha, dataBase.f16_AerodynamicData.BP.Cl.Clda20.beta, ...
                dataBase.f16_AerodynamicData.data.Cl.Clda20, rad2deg(Alpha), rad2deg(Beta), 'linear');

Cl_dr30 = interpn(dataBase.f16_AerodynamicData.BP.Cl.Cldr30.alpha, dataBase.f16_AerodynamicData.BP.Cl.Cldr30.beta, ...
                dataBase.f16_AerodynamicData.data.Cl.Cldr30, rad2deg(Alpha), rad2deg(Beta), 'linear');
   
Cl_da20_LEF = interpn(dataBase.f16_AerodynamicData.BP.Cl.Clda20lef.alpha, dataBase.f16_AerodynamicData.BP.Cl.Clda20lef.beta, ...
                dataBase.f16_AerodynamicData.data.Cl.Clda20lef, rad2deg(Alpha), rad2deg(Beta), 'linear');

Cl_r = interp1(dataBase.f16_AerodynamicData.BP.Cl.Clr.alpha,dataBase.f16_AerodynamicData.data.Cl.Clr, rad2deg(Alpha), 'linear', 'extrap');

deltaCl_r_LEF = interp1(dataBase.f16_AerodynamicData.BP.Cl.deltaClrlef.alpha,dataBase.f16_AerodynamicData.data.Cl.deltaClrlef, rad2deg(Alpha), 'linear', 'extrap');

Cl_p = interp1(dataBase.f16_AerodynamicData.BP.Cl.Clp.alpha,dataBase.f16_AerodynamicData.data.Cl.Clp, rad2deg(Alpha), 'linear', 'extrap');

deltaCl_p_LEF = interp1(dataBase.f16_AerodynamicData.BP.Cl.deltaClplef.alpha,dataBase.f16_AerodynamicData.data.Cl.deltaClplef, rad2deg(Alpha), 'linear', 'extrap');

deltaCl_beta = interp1(dataBase.f16_AerodynamicData.BP.Cl.deltaClBeta.alpha,dataBase.f16_AerodynamicData.data.Cl.deltaClBeta, rad2deg(Alpha), 'linear', 'extrap');

deltaCl_LEF = Cl_LEF - Cl_HT0;

deltaCl_da20 = Cl_da20 - Cl_HT0;

deltaCl_dr30 = Cl_dr30 - Cl_HT0;

deltaCl_da20_LEF = Cl_da20_LEF - Cl_LEF - deltaCl_da20;

coeffs.Cl_total = Cl + (deltaAil/20)*(deltaCl_da20 + deltaCl_da20_LEF*(1-deltaLEF/25)) + deltaCl_dr30*deltaRud/30 ...
    + deltaCl_LEF*(1-deltaLEF/25) + R*b*(Cl_r + deltaCl_r_LEF*(1-deltaLEF/25))/(2*Velocity)...
    + P*b*(Cl_p + deltaCl_p_LEF*(1-deltaLEF/25))/(2*Velocity) + deltaCl_beta*rad2deg(Beta);

%% Propulsion Model

He = 216.9; % kgm^2/s

Tidle = interpn(dataBase.Engine.f16_EngineData.BP.idle.mach,dataBase.Engine.f16_EngineData.BP.idle.alt_ft,...
                dataBase.Engine.f16_EngineData.data.Thrust_lbs.idle, Mach, Altitude/0.3048, 'linear')*4.4482; % N
               
Tmil = interpn(dataBase.Engine.f16_EngineData.BP.mil.mach,dataBase.Engine.f16_EngineData.BP.mil.alt_ft,...
                dataBase.Engine.f16_EngineData.data.Thrust_lbs.mil, Mach, Altitude/0.3048, 'linear')*4.4482; % N

Tmax = interpn(dataBase.Engine.f16_EngineData.BP.max.mach,dataBase.Engine.f16_EngineData.BP.max.alt_ft,...
                dataBase.Engine.f16_EngineData.data.Thrust_lbs.max, Mach, Altitude/0.3048, 'linear')*4.4482; % N
            
if Throttle <= 77
    Pc = 64.94*Throttle/100;
elseif Throttle > 77
    Pc = 217.38*Throttle/100 - 117.38;
end

if Pc < 50
    Thrust = Tidle + (Tmil - Tidle)*Pc/50;
elseif 50 <= Pc
    Thrust = Tmil + (Tmax - Tmil)*(Pc-50)/50;
end

%% Forces and Moments in Body Frame

Forces_BF(1) = coeffs.CX_total*Qdyn*Sw; % X-axis Force
Forces_BF(2) = coeffs.CY_total*Qdyn*Sw; % Y-axis Force
Forces_BF(3) = coeffs.CZ_total*Qdyn*Sw; % Z-axis Force

Moments(1) = coeffs.Cm_total*Qdyn*Sw*MAC - He*R; % Pitching Moment
Moments(2) = coeffs.Cn_total*Qdyn*Sw*b + He*Q; % Yawing Moment    
Moments(3) = coeffs.Cl_total*Qdyn*Sw*b; % Rolling Moment    

%% Forces in Wind Frame

transFromBtoW = [cos(Beta)*cos(Alpha) sin(Beta) cos(Beta)*sin(Alpha);...
                 -sin(Beta)*cos(Alpha) cos(Beta) -sin(Beta)*sin(Alpha);...
                 -sin(Alpha) 0 cos(Alpha)];
        
forceCoeffsBody = [coeffs.CX_total; coeffs.CY_total; coeffs.CZ_total];
forceCoeffsWind = transFromBtoW*forceCoeffsBody;

CD = -forceCoeffsWind(1);
Cside = forceCoeffsWind(2);
CL = -forceCoeffsWind(3);

L = CL*Qdyn*Sw;
D = CD*Qdyn*Sw;
Y = Cside*Qdyn*Sw;

Nx = -Forces_BF(1)/(Mass*9.8065);
Ny = Forces_BF(2)/(Mass*9.8065);
Nz = -Forces_BF(3)/(Mass*9.8065);

loadFactor = [Nx, Ny, Nz];

%% Equations of Motion

U = Velocity*cos(Alpha)*cos(Beta);
V = Velocity*sin(Beta);
W = Velocity*sin(Alpha)*cos(Beta);

% Translational Eq.s of Motion

UDot = R*V - Q*W - g*sin(Theta) + Forces_BF(1)/Mass + Thrust/Mass;
VDot = P*W - R*U + g*cos(Theta)*sin(Phi) + Forces_BF(2)/Mass;
WDot = Q*U - P*V + g*cos(Theta)*cos(Phi) + Forces_BF(3)/Mass;

% Rotational Eq.s of Motion

F = Ixx*Izz - Ixz^2;
PDot = (Ixz*(Ixx-Iyy+Izz)*P*Q - (Izz*(Izz-Iyy)+Ixz^2)*Q*R + Izz*Moments(3) + Ixz*Moments(2))/F;
QDot = ((Izz-Ixx)*P*R - Ixz*(R^2 - P^2) + Moments(1))/Iyy;
RDot = ((Ixx*(Ixx-Iyy)+Ixz^2)*P*Q - Ixz*(Ixx-Iyy+Izz)*Q*R + Ixz*Moments(3) + Ixx*Moments(2))/F;

% Kinematic Eq.s

ThetaDot = Q*cos(Phi) - R*sin(Phi);
PhiDot = P + tan(Theta)*(Q*sin(Phi) + R*cos(Phi));

if trimOption == 3 && trimOption == 4
    PsiDot = (g/Velocity)*tan(Phi);
else
    PsiDot = (Q*sin(Phi) + R*cos(Phi))/cos(Theta);
end

% Navigational Eq.s 

NorthDot = U*cos(Theta)*cos(Psi) + V*(-cos(Phi)*sin(Psi) + sin(Phi)*sin(Theta)*cos(Psi)) ...
                                        + W*(sin(Phi)*sin(Psi)+cos(Phi)*sin(Theta)*cos(Psi));
EastDot = U*cos(Theta)*sin(Psi) +  V*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Theta)*sin(Psi)) ...
                                     + W*(-sin(Phi)*cos(Psi) + cos(Phi)*sin(Theta)*sin(Psi));
AltitudeDot = Velocity*(cos(Alpha)*cos(Beta)*sin(Theta) - sin(Beta)*sin(Phi)*cos(Theta)...
                                            - sin(Alpha)*cos(Beta)*cos(Phi)*cos(Theta));

% Auxiliary

AlphaDot = (-L - Thrust*sin(Alpha))/(Mass*Velocity*cos(Beta)) + g*(cos(Theta)*cos(Phi)*cos(Alpha)...
    + sin(Theta)*sin(Alpha))/(Velocity*cos(Beta)) + Q - tan(Beta)*(P*cos(Alpha) + R*sin(Alpha));

BetaDot = (D*sin(Beta)+Y*cos(Beta) - Thrust*cos(Alpha)*sin(Beta))/(Mass*Velocity)...
    + g*(sin(Theta)*cos(Alpha)*sin(Beta) + cos(Theta)*sin(Phi)*cos(Beta)...
    - cos(Theta)*cos(Phi)*sin(Alpha)*sin(Beta))/Velocity + P*sin(Alpha) - R*cos(Alpha);

VelocityDot = (-D*cos(Beta) + Y*sin(Beta) + Thrust*cos(Alpha)*cos(Beta))/Mass + ...
                g*(-sin(Theta)*cos(Alpha)*cos(Beta) +...
                cos(Theta)*sin(Phi)*sin(Beta) + cos(Theta)*cos(Phi)*sin(Alpha)*cos(Beta));


statesDot = [AlphaDot, BetaDot, ThetaDot, PhiDot, PsiDot, PDot, QDot, RDot, VelocityDot, ...
                                                             NorthDot, EastDot, AltitudeDot];
statesUpdated =  [Alpha, Beta, Theta, Phi, Psi, P, Q, R, Velocity, North, East, Altitude] + statesDot;

end
