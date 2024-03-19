function [controlDeflections, statesFinal, deltaLEF, Velocity,costResults] = trimOptimization_for_Autopilot(states_, dataBase, trimOption, gamma, n)

if trimOption == 1
    
    A = [];
    b = [];
    Aeq = [];
    beq = [];
    lb = [-25,  -21.5, -30,   0, deg2rad(-20), deg2rad(states_(4)), deg2rad(0)]; % HT - Aileron - Rudder - Throttle - Theta - Phi - Psi
    ub = [ 25,   21.5,  30, 100,  deg2rad(45),  deg2rad(states_(4)),  deg2rad(0)]; 
    Initialize = [-2 0 0 75 1 states_(4) 0];

    costF = @(u) StraightFlight(states_, u, dataBase);
    options = optimoptions(@fmincon,'Algorithm','sqp','TolX',1e-60);
    options.MaxFunctionEvaluations = 2000;
    options.OptimalityTolerance = 1.00e-60;
    options.MaxIterations = 1000;
    options.ConstraintTolerance = 1.00e-30;
    options.Display = 'iter';

    [controlDeflections,costResults] = fmincon(costF, Initialize, A, b, Aeq, beq, lb, ub, [], options);
    statesFinal = [controlDeflections(5) - gamma, states_(2), controlDeflections(5), controlDeflections(6),...
                    controlDeflections(7), states_(6), states_(7), states_(8), Velocity,...
                    states_(10), states_(11), states_(12)];

elseif trimOption == 2
    
    A = [];
    b = [];
    Aeq = [];
    beq = [];
    lb = [-25,  -21.5, -30,   0, deg2rad(-20), deg2rad(0)]; % HT - Aileron - Rudder - Throttle - AoA - SS
    ub = [ 25,   21.5,  30, 100,  deg2rad(45),  deg2rad(0)]; 
    Initialize = [0 0 0 75 0.05 0];

    costF = @(u) StraightFlight2(states_, u, dataBase);
    options = optimoptions(@fmincon,'Algorithm','sqp','TolX',1e-60);
    options.MaxFunctionEvaluations = 2000;
    options.OptimalityTolerance = 1.00e-60;
    options.MaxIterations = 1000;
    options.ConstraintTolerance = 1.00e-30;

    controlDeflections = fmincon(costF, Initialize, A, b, Aeq, beq, lb, ub, [], options);
    statesFinal = [controlDeflections(5), controlDeflections(6), controlDeflections(5) + gamma, states_(4),...
                    states_(5), states_(6), states_(7), states_(8), Velocity,...
                    states_(10), states_(11), states_(12)];
        
elseif trimOption == 3
    
    A = [];
    b = [];
    Aeq = [];
    beq = [];
    lb = [-25, -21.5, -30,   0, deg2rad(-20), deg2rad(-20), states_(4)]; % HT - Aileron - Rudder - Throttle - Alpha - Theta - Phi
    ub = [ 25,  21.5,  30, 100,  deg2rad(45),  deg2rad(45),  states_(4)]; 
    Initialize = [-10 5 -5 75 0.02 0 states_(4)];
    
    costF = @(u) SteadyTurnCase1(states_, u, dataBase);
    options = optimoptions(@fmincon,'Algorithm','sqp','TolX',1e-60);
    options.MaxFunctionEvaluations = 2000;
    options.OptimalityTolerance = 1.00e-60;
    options.MaxIterations = 1000; 
    options.ConstraintTolerance = 1.00e-30;
    
    g = 9.80665*(6371020/(6371020 + states_(12)))^2;
    Temperature = 288.15 - 6.5*states_(12)/1000;
    k = 1.4;
    R_c = 287.06;
    Velocity = states_(9)*sqrt(k*R_c*Temperature);
    PsiDot = (g/Velocity)*tan(states_(4));
    
    controlDeflections = fmincon(costF, Initialize, A, b, Aeq, beq, lb, ub, [], options);
    
    statesFinal = [controlDeflections(5), states_(2), controlDeflections(6),...
                   states_(4), states_(5), states_(6), controlDeflections(7),...
                   states_(8), Velocity, states_(10), states_(11), states_(12)];
               
    statesFinal(6) = -PsiDot*sin(controlDeflections(6));                % P
    statesFinal(7) = PsiDot*cos(controlDeflections(6))*sin(states_(4)); % Q
    statesFinal(8) = PsiDot*cos(controlDeflections(6))*cos(states_(4)); % R
    
elseif trimOption == 4
    
    A = [];
    b = [];
    Aeq = [];
    beq = [];
    lb = [-25, -21.5, -30,   0, deg2rad(-20), deg2rad(-20), states_(4)]; % HT - Aileron - Rudder - Throttle - Alpha - Theta - Phi 
    ub = [ 25,  21.5,  30, 100,  deg2rad(45),  deg2rad(45), states_(4)]; 
    Initialize = [-10 -5 5 75 0.02 0.02 states_(4)];
    
    costF = @(u) SteadyTurnCase2(states_, u, dataBase);
    options = optimoptions(@fmincon,'Algorithm','sqp','TolX',1e-60);
    options.MaxFunctionEvaluations = 2000;
    options.OptimalityTolerance = 1.00e-60;
    options.MaxIterations = 1000; 
    options.ConstraintTolerance = 1.00e-30;
    
    g = 9.80665*(6371020/(6371020 + states_(12)))^2;
    Temperature = 288.15 - 6.5*states_(12)/1000;
    k = 1.4;
    R_c = 287.06;
    Velocity = states_(9)*sqrt(k*R_c*Temperature);
    PsiDot = (g/Velocity)*tan(states_(4));
   
    controlDeflections = fmincon(costF, Initialize, A, b, Aeq, beq, lb, ub, [], options);
    
    statesFinal = [controlDeflections(5), states_(2), controlDeflections(6),...
                   states_(4), states_(5), controlDeflections(7), states_(7),...
                   states_(8), Velocity, states_(10), states_(11), states_(12)];
               
    statesFinal(6) = -PsiDot*sin(controlDeflections(6));                % P
    statesFinal(7) = PsiDot*cos(controlDeflections(6))*sin(states_(4)); % Q
    statesFinal(8) = PsiDot*cos(controlDeflections(6))*cos(states_(4)); % R
    
elseif trimOption == 5
    
    A = [];
    b = [];
    Aeq = [];
    beq = [];
    lb = [-25,  -21.5, -30,   0, deg2rad(-20), deg2rad(states_(5)), deg2rad(0), -deg2rad(60)]; % HT - Aileron - Rudder - Throttle - Theta - Beta - Psi - Pitch Rate
    ub = [ 25,   21.5,  30, 100,  deg2rad(45), deg2rad(states_(5)), deg2rad(0),  deg2rad(60)]; 
    Initialize = [-2 0 0 75 0.1 states_(2) states_(5) 0.087];

    costF = @(u) PullUp_PushOver(states_, u, dataBase);
    options = optimoptions(@fmincon,'Algorithm','sqp','TolX',1e-60);
    options.MaxFunctionEvaluations = 2000;
    options.OptimalityTolerance = 1.00e-60;
    options.MaxIterations = 1000;
    options.ConstraintTolerance = 1.00e-30;

    controlDeflections = fmincon(costF, Initialize, A, b, Aeq, beq, lb, ub, [], options);
    statesFinal = [controlDeflections(5) - gamma, controlDeflections(6), controlDeflections(5), states_(4),...
                    states_(5), states_(6), controlDeflections(8), states_(8), Velocity,...
                    states_(10), states_(11), states_(12)];
                
elseif trimOption == 6
    
    A = [];
    b = [];
    Aeq = [];
    beq = [];
    lb = [-25,  -21.5, -30,   0, deg2rad(-20), deg2rad(-90), deg2rad(-60), deg2rad(-20)]; % HT - Aileron - Rudder - Throttle - Theta - Phi - Psi - AoA
    ub = [ 25,   21.5,  30, 100,  deg2rad(45),  deg2rad(90),  deg2rad(60), deg2rad(45)]; 
    Initialize = [-2 -1 10 75 0 deg2rad(10) -deg2rad(states_(2)) 0];

    costF = @(u) SteadySideslip(states_, u, dataBase);
    options = optimoptions(@fmincon,'Algorithm','sqp','TolX',1e-60);
    options.MaxFunctionEvaluations = 3000;
    options.OptimalityTolerance = 1.00e-60;
    options.MaxIterations = 3000;
    options.ConstraintTolerance = 1.00e-30;

    controlDeflections = fmincon(costF, Initialize, A, b, Aeq, beq, lb, ub, [], options);
    statesFinal = [controlDeflections(8), states_(2), controlDeflections(5), controlDeflections(6),...
                    controlDeflections(7), states_(6), states_(7), states_(8), Velocity,...
                    states_(10), states_(11), states_(12)];
                
elseif trimOption == 7
    
    A = [];
    b = [];
    Aeq = [];
    beq = [];
    lb = [-25,  -21.5, -30,   0, deg2rad(-20), deg2rad(-20), deg2rad(states_(6))]; % HT - Aileron - Rudder - Throttle - Theta - Alpha - Roll Rate
    ub = [ 25,   21.5,  30, 100,  deg2rad(45),  deg2rad(45), deg2rad(states_(6))]; 
    Initialize = [-5 0 0 85 0.1 0.05 0.05 deg2rad(states_(6))];

    costF = @(u) SteadyRoll(states_, u, dataBase);
    options = optimoptions(@fmincon,'Algorithm','sqp','TolX',1e-60);
    options.MaxFunctionEvaluations = 2000;
    options.OptimalityTolerance = 1.00e-60;
    options.MaxIterations = 1000;
    options.ConstraintTolerance = 1.00e-30;

    controlDeflections = fmincon(costF, Initialize, A, b, Aeq, beq, lb, ub, [], options);
    statesFinal = [controlDeflections(6), 0, controlDeflections(5), states_(4),...
                    states_(5), controlDeflections(7), states_(7), states_(8), Velocity,...
                    states_(10), states_(11), states_(12)];

elseif trimOption == 8
    
    A = [];
    b = [];
    Aeq = [];
    beq = [];
    lb = [-25,  -21.5, -30,   0, deg2rad(-20), deg2rad(states_(6)), deg2rad(-60)]; % HT - Aileron - Rudder - Throttle - Alpha - Roll Rate - Pitch Rate
    ub = [ 25,   21.5,  30, 100,  deg2rad(45), deg2rad(states_(6)),  deg2rad(60)]; 
    Initialize = [-5 0 0 100 0.1 0.05 deg2rad(states_(6)) 0];

    costF = @(u) RollingPullUp(states_, u, dataBase);
    options = optimoptions(@fmincon,'Algorithm','sqp','TolX',1e-60);
    options.MaxFunctionEvaluations = 2000;
    options.OptimalityTolerance = 1.00e-60;
    options.MaxIterations = 1000;
    options.ConstraintTolerance = 1.00e-30;

    controlDeflections = fmincon(costF, Initialize, A, b, Aeq, beq, lb, ub, [], options);
    statesFinal = [controlDeflections(5), 0, controlDeflections(5) + gamma, states_(4),...
                    states_(5), controlDeflections(6), controlDeflections(7), states_(8), Velocity,...
                    states_(10), states_(11), states_(12)];
end
    
%% Straight Flight
function J = StraightFlight(states_, u, dataBase)
    states_(3) = u(5); % Theta
    states_(4) = u(6); % Phi
    states_(5) = u(7); % Psi
    
    states_(1) = u(5) - gamma; % AoA

    gammaCons = asin(cos(-u(7))*(cos(states_(1))*sin(u(5))) - (sin(states_(1))*cos(u(5))));
    
    [statesUpdated, statesDot, loadFactor, deltaLEF, Velocity] = aircraftModel(states_, [u(1) u(2) u(3) u(4)], dataBase, trimOption);
    W = diag([1 1 0 0 0 1 1 1 1 0 0 0]);
    J = statesDot*W*statesDot' + (gammaCons - gamma)^2;
    
    %% disp('Cost Value:'); disp(J)
    %% disp('Nz:'); disp(loadFactor(3))
    %% disp('Gamma:'); disp(rad2deg(statesUpdated(3)-statesUpdated(1)))
    %% disp('Phi:'); disp(rad2deg(statesUpdated(4)))

end
%% Straight Flight for Special Case
function J = StraightFlight2(states_, u, dataBase)
    states_(1) = u(5); % AoA
    states_(2) = u(6); % SS
    
    states_(3) = u(5); % Theta
    states_(5) = -u(6); % Psi
    
    [statesUpdated, statesDot, loadFactor, deltaLEF, Velocity] = aircraftModel(states_, [u(1) u(2) u(3) u(4)], dataBase, trimOption);
    W = diag([10 10 0 0 0 10 10 10 10 0 0 0]);
    J = statesDot*W*statesDot';
    
    disp('Cost Value:'); disp(J)
    %disp('Nz:'); disp(loadFactor(3))
    %disp('Gamma:'); disp(rad2deg(statesUpdated(3)-statesUpdated(1)))
    %disp('Phi:'); disp(rad2deg(statesUpdated(4)))

end

%% Steady State Turn- Phi Input
function J = SteadyTurnCase1(states_, u, dataBase)
    states_(1) = u(5); % Alpha
    states_(3) = u(6); % Theta
    states_(4) = u(7); % Phi
    
    states_(3) = atan(tan(u(5))*cos(states_(4))); % Gamma constraint
    
    g = 9.80665*(6371020/(6371020 + states_(12)))^2;
    Temperature = 288.15 - 6.5*states_(12)/1000;
    k = 1.4;
    R_c = 287.06;
    Velocity = states_(9)*sqrt(k*R_c*Temperature);
    PsiDot = (g/Velocity)*tan(states_(4));
    
    states_(6) = -PsiDot*sin(u(6));          % P
    states_(7) = PsiDot*cos(u(6))*sin(u(7)); % Q
    states_(8) = PsiDot*cos(u(6))*cos(u(7)); % R
    
    [statesUpdated, statesDot, loadFactor, deltaLEF, Velocity] = aircraftModel(states_, [u(1) u(2) u(3) u(4)], dataBase, trimOption);
    W = diag([1 1 0 0 0 1 1 1 1 0 0 1]);
    J = statesDot*W*statesDot'; 
    
    disp('Cost Value:'); disp(J)
    %disp('Nz:'); disp(loadFactor(3))
    %disp('Beta:'); disp(rad2deg(statesUpdated(2)))
    %disp('Phi:'); disp(rad2deg(statesUpdated(4)))

end

%% Steady State Turn- N Input
function J = SteadyTurnCase2(states_, u, dataBase)
    states_(1) = u(5); % Alpha
    states_(3) = u(6); % Theta
    states_(4) = u(7); % Phi
    
    g = 9.8065*(6371020/(6371020 + states_(12)))^2;
    Temperature = 288.15 - 6.5*states_(12)/1000;
    k = 1.4;
    R_c = 287.06;
    Velocity = states_(9)*sqrt(k*R_c*Temperature);
    PsiDot = (g/Velocity)*tan(states_(4));
    
    states_(6) = -PsiDot*sin(u(6));
    states_(7) = PsiDot*cos(u(6))*sin(u(7));
    states_(8) = PsiDot*cos(u(6))*cos(u(7));
    
    mu = atan(cos(u(6))*sin(u(7))/(sin(u(5))*sin(u(6))+cos(u(5))*cos(u(6))*cos(u(7))));
    
    [statesUpdated, statesDot, loadFactor, deltaLEF, Velocity] = aircraftModel(states_, [u(1) u(2) u(3) u(4)], dataBase, trimOption);
    W = diag([1 1 0 0 0 10 1 1 1 0 0 1]);
    J = statesDot*W*statesDot' + (loadFactor(3) - 1/cos(mu))^2; 
    
    disp('Cost Value:'); disp(J)
    %disp('Nz:'); disp(loadFactor(3))
    %disp('Beta:'); disp(rad2deg(statesUpdated(2)))
    %disp('Phi:'); disp(rad2deg(statesUpdated(4)))

end

%% Pull-Up / Push Over
function J = PullUp_PushOver(states_, u, dataBase)
    states_(3) = u(5); % Theta
    states_(4) = u(6); % Beta
    states_(5) = u(7); % Psi
    states_(7) = u(8); % Pitch Rate
       
    states_(1) = u(5) - gamma; % AoA
    
    [statesUpdated, statesDot, loadFactor, deltaLEF, Velocity] = aircraftModel(states_, [u(1) u(2) u(3) u(4)], dataBase, trimOption);
    W = diag([10 10 0 0 0 10 10 10 1 0 10 1]);
    J = statesDot*W*statesDot' + (loadFactor(3) - n)^2;
    
    disp('Cost Value:'); disp(J)
    %disp('Nz:'); disp(loadFactor(3))
    %disp('Pitch Rate:'); disp(rad2deg(statesUpdated(7)))

end

%% Steady Heading Sideslip
function J = SteadySideslip(states_, u, dataBase)
    states_(3) = u(5); % Theta
    states_(4) = u(6); % Phi
    states_(5) = u(7); % Psi
    states_(1) = u(8); % AoA

    gammaCons = asin(cos(u(8))*cos(states_(2))*sin(u(5)) - sin(states_(2))*cos(u(5))*sin(u(6))...
               - sin(u(8))*cos(states_(2))*cos(u(5))*cos(u(6)));
    
    [statesUpdated, statesDot, loadFactor, deltaLEF, Velocity] = aircraftModel(states_, [u(1) u(2) u(3) u(4)], dataBase, trimOption);
    W = diag([1 1 0 0 0 1 1 1 1 0 1 0]);
    J = statesDot*W*statesDot' + (gammaCons - gamma)^2;
    
    disp('Cost Value:'); disp(J)
    %disp('Nz:'); disp(loadFactor(3))
    %disp('Beta:'); disp(rad2deg(statesUpdated(2)))
    
end

%% Roll Trim
function J = SteadyRoll(states_, u, dataBase)
    states_(3) = u(5); % Theta
    states_(1) = u(6); % AoA
    states_(6) = u(7); % Roll Rate
    
    [statesUpdated, statesDot, loadFactor, deltaLEF, Velocity] = aircraftModel(states_, [u(1) u(2) u(3) u(4)], dataBase, trimOption);
    W = diag([1 1 0 0 0 1 1 1 1 0 0 1]);
    J = statesDot*W*statesDot';
    
    disp('Cost Value:'); disp(J)
    %disp('Nz:'); disp(loadFactor(3))
    %disp('Roll Rate:'); disp(rad2deg(statesUpdated(6)))
    
end

%% Rolling Pull Up / Rolling Push Over
function J = RollingPullUp(states_, u, dataBase)
    states_(1) = u(5); % AoA
    states_(6) = u(6); % Roll Rate
    states_(7) = u(7); % Pitch Rate
    
    states_(3) = u(5) + gamma; % Theta
    
    [statesUpdated, statesDot, loadFactor, deltaLEF, Velocity] = aircraftModel(states_, [u(1) u(2) u(3) u(4)], dataBase, trimOption);
    W = diag([1 1 0 0 0 1 1 1 1 0 0 0]);
    J = statesDot*W*statesDot' + (loadFactor(3)-n)^2;
    
    disp('Cost Value:'); disp(J)
    %disp('Nz:'); disp(loadFactor(3))
    %disp('Roll Rate:'); disp(rad2deg(statesUpdated(6)))
    
end


end