classdef Node<handle
    properties
        state % x,y,theta 
        statesOfBranches
        controlInputSet
        neighbours
        cameFrom Node
        gScore = 0 % Cost from Start Node
        hScore = inf % Cost from End Node
        fScore = inf % Final Cost hScore + gScore
        chasisLength_m
        ODE_Solver_Fcn
    end

    methods
        function obj = Node(state,controlInputSet)
                obj.state           = state;    
                obj.controlInputSet = controlInputSet;
                obj.neighbours      = cell(1,length(controlInputSet));
                obj.chasisLength_m  = 1;
                obj.ODE_Solver_Fcn  = @Solver.Heun;
        end
        function calculate_g_Score(obj)
            if (~isempty(obj.cameFrom))
                obj.gScore = obj.cameFrom.gScore + Node.FindDistance([obj.state(1),obj.state(2)],[obj.cameFrom.state(1),obj.cameFrom.state(2)])*0.1;
            else
                obj.gScore = 0;
            end
        end
        function calculate_h_Score(obj,endPoint)
            obj.hScore = Node.FindDistance([obj.state(1),obj.state(2)],endPoint);
        end
        function calculate_f_Score(obj,endPoint)
            obj.calculate_h_Score(endPoint);
            obj.calculate_g_Score();
            obj.fScore = obj.gScore + obj.hScore;
        end
        function createNeighbours(obj,map)
           obj.statesOfBranches = obj.simulateInputSet();
           for i = 1:length(obj.statesOfBranches)
               currentStates = obj.statesOfBranches{i};
    
               finalState = currentStates (end,:);
               hasHit = Node.CheckOccupancyHit(currentStates,map);
               if hasHit
                   obj.neighbours{i} = [];
                   obj.statesOfBranches{i} = [];
               else
                   obj.neighbours{i} = Node(finalState,obj.controlInputSet);
                   obj.neighbours{i}.cameFrom = obj;
               end
           end
        end

        %% Simulation Functions
        function states = simulate(obj,controlInput,tSpan)
            u       = [controlInput.velocity_m_s,controlInput.steeringAngle_deg];
            fcn     = @(y,t)obj.EoM(y,t,u);
            states  = obj.ODE_Solver_Fcn(fcn,obj.state,tSpan);
        end
        function states = simulateInputSet(obj)
            sx     = length(obj.controlInputSet.set);
            tSpan  = obj.controlInputSet.tSpan;
            states = cell(sx,1); 
            for i = 1:sx
                controlInput = obj.controlInputSet.set(i);
                states{i}   = simulate(obj,controlInput,tSpan);
            end
        end
        function statesDot = EoM(obj,states,t,controlInputs) % Equations of Motion
            % x_m                 = states(1);
            % y_m                 = states(2);
            theta_deg           = states(3);
            theta_rad           = deg2rad(theta_deg);
            velocity_m_s        = controlInputs(1);
            steeringAngle_deg   = controlInputs(2);

            l_m             = obj.chasisLength_m;
            alpha_deg       = steeringAngle_deg;
            alpha_rad       = deg2rad(alpha_deg);

            R_m             = l_m/tan(alpha_rad); %instantaneous turning radious

            x_dot_m_s       = velocity_m_s*cos(theta_rad);
            y_dot_m_s       = velocity_m_s*sin(theta_rad);
            theta_dot_rad_s = velocity_m_s./R_m;
            theta_dot_deg_s = rad2deg(theta_dot_rad_s);

            statesDot = [x_dot_m_s,y_dot_m_s,theta_dot_deg_s];
        end
    end
    methods(Static)
        function length = FindDistance(Point1,Point2)
                deltaLoc = (Point2 - Point1);
                length = sqrt(deltaLoc*deltaLoc');
        end
        function hasHit = CheckOccupancyHit(states,map)
            k = flip(map.map.occupancyMatrix)';
            bp1 = 1:map.sx;
            bp2 = 1:map.sy;
            v   = double(k);
            [rn,~]=size(states);

            for i = 1:rn-1
                startXY = states(i,1:2);
                endXY   = states(i+1,1:2);
                deltaXY = endXY-startXY;
                [azimuth,~,r ]= cart2sph(deltaXY(1),deltaXY(2),0);
                rlist = 0:0.1:r;
                listXY(:,1) = startXY(1) + rlist'.*cos(azimuth);
                listXY(:,2) = startXY(2) + rlist'.*sin(azimuth);
                res = interpn(bp1,bp2,v,listXY(:,1),listXY(:,2));
                if any(res~=0)
                    hasHit = true;
                else
                    hasHit = false;
                end
            end
            % hasHit = false;
        end
    end
end

