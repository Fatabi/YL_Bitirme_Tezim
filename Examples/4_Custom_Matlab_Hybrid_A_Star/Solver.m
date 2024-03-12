classdef Solver
    methods(Static)
        function states = Heun(fcn, state0, tSpan)
            % Solve the initial value problem using Heun's method.
            f           = fcn;
            t_values    = tSpan;
            h = t_values(2) - t_values(1);
            states      = nan(length(t_values),length(state0));
            states(1,:) = state0;

            for i = 1:(length(t_values)-1)
                t = t_values(i);
                y = states(i,:);

                k1 = h * f(y,t);
                k2 = h * f(y + k1,t + h);

                states(i + 1,:) = y + 0.5 * (k1 + k2);
            end
        end
    end
end

