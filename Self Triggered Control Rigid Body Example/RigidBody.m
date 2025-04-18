classdef RigidBody < handle
    %% Parameters & State
    properties
        TauPeriodic double = 4.5e-4
        TauTrigger  double = 5.1e-3
        NoiseStd    double = 0
        X           double
    end
    properties (SetAccess = private)
        InitialState double
    end

    %% Construction
    methods
        function obj = RigidBody(InitCond, NoiseStd)
            arguments
                InitCond    (1, 3) double
                NoiseStd    (1, 1) double = 0
            end
            obj.InitialState = InitCond(:);
            obj.X            = obj.InitialState;
            obj.NoiseStd     = NoiseStd;
        end
    end

    %% Utilities
    methods
        function st = GetState(obj)
            st = obj.X';
        end
    end

    %% Control Law & Triggers
    methods
        function u = InputCalc(~, state)
            X1 = state(1);
            X2 = state(2);
            X3 = state(3);

            U1 = -X1*X2 - 2*X2*X3 - X1 - X3;
            U2 =  2*X1*X2*X3 + 3*X3^2 - X2;

            u  = [U1; U2];
        end

        function dt = PeriodicCondition(obj, ~)
            dt = obj.TauPeriodic;
        end

        function dt = TriggerCondition(obj, state)
            dt = obj.TauTrigger ./ (1 + norm(state)^2);
        end
    end

    %% Plant ODE + Noise
    methods
        function InputApply(obj, U, TMax)
            odeFun = @(~, X)[U(1); U(2); X(1).*X(2)];
            Sol = ode45(odeFun, [0, TMax], obj.X, ...
                odeset('RelTol', 1e-9, 'AbsTol', 1e-12));
            obj.X = Sol.y(:, end);

            if obj.NoiseStd > 0
                obj.X = obj.X + abs(obj.X).*obj.NoiseStd.*randn(size(obj.X));
            end
        end
    end

    %% Full Closed‑Loop Simulation
    methods
        function [TimeVec, RespMat] = Simulate(obj, TotalTime, TriggerFcn, Disturbance)
            arguments
                obj
                TotalTime   (1, 1) double {mustBePositive}
                TriggerFcn  function_handle
                Disturbance double = []
            end
            obj.X   = obj.InitialState;
            t       = 0;
            TimeVec = t;
            RespMat = obj.X';
            Dist    = Disturbance;

            while t <= TotalTime
                CurrState = obj.X;
                U         = obj.InputCalc(CurrState);

                if ~isempty(Dist) && t >= Dist(1)
                    U    = U + Dist(2:3)';
                    Dist = [];
                end

                dt = TriggerFcn(obj, CurrState);
                obj.InputApply(U, dt);

                t        = t + dt;
                TimeVec  = [TimeVec; t];            %#ok<AGROW>
                RespMat  = [RespMat; obj.X'];       %#ok<AGROW>
            end
        end
    end

    %% Static Coordinate Conversions Methods
    methods (Static)
        function V = Lyapunov(X)
            X1 = X(:, 1);
            X2 = X(:, 2);
            X3 = X(:, 3);

            V  = 0.5*(X1 + X3).^2 + 0.5*(X2 - X3.^2).^2 + X3.^2;
        end
    end
end
