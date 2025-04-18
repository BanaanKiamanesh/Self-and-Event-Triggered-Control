classdef JetEngine < handle
    %% Parameters & State
    properties
        Beta        double  = 1
        TauTrigger  double  = 7.63e-3
        TauPeriodic double  = 7.63e-3
        NoiseStd    double  = 0
        X           double
    end
    properties (SetAccess = private)
        InitialState double
    end

    %% Construction
    methods
        function obj = JetEngine(InitCond, NoiseStd)
            arguments
                InitCond    (1, 2) double
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
        function U = InputCalc(obj, X)
            X1 = X(1);  
            X2 = X(2);
            Y  = JetEngine.X22Y(X1, X2);
            
            U  = X1 + 0.5 * obj.Beta^2 * (X1^2 + 1)* ...
                (2 * X1^2 * Y + Y + X1 * Y^2 - 2 * X1^2 - 2 * X1 * Y);
        end

        function dt = PeriodicCondition(obj, ~)
            dt = obj.TauPeriodic;
        end

        function dt = TriggerCondition(obj, X)
            X1   = X(1);
            Y    = JetEngine.X22Y(X(1), X(2));
            Norm = hypot(X1, Y);

            dt   = (29*X1 + Norm^2) / (5.36*Norm*X1^2 + Norm^2) * obj.TauTrigger;
        end
    end

    %% Plant ODE + Noise
    methods
        function InputApply(obj, U, TMax)
            % ODE Function
            ODEFun = @(~, X)[-X(2) - 1.5*X(1).^2 - 0.5*X(1).^3
                             (X(1) - U) / obj.Beta^2];

            Sol = ode45(ODEFun, [0, TMax], obj.X, ...
                        odeset('RelTol', 1e-9, 'AbsTol', 1e-12));
            obj.X = Sol.y(:, end);

            if obj.NoiseStd > 0
                obj.X = obj.X + abs(obj.X) .* obj.NoiseStd .* randn(size(obj.X));
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

            obj.X    = obj.InitialState;
            t        = 0;
            TimeVec  = t;
            RespMat  = obj.X';
            DistInfo = Disturbance;     % [time value] or []

            while t <= TotalTime
                CurrState = obj.X;
                U         = obj.InputCalc(CurrState);

                if ~isempty(DistInfo) && t >= DistInfo(1)
                    U = U + DistInfo(2);
                    DistInfo = [];                       % single shot
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
        function Y = X22Y(X1, X2)
            Y = 2 * (X1.^2 + X2) ./ (X1.^2 + 1);
        end

        function X2 = Y2X2(X1, Y)
            X2 = 0.5 * Y .* (X1.^2 + 1) - X1.^2;
        end
    end
end
