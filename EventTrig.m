clc;
clear;
close all;

%% System Definition
Ap = [ 1.280  -0.208   6.715  -5.676
      -0.581  -4.290   0       0.675
       1.067   4.273  -6.654   5.893
       0.048   4.273   1.343  -2.104];

Bp = [0       0
      5.679   0
      1.136  -3.146
      1.136   0];

% State Feedback
K = [0.0360  -0.5373  -0.3344  -0.0147
     1.6301   0.5716   0.8285  -0.2821];

A_cl = Ap + Bp*K;       % Closed-Loop State Matrix

%% Lyapunov Function Setup
Q = eye(4);                 % Chosen positive definite matrix
P = lyap(A_cl', Q);         % Solve P from Lyapunov Equation

%% Simulation Parameters
dt = 0.001;          % Sampling Time
Tf = 4;              % Final Simulation Time
t  = 0:dt:Tf;        % Time Vector
N  = length(t);

% Mem Alloc
X = zeros(4, N);        % State trajectory
X(:,1) = [1; 0; 1; 0];  % Init Cond

% Event-Triggered Controller Params
Sigma  = 0.9;           % Decay Rate Param (0 < Sigma < 1)
Err    = zeros(4, 1);   % Initial Error
U      = zeros(2, N);   % Control Inputs
Events = 0;             % Record Event Instants
Cnt    = 1;             % Event Counter

X_last = X(:, 1);       % Last Triggered State

%% Simulation System

for i = 1:N-1
    % Current State and Error
    X_curr = X(:, i);
    Err = X_last - X_curr;

    % % Check Triggering Condition
    % TriggerCond = [X_curr; Err]' * [ (Sigma-1)*Q,  P*Bp*K
    %                                     K'*Bp'*P, zeros(4)] * [X_curr; Err];
    
    TriggerCond = norm(Err) - Sigma * norm(X_curr);

    % Triggering Logic
    if TriggerCond >= 1e-5 || i == 1
        U(:, i) = K * X_curr;        % Update control input
        X_last = X_curr;

        Events = [Events, t(i)];     % Store Event Time
        Cnt = Cnt + 1;
    else
        U(:, i) = U(:, i-1);
    end

    dX        = Ap * X_curr + Bp * U(:, i);
    X(:, i+1) = X_curr + dX * dt;
end

%% Post-Processing
% Calc Inter-Event Intervals
InterEventTime = diff(Events);

% Compute Lyapunov Function Values
V = zeros(1,N);
for i = 1:N
    V(i) = X(:, i)' * P * X(:, i);
end

%% Plotting Results
figure
set(gcf, 'Units', 'normalized')
set(gcf, 'OuterPosition', [0, 0, 1, 1])

subplot(2, 1, 1)
plot(t, X, 'LineWidth', 2)
grid on
title('State Trajectories')
xlabel('Time (s)')
ylabel('States')
legend('x_1', 'x_2', 'x_3', 'x_4')

subplot(2, 1, 2)
stairs(t, U(1, :), 'LineWidth', 2), hold on
stairs(t, U(2, :), 'LineWidth', 2)
grid on
title('Control Inputs')
xlabel('Time (s)')
ylabel('U')
legend('u_1', 'u_2')

figure
set(gcf, 'Units', 'normalized')
set(gcf, 'OuterPosition', [0, 0, 1, 1])

stem(Events(2:end), InterEventTime, 'filled')
title('Inter-Event Times')
xlabel('Event Number')
ylabel('Time Between Events (s)')
grid on

figure
set(gcf, 'Units', 'normalized')
set(gcf, 'OuterPosition', [0, 0, 1, 1])

subplot(2, 1, 1)
plot(t, V, 'b', 'LineWidth', 1.5), hold on
plot(Events, V(round(Events/dt)+1), 'ro', 'MarkerSize', 5)
grid on
title('Lyapunov Function Decay')
xlabel('Time (s)')
ylabel('V(x)')
legend('Lyapunov Function', 'Trigger Events')

subplot(2, 1, 2);
semilogy(t, V, 'b', 'LineWidth', 1.5), hold on
semilogy(Events, V(round(Events/dt)+1), 'ro', 'MarkerSize', 5)
grid on
title('Lyapunov Function Decay(log scale)')
xlabel('Time (s)')
ylabel('log(V(x))')

% Event Properties
fprintf('Total Events: %d\n', length(Events))
fprintf('Average Inter-Event Time: %.3f s\n', mean(InterEventTime))
