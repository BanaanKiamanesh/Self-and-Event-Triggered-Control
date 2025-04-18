clear
close all
clc

%% Parameter Declaration
InitialCond = [1, 2, 3];
Tfinal      = 8;

R = RigidBody(InitialCond);

TrigPer  = @(RB, X) RB.PeriodicCondition(X);
TrigSelf = @(RB, X)  RB.TriggerCondition(X);

%% System Simulation
[tPeriodic, XPeriodic] = R.Simulate(Tfinal, TrigPer);
[tSelfTrig, XSelfTrig] = R.Simulate(Tfinal, TrigSelf);

%% Plotting Results
% States
figure('Name', 'State Response', 'Units', 'normalized', 'OuterPosition', [0, 0, 0.5, 1])
plot(tPeriodic, XPeriodic(:, 1), 'b', ...
     tPeriodic, XPeriodic(:, 2), 'r', ...
     tPeriodic, XPeriodic(:, 3), 'k', ...
     tSelfTrig, XSelfTrig(:, 1), 'c--', ...
     tSelfTrig, XSelfTrig(:, 2), 'y--', ...
     tSelfTrig, XSelfTrig(:, 3), 'g--');

xlabel('Time(s)')
ylabel('State')
legend({'x_1 Periodic', 'x_2 Teriodic', 'x_3 Periodic', ...
         'x_1 Trigger',  'x_2 Trigger',  'x_3 Trigger'})
grid on
xlim([0, Tfinal])

% Inter‑Execution Times(Noise‑Free)
figure('Name', 'dt', 'Units', 'normalized', 'OuterPosition', [0.5, 0, 0.5, 0.5])
plot(tSelfTrig(2:end), diff(tSelfTrig), 'b.', ...
     tPeriodic(2:end), diff(tPeriodic), 'r')
legend({'Self‑Triggered', 'Periodic'})
xlabel('Time(s)')
ylabel('\Deltat(s)')
title('\Deltat')
grid on
xlim([0, Tfinal])

% Lyapunov Function Decay
LyapPeriodic = vecnorm(XPeriodic, 2, 2).^2;
LyapSelfTrig = vecnorm(XSelfTrig, 2, 2).^2;

figure('Name', 'Lyapunov Function', 'Units', 'normalized', 'OuterPosition', [0.5, 0.5, 0.5, 0.5])
plot(tPeriodic, LyapPeriodic,   'b', ...
     tSelfTrig, LyapSelfTrig, '--r')
legend({'periodic', 'self‑triggered'})
title('Lyapunov Function V(x)')
xlabel('Time(s)')
ylabel('||x||^2')
grid on
xlim([0, Tfinal])

% Increase the LineWidths in All the Open Figures(I'm too lazy to add it to all the plots but not enough not to write it here!!)
set(findall(0, 'Type',  'line'), 'LineWidth', 2);

% Number of Executions
disp(' ---> Number of Controller Executions');
NumExec = table([numel(tPeriodic) - 1; numel(tSelfTrig) - 1], ...
                'VariableNames', {'Executions'}, ...
                'RowNames', {'Periodic', 'Self‑Triggered'});
disp(NumExec);
