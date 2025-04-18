clear
close all
clc

%% Parameter Declaration
InitCond = [5.37, JetEngine.Y2X2(5.37, 0.34)];
Tfinal   = 3;

% Aux Functions
TrigPer  = @(JE, State) JE.PeriodicCondition(State);
TrigSelf = @(JE, State) JE.TriggerCondition(State);

% Object Creation
J = JetEngine(InitCond);

%% System Simulation
[tPeriodic, XPeriodic] = J.Simulate(Tfinal,  TrigPer);
[tSelfTrig, XSelfTrig] = J.Simulate(Tfinal, TrigSelf);

%% Plotting Results
% States
figure('Name', 'State Response', 'Units', 'normalized', 'OuterPosition', [0, 0, 0.5, 0.5])
plot(tPeriodic, XPeriodic(:, 1), 'b*', tPeriodic, XPeriodic(:, 2), 'r*', ...
     tSelfTrig, XSelfTrig(:, 1), 'b+', tSelfTrig, XSelfTrig(:, 2), 'r+')
title('State Response')
xlabel('Time (s)')
ylabel('State')
legend({'x_1 periodic', 'x_2 periodic', ...
        'x_1 trigger', 'x_2 trigger'})
grid on
xlim([0 Tfinal])

% Inter‑Execution Times (Noise‑Free)
figure('Name', 'dt(noise‑free)', 'Units', 'normalized', 'OuterPosition', [0, 0.5, 0.5, 0.5])
stairs(tPeriodic(2:end), diff(tPeriodic),   'b'), hold on
stairs(tSelfTrig(2:end), diff(tSelfTrig), '--r')
title('\Deltat (noise‑free)')
legend({'Periodic', 'Self‑Triggered'})
xlabel('Time (s)')
ylabel('\Deltat (s)')
grid on
xlim([0, Tfinal])

% Perturbed Response
JDist = JetEngine(InitCond, 0.05);
Disturbance = [0.7, 200];

[tPeriodicDisturbed, XPeriodicDisturbed] = JDist.Simulate(Tfinal,  TrigPer, Disturbance);
[tSelfTrigDisturbed, XSelfTrigDisturbed] = JDist.Simulate(Tfinal, TrigSelf, Disturbance);

figure('Name', 'Perturbed State Response', 'Units', 'normalized', 'OuterPosition', [0.5, 0, 0.5, 0.5])
plot(tPeriodicDisturbed, XPeriodicDisturbed(:, 1),   'b', tPeriodicDisturbed, XPeriodicDisturbed(:, 2),   'r', ...
     tSelfTrigDisturbed, XSelfTrigDisturbed(:, 1), '--b', tSelfTrigDisturbed, XSelfTrigDisturbed(:, 2), '--r')
title('Perturbed State Response')
xlabel('Time (s)')
ylabel('State')
legend({'x_1 periodic', 'x_2 periodic', 'x_1 trigger', 'x_2 trigger'})
grid on
xlim([0 Tfinal])

% dt with & without Disturbance
figure('Name', 'dt with / without Disturbance', 'Units', 'normalized', 'OuterPosition', [0.5, 0.5, 0.5, 0.5])
stairs(tSelfTrigDisturbed(2:end), diff(tSelfTrigDisturbed), 'b'), hold on
stairs(tSelfTrig(2:end), diff(tSelfTrig), '--r')
title('\Deltat with/without Disturbance')
legend({['Disturbance at ', num2str(Disturbance(1)), ' s'], 'No Disturbance'})
xlabel('Time (s)')
ylabel('\Deltat (s)')
grid on
xlim([0 Tfinal])

% Lyapunov Function Decay
LyapFun = @(X1, Y) 1.46*X1.^2 - 0.35*X1.*Y + 1.16*Y.^2;
YTrajPeriodic = JetEngine.X22Y(XPeriodic(:, 1), XPeriodic(:, 2));
YTrajSelfTrig = JetEngine.X22Y(XSelfTrig(:, 1), XSelfTrig(:, 2));
LyapPeriodic  = LyapFun(XPeriodic(:, 1), YTrajPeriodic);
LyapSelfTrig  = LyapFun(XSelfTrig(:, 1), YTrajSelfTrig);

figure('Name', 'Lyapunov Function', 'Units', 'normalized', 'OuterPosition', [0.25, 0.25, 0.5, 0.5])
plot(tPeriodic, LyapPeriodic, 'b', ...
     tSelfTrig, LyapSelfTrig, '--r')
title('Lyapunov Function V(x)')
legend({'Periodic', 'Self‑Triggered'})
xlabel('Time (s)')
ylabel('V(x)')
grid on
xlim([0 Tfinal])

% Increase the LineWidths in All the Open Figures(I'm too lazy to add it to all the plots but not enough not to write it here!!)
set(findall(0, 'Type',  'line'), 'LineWidth', 2);
set(findall(0, 'Type', 'stair'), 'LineWidth', 2);


% Number of Executions
disp(' ---> Number of Controller Executions');
NumExec = table([numel(tPeriodic) - 1; numel(tSelfTrig) - 1], ...
                'VariableNames', {'Executions'}, ...
                'RowNames', {'Periodic', 'Self‑Triggered'});
disp(NumExec);
