function PlotResult(Torque, ToolPos, fig1, fig2 , str1)
% PlotResult(Torque, ToolPos, fig1, fig2 , str1)
%
% Plots Torque and ToolPosition

Torq = Torque.signals.values(:,1);
TimeT = Torque.time;
figure(fig1);
plot(TimeT,Torq,'k-');
grid on;
hold on;
title(sprintf('Motor Torque for %s', str1));
xlabel('Time [s]');
ylabel('Torque [Nm]');

ToolPosition = ToolPos.signals.values(:,1);
Time = ToolPos.time;
figure(fig2);
plot(Time,ToolPosition,'k-');
grid on;
hold on;
title(sprintf('Tool Position for %s', str1));
xlabel('Time [s]');
ylabel('Tool position [mm]');
