function PlotRobotTransferFunction(model, fig1, str1)
% PlotRobotTransferFunction(model, fig1, str1)
%
% Plot transfer function amplitude of high stiffness region for given model parameters
w = 2*pi*logspace(0,2,10000);
% High stiffness region of gearbox
G = model(15) * tf(FourMassRobot(model(1),model(2),model(3),model(4),model(5),model(6),model(7),model(8),...
    model(9),model(10),model(11),model(12),model(13),model(14),model(19),model(20),model(21),model(22)));
[mag,pha] = bode(G(1,1)* tf([1 0 0],[1]),w);
figure(fig1);
semilogx(w/2/pi,20*log10(mag(:)),'k-');
hold on;
grid on;
% % Low stiffness region of gearbox
% G = model(15) * tf(FourMassRobot(model(1),model(2),model(3),model(4),model(17),model(6),model(7),model(8),...
%     model(9),model(10),model(11),model(12),model(13),model(14),model(19),model(20),model(21),model(22)));
% [mag,pha] = bode(G(1,1)* tf([1 0 0],[1]),w);
% semilogx(w/2/pi,20*log10(mag(:)),'k--');
title(sprintf('Transfer functions from motor torque to motor acceleration %s', str1));
xlabel('Frequency [Hz]');
ylabel('Magnitude [dB]');