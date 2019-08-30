function [Jm, Ja1, Ja2, Ja3, k1, k2, k3, d1, d2, d3, fm, fa1, fa2, fa3, gear_ratio, k1_low, k1_low_pos, k1_pos, len1, len2, len3, Gain, DelayTime] = ...
    MakeFourMassRobot (Jm_err, Ja1_err, Ja2_err, Ja3_err, k1_err, k2_err, k3_err, ...
    d1_err, d2_err, d3_err, fm_err, fa1_err, fa2_err, fa3_err, Gain_err, DelayTime_err, k1_low_err, k1_pos_err)
% [Jm, Ja1, Ja2, Ja3, k1, k2, k3, d1, d2, d3, fm, fa1, fa2, fa3, gear_ratio, 
%  k1_low, k1_low_pos, k1_pos, len1, len2, len3, Gain, DelayTime]
%  = MakeFourMassRobot (Jm_err, Ja1_err, Ja2_err, Ja3_err, k1_err, k2_err, k3_err,
%    d1_err, d2_err, d3_err, fm_err, fa1_err, fa2_err, fa3_err, Gain_err, DelayTime_err, 
%    k1_low_err, k1_pos_err)
% 
% Returns parameter values for nonlinear robot model including delay time
% and gain for stability check. Some of the nominal parameter values are
% multiplied with the input error constants for creation of different model 
% parameters for robustness and stability check
%

% Nominal Robot Data
Jm = 0.005;          % Motor Inertia
Ja1 = 0.002;         % Arm Inertia 1
Ja2 = 0.02;          % Arm Inertia 2
Ja3 = 0.02;          % Arm Inertia 3
k1 = 100;            % Gear Box Stiffness at high torque
k1_low = k1/6;       % Gear box stiffness at low torque
k1_pos = 1;          % Position for k1 [bm]
k1_low_pos = 0;      % Position for k1_low
k2 = 110;            % Arm Stiffness 1
k3 = 80;             % Arm Stiffness 2
d1 = 0.08;           % Gear Box Damping
d2 = 0.06;           % Arm Damping 1
d3 = 0.08;           % Arm Damping 2
fm = 0.006;          % Motor Friction
fa1 = 0.001;         % Arm Friction 1
fa2 = 0.001;         % Arm Friction 2
fa3 = 0.001;         % Arm Friction 3
gear_ratio = 220;    % gear_ratio
len1 = 20;          % Lengths for tool position computation [mm]
len2 = 600;          % Lengths for tool position computation [mm]
len3 = 1530;         % Lengths for tool position computation [mm] 
Gain = 1;            % Normal Gain in loop
DelayTime = 0.5E-3;  % Normal Delay in loop  

Jm = Jm * Jm_err;
Ja1 = Ja1 * Ja1_err;
Ja2 = Ja2 * Ja2_err;
Ja3 = Ja3 * Ja3_err;
k1 = k1 * k1_err;
k2 = k2 * k2_err;
k3 = k3 * k3_err;
d1 = d1 * d1_err;
d2 = d2 * d2_err;
d3 = d3 * d3_err;
fm = fm * fm_err;
fa1 = fa1 * fa1_err;
fa2 = fa2 * fa2_err;
fa3 = fa3 * fa3_err;
Gain = Gain * Gain_err;
DelayTime = DelayTime * DelayTime_err;
k1_low = k1_low * k1_low_err;
k1_pos =  k1_pos * k1_pos_err;