SetParameters; % Put everything in order.

[Jm, Ja1, Ja2, Ja3, k1, k2, k3, d1, d2, d3, fm, fa1, fa2, fa3, gear_ratio, k1_low, k1_low_pos, k1_pos, len1, len2, len3, Gain, DelayTime] = ...
    MakeFourMassRobot(1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1);

% Four mass state space system
A = [0 0 0 0 1 0 0 0; ...
     0 0 0 0 0 1 0 0; ...
     0 0 0 0 0 0 1 0; ...
     0 0 0 0 0 0 0 1; ...
     -k1/Jm k1/Jm  0 0 -(fm+d1)/Jm d1/Jm 0 0; ...
     k1/Ja1 -(k1+k2)/Ja1 k2/Ja1 0 d1/Ja1 -(d1+d2+fa1)/Ja1 d2/Ja1 0; ...
     0 k2/Ja2 -(k2+k3)/Ja2 k3/Ja2 0 d2/Ja2 -(d2+d3+fa2)/Ja2 d3/Ja2; ...
     0 0 k3/Ja3 -k3/Ja3 0 0 d3/Ja3 -(fa3+d3)/Ja3];

B = [0; ...
     0; ...
     0; ...
     0; ...
     1/Jm; ...
     0; ...
     0; ...
     0];

C = [1 0 0 0 0 0 0 0];

D = [0];

RobotSS=ss(A,B,C,D);

[b,a]=ss2tf(A,B,C,D,1);

RobotLaplace=tf(b,a);