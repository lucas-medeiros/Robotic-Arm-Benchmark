% RobotSystemSimulate - Main File 
%
% 1. Simulates Nominal Model & Controller 1
%    Compute Performance Criterion K_nom. Stability must be manually inspected.
% 2. Check if Gain Margin and Delay Margin are fullfilled for Nominal Model and Controller 1.
%    Result must be manually inspected
% 3. Check if Gain Margin and Delay Margin are fullfilled for Nominal Model and Controller 2.
%    Result must be manually inspected
% 4. Simulates Model Set 1 & Controller 2
%    Compute Performance Criterion K_1. Stability must be manually inspected.
% 5. Simulates Model Set 2 & Controller 2
%    Compute Performance Criterion K_2. Stability must be manually inspected.
% 6. Outputs Total Performance Criterion if all requirements are fullfilled
% 
% N.B.1. Stability has to be manually inspected
% N.B.2. The criterion computation is probably not fail-safe. Manual inspection is recommended.
% N.B.3. Tool Position, Torque and Amplitude Frequency Response of linearized robot system 
%        are plotted for each simulation.
% N.B.4. Simulation of model set 1 and 2 can be disabled.
% N.B.5. Of course it is allowed to have Controller 1 = Controller 2, i.e.
%        Controller_1.m == Controller2.m & RobotSystem_1.mdl == RobotSystem_2.mdl
% N.B.6. Controller 1 & 2 may have the same structure but different tuning parameters
%        (Controller_1.m != Controller2.m & RobotSystem_1.mdl == RobotSystem_2.mdl)
%        or have completely different structures 
%        (Controller_1.m != Controller2.m & RobotSystem_1.mdl != RobotSystem_2.mdl)

clear;
close all;
error = 1;
simulate_model_set_1 = 1; % 0 - do not simulate, 1 - simulate
simulate_model_set_2 = 1; % 0 - do not simulate, 1 - simulate
% Set Parameters for simulation
SetParameters;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Controller 1 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Controller Parameters Controller 1
Controller_1;

% Nominal Robot
[Jm, Ja1, Ja2, Ja3, k1, k2, k3, d1, d2, d3, fm, fa1, fa2, fa3, gear_ratio, k1_low, k1_low_pos, k1_pos, len1, len2, len3, Gain, DelayTime] = ...
    MakeFourMassRobot(1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1);
% Simulate nominal
sim RobotSystem_1;

% Plot Result
PlotResult(Torque, ToolPos, 1, 2 , 'Nominal System & Controller 1');
% Plot transfer function
PlotRobotTransferFunction([Jm, Ja1, Ja2, Ja3, k1, k2, k3, d1, d2, d3, fm, fa1, fa2, fa3, ...
        Gain, DelayTime, k1_low, k1_pos,len1,len2,len3,gear_ratio], 3, 'for nominal model');
% Check Performance
disp('PERFORMANCE NOMINAL MODEL:')
OldErrors = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];
[Result,NewErrors] = CheckPerformance(Torque, ToolPos, RmsTorque,OldErrors,-1, 3.0, 5, 0.1);
OldErrors = NewErrors;
[ResultNom,NewErrors] = CheckPerformance(Torque, ToolPos, RmsTorque,OldErrors,1, 3.0, 5, 0.1);
if ResultNom > 0
    disp(sprintf('Nominal Performance Criterion = %.1f', ResultNom));
else
    disp('Nominal Performance not satisfied');
    error = -1;
end

% Test of stability for Gain x 2.5
[Jm, Ja1, Ja2, Ja3, k1, k2, k3, d1, d2, d3, fm, fa1, fa2, fa3, gear_ratio, k1_low, k1_low_pos, k1_pos, len1, len2, len3, Gain, DelayTime] = ...
    MakeFourMassRobot(1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2.5, 1, 1, 1);
% Simulate system
sim RobotSystem_1;

% Plot Result
PlotResult(Torque, ToolPos, 4,5 , 'gain increase Nominal System & Controller 1');

% Test of stability for DelayTime * 4 
[Jm, Ja1, Ja2, Ja3, k1, k2, k3, d1, d2, d3, fm, fa1, fa2, fa3, gear_ratio, k1_low, k1_low_pos, k1_pos, len1, len2, len3, Gain, DelayTime] = ...
    MakeFourMassRobot(1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 4, 1, 1);
% Simulate system
sim RobotSystem_1;

% Plot Result
PlotResult(Torque, ToolPos, 6,7 , 'delay increase Nominal System & Controller 1');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Controller 2 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Controller Parameters Controller 2
Controller_2;

% Test of stability for Gain x 2.5
[Jm, Ja1, Ja2, Ja3, k1, k2, k3, d1, d2, d3, fm, fa1, fa2, fa3, gear_ratio, k1_low, k1_low_pos, k1_pos, len1, len2, len3, Gain, DelayTime] = ...
    MakeFourMassRobot(1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2.5, 1, 1, 1);
% Simulate system
sim RobotSystem_2;

% Plot Result
PlotResult(Torque, ToolPos, 8,9 , 'gain increase Nominal System & Controller 2');

% Test of stability for DelayTime * 4 
[Jm, Ja1, Ja2, Ja3, k1, k2, k3, d1, d2, d3, fm, fa1, fa2, fa3, gear_ratio, k1_low, k1_low_pos, k1_pos, len1, len2, len3, Gain, DelayTime] = ...
    MakeFourMassRobot(1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 4, 1, 1);
% Simulate system
sim RobotSystem_2;

% Plot Result
PlotResult(Torque, ToolPos, 10,11 , 'delay increase Nominal System & Controller 2');

if simulate_model_set_1
    % Robust Performance for model set 1
    if error > 0 
        disp('ROBUST PERFORMANCE FOR MODEL SET 1:')
        rp_models = [...
                1.1 0.9 1.5*0.9 0.5*0.9 1.15 0.75*1.15 1.25*1.15 0.5 0.5 0.5 0.2 0.2 0.0 0.0 1.1 1.1 0.8 1.5;
            0.9 0.9 1.8*0.9 0.2*0.9 1.15 0.72*1.15 1.28*1.15 0.5 1.5 0.5 2.0 2.0 0.0 0.0 0.9 1.1 1.5 0.5;
            1.1 0.9 0.5*0.9 1.5*0.9 1.15 1.40*1.15 1.30*1.15 1.5 0.5 0.5 0.2 0.2 0.0 0.0 1.1 1.1 0.8 0.5;
            0.9 0.9 1.0*0.9 1.0*0.9 1.15 1.70*1.15 0.55*1.15 0.5 0.5 1.5 0.2 0.2 0.0 0.0 0.9 1.1 1.5 1.5;
            1.0 1.1 0.05*1.0 1.95*1 0.85 2.00*1.00 0.50*1.00 0.5 0.5 0.5 0.2 0.2 0.0 0.0 0.9 1.1 0.8 1.5;
            0.9 1.1 1.0*1.1 1.0*1.1 0.85 0.80*0.85 2.20*0.85 0.5 0.5 0.5 0.2 0.2 0.0 0.0 1.1 1.1 1.5 0.5;
            1.1 1.1 1.5*1.1 0.5*1.1 0.85 0.90*0.85 0.40*0.85 1.5 0.5 0.5 0.2 0.2 0.0 0.0 0.9 1.1 0.8 0.5;
            0.9 1.1 0.5*1.1 1.5*1.1 0.85 1.10*0.85 1.80*0.85 0.5 1.5 0.5 2.0 2.0 0.0 0.0 1.1 1.1 1.5 1.5;
            1.1 1.1 1.0*0.9 1.0*1.1 0.85 1.00*0.85 1.00*1.15 0.5 0.5 1.0 0.2 0.2 0.0 0.0 1.1 1.1 0.8 1.5;
            1.0 1.0 1.4*1.0 0.6*1.0 1.00 1.40*1.00 0.25*1.00 0.5 0.5 0.5 0.2 0.2 0.0 0.0 1.1 1.1 1.5 0.5];
        OldErrors = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];
        for ii = 1:10,
            [Jm, Ja1, Ja2, Ja3, k1, k2, k3, d1, d2, d3, fm, fa1, fa2, fa3, gear_ratio, k1_low, k1_low_pos, k1_pos, len1, len2, len3, Gain, DelayTime] = ...
                MakeFourMassRobot(rp_models(ii,1),rp_models(ii,2),rp_models(ii,3),rp_models(ii,4),rp_models(ii,5),...
                rp_models(ii,6),rp_models(ii,7),rp_models(ii,8),rp_models(ii,9),rp_models(ii,10),rp_models(ii,11),...
                rp_models(ii,12),rp_models(ii,13),rp_models(ii,14),rp_models(ii,15),rp_models(ii,16),rp_models(ii,17),...
                rp_models(ii,18));
            % Simulate system
            sim RobotSystem_2;

            % Plot Result
            PlotResult(Torque, ToolPos, 12, 13 , 'Model Set 1 & Controller 2');
            % Plot transfer function
            PlotRobotTransferFunction([Jm, Ja1, Ja2, Ja3, k1, k2, k3, d1, d2, d3, fm, fa1, fa2, fa3, ...
                    Gain, DelayTime, k1_low, k1_pos,len1,len2,len3,gear_ratio], 14, 'for Model Set 1');
            % Check Performance
            [NewResult,NewErrors] = CheckPerformance(Torque, ToolPos, RmsTorque,OldErrors,-1, 3.0, 5, 0.1);
            OldErrors = NewErrors;
        end
        [ResultM1,NewErrors] = CheckPerformance(Torque, ToolPos, RmsTorque,OldErrors,1, 3.0, 5, 0.1);
        if ResultM1 > 0
            disp(sprintf('Robust Performance Criterion for Model Set 1 = %.1f', ResultM1));
        else
            disp('Robust Performance for Model Set 1 not satisfied');
            error = -1;
        end
    end
end

if simulate_model_set_2
    
    % Robust Performance for model set 2
    if error > 0
        disp('ROBUST PERFORMANCE FOR MODEL SET 2:')
        
        rs_models = [...
                1.1 0.9 1.5*0.5 0.5*0.5 1.30 0.75*1.30 1.25*1.30 0.5 0.5 0.5 0.2 0.2 0.0 0.0 1.1 1.1 0.5 1.5;
            0.9 0.9 1.8*0.5 0.2*0.5 1.30 0.72*1.30 1.28*1.30 0.5 0.5 0.5 0.2 0.2 0.0 0.0 0.9 1.1 1.5 0.5;
            1.1 0.9 0.5*0.5 1.5*0.5 1.30 1.40*1.30 1.30*1.30 0.5 0.5 0.5 0.2 0.2 0.0 0.0 1.1 1.1 0.5 0.5;
            0.9 0.9 1.0*0.5 1.0*0.5 1.30 1.70*1.30 0.55*1.30 0.5 0.5 0.5 0.2 0.2 0.0 0.0 0.9 1.1 1.5 1.5;
            1.0 1.1 0.05*1.0 1.95*1 0.70 2.00*1.30 0.50*0.80 0.5 0.5 0.5 0.2 0.2 0.0 0.0 0.9 1.1 0.8 1.5;
            0.9 1.5 1.0*1.5 1.0*1.5 0.70 0.80*0.70 2.20*0.70 0.5 0.5 0.5 0.2 0.2 0.0 0.0 1.1 1.1 1.5 0.5;
            1.1 1.5 1.5*1.5 0.5*1.5 0.70 0.90*0.70 0.40*0.70 0.5 0.5 0.5 0.2 0.2 0.0 0.0 0.9 1.1 0.5 0.5;
            0.9 1.5 0.5*1.5 1.5*1.5 0.70 1.10*0.70 1.80*0.70 0.5 0.5 0.5 0.2 0.2 0.0 0.0 1.1 1.1 1.5 1.5;
            1.1 1.0 1.0*1.5 1.0*1.5 0.70 1.00*0.70 1.00*0.70 0.5 0.5 0.5 0.2 0.2 0.0 0.0 0.9 1.1 0.5 0.5;
            1.0 1.0 1.4*0.5 0.6*0.5 1.30 1.40*1.30 0.25*1.30 0.5 0.5 0.5 0.2 0.2 0.0 0.0 1.1 1.1 1.5 0.5];    
        OldErrors = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]; 
        for ii = 1:10
            [Jm, Ja1, Ja2, Ja3, k1, k2, k3, d1, d2, d3, fm, fa1, fa2, fa3, gear_ratio, k1_low, k1_low_pos, k1_pos, len1, len2, len3, Gain, DelayTime] = ...
                MakeFourMassRobot(rs_models(ii,1),rs_models(ii,2),rs_models(ii,3),rs_models(ii,4),rs_models(ii,5),...
                rs_models(ii,6),rs_models(ii,7),rs_models(ii,8),rs_models(ii,9),rs_models(ii,10),rs_models(ii,11),...
                rs_models(ii,12),rs_models(ii,13),rs_models(ii,14),rs_models(ii,15),rs_models(ii,16),rs_models(ii,17),...
                rs_models(ii,18));           
            % Simulate system
            sim RobotSystem_2;
     
            % Plot Result
            PlotResult(Torque, ToolPos, 15,16 , 'for Model Set 2 & Controller 2');  
            % Plot transfer function
            PlotRobotTransferFunction([Jm, Ja1, Ja2, Ja3, k1, k2, k3, d1, d2, d3, fm, fa1, fa2, fa3, ...
                    Gain, DelayTime, k1_low, k1_pos,len1,len2,len3,gear_ratio], 17, 'for Model Set 2');
            % Check Performance
            [NewResult,NewErrors] = CheckPerformance(Torque, ToolPos, RmsTorque,OldErrors,-1, 4, 5, 0.3);
            OldErrors = NewErrors;
        end
        [ResultM2,NewErrors] = CheckPerformance(Torque, ToolPos, RmsTorque,OldErrors,1, 4, 5, 0.3);
        if ResultM2 > 0
            disp(sprintf('Robust Performance Criterion for Model Set 2 = %.1f', ResultM2));
            if (simulate_model_set_1 & simulate_model_set_2)
                FinalCriterion = beta_nom * ResultNom + beta_1 * ResultM1 + beta_2 * ResultM2;
                disp('Weighted Robust Performance Criterion Nominal Model,');
                disp(sprintf('Model Set 1 and Model Set 2 = %.1f', FinalCriterion));
                disp('Stability must be manually checked by inspection of the signals');
            end
        else
            disp('Robust Performance for Model Set 2 not satisfied');
            error = -1;
        end
    end
end