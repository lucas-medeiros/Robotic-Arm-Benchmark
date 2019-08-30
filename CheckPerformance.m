function [Result, Errors] = CheckPerformance(Torque, ToolPos, RmsTorque, OldErrors, WriteResult, max_T_settle, max_torque_noise, pos_tolerance)
% [Result, Errors] = CheckPerformance(Torque, ToolPos, RmsTorque, OldErrors, WriteResult, 
%                                     max_T_settle, max_torque_noise, pos_tolerance)
%
% Computes errors and store maximum errors in return parameter Errors if
% WriteResult < 1. User must supply OldErrors as argument
% 
% Compute the performance criterion if WriteResult > 0
%
% Returns criterion value if requirements set by max_T_settle
% (pos_tolerance) and max_torque_noise are fulfilled else returns -1

% Set tolerances and weights
SetParameters; 
alfa_1 = 1;
alfa_2 = 2;
alfa_3 = 2;
alfa_4 = 4;
alfa_5 = 1;
alfa_6 = 2;
alfa_7 = 2;
alfa_8 = 4;
alfa_9 = 4;
alfa_10 = 4;
alfa_11 = 4;
alfa_12 = 4;
alfa_13 = 2;
alfa_14 = 2;
alfa_15 = 5;
gamma = 0.702;
if WriteResult < 0
    % Get Signals
    Torq = Torque.signals.values(:,1);
    TimeT = Torque.time;
    ToolPosition = ToolPos.signals.values(:,1);
    Time = ToolPos.time;
    Rms_Torque = RmsTorque.signals.values(:,1);
    % Evaluate index 
    ind1 = find(Time < ToolDisturbanceOff1);
    ind2 = find(Time < MotorDisturbanceOn1);
    ind3 = find(Time < MotorDisturbanceOn2);
    ind4 = find(Time < ToolDisturbanceChirpStartTime1);
    ind5 = find(Time < MotorDisturbanceChirpStartTime1);
    ind6 = find(Time < MotorDisturbanceOn3);
    ind7 = find(Time < ToolDisturbanceOn3);
    ind75 = find(Time < ToolDisturbanceOff3);
    ind8 = find(Time < MotorDisturbanceOn4);
    ind85 = find(Time < MotorDisturbanceOff4);
    ind9 = find(Time < ToolDisturbanceChirpStartTime2);
    ind10 = find(Time < MotorDisturbanceChirpStartTime2);
    ind11 = find(Time < SimulationTime);
    ind12 = find(TimeT < MotorDisturbanceOn3 - 1);
    ind13 = find(TimeT < MotorDisturbanceOn3);
    ind14 = find(TimeT < SimulationTime - 1);
    ind15 = find(TimeT < SimulationTime);
    % Compute tool position peak-to-peak errors
    err1 = abs(max(ToolPosition(1:ind2(length(ind2))))-min(ToolPosition(1:ind2(length(ind2)))));
    err2 = abs(max(ToolPosition(ind2(length(ind2)):ind4(length(ind4))))-min(ToolPosition(ind2(length(ind2)):ind4(length(ind4)))));
    err3 = abs(max(ToolPosition(ind4(length(ind4)):ind5(length(ind5))))-min(ToolPosition(ind4(length(ind4)):ind5(length(ind5)))));
    err4 = abs(max(ToolPosition(ind5(length(ind5)):ind6(length(ind6))))-min(ToolPosition(ind5(length(ind5)):ind6(length(ind6)))));
    err5 = abs(max(ToolPosition(ind7(length(ind7)):ind8(length(ind8))))-min(ToolPosition(ind7(length(ind7)):ind8(length(ind8)))));
    err6 = abs(max(ToolPosition(ind8(length(ind8)):ind9(length(ind9))))-min(ToolPosition(ind8(length(ind8)):ind9(length(ind9)))));
    err7 = abs(max(ToolPosition(ind9(length(ind9)):ind10(length(ind10))))-min(ToolPosition(ind9(length(ind9)):ind10(length(ind10)))));
    err8 = abs(max(ToolPosition(ind10(length(ind10)):ind11(length(ind11))))-min(ToolPosition(ind10(length(ind10)):ind11(length(ind11)))));
    % Evaluate settling times
    % Ts1
    TempPos1 = ToolPosition(ind1(length(ind1)):ind2(length(ind2)));
    ind100 = find(abs(TempPos1) > pos_tolerance);
    if length(ind100) == 0
        ind100 = 0;
    end
    Ts1 = Time(ind100(length(ind100))+ind1(length(ind1)))-Time(ind1(length(ind1)));
    % Ts2
    TempPos2 = ToolPosition(ind3(length(ind3)):ind4(length(ind4)));
    ind200 = find(abs(TempPos2) > pos_tolerance);
    if length(ind200) == 0
        ind200 = 0;
    end
    Ts2 = Time(ind200(length(ind200))+ind3(length(ind3)))-Time(ind3(length(ind3)));
    % Ts3
    % First check the final position (we have a static error due to the arm disturbance)
    final_position = ToolPosition(length(ToolPosition));
    TempPos3 = ToolPosition(ind75(length(ind75)):ind8(length(ind8)));
    ind300 = find(abs(TempPos3-final_position) > pos_tolerance);
    if length(ind300) == 0
        ind300 = 0;
    end
    Ts3 = Time(ind300(length(ind300))+ind75(length(ind75)))-Time(ind75(length(ind75)));
    % Ts4
    TempPos4 = ToolPosition(ind85(length(ind85)):ind9(length(ind9)));
    ind400 = find(abs(TempPos4-final_position) > pos_tolerance);
    if length(ind400) == 0
        ind400 = 0;
    end
    Ts4 = Time(ind400(length(ind400))+ind85(length(ind85)))-Time(ind85(length(ind85)));
    % Evaluate torque criterions
    torque_noise_1 = abs(max(Torq(ind12(length(ind12)):ind13(length(ind13))))-min(Torq(ind12(length(ind12)):ind13(length(ind13)))));
    torque_noise_2 = abs(max(Torq(ind14(length(ind14)):ind15(length(ind15))))-min(Torq(ind14(length(ind14)):ind15(length(ind15)))));
    torque_noise = max([torque_noise_1 torque_noise_2]);
    MaxTorque = max(abs(Torq));
    RMS_Torque = Rms_Torque(length(Rms_Torque));
    Errors = OldErrors;
    if err1 > OldErrors(1)
        Errors(1) = err1;
    end
    if err2 > OldErrors(2)
        Errors(2) = err2;
    end
    if err3 > OldErrors(3)
        Errors(3) = err3;
    end
    if err4 > OldErrors(4)
        Errors(4) = err4;
    end
    if err5 > OldErrors(5)
        Errors(5) = err5;
    end
    if err6 > OldErrors(6)
        Errors(6) = err6;
    end
    if err7 > OldErrors(7)
        Errors(7) = err7;
    end
    if err8 > OldErrors(8)
        Errors(8) = err8;
    end
    if Ts1 > OldErrors(9)
        Errors(9) = Ts1;
    end
    if Ts2 > OldErrors(10)
        Errors(10) = Ts2;
    end
    if Ts3 > OldErrors(11)
        Errors(11) = Ts3;
    end
    if Ts4 > OldErrors(12)
        Errors(12) = Ts4;
    end
    if torque_noise > OldErrors(13)
        Errors(13) = torque_noise;
    end
    if MaxTorque > OldErrors(14)
        Errors(14) = MaxTorque;
    end
    if RMS_Torque > OldErrors(15)
        Errors(15) = RMS_Torque;
    end 
    Result = 1;
else  
    err1 = OldErrors(1);
    err2 = OldErrors(2);
    err3 = OldErrors(3);
    err4 = OldErrors(4);
    err5 = OldErrors(5);
    err6 = OldErrors(6);
    err7 = OldErrors(7);
    err8 = OldErrors(8);
    Ts1 = OldErrors(9);
    Ts2 = OldErrors(10);
    Ts3 = OldErrors(11);
    Ts4 = OldErrors(12);
    torque_noise = OldErrors(13);
    MaxTorque = OldErrors(14);
    RMS_Torque = OldErrors(15);
    
    disp(sprintf('[e1 e2 e3 e4 e5 e6 e7 e8] = [%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f]',err1,err2,err3,err4,err5,err6,err7,err8));
    disp(sprintf('[Ts1 Ts2 Ts3 Ts4] = [%.2f %.2f %.2f %.2f] (Ts < %.1f for tolerance %.1f mm)', Ts1, Ts2, Ts3, Ts4, max_T_settle, pos_tolerance));
    disp(sprintf('Torque noise = %.2f (< %.1f), Max Torque = %.2f, RMS Torque = %.2f', ...
        torque_noise, max_torque_noise, MaxTorque, RMS_Torque));
    % Compute performance criterion
    if ((Ts1 < max_T_settle) & (Ts2 < max_T_settle) & (Ts3 < max_T_settle) & (Ts4 < max_T_settle) & (torque_noise < max_torque_noise))
        PerformanceCriterion = alfa_1*err1 + alfa_2*err2 + alfa_3*err3 + alfa_4*err4 + alfa_5*err5 + alfa_6*err6 ...
            + alfa_7*err7 + alfa_8*err8 + alfa_9*Ts1 + alfa_10*Ts2 + alfa_11*Ts3 + alfa_12*Ts4 ...
            + alfa_13*torque_noise + alfa_14*MaxTorque + alfa_15*RMS_Torque;
        Result = PerformanceCriterion * gamma;
        Errors = OldErrors;
    else
        Errors = OldErrors;
        Result = -1;
    end 
end

