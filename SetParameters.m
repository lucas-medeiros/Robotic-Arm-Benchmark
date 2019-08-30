% Setting of global parameters
% Disturbances, Actuator Limits, Weights, Measurement Noise and Sample Time

% Actuator limits and measurement system parameters
TorqueLimitMax = 20;    % Maximum Torque
TorqueLimitMin = -20;   % Minimum Torque
NoiseP = 0.5E-10;       % Measurement noise power
NoiseTs = 0.5E-3;       % Measurement noise sample time
SampleTime = 0.5E-3;    % Sample Time

% Weights used when summing the criterion function
beta_nom = 0.6;
beta_1 = 1.0;
beta_2 = 0.3;

% Step Disturbance Torques on Tool
ToolDisturbance1 = 5;
ToolDisturbanceOn1 = 0;
ToolDisturbanceOff1 = 0.5;

% Step Disturbance Torques on Motor
MotorDisturbance1 = 3.5;
MotorDisturbanceOn1 = 5; 
MotorDisturbanceOff1 = 5.5;
MotorDisturbance2 = 3.5;
MotorDisturbanceOn2 = 6; 
MotorDisturbanceOff2 = 25;

% Chirp Disturbance Torques on Tool
ToolDisturbanceChirpStartFrequency1 = 15;
ToolDisturbanceChirpEndFrequency1 = 2;
ToolDisturbanceChirpStartTime1 = 10.5;
ToolDisturbanceChirpEndTime1 = 15.5;
ToolDisturbanceChirpAmplitude1 = 1.5;

% Chirp Disturbance Torques on Motor
MotorDisturbanceChirpStartFrequency1 = 15;
MotorDisturbanceChirpEndFrequency1 = 2;
MotorDisturbanceChirpStartTime1 = 16.5;
MotorDisturbanceChirpEndTime1 = 21.5;
MotorDisturbanceChirpAmplitude1 = 1.5;

% Step Disturbance Torques on Motor & Tool
MotorDisturbance3 = 7;
MotorDisturbanceOn3 = 25; 
MotorDisturbanceOff3 = 1000;
ToolDisturbance2 = -7;
ToolDisturbanceOn2 = 25; 
ToolDisturbanceOff2 = 1000;

% Step Disturbance Torques on Tool
ToolDisturbance3 = 5;
ToolDisturbanceOn3 = 27;
ToolDisturbanceOff3 = 27.5;

% Step Disturbance Torques on Motor
MotorDisturbance4 = 4.5;
MotorDisturbanceOn4 = 32;
MotorDisturbanceOff4 = 32.5;

% Chirp Disturbance Torques on Tool
ToolDisturbanceChirpStartFrequency2 = 15;
ToolDisturbanceChirpEndFrequency2 = 2;
ToolDisturbanceChirpStartTime2 = 37;
ToolDisturbanceChirpEndTime2 = 42;
ToolDisturbanceChirpAmplitude2 = 1.5;

% Chirp Disturbance Torques on Motor
MotorDisturbanceChirpStartFrequency2 = 15;
MotorDisturbanceChirpEndFrequency2 = 2;
MotorDisturbanceChirpStartTime2 = 43;
MotorDisturbanceChirpEndTime2 = 48;
MotorDisturbanceChirpAmplitude2 = 1.5;

% Simulation Time
SimulationTime = 55;