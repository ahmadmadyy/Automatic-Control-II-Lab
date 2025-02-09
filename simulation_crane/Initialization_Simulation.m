clear all; close all; clc;
ToDo = NaN;

% Initalization
disp('Initialization of the flatness-based control...')

% Simulation model
% 0: kinematic model
% 1: reference model
param.Simulationsmodell = 0;


% Coefficients of the parabola y_L = a2 * x_L^2 + a1 * x_L + a0
param.a0 = 0.35;
param.a1 = 0;
param.a2 = 1.48;

% Type of trajectory:
% 1 = SP change
% 2 = Sequence of SP changes, start at [0, l0].
% 3 = Trajectory (parabola) over container
param.Sollvorgabe = 1;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initial state and position of the load
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Time period between setpoint changes for type of trajectory = 2
param.tTask = 4.0;

% Initial positions 
% Transition times of desired trajectories
% Simulation time
switch  param.Sollvorgabe
    case 1 % SP-Change
        param.xW0 = -0.50; 
        param.l0  =  0.72; 
        param.T_traj = 3.0;
        param.Tsim = param.T_traj + 2;
    case 2 %  Sequence of SP-Changes
        param.xW0 =  0.00; 
        param.l0  =  0.72; 
        param.T_traj = 3.0;
        param.Tsim = param.tTask * 6 + 2;
    case 3 % Parabola
        param.xW0 = -0.50; 
        param.l0  =  0.72; 
        param.T_traj = 8.0;
        param.Tsim = param.T_traj + 2;
    otherwise
        error(['Sollvorgabe #' num2str(param.Sollvorgabe) ' nicht definiert!'])
end

% Target end position for SP change
param.xLsoll_0 =  -0.50;   
param.yLsoll_0 =  0.72;
param.xLsoll_T =  0.50;   
param.yLsoll_T =  0.37;

% Sequence of AP changes positions
param.xLsoll_links  = -0.5;
param.xLsoll_rechts =  0.5;
param.yLsoll_oben   =  0.72-0.35;
param.yLsoll_unten  =  0.72;

if isnan(param.a0) || isnan(param.a1) || isnan(param.a2)
   error('Tragen Sie die Werte param.a0, param.a1 und param.a2 ein!')
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialization of the flatness-based control
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Controller selection:
% 1: Feed forward control
% 2: Quasi-static tracking control
% 3: Dynamic tracking control
param.aktFolgeregelung = 1; 

% Control amplifications
% x-load
% Coefficient for the polynomial pX01 * x^3 + pX02 * x^2 + pX03 * x + pX04
param.pX01 = 256;         
param.pX02 = 256;
param.pX03 = 96;
param.pX04 = 16;

% y-load
% Control amplifications
% Quasi-static tracking control
% Coefficient for the polynomial pY01 * x + pX02
param.pY01_QS = 8;
param.pY02_QS = 16;

% y-load
% Control amplifications
% Danamic tracking control
% Coefficient for the polynomial pY01 * x^3 + pY02 * x^2 + pY03 * x + pY04
param.pY01_DYN = 256;
param.pY02_DYN = 256;
param.pY03_DYN = 96; 
param.pY04_DYN = 16;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% System parameters - Do not change!
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
param.g=9.81;    % Erdbeschleunigung
param.m1=1.4337; % Masse Wagen
param.m2=0.345;  % Masse Last
param.J=0.012477;% Tr gheitsmoment der Tromme
param.rho=0.03;  % Radius der Trommel

% Begrenzungen fuer Seillaenge
param.lmax = 0.85;                      % Seill nge untere Lage
param.lmin = 0.25;                      % Seill nge obere Lage
param.l_m  = (param.lmax+param.lmin)/2; % mittlere Seillaenge

% Begrenzungen fuer die Wagenposition
param.xmax =  0.6;  % rechte Begrenzung
param.xmin = -0.6;  % linke  Begrenzung

% Dimensionen Last
param.BreiteLast = 0.035;
param.HoeheLast = 0.045;

% Fundamentale Abtastrate
param.tSample = 1e-2;

param.p0 = param.a0;
param.p1 = param.a1;
param.p2 = param.a2;

disp('Initialization completed')