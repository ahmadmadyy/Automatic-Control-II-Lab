function [grampc,Tsim,grampc_sdata, cost, param] = initData()

% Startposition Last / Initial position load r_l,0
param.xL0 =  -0.55;   %TODO x_L
param.yL0 = 0;    %TODO y_tilde_L

% Zielposition Last / target position load r_L,z
param.xLsoll = 0.55; %TODO  
param.yLsoll = 0.0; %TODO

% Startzustaende System abh. von Startposition Last 
% Initial states dependent on initial load position 
x0 =   -0.55;   %TODO Wagenposition / Cart position (Annahme: theta = 0) /(Assumption: theta = 0)
ly0 =  0.72;   %TODO Seillänge Unterkante / Rope length lower edge
l0 =  0.72;    %TODO y_L Seillänge / Rope length

% Zielzustaende System abh. von Zielposition Last 
% Target states dependent on target load position 
xdes =   param.xLsoll;   %TODO Wagenposition / Cart position (Annahme: theta = 0) /(Assumption: theta = 0)
ldes =  param.yLsoll;    %TODO y_L Seillänge / Rope length


% Initial values and setpoints of the states
user.param.x0    = [x0, 0.0, l0, 0.0, 0.0, 0.0];
user.param.xdes  = [xdes, 0.0, ldes, 0.0, 0.0, 0.0];

% Initial values, setpoints and limits of the inputs
user.param.u0    = [0.0,0.0];
user.param.udes  = [0.0,0.0];
user.param.umax  = [9,3.5];   %TODO Stellgrößenbeschränkung / Input constraints
user.param.umin  = [-9,-3.5];

% Time variables
user.param.Thor  =     1;     %TODO Prädiktionshorizont / Prediction horizon

user.param.dt    =     0.001;     %TODO Sampling time

user.param.t0    = 0.0;     % time at the current sampling step

%% Option definition
% Basic algorithmic options
user.opt.Nhor =    20;         %TODO Number of steps for the system integration

user.opt.TerminalCost = 'on'; %TODO 'on' or 'off'

user.opt.MaxGradIter =   5;   %TODO Gradientiterationen
user.opt.MaxMultIter =   2;   %TODO Multplikator-Updates

user.opt.Integrator = 'euler';

% Constraints tolerances
user.opt.InequalityConstraints = 'on';
%user.opt.ConstraintsAbsTol = 1e-4*[1 1 1 1 1 1 1]; 

user.opt.LineSearchExpAutoFallback = 'on';

% optional settings for a better performance
% user.opt.LineSearchMax = 1e1;
% user.opt.LineSearchInit = 1e-1;
% user.opt.LineSearchExpAutoFallback = 'off';
user.opt.PenaltyMin = 1e3; % Comment line (grampc = CmexFiles.grampc_estim_penmin_Cmex(grampc,1);) to use this option value


%% Userparameter definition 
% e.g. system parameters or weights for the cost function

% Integralkosten Gewichtungen / Integral cost weights
% Position & Geschwindigkeiten / Position & Velocities
cost.xL = 2.5; % 1
cost.x_dot = 2;
cost.yL = 2;
cost.l_dot = 1;
% Winkel & Geschwindigkeit / Angle & Velocity
cost.theta = 1;
cost.theta_dot = 4;
%Stellgrößen / Control inputs
cost.ax = 0.05; %0.05
cost.al = 0.05; %0.05

%Endkosten Gewichtungen / Terminal cost weights
cost.xLend = 1;
cost.yLend = 1;
%TODO add more if necessary


pCost = [cost.xL, cost.x_dot, cost.yL, cost.l_dot, ...
         cost.theta, cost.theta_dot, cost.ax, cost.al, ...
         cost.xLend, cost.yLend]; 
%TODO add more if necessary

userparam = pCost*20;

%%
param.yL0offset = 0.72;    % ly0;
param.tSample = user.param.dt;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Grampc initialization
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
grampc = CmexFiles.grampc_init_Cmex(userparam);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Update grampc struct while ensuring correct data types
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
grampc = grampc_update_struct_grampc(grampc,user);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Estimate and set PenaltyMin
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [grampc, ~] = CmexFiles.grampc_estim_penmin_Cmex(grampc,1);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Simulation time
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if nargout>1
    Tsim = 12.5;
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Simulink data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Converting the grampc struct to a code generation compatible format for
% the use in simulink
if nargout>2
    grampc_sdata = grampc_generate_sdata(grampc);
end
