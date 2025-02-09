function param = load_parameter(param, grampc)


% 1 = AP-Wechsel, 2 = Folge von Ap-Wechseln, Start bei [0, 0], 3 =
% Periodische Solltrajektorienfolgereglung
param.Sollvorgabe = 1;
% Constrain y-Position
param.constrainYLast = 0;
ly0 = 0.72;

%%
param.yL0offset = ly0;
param.tSample = grampc.param.dt;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Systemparameter
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
param.g=9.80665;
param.m1=1.4337; 
param.m2=0.345;
param.J=0.012477;
param.rho=0.03;

% Begrenzungen fuer Seillaenge
param.lmax = 0.85;   
param.lmin = 0.25;  
param.l_m = 0.55;     % mittlere Seillaenge

% Begrenzungen fuer die Wagenposition
param.xmax = 0.6;  
param.xmin = -0.6;  

% Dimensionen Last
param.BreiteLast = 0.035;
param.HoeheLast = 0.045;

% Maximale Antriebskraft des Wagens, maximales Moment des Trommelmotors
param.Fmax = 22.5;  
param.ScaleFmax = 0.95;

param.M_Tmax = 3.75*param.rho*13; 


% Beschraenkungen MPC
% Stellgroessenbeschraenkungen
param.aXmax = 9.0; % ((param.Fmax) / param.m1);
param.aLmax = param.M_Tmax / param.J * param.rho;

% Max. Wagengeschwindigkeit [m/s]
param.vWmax = 2.0;




% Max. Seilwinkel [rad]
param.phimax = 1.0; %0.3; %5*pi/180;

param.yL0offset = 0.72;

% Startzustaende System abh. von Startposition Last (Annahme: phi = 0)
param.xW0 = param.xL0;
param.l0 = param.yL0offset - param.yL0; 



end

