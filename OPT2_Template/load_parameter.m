function param = load_parameter(param)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Parameter Solltrajektorie
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
param.trajType = param.bx;
param.amplX = param.ax;
param.amplY = param.ay;

% Trajectorenfolgeregelung (v-Traj)
param.vsoll = 0.5;



% Fundamentale Abtastrate
param.tSample = 1e-3;

param.g=9.80665;

% Kugelparameter / Ball parameters
param.mK = 0.096; 
param.rK = 14.3e-3;
param.JK = (2/5)*param.mK*param.rK^2;

param.B = param.mK / ( (param.JK/(param.rK*param.rK)) + param.mK);


% Systembeschraenkungen / System constraints
param.yKmax = 0.2365;
param.xKmax = 0.1775;

% Maximale Plattenwinkel / Maximum plate angles
% Innen / inner
param.phiMax = deg2rad(14);
% Aussen / outer
param.thetaMax = deg2rad(13);


% Traegheitsmomente Platten / Moment of inertia plates
param.JIx = 0.11108;
param.JAy = 0.34602;


% Beschraenkungen MPC / Constraints MPC
% Stellgroessenbeschraenkungen / Input constraints
param.angAccMaxA = 10.0;
param.angAccMaxI = 23.0;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Feste Parameter für die Sensorik und Aktorik:                           %
% Fixed Parameter for the sensors and actors:                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 0° - Festlegung für die Absolut Inkrementalgeber / 
% 0° - Setting for the absolute incremental encoder
param.Ink1_zero_point = -61.509;      % [°] Innerer Rahmen / Inner frame
param.Ink2_zero_point = -96.623;       % [°] Äußerer Rahmen / Outer frame

% 0-Punkt-Offset für das Touchpad / 0-point-offset for the touchpad
param.x_offset = -1975;               % [dots]
param.y_offset = -1975;               % [dots] 

% Umrechnungsfaktor von Volt zu Ampere für den aktuellen Motorstrom
% Conversion factor from Volt to Ampere for current motor current
param.Volt2Amp = 30;                  % [A/V]

% Maximales Moment, dass auf den Rahmen ausgeübt werden kann /
% Maximum moment that can be applied to the frame
% Übersetzungsverhältnis des Riemens * Drehmomentkonstante * I_max * 95% /
% Transmission ratio of the belt * Torque constant * I_max * 95%
param.M_max = 2*0.114*16;             % [Nm]
param.Ma_max = param.M_max;
param.Mi_max = param.M_max/2;

% Gegenmoment bei Winkelüberschreitung, welches auf den Rahmen eingreifen soll /
% Counter torque in case of angular overshoot, which should act on the frame
param.Mi_gegen = param.Mi_max;                % [Nm]
param.Ma_gegen = param.Ma_max;                % [Nm]

% Bandbreite der Drehmomentregelung (in [Hz]) / Torque control bandwidth (in [Hz])
param.f_I = 1000;                       % [Hz]

% Umrechnungsfaktor von Moment zu Ampere / Conversion factor from moment to ampere
param.M2A = 0.5/0.114;                % [A/Nm]

% Umrechnungsfaktor von Ampere zu Volt für Motoransteuerung /
% Conversion factor from ampere to Volt for motor control
param.Amp2Volt = 0.0333;              % [V/A]

% Vorlaufzeit für Beobachter für Positionsregelung /
% Lead time for observer for position control
param.pos_reg_delay = 100;            % [0.2s]

% Vorlaufzeit für Beobachter für Winkelregelung /
% Lead time for observer for angular control
param.wink_reg_delay = 1;            % [0.04s]
       
% Drehmomentkonstante / Torque constant
param.k = 0.114;      % [Nm/A]


%% Parameter für Winkelregelung / Parameter for angle control
 % Berechnung der Koeffizienten für die Positionssteuerung

    % allgemeine Größen:
    % Gewichtskraft g:
    g   = 9.81;       % [m/s^2]

    % Motor 1:
    % Trägheitsmoment J1:
    J1  = 0.099;     % [kgm^2]
    % Übersetzungsverhältnis u1: (es wird das Moment direkt für den Rahmen
    % berechnet)
    u1  = 1;        % []
    % viskose Reibkonstante
    Rx = 0.1;
    % Konstanten K1_1
    param.wink_reg_K1_1 = u1/J1;
    K1_1 = u1/J1;
    K1_3 = -Rx/J1;

    % Zustandsraumdarstellung:
    A1 = [0 1 0; 0 K1_3 0; 1 0 0];
    B1 = [0; K1_1; 0];

    %Eigenwerte:
    P1 = [-7, -7, -7];

    % Berechnung mit acker:
    param.wink_reg_r1 = acker(A1,B1,P1);
%     r1 = wink_reg_r1;

    % Vorsteuerung:
    param.wink_reg_Mx1 = 0.333;

    % Beobachter:
    param.wink_reg_L1_1 = 18;


    % Motor 2:
    % Trägheitsmoment J2:
    J2  = 0.435;     % [kgm^2]
    % Übersetzungsverhältnis u1: (es wird das Moment direkt für den Rahmen
    % berechnet)
    u2  = 1;        % []
    % Masse m:
    m = 11.343;        % [kg]
    % Abstand Achse -> Schwerpunkt
    a = 0.0353;      % [m]
    % viskose Reibkonstante
    Ry = 0.348;
    % a = J2/m/g*pi^2;
    % Konstanten K2_1
    param.wink_reg_K2_1 = -u2/J2;
    K2_1 = -u2/J2;
    % Konstanten K2_2
    K2_2 = -a*g*m/J2/u2;
    % Konstante K2_3
    K2_3 = -Ry/J2;
    % Konstante K2_4
    param.wink_reg_K2_4 = -0.048;
    %Vorsteuerung Mu:
    param.wink_reg_Mu2 = -K2_2/K2_1;
    param.wink_reg_Mx2 = 0.3826;

    % Zustandsraumdarstellung:
    A2 = [0 1 0;K2_2 K2_3 0;1 0 0];
    B2 = [0; K2_1; 0];

    %Eigenwerte:
    P2 = [-6 -6 -6];

    % Berechnung mit acker:
    param.wink_reg_r2 = acker(A2,B2,P2);

    % Beobachter:
    param.wink_reg_L2_1 = 18;

    display('Parameter für Winkel-Regelung geladen');



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Parameter unterlagerte Winkelgeschwindigkeitsregelung Platten
%  Parameter for inner angular velocity control of the plates
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
param.vRegelungSollType = 0;

% Innere Platte / Inner plate
param.kvPhi = 100;  
param.kvPhiInt = 0;
param.kdirektI = 1;

% Aeussere Platte / Outer plate
param.kvTheta = 100;
param.kvThetaInt = 0;
param.kdirektA = 1;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Parameter Messwertfilterung
%  Parameter measurement filtering
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Offset in der Winkelmessung [deg] / Offset in angle measurements
param.offsetPhi = -0.55;
param.offsetTheta = 0.05;


% Savitzky-Golay Ableitungsfilter (v phi) /
% Savitzky-Golay derivative filter (v phi)
window = 21;
polyorder = 1;
param.deriv_vPhi = 1;
[~, g] = sgolay(polyorder, window);
param.coeffs_vPhi = fliplr(g(:,param.deriv_vPhi+1)');

% Savitzky-Golay Ableitungsfilter (v theta)
window = 21;
polyorder = 1;
param.deriv_vTheta = 1;
[~, g] = sgolay(polyorder, window);
param.coeffs_vTheta = fliplr(g(:,param.deriv_vTheta+1)');

% Savitzky-Golay Ableitungsfilter (xK)
window = 201;
polyorder = 1;
param.deriv_vXk = 1;
[~, g] = sgolay(polyorder, window);
param.coeffs_vXk = fliplr(g(:,param.deriv_vXk+1)');

% Savitzky-Golay Ableitungsfilter (yK)
window = 201;
polyorder = 1;
param.deriv_vYk = 1;
[~, g] = sgolay(polyorder, window);
param.coeffs_vYk = fliplr(g(:,param.deriv_vYk+1)');


% Filterung phi / Filtering phi
param.expMAparam_phi = 0.99;

% Filterung theta / Filtering theta
param.expMAparam_theta = 0.99;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Parameter flachheitsbasierte Folgeregelung zum Startpunkt anfahren
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Zeit fuer SP anfahren
param.tApWechselSP = 4.0;

% Linearisierungsart (0 = feedback linearization, 1 = feedforward linearization) 
param.linearizationTypeSP = 0;

% Eigenwerte Folgeregelung yK (innerer Rahmen)
param.eigYKSP = 4;

% Eigenwerte Folgeregelung xK (aeusserer Rahmen)
param.eigXKSP = 4;


% Winkelgeschwindigkeitsregelung Platte
param.amplVphi = 0.1; %deg2rad(5);
param.freqVphi = 3; %0.3*2*pi;
param.amplVtheta = -0.1; %deg2rad(5);
param.freqVtheta = 3; %0.3*2*pi;

end

