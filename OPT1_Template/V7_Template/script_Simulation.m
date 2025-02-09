clear all; close all; clc;

% Initialisierung / Initialization
disp('Init ...')


% Pfad zu GRAMPC / Path to GRAMPC
grampc_root_path =  'GRAMPC_v2.2/';      %TODO


%% Compilation
compile = 1;
varargin = {}; 

addpath([grampc_root_path 'matlab/mfiles']);
% name of problem function
probfct = 'probfct_Verladebruecke.c';

% compile toolbox
if compile > 1 || ~exist([grampc_root_path 'matlab/bin'], 'dir')
    grampc_make_toolbox(grampc_root_path, varargin{:});
end
% compile problem
if compile > 0 || ~exist('+CmexFiles', 'dir')
    grampc_make_probfct(grampc_root_path, probfct, varargin{:});
end

% Init GRAMPC
[grampc,Tsim,grampc_sdata, cost, param] = initData();

%% Zusaetzliche Parameter (NICHT AENDERN!) / Additional Parameters (DON'T CHANGE!)

param = load_parameter(param,grampc);


disp('Initialisierung abgeschlossen')

