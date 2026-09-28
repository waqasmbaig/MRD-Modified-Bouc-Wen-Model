%% Parameters for Spencer Modified Bouc-Wen MR Damper Model
% Author: W. M. Baig
% Model file: MRD_FDFV.slx
%
% When using this model, please cite:
%   [1] Z. Yu, R. Luo, P. Wu, W. M. Baig, H. Ma, Z. Hou, IEEE TTE 2025 (DOI: 10.1109/TTE.2025.3535765)
%   [2] W. M. Baig, Z. Yu, H. Ma, Z. Hou, IEEE VTC2025-Spring (DOI: 10.1109/VTC2025-Spring65109.2025.11174543)
%   [3] W. M. Baig, Z. Hou, S. Ijaz, CCDC 2017 (pp. 2808-2813)

clear;
clc;

%% Simulation Settings
SimTime = 2.0;               % Simulation Time [s]
fz = 10.0;                   % Harmonic test frequency [Hz]
wc = 2 * pi * fz;            % Excitation angular frequency [rad/s]
X0 = 0.008;                  % Stroke amplitude [m] (8 mm)
Volt = 1.0;                  % Nominal command voltage [V]
Vmax = 2.0;                  % Maximum rated voltage [V]

%% Physical Model Parameters (SI Units)
% Viscous damping parameters
c0_a = 784.0;                % Base dashpot damping [N*s/m]
c0_b = 1803.0;               % Field-induced damping gain [N*s/(m*V)]
k0   = 3610.0;               % Post-yield stiffness [N/m]

% Gas accumulator parameters
c1_a = 14649.0;              % Accumulator damping [N*s/m]
c1_b = 34622.0;              % Field accumulator damping gain [N*s/(m*V)]
k1   = 840.0;                % Accumulator stiffness [N/m]
x0   = 0.0245;               % Accumulator offset displacement [m]

% Bouc-Wen hysteretic parameters
alpha_a = 12441.0;           % Base hysteretic force coefficient [N/m]
alpha_b = 38430.0;           % Field-dependent hysteretic force gain [N/(m*V)]
gamma   = 136320.0;          % Hysteresis shape factor [m^-2]
beta    = 2059020.0;         % Hysteresis shape factor [m^-2]
A       = 58.0;              % Restoring force scale parameter [-]
n       = 2.0;               % Smoothness exponent [-]

% Coil electromagnetic dynamics
eta     = 190.0;             % First-order time response rate [s^-1] (tau = 1/eta ~ 5.26 ms)

fprintf('MR Damper parameters successfully initialized in workspace.\n');
