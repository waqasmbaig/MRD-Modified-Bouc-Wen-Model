%% Script to simulate MRD_FDFV.slx across varying control voltages
% and plot Force-Displacement and Force-Velocity curves.

clear; clc; close all;

%% Load parameters
run('init_params.m');

model_name = 'MRD_FDFV';
load_system(model_name);

voltages = [0.0, 0.5, 1.0, 1.5, 2.0];
colors = lines(length(voltages));

figure('Name', 'MR Damper Hysteresis Loops', 'Position', [100, 100, 1100, 480]);

for i = 1:length(voltages)
    V_val = voltages(i);
    
    % Set Constant block voltage in Simulink
    set_param([model_name '/Constant'], 'Value', num2str(V_val));
    
    % Run simulation
    simOut = sim(model_name, 'StopTime', num2str(SimTime));
    
    % Extract signals from simulation output / logged scopes
    % Assuming default scope or outport logs
    t = simOut.tout;
    % Piston displacement and velocity from Sinusoid generator:
    x = X0 * sin(wc * t - pi/2);
    x_dot = X0 * wc * cos(wc * t - pi/2);
    
    % If F_MR is an outport in MRD_FDFV
    if isfield(simOut, 'F_MR')
        F_MR = simOut.F_MR.Data;
    elseif isfield(simOut, 'yout')
        F_MR = simOut.yout{1}.Values.Data;
    else
        % Fallback for direct scope extraction
        warning('Extracting F_MR from Scope data or workspace variable');
    end
    
    % Steady-state selection (last 2 cycles)
    idx_ss = t >= (SimTime - 2/fz);
    
    % 1. Force - Displacement Plot
    subplot(1, 2, 1);
    plot(x(idx_ss) * 1000, F_MR(idx_ss), 'LineWidth', 1.8, 'Color', colors(i, :));
    hold on;
    
    % 2. Force - Velocity Plot
    subplot(1, 2, 2);
    plot(x_dot(idx_ss), F_MR(idx_ss), 'LineWidth', 1.8, 'Color', colors(i, :));
    hold on;
end

% Format subplots
subplot(1, 2, 1);
title(sprintf('Force vs. Displacement (f = %.0f Hz)', fz), 'FontWeight', 'bold');
xlabel('Displacement [mm]');
ylabel('Damper Force [N]');
grid on;
legend(arrayfun(@(v) sprintf('V = %.1f V', v), voltages, 'UniformOutput', false), 'Location', 'northwest');

subplot(1, 2, 2);
title(sprintf('Force vs. Velocity (f = %.0f Hz)', fz), 'FontWeight', 'bold');
xlabel('Piston Velocity [m/s]');
ylabel('Damper Force [N]');
grid on;
legend(arrayfun(@(v) sprintf('V = %.1f V', v), voltages, 'UniformOutput', false), 'Location', 'northwest');

fprintf('Simulation and plotting completed.\n');
