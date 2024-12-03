% =========================================================================
% FILE NAME: beam_and_ball_param_and_sim.m
% =========================================================================
% PURPOSE:
%   Simulates the dynamics of a ball-and-beam system controlled using an
%   LQR (Linear Quadratic Regulator). The script evaluates system
%   performance using metrics such as IAE, ISE, ITAE, settling time, and
%   overshoot, and visualizes the results.
%
% AUTHOR: Alexander Brown
% DATE:   2024-12-02
%
% VERSION: 1.0
%   - Initial implementation of the ball-and-beam simulation.
%
% INPUTS:
%   - Initial conditions for ball position and velocity.
%   - Reference state (desired position).
%   - System and control parameters (LQR weights, system matrices).
%
% OUTPUTS:
%   - Performance metrics: IAE, ISE, ITAE, settling time, percent overshoot
%   - Plots of state trajectories and control effort over time.
%
% SYSTEM PARAMETERS:
%   - Beam length, height, and width.
%   - Ball mass, radius, and moment of inertia.
%   - Gravitational acceleration.
%
% INSTRUCTIONS:
%   1. Configure the system and simulation parameters as desired.
%   2. Run the script in MATLAB.
%   3. Analyze the generated plots and performance metrics.
%
% DEPENDENCIES:
%   This script requires the following toolboxes and files:
%
%   1. MATLAB (Core) [Version 24.2 or higher]
%      - Required for basic operations, numerical computation, and plots.
%
%   2. Control System Toolbox [Version 24.2 or higher]
%      - Required for state-space modeling and LQR design.
%      - Functions used: lqr, ss, etc.
%
%   Files:
%   - beam_and_ball_param_and_sim.m (this script)
%   - beam_and_ball_LQR_controller.slx (required for system 3D modeling)
%
%   Note:
%   If used in combination with Simulink model for 3D visualization:
%      - Simscape Multibody Toolbox required for the `Mechanics Explorer`.
%      - Dependencies for Simulink models should be verified separately.
%
%   Ensure all dependencies are installed and accessible before running.
%
% =========================================================================

%% Housekeeping
clear, close all, clc;

%% System Parameters
% beam
L = 1.0;
w = 0.05;
h = 0.1;

% ball
m = 0.5;
r = 0.05;
g = 9.81;
I = 0.02;

% system matrices
A = [0 1 0 0; 0 0 g/L 0; 0 0 0 1; 0 0 -m*g*r/I 0];
B = [0; 0; 0; 1/I];
C = eye(4);
D = 0;

% LQR weights
Q = diag([200, 10, 10, 10]);
R = 1;

% desired_poles = [-2, -3, -4, -5];
% K = acker(A, B, desired_poles);

% gain calculation
K = lqr(A, B, Q, R);
Ki = 1;

% Simulation Parameters
T = 10;
x0 = [0.1; 0; 0; 0];
x_ref = [0; 0; 0; 0];

% system
dt = 0.01; % step
time = 0:dt:T;
x = x0;
u_hist = []; % control input history
x_hist = []; % state history
error_hist = []; % error history

for t = time
    e = x_ref - x;
    u = -K*x;
    u_hist = [u_hist, u];
    x_hist = [x_hist, x];
    error_hist = [error_hist, e];
    
    % euler integration
    dx = A*x + B*u;
    x = x + dx*dt;
end

% cost function calculations
e_pos = error_hist(1, :);
IAE = trapz(time, abs(e_pos));
ISE = trapz(time, e_pos.^2);
ITAE = trapz(time, time .* abs(e_pos));

% % settling time
% tolerance = 0.02;
% x_final = x_hist(1, end);
% settling_indices = find(abs(x_hist(1, :) - x_final) ...
%     > tolerance*abs(x_final));
% if isempty(settling_indices)
%     settling_time = 0;
% else
%     settling_time = time(settling_indices(end));
% end
% 
% % percent overshoot
% x_max = max(x_hist(1, :));
% percent_overshoot = ((x_max - x_final) / abs(x_final))*100;

fprintf('Cost Analysis Results:\n');
fprintf('IAE: %.5f\n', IAE);
fprintf('ISE: %.5f\n', ISE);
fprintf('ITAE: %.5f\n', ITAE);
% fprintf('Settling Time: %.2f seconds\n', settling_time);
% fprintf('Percent Overshoot: %.2f%%\n', percent_overshoot);

% visualize system simulation
figure;
% subplot(2, 1, 1);
plot(time, x_hist(1, :), 'r', 'LineWidth', 1.5); hold on;
plot(time, x_hist(3, :), 'b', 'LineWidth', 1.5);
legend('Ball Position (x)', 'Beam Angle (\theta)');
xlabel('Time (s)');
ylabel('State Values');
title('System States');
grid on;

figure;
% subplot(2, 1, 2);
plot(time, u_hist, 'k', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Control Input (u)');
title('Control Input');
grid on;