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
% Beam
L = 1.0; w = 0.05; h = 0.1;

% Ball
m = 0.5; r = 0.05; g = 9.81; I = 0.02;

% System matrices
A = [0 1 0 0; 0 0 g/L 0; 0 0 0 1; 0 0 -m*g*r/I 0];
B = [0; 0; 0; 1/I];
C = eye(4); D = 0;

%% Controller Design
% LQR weights
Q = diag([200, 10, 10, 10]);
R = 1;

% Controller Gains
desired_poles = [-158, -2+3i, -2-3i, -3.5];
K_acker = acker(A, B, desired_poles);
K_lqr = lqr(A, B, Q, R);

%% Simulation
% Simulation Parameters
T = 10; dt = 0.01;
time = 0:dt:T;
x0 = [-0.2; 0; 0; 0];
x_ref = [0; 0; 0; 0];

% Initialize histories
x_hist_acker = [];
u_hist_acker = [];
x_hist_lqr = [];
u_hist_lqr = [];

% Simulate Pole Placement Controller
x = x0;
for t = time
    e = x_ref - x;
    u = -K_acker * x;
    u_hist_acker = [u_hist_acker, u];
    x_hist_acker = [x_hist_acker, x];

    % Euler integration
    dx = A * x + B * u;
    x = x + dx * dt;
end

% Simulate LQR Controller
x = x0;
for t = time
    e = x_ref - x;
    u = -K_lqr * x;
    u_hist_lqr = [u_hist_lqr, u];
    x_hist_lqr = [x_hist_lqr, x];

    % Euler integration
    dx = A * x + B * u;
    x = x + dx * dt;
end


%% Report
% Performance Metrics
e_pos_acker = x_hist_acker(1, :) - x_ref(1);
IAE_acker = trapz(time, abs(e_pos_acker));
ISE_acker = trapz(time, e_pos_acker.^2);
ITAE_acker = trapz(time, time .* abs(e_pos_acker));

e_pos_lqr = x_hist_lqr(1, :) - x_ref(1);
IAE_lqr = trapz(time, abs(e_pos_lqr));
ISE_lqr = trapz(time, e_pos_lqr.^2);
ITAE_lqr = trapz(time, time .* abs(e_pos_lqr));

% Display Results
fprintf('Cost Analysis Results:\n');
fprintf('Acker - IAE: %.5f, ISE: %.5f, ITAE: %.5f\n', IAE_acker, ISE_acker, ITAE_acker);
fprintf('LQR - IAE: %.5f, ISE: %.5f, ITAE: %.5f\n', IAE_lqr, ISE_lqr, ITAE_lqr);

% Visualization
figure;
% State Responses
subplot(2, 1, 1);
plot(time, x_hist_acker(1, :), 'b', 'LineWidth', 1.5); hold on;
plot(time, x_hist_acker(3, :), 'r', 'LineWidth', 1.5);
plot(time, x_hist_lqr(1, :), '--b', 'LineWidth', 1.5);
plot(time, x_hist_lqr(3, :), '--r', 'LineWidth', 1.5);
legend('x - Acker', '\theta - Acker', 'x - LQR', '\theta - LQR');
xlabel('Time (s)');
ylabel('State Values');
title('State Response Comparison - Pole Placement vs. LQR');
grid on;

% Control Inputs
subplot(2, 1, 2);
plot(time, u_hist_acker, 'b', 'LineWidth', 1.5); hold on;
plot(time, u_hist_lqr, '--r', 'LineWidth', 1.5);
legend('u - Acker', 'u - LQR');
xlabel('Time (s)');
ylabel('Control Input');
title('Control Input Comparison - Pole Placement vs. LQR');
grid on;
