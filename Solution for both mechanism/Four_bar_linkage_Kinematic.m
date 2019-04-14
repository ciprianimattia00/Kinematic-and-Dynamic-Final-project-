%% CMIM Final Project

% Daphne van Dijken
% Mattia Cipriani
% Vojtech Pospisil

clear all
close all
clc

%% kINEMATIC ANALYSIS
%% initial values
BD = sqrt(4^2 + 10^2 - 2*4*10*cosd(120));
gamma = acosd((BD^2 - 10^2 - 12^2) / (-2*12*10));
alpha = asind(10/BD*sind(gamma));
beta = asind(4/BD*sind(120));
zeta = alpha + beta;
teta = 360 - 120 - gamma - zeta;
AG2 = sqrt(4^2 + 5^2 - 2*4*5*cosd(teta));
eta = asind(5/AG2*sind(teta));
omega = 120 - eta;
xG2 = AG2*cosd(omega);
yG2 = AG2*sind(omega);
delta = teta - (360 - 120 -90 - 90);

%% Coordinates
% ground 1
q1 = [0; 0; 0]; 
% link 1
q2 = [-0.02 * sind(30)
      0.02 * cosd(30)
      deg2rad(120)];
% link 2
q3 = [xG2/100
      yG2/100
      deg2rad(delta)];
% link 3
q4 = [0.1-0.06*cosd(zeta)
      0.06*sind(zeta)
      deg2rad(-zeta)];
% ground 2
q5 = [0.1; 0; 0];

q_0 = [q1; q2; q3; q4; q5]; % initial coordinates

%% Revolute joints
% 1 connects ground 1 and link 1
revolute(1).i = 1;
revolute(1).j = 2;
revolute(1).s_i = [0; 0];
revolute(1).s_j = [-0.02;0];

% 2 connects link 1 and link 2
revolute(2).i = 2;
revolute(2).j = 3;
revolute(2).s_i = [0.02; 0];
revolute(2).s_j = [-0.05; 0];

% 3 connects link 2 and link 3
revolute(3).i = 3;
revolute(3).j = 4;
revolute(3).s_i = [0.05; 0];
revolute(3).s_j = [-0.06; 0];

% 4 connects link 3 and ground 2
revolute(4).i = 4;
revolute(4).j = 5;
revolute(4).s_i = [0.06; 0];
revolute(4).s_j = [0; 0];

%% Simple constraints

% Three simple joints to fix the ground 1 origin
simple(1).i = 1;
simple(1).k = 1;
simple(1).c_k = 0;

simple(2).i = 1;
simple(2).k = 2;
simple(2).c_k = 0;

simple(3).i = 1;
simple(3).k = 3;
simple(3).c_k = 0;

% slider - use simple joints instead of translational
simple(4).i = 5;
simple(4).k = 1;
simple(4).c_k = 0.1;

simple(5).i = 5;
simple(5).k = 2;
simple(5).c_k = 0;

simple(6).i = 5;
simple(6).k = 3;
simple(6).c_k = 0;

%% Driving constraints
driving.i = 2;
driving.k = 3;
driving.d_k = @(t) (2/3)*pi + 45 * t;
driving.d_k_t = @(t) 45;
driving.d_k_tt = @(t) 0;

%% Jacobian of our constraints
Cq = constraint_dq(revolute, simple, driving, 0, q_0);

%% Solve constraint equation using NR for position and velocity
C_fun = @(t, q) constraint(revolute, simple, driving, t, q);
Cq_fun = @(t, q) constraint_dq(revolute, simple, driving, t, q);
Ct_fun = @(t, q) constraint_dt(revolute, simple, driving, t, q);
Ctt_fun = @(t, q, dq) constraint_dtt(revolute, simple, driving, t, dq, q);
[T, Q, QP, QPP] = pos_vel_acc_NR(C_fun, Cq_fun, Ct_fun, Ctt_fun, 0.2, q_0, 0.001);

% %% Plot for POSITIONS
% figure
% plot(Q(:, 4), Q(:, 5), ...
%     Q(:, 7), Q(:, 8), ...
%     Q(:, 10), Q(:, 11), ...
%     0.1, 0, '*', ...
%     0, 0, '*', 'LineWidth', 2);
% axis equal
% title('Positions');
% legend('link 1','link 2','link 3');
% xlabel('x [m]');
% ylabel('y [m]');
% 
% %% Plot for VELOCITIES
% figure
% plot(QP(:, 4), QP(:, 5), ...
%     QP(:, 7), QP(:, 8), ...
%     QP(:, 10), QP(:, 11), ...
%     0, 0, '*', 'LineWidth', 2);
% axis equal
% title('Velocities');
% legend('link 1','link 2','link 3');
% xlabel('v_x [m/s]');
% ylabel('v_y [m/s]');
% 
% %% Plot for ACCELERATIONS
% figure
% plot(QPP(:, 4), QPP(:, 5), ...
%     QPP(:, 7), QPP(:, 8), ...
%     QPP(:, 10), QPP(:, 11), ...
%     0, 0, '*', 'LineWidth', 2);
% axis equal
% title('Accelerations');
% legend('link 1','link 2','link 3');
% xlabel('a_x [m/s^2]');
% ylabel('a_y [m/s^2]');