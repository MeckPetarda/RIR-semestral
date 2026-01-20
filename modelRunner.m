clc;
clear;
clf;
close all;
format compact

% Simulation parameters
deltaT = 0.1;              % [s]
g = 9.81;

% Constants
% Radians to degree
RadianToDegree = 180/pi; 
% Degree to radians
DegreeToRadian = pi/180;

% Quadcopter parameters
Mass = 1.3;                 % [kg]
ArmLenght = 0.27;           % [m]
XMomentOfInertia =  0.023;  % [kg m^2]
YMomentOfInertia =  0.023;  % [kg m^2]
ZMomentOfInertia =  0.047;  % [kg m^2]

   
% Define waypoints and their required passage times
timeForWaypointPasage = [140, 280, 360, 480, 600];  % [s]
wayPoints = [0 0 -6;        % Waypoint 1
             1.2 1 -6;      % Waypoint 2
             1.5 0 -5;      % Waypoint 3
             0 1.5 -7;      % Waypoint 4
             -1 1 -6];      % Waypoint 5

%% --- Z REGULATOR ---
% State: [z, v_z]
% Control: Î"T (thrust perturbation from hover)
A_z = [0  1;
       0  0];
B_z = [0; -1/Mass];
poles_z = [-0.5, -0.6];  % Slower poles for altitude
R_z = place(A_z, B_z, poles_z);
fprintf('Z regulator gains R_z: %f, %f\n', R_z(1), R_z(2));

%% --- X REGULATOR ---
% State: [x, v_x]
% Control: Î"Î¸ (desired pitch angle)
A_x = [0  1;
       0  0];
B_x = [0; -g];
poles_x = [-0.3, -0.4];  % Slower poles for position
R_x = place(A_x, B_x, poles_x);
fprintf('X regulator gains R_x: %f, %f\n', R_x(1), R_x(2));

% State: [theta_error, omega_theta]
% Control: M2
A_theta = [0  1;
           0  0];
B_theta = [0; 1/YMomentOfInertia];
poles_theta = [-1.5, -2.0];  % Faster than position loop
R_theta = place(A_theta, B_theta, poles_theta);

%% --- Y REGULATOR ---
% State: [y, v_y]
% Control: Î"Ï† (desired roll angle)
A_y = [0  1;
       0  0];
B_y = [0; g];
poles_y = [-0.3, -0.4];  % Same as X for symmetry
R_y = place(A_y, B_y, poles_y);
fprintf('Y regulator gains R_y: %f, %f\n', R_y(1), R_y(2));

% State: [phi_error, omega_phi]  
% Control: M1
A_phi = [0  1;
         0  0];
B_phi = [0; 1/XMomentOfInertia];
poles_phi = [-1.5, -2.0];  % Same speed as theta
R_phi = place(A_phi, B_phi, poles_phi);

%% --- PSI REGULATOR ---
% State: [psi, psi_dot]
% Control: M3 (yaw moment)
A_psi = [0  1;
         0  0];
B_psi = [0; 1/ZMomentOfInertia];
poles_psi = [-0.2, -0.25];  % Slow yaw correction
R_psi = place(A_psi, B_psi, poles_psi);
fprintf('Psi regulator gains R_psi: %f, %f\n', R_psi(1), R_psi(2));

trajectory_data = TrajectorySetup(wayPoints, timeForWaypointPasage);

% Create Estimation Data Bus for Simulink Model
trajectory_data_bus_info = Simulink.Bus.createObject(trajectory_data);
trajectory_data_bus = evalin('base', trajectory_data_bus_info.busName);

out = sim("model.mdl");

PlotTrajectory3D(out, trajectory_data)
PlotSimulationResults(out)
