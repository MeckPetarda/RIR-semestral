clc;
clear;
clf;
close all;
format compact

% At these times, the quadcopter must pass the appropriate waypoint
timeForWaypointPasage = [140,280,360,480,600]; % [s]

% Waypoints - these points must be flown by quadcopter
wayPoints = [0 0 -6;        % [X, Y, Z] - waypoint in [m]
             1.2 1 -6;
             1.5 0 -5;
             0 1.5 -7;
             -1 1 -6];
% Position tolerance
positionTolerance = 0.1;    % [m]

% Simulation parameters
deltaT = 0.1;              % [s]
simulationTime = max(timeForWaypointPasage) + 20; % [s]
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

% Initial state of quadcopter
% quadcopterInitState.BodyXYZPosition.X = 0;  % [m]
% quadcopterInitState.BodyXYZPosition.Y = 0;  % [m]
% quadcopterInitState.BodyXYZPosition.Z = -6; % [m]           
% quadcopterInitState.BodyXYZVelocity.X = 0;            
% quadcopterInitState.BodyXYZVelocity.Y = 0;            
% quadcopterInitState.BodyXYZVelocity.Z = 0;
% quadcopterInitState.BodyEulerAngle.Phi = 0;
% quadcopterInitState.BodyEulerAngle.Theta = 0;
% quadcopterInitState.BodyEulerAngle.Psi = 0;
% quadcopterInitState.BodyAngularRate.dPhi = 0;
% quadcopterInitState.BodyAngularRate.dTheta = 0;
% quadcopterInitState.BodyAngularRate.dPsi = 0;

% --- Z REGULATOR ---
% State: [z, v_z]
% Control: Î"T (thrust perturbation from hover)
A_z = [0  1;
       0  0];
B_z = [0; -1/Mass];
poles_z = [-0.5, -0.6];  % Slower poles for altitude
R_z = place(A_z, B_z, poles_z);
fprintf('Z regulator gains R_z: %f, %f\n', R_z(1), R_z(2));

% --- X REGULATOR ---
% State: [x, v_x]
% Control: Î"Î¸ (desired pitch angle)
A_x = [0  1;
       0  0];
B_x = [0; -g];
poles_x = [-0.3, -0.4];  % Slower poles for position
R_x = place(A_x, B_x, poles_x);
fprintf('X regulator gains R_x: %f, %f\n', R_x(1), R_x(2));

% --- Y REGULATOR ---
% State: [y, v_y]
% Control: Î"Ï† (desired roll angle)
A_y = [0  1;
       0  0];
B_y = [0; g];
poles_y = [-0.3, -0.4];  % Same as X for symmetry
R_y = place(A_y, B_y, poles_y);
fprintf('Y regulator gains R_y: %f, %f\n', R_y(1), R_y(2));

% --- PSI REGULATOR ---
% State: [psi, psi_dot]
% Control: M3 (yaw moment)
A_psi = [0  1;
         0  0];
B_psi = [0; 1/ZMomentOfInertia];
poles_psi = [-0.2, -0.25];  % Slow yaw correction
R_psi = place(A_psi, B_psi, poles_psi);
fprintf('Psi regulator gains R_psi: %f, %f\n', R_psi(1), R_psi(2));

% State: [theta_error, omega_theta]
% Control: M2
A_theta = [0  1;
           0  0];
B_theta = [0; 1/YMomentOfInertia];
poles_theta = [-1.5, -2.0];  % Faster than position loop
R_theta = place(A_theta, B_theta, poles_theta);

% State: [phi_error, omega_phi]  
% Control: M1
A_phi = [0  1;
         0  0];
B_phi = [0; 1/XMomentOfInertia];
poles_phi = [-1.5, -2.0];  % Same speed as theta
R_phi = place(A_phi, B_phi, poles_phi);

% Control variables - total thrust and moments on each control axis
% quadcopterInitControlInputs = [0, 0, 0, 0]';     % (T, M1, M2, M3)
                           
% Initiate quadcopter
% quadcopter = Quadcopter(Mass, ...               
%                XMomentOfInertia, ...
%                YMomentOfInertia, ...
%                ZMomentOfInertia, ...
%                quadcopterInitState, ...
%                quadcopterInitControlInputs,...
%                deltaT);

% Store history for plotting
% history.time = [];
% history.z = [];
% history.x = [];
% history.y = [];
% history.psi = [];
% history.T = [];
% history.theta_des = [];
% history.phi_des = [];
% history.psi_meas = [];

trajectory_data = TrajectorySetup();

% Create Estimation Data Bus for Simulink Model
trajectory_data_bus_info = Simulink.Bus.createObject(trajectory_data);
trajectory_data_bus = evalin('base', trajectory_data_bus_info.busName);

out = sim("model.slx");

PlotTrajectory3D(out)
PlotSimulationResults(out)

% % Simulation
% for t = 1 : deltaT : simulationTime
% 
%     % ===== POSITION/ALTITUDE REGULATORS (OUTER LOOP) =====
%     % All regulators compute error from reference and apply feedback
% 
%     state = quadcopter.GetState();
% 
%     X = state.BodyXYZPosition.X;
%     Y = state.BodyXYZPosition.Y;
%     Z = state.BodyXYZPosition.Z;
% 
%     vX = state.BodyXYZVelocity.X;
%     vY = state.BodyXYZVelocity.Y;
%     vZ = state.BodyXYZVelocity.Z;
% 
%     phi = state.BodyEulerAngle.Phi;
%     theta = state.BodyEulerAngle.Theta;
%     psi = state.BodyEulerAngle.Psi;
% 
%     omega_phi = state.BodyAngularRate.dPhi;
%     omega_theta = state.BodyAngularRate.dTheta;
%     omega_psi = state.BodyAngularRate.dPsi;
% 
%     target = [X, Y, Z];
% 
%     for j = 1:length(timeForWaypointPasage)
%         if (timeForWaypointPasage(j) > t)
%             target = wayPoints(j, :);
%             break;
%         end
%     end
% 
%     x_ref = target(1);
%     y_ref = target(2);
%     z_ref = target(3);
% 
%     phi_ref = 0;
%     theta_ref = 0;
%     psi_ref = 0;
% 
%     % Z regulator
%     z_error = [Z - z_ref; vZ];
%     delta_T = -R_z * z_error;  % Î"T from regulator
%     T_command = Mass*g + delta_T;  % Total thrust = hover thrust + perturbation
% 
%     max_angle = 10*DegreeToRadian;
% 
%     % X regulator
%     x_error = [X - x_ref; vX];
%     theta_des = max(min(-R_x * x_error, max_angle), -max_angle);  % Desired pitch angle (note the sign)
% 
%     % Y regulator
%     y_error = [Y - y_ref; vY];
%     phi_des = max(min(-R_y * y_error, max_angle), -max_angle);   % Desired roll angle
% 
%     max_angle = 30 * DegreeToRadian;  % Max 30 degree tilt
%     theta_des = max(min(theta_des, max_angle), -max_angle);
%     phi_des = max(min(phi_des, max_angle), -max_angle);
% 
%     % Psi regulator
%     psi_error = [psi - psi_ref; omega_psi];
%     M3_command = -R_psi * psi_error;  % Yaw moment for heading control
% 
%     % ===== ATTITUDE CONTROLLER (MIDDLE LAYER) =====
%     % Takes desired Î¸, Ï† and compares with actual Î¸, Ï†
% 
%     theta_error_state = [theta - theta_des; omega_theta];
%     M2_command = -R_theta * theta_error_state;
% 
%     phi_error_state = [phi - phi_des; omega_phi];
%     M1_command = -R_phi * phi_error_state;
% 
%     T_max = 2 * Mass * g;        % Max thrust (twice hover)
%     T_min = 0;                    % No negative thrust
%     max_moment = 0.1;             % Max moment in Nâ‹…m
% 
%     % Then apply saturation:
%     T_command = max(min(T_command, T_max), T_min);
% 
%     M1_command = max(min(M1_command, max_moment), -max_moment);
%     M2_command = max(min(M2_command, max_moment), -max_moment);
%     M3_command = max(min(M3_command, max_moment), -max_moment);
% 
%     % Action for total thrust
%     quadcopter.TotalThrustControlAction(T_command);
%     % Action for attitude
%     quadcopter.AttitudeControlAction(M1_command, M2_command, M3_command);
% 
%     % Update state of quadcopter
%     quadcopter.UpdateState();
% 
%     % Get actual state of quadcopter
%     quadcopterActualState = quadcopter.GetState();
% 
%     % Crash check
%     if (quadcopterActualState.BodyXYZPosition.Z >= 0)
%         msgbox('Quadcopter Crashed!', 'Error', 'error');
%         break;
%     end
% 
%     % Waypoint check
%     if (CheckWayPointTrack(...
%                 quadcopterActualState.BodyXYZPosition,...
%                 t,...
%                 timeForWaypointPasage,...
%                 wayPoints,...
%                 positionTolerance))
%         %msgbox('Quadcopter did not passed waypoint', 'Error', 'error');
%         %break;
%     end
% 
%         % ===== LOGGING =====
%     history.time = [history.time; t];
%     history.z = [history.z; Z];
%     history.x = [history.x; X];
%     history.y = [history.y; Y];
%     history.psi = [history.psi; psi];
%     history.T = [history.T; T_command];
%     history.theta_des = [history.theta_des; theta_des];
%     history.phi_des = [history.phi_des; phi_des];
%     history.psi_meas = [history.psi_meas; psi];
% end                              
% 
% %% ========== PLOTTING ==========
% figure('Position', [100 100 1200 800]);
% 
% subplot(2,3,1);
% plot(history.time, history.z, 'b', 'LineWidth', 2);
% hold on; plot(history.time, z_ref*ones(size(history.time)), 'r--', 'LineWidth', 1.5);
% xlabel('Time (s)'); ylabel('Altitude Z (m)'); legend('Actual', 'Reference');
% grid on;
% 
% subplot(2,3,2);
% plot(history.time, history.x, 'b', 'LineWidth', 2);
% hold on; plot(history.time, x_ref*ones(size(history.time)), 'r--', 'LineWidth', 1.5);
% xlabel('Time (s)'); ylabel('Position X (m)'); legend('Actual', 'Reference');
% grid on;
% 
% subplot(2,3,3);
% plot(history.time, history.y, 'b', 'LineWidth', 2);
% hold on; plot(history.time, y_ref*ones(size(history.time)), 'r--', 'LineWidth', 1.5);
% xlabel('Time (s)'); ylabel('Position Y (m)'); legend('Actual', 'Reference');
% grid on;
% 
% subplot(2,3,4);
% plot(history.time, history.T, 'g', 'LineWidth', 2);
% hold on; plot(history.time, Mass*g*ones(size(history.time)), 'k--', 'LineWidth', 1);
% xlabel('Time (s)'); ylabel('Thrust T (N)');
% legend('Command', 'Hover thrust');
% grid on;
% 
% subplot(2,3,5);
% plot(history.time, history.theta_des, 'r', 'LineWidth', 1.5);
% hold on; plot(history.time, history.phi_des, 'b', 'LineWidth', 1.5);
% xlabel('Time (s)'); ylabel('Angle (rad)');
% legend('Î¸_{des}', 'Ï†_{des}');
% grid on;
% 
% subplot(2,3,6);
% plot(history.time, history.psi, 'b', 'LineWidth', 2);
% hold on; plot(history.time, psi_ref*ones(size(history.time)), 'r--', 'LineWidth', 1.5);
% xlabel('Time (s)'); ylabel('Heading Ïˆ (rad)');
% legend('Actual', 'Reference');
% grid on;
