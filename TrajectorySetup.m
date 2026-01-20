function trajectory_data = TrajectorySetup()
    % TrajectorySetup - Pre-compute cubic spline trajectory
    %
    % This script generates the complete trajectory offline (before Simulink runs)
    % and saves it to a MAT file that the Simulink function can load.
    %
    % Call this once before running your Simulink simulation:
    %   TrajectorySetup()
    %
    % This creates 'trajectory_data.mat' in the current directory
    
    clear all;
    
    % Define waypoints and their required passage times
    timeForWaypointPasage = [140, 280, 360, 480, 600];  % [s]
    wayPoints = [0 0 -6;        % Waypoint 1
                 1.2 1 -6;      % Waypoint 2
                 1.5 0 -5;      % Waypoint 3
                 0 1.5 -7;      % Waypoint 4
                 -1 1 -6];      % Waypoint 5
    
    % Create cubic spline interpolants (position)
    pp_x = spline(timeForWaypointPasage, wayPoints(:,1));
    pp_y = spline(timeForWaypointPasage, wayPoints(:,2));
    pp_z = spline(timeForWaypointPasage, wayPoints(:,3));
    
    % Create velocity splines (first derivatives)
    pp_x_dot = fnder(pp_x);
    pp_y_dot = fnder(pp_y);
    pp_z_dot = fnder(pp_z);
    
    % Create acceleration splines (second derivatives)
    pp_x_ddot = fnder(pp_x_dot);
    pp_y_ddot = fnder(pp_y_dot);
    pp_z_ddot = fnder(pp_z_dot);
    
    % Generate fine time grid for lookup table
    t_min = timeForWaypointPasage(1);
    t_max = timeForWaypointPasage(end);
    dt_grid = 1;  % 100 ms resolution - adjust if needed
    time_grid = (t_min : dt_grid : t_max)';
    
    n_points = length(time_grid);
    
    % Pre-compute all trajectory parameters on the grid
    pos_x = ppval(pp_x, time_grid);
    pos_y = ppval(pp_y, time_grid);
    pos_z = ppval(pp_z, time_grid);
    
    vel_x = ppval(pp_x_dot, time_grid);
    vel_y = ppval(pp_y_dot, time_grid);
    vel_z = ppval(pp_z_dot, time_grid);
    
    acc_x = ppval(pp_x_ddot, time_grid);
    acc_y = ppval(pp_y_ddot, time_grid);
    acc_z = ppval(pp_z_ddot, time_grid);
    
    % Compute heading and speed
    heading = atan2(vel_y, vel_x);
    speed = sqrt(vel_x.^2 + vel_y.^2 + vel_z.^2);
    
    % Pack everything into a structure
    trajectory_data.time = time_grid;
    trajectory_data.pos_x = pos_x;
    trajectory_data.pos_y = pos_y;
    trajectory_data.pos_z = pos_z;
    trajectory_data.vel_x = vel_x;
    trajectory_data.vel_y = vel_y;
    trajectory_data.vel_z = vel_z;
    trajectory_data.acc_x = acc_x;
    trajectory_data.acc_y = acc_y;
    trajectory_data.acc_z = acc_z;
    trajectory_data.heading = heading;
    trajectory_data.speed = speed;
    trajectory_data.t_min = t_min;
    trajectory_data.t_max = t_max;
    
    fprintf('Trajectory setup complete!\n');
    fprintf('  Time range: %.0f to %.0f seconds\n', t_min, t_max);
    fprintf('  Grid resolution: %.3f seconds (%.0f points)\n', dt_grid, n_points);
    fprintf('  Saved to: trajectory_data.mat\n\n');
    
    % Print some statistics
    fprintf('Trajectory statistics:\n');
    fprintf('  Speed range: %.4f to %.4f m/s\n', min(speed), max(speed));
    fprintf('  Max acceleration: %.4f m/s^2\n', max(sqrt(acc_x.^2 + acc_y.^2 + acc_z.^2)));
    fprintf('  X position range: %.4f to %.4f m\n', min(pos_x), max(pos_x));
    fprintf('  Y position range: %.4f to %.4f m\n', min(pos_y), max(pos_y));
    fprintf('  Z position range: %.4f to %.4f m\n', min(pos_z), max(pos_z));
    
end