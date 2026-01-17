%% Extract and visualize Simulink simulation results

function PlotSimulationResults(out)
    
    % Extract time and state data

    t = out.simout.Time;
    % Data is [12x1x1241], convert to [1241x12]
    states = permute(squeeze(out.simout.Data), [2, 1]);

    
    % Verify dimensions
    if size(states, 1) ~= length(t)
        fprintf('ERROR: Time length = %d, States rows = %d\n', length(t), size(states, 1));
        fprintf('States size: %d x %d\n', size(states));
        error('Mismatch between time vector and state data');
    end
    
    if size(states, 2) ~= 12
        error('Expected 12 states, got %d', size(states, 2));
    end
    
    % Extract individual states
    pos_x = states(:, 1);
    pos_y = states(:, 2);
    pos_z = states(:, 3);
    vel_x = states(:, 4);
    vel_y = states(:, 5);
    vel_z = states(:, 6);
    phi = states(:, 7);
    theta = states(:, 8);
    psi = states(:, 9);
    dPhi = states(:, 10);
    dTheta = states(:, 11);
    dPsi = states(:, 12);
    
    % Create figure with subplots
    fig = figure('Position', [100, 100, 1600, 1000]);
    
    % Position
    subplot(4, 3, 1);
    plot(t, pos_x, 'r-', 'LineWidth', 1.5);
    grid on; xlabel('Time (s)'); ylabel('X (m)');
    title('Position - X');
    
    subplot(4, 3, 2);
    plot(t, pos_y, 'g-', 'LineWidth', 1.5);
    grid on; xlabel('Time (s)'); ylabel('Y (m)');
    title('Position - Y');
    
    subplot(4, 3, 3);
    plot(t, pos_z, 'b-', 'LineWidth', 1.5);
    grid on; xlabel('Time (s)'); ylabel('Z (m)');
    title('Position - Z');
    
    % Velocity
    subplot(4, 3, 4);
    plot(t, vel_x, 'r-', 'LineWidth', 1.5);
    grid on; xlabel('Time (s)'); ylabel('vX (m/s)');
    title('Velocity - X');
    
    subplot(4, 3, 5);
    plot(t, vel_y, 'g-', 'LineWidth', 1.5);
    grid on; xlabel('Time (s)'); ylabel('vY (m/s)');
    title('Velocity - Y');
    
    subplot(4, 3, 6);
    plot(t, vel_z, 'b-', 'LineWidth', 1.5);
    grid on; xlabel('Time (s)'); ylabel('vZ (m/s)');
    title('Velocity - Z');
    
    % Euler angles (rad to deg)
    subplot(4, 3, 7);
    plot(t, phi * 180/pi, 'r-', 'LineWidth', 1.5);
    grid on; xlabel('Time (s)'); ylabel('φ (deg)');
    title('Euler Angle - Roll');
    
    subplot(4, 3, 8);
    plot(t, theta * 180/pi, 'g-', 'LineWidth', 1.5);
    grid on; xlabel('Time (s)'); ylabel('θ (deg)');
    title('Euler Angle - Pitch');
    
    subplot(4, 3, 9);
    plot(t, psi * 180/pi, 'b-', 'LineWidth', 1.5);
    grid on; xlabel('Time (s)'); ylabel('ψ (deg)');
    title('Euler Angle - Yaw');
    
    % Angular velocities
    subplot(4, 3, 10);
    plot(t, dPhi * 180/pi, 'r-', 'LineWidth', 1.5);
    grid on; xlabel('Time (s)'); ylabel('dφ/dt (deg/s)');
    title('Angular Velocity - Roll Rate');
    
    subplot(4, 3, 11);
    plot(t, dTheta * 180/pi, 'g-', 'LineWidth', 1.5);
    grid on; xlabel('Time (s)'); ylabel('dθ/dt (deg/s)');
    title('Angular Velocity - Pitch Rate');
    
    subplot(4, 3, 12);
    plot(t, dPsi * 180/pi, 'b-', 'LineWidth', 1.5);
    grid on; xlabel('Time (s)'); ylabel('dψ/dt (deg/s)');
    title('Angular Velocity - Yaw Rate');
    
    sgtitle('Quadcopter Simulation Results');
    
    % Print statistics
    fprintf('\n=== Simulation Statistics ===\n');
    fprintf('Simulation time: %.2f seconds\n', t(end) - t(1));
    fprintf('Number of samples: %d\n\n', length(t));
    
    fprintf('Position ranges:\n');
    fprintf('  X: %.4f to %.4f m\n', min(pos_x), max(pos_x));
    fprintf('  Y: %.4f to %.4f m\n', min(pos_y), max(pos_y));
    fprintf('  Z: %.4f to %.4f m\n', min(pos_z), max(pos_z));
    
    fprintf('\nVelocity ranges:\n');
    fprintf('  vX: %.4f to %.4f m/s\n', min(vel_x), max(vel_x));
    fprintf('  vY: %.4f to %.4f m/s\n', min(vel_y), max(vel_y));
    fprintf('  vZ: %.4f to %.4f m/s\n', min(vel_z), max(vel_z));
    
    fprintf('\nEuler angle ranges (degrees):\n');
    fprintf('  φ (Roll): %.2f to %.2f°\n', min(phi)*180/pi, max(phi)*180/pi);
    fprintf('  θ (Pitch): %.2f to %.2f°\n', min(theta)*180/pi, max(theta)*180/pi);
    fprintf('  ψ (Yaw): %.2f to %.2f°\n', min(psi)*180/pi, max(psi)*180/pi);
    
end