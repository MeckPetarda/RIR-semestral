%% 3D trajectory and additional visualizations

function PlotTrajectory3D(out)
    
    % Extract time and state data
    t = out.simout.Time;
    % Data is [12x1x1241], convert to [1241x12]
    states = permute(squeeze(out.simout.Data), [2, 1]);

    
    % Verify dimensions
    if size(states, 1) ~= length(t)
        error('Mismatch between time vector and state data');
    end
    
    if size(states, 2) ~= 12
        error('Expected 12 states, got %d', size(states, 2));
    end
    
    % Extract position and angles
    pos_x = states(:, 1);
    pos_y = states(:, 2);
    pos_z = states(:, 3);
    vel_x = states(:, 4);
    vel_y = states(:, 5);
    vel_z = states(:, 6);
    phi = states(:, 7);
    theta = states(:, 8);
    psi = states(:, 9);
    
    fig = figure('Position', [100, 100, 1400, 900]);
    
    % 3D trajectory
    subplot(3, 3, [1, 4, 7]);
    plot3(pos_x, pos_y, -pos_z, 'b-', 'LineWidth', 2);
    hold on;
    plot3(pos_x(1), pos_y(1), -pos_z(1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g'); % Start
    plot3(pos_x(end), pos_y(end), -pos_z(end), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r'); % End
    grid on; xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title('3D Trajectory');
    axis equal;
    view(45, 30);
    legend('Path', 'Start', 'End');
    
    % XY plane (top view)
    subplot(3, 3, [2, 3]);
    plot(pos_x, pos_y, 'b-', 'LineWidth', 2);
    hold on;
    plot(pos_x(1), pos_y(1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    plot(pos_x(end), pos_y(end), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    % Add arrows showing direction at several points
    n_arrows = min(10, length(t));
    arrow_idx = round(linspace(1, length(t), n_arrows));
    for i = 1:length(arrow_idx)
        idx = arrow_idx(i);
        scale = 0.3;
        quiver(pos_x(idx), pos_y(idx), scale*cos(psi(idx)), scale*sin(psi(idx)), 0, 'k-', 'LineWidth', 1);
    end
    grid on; xlabel('X (m)'); ylabel('Y (m)');
    title('XY Plane (Top View) with Heading');
    axis equal;
    
    
    % XZ plane (side view)
    subplot(3, 3, [5, 8]);
    plot(pos_x, -pos_z, 'b-', 'LineWidth', 2);
    hold on;
    plot(pos_x(1), -pos_z(1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    plot(pos_x(end), -pos_z(end), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    grid on; xlabel('X (m)'); ylabel('Z (m)');
    title('XZ Plane (Side View)');
    axis equal;
    
    sgtitle('Trajectory Visualization');

    % YZ plane (side view)
    subplot(3, 3, [6, 9]);
    plot(pos_y, -pos_z, 'b-', 'LineWidth', 2);
    hold on;
    plot(pos_y(1), -pos_z(1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    plot(pos_y(end), -pos_z(end), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    grid on; xlabel('Y (m)'); ylabel('Z (m)');
    title('YZ Plane (Side View)');
    axis equal;
    
    sgtitle('Trajectory Visualization');
    
end