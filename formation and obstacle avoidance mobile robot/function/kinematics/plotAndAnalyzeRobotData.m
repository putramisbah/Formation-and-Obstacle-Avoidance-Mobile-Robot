function plotAndAnalyzeRobotData(tVec, v_safe1_array, w_safe1_array, v_safe2_array, w_safe2_array, ...
                                 v_safe3_array, w_safe3_array, errorPositionFollower2, errorPositionFollower3, ...
                                 xTargetPosition2_array, yTargetPosition2_array, xcurPose2_array, ycurPose2_array, ...
                                 xTargetPosition3_array, yTargetPosition3_array, xcurPose3_array, ycurPose3_array, ...
                                 pose1, waypoints1, collisionCount, collisionTimes, outputFolder)
    % Set up screen and resolution
    screenSize = get(0, 'ScreenSize');  % Full screen
    fontSize = 20;                      % Font size for labels, titles, legends, and tick marks

    %% Plot v_safe and w_safe for each robot
    figure('Position', [0, 0, screenSize(3), screenSize(4)], 'PaperPositionMode', 'auto');

    subplot(3,2,1);
    plot(tVec, v_safe1_array, 'LineWidth', 2);
    xlabel('Time [s]', 'FontSize', fontSize);
    ylabel('v_{safe1} [m/s]', 'FontSize', fontSize);
    title('v_{safe1} vs time', 'FontSize', fontSize);
    set(gca, 'FontSize', fontSize);
    grid on;

    subplot(3,2,2);
    plot(tVec, w_safe1_array, 'LineWidth', 2);
    xlabel('Time [s]', 'FontSize', fontSize);
    ylabel('w_{safe1} [rad/s]', 'FontSize', fontSize);
    title('w_{safe1} vs time', 'FontSize', fontSize);
    set(gca, 'FontSize', fontSize);
    grid on;

    subplot(3,2,3);
    plot(tVec, v_safe2_array, 'LineWidth', 2);
    xlabel('Time [s]', 'FontSize', fontSize);
    ylabel('v_{safe2} [m/s]', 'FontSize', fontSize);
    title('v_{safe2} vs time', 'FontSize', fontSize);
    set(gca, 'FontSize', fontSize);
    grid on;

    subplot(3,2,4);
    plot(tVec, w_safe2_array, 'LineWidth', 2);
    xlabel('Time [s]', 'FontSize', fontSize);
    ylabel('w_{safe2} [rad/s]', 'FontSize', fontSize);
    title('w_{safe2} vs time', 'FontSize', fontSize);
    set(gca, 'FontSize', fontSize);
    grid on;

    subplot(3,2,5);
    plot(tVec, v_safe3_array, 'LineWidth', 2);
    xlabel('Time [s]', 'FontSize', fontSize);
    ylabel('v_{safe3} [m/s]', 'FontSize', fontSize);
    title('v_{safe3} vs time', 'FontSize', fontSize);
    set(gca, 'FontSize', fontSize);
    grid on;

    subplot(3,2,6);
    plot(tVec, w_safe3_array, 'LineWidth', 2);
    xlabel('Time [s]', 'FontSize', fontSize);
    ylabel('w_{safe3} [rad/s]', 'FontSize', fontSize);
    title('w_{safe3} vs time', 'FontSize', fontSize);
    set(gca, 'FontSize', fontSize);
    grid on;

    % Save figure to the 'figures' folder
    savefig(fullfile(outputFolder, 'velocity_plots.fig'));

    %% Plot errors for both followers
    figure('Position', [0, 0, screenSize(3), screenSize(4)], 'PaperPositionMode', 'auto');

    subplot(2,1,1);
    plot(tVec, errorPositionFollower2, 'LineWidth', 2);
    xlabel('Time [s]', 'FontSize', fontSize);
    ylabel('errorPositionFollower2 [m]', 'FontSize', fontSize);
    title('errorPositionFollower2 vs. time', 'FontSize', fontSize);
    set(gca, 'FontSize', fontSize);
    grid on;

    subplot(2,1,2);
    plot(tVec, errorPositionFollower3, 'LineWidth', 2);
    xlabel('Time [s]', 'FontSize', fontSize);
    ylabel('errorPositionFollower3 [m]', 'FontSize', fontSize);
    title('errorPositionFollower3 vs. time', 'FontSize', fontSize);
    set(gca, 'FontSize', fontSize);
    grid on;

    % Save figure
    savefig(fullfile(outputFolder, 'error_plots.fig'));

    %% Target Position vs. Current Position of Followers
    nonZeroIndices2 = xTargetPosition2_array ~= 0 & yTargetPosition2_array ~= 0 & ...
                     xcurPose2_array ~= 0 & ycurPose2_array ~= 0;

    nonZeroIndices3 = xTargetPosition3_array ~= 0 & yTargetPosition3_array ~= 0 & ...
                     xcurPose3_array ~= 0 & ycurPose3_array ~= 0;

    figure('Position', [0, 0, screenSize(3), screenSize(4)], 'PaperPositionMode', 'auto');

    subplot(2,1,1);
    plot(xTargetPosition2_array(nonZeroIndices2), yTargetPosition2_array(nonZeroIndices2), 'r--', 'LineWidth', 2);
    hold on;
    plot(xcurPose2_array(nonZeroIndices2), ycurPose2_array(nonZeroIndices2), 'b', 'LineWidth', 2);
    xlabel('X Position [m]', 'FontSize', fontSize);
    ylabel('Y Position [m]', 'FontSize', fontSize);
    title('Target Position vs. Current Position of Follower 2', 'FontSize', fontSize);
    legend('Target Position 2', 'Current Pose 2', 'Location', 'southeast', 'FontSize', fontSize);
    set(gca, 'FontSize', fontSize);
    grid on;
    hold off;

    subplot(2,1,2);
    plot(xTargetPosition3_array(nonZeroIndices3), yTargetPosition3_array(nonZeroIndices3), 'r--', 'LineWidth', 2);
    hold on;
    plot(xcurPose3_array(nonZeroIndices3), ycurPose3_array(nonZeroIndices3), 'b', 'LineWidth', 2);
    xlabel('X Position [m]', 'FontSize', fontSize);
    ylabel('Y Position [m]', 'FontSize', fontSize);
    title('Target Position vs. Current Position of Follower 3', 'FontSize', fontSize);
    legend('Target Position 3', 'Current Pose 3', 'Location', 'southeast', 'FontSize', fontSize);
    set(gca, 'FontSize', fontSize);
    grid on;
    hold off;

    % Save figure
    savefig(fullfile(outputFolder, 'target_vs_position.fig'));

    %% Calculate averages and distances
    avg_v_safe1 = mean(v_safe1_array);
    avg_v_safe2 = mean(v_safe2_array);
    avg_v_safe3 = mean(v_safe3_array);

    avg_w_safe1 = mean(w_safe1_array);
    avg_w_safe2 = mean(w_safe2_array);
    avg_w_safe3 = mean(w_safe3_array);

    distance1 = trapz(tVec, v_safe1_array);
    distance2 = trapz(tVec, v_safe2_array);
    distance3 = trapz(tVec, v_safe3_array);

    goal_position_robot1 = waypoints1(end, :);
    distance_to_goal_robot1 = sqrt((pose1(1,:) - goal_position_robot1(1)).^2 + (pose1(2,:) - goal_position_robot1(2)).^2);
    goal_threshold_position = 0.1;
    goal_idx_robot1_position = find(distance_to_goal_robot1 < goal_threshold_position, 1);
    time_to_goal_robot1 = tVec(goal_idx_robot1_position);

    distance12 = mean(errorPositionFollower2);
    distance13 = mean(errorPositionFollower3);

    %% Prepare data for simulation result table
    data = [
        avg_v_safe1, avg_v_safe2, avg_v_safe3;
        avg_w_safe1, avg_w_safe2, avg_w_safe3;
        NaN, distance12, distance13;  % NaN for unavailable data
        distance1, distance2, distance3;
        time_to_goal_robot1, NaN, NaN;  % NaN for non-applicable data
    ];

    % Define variable names for the simulation result table
    resultTable = table( ...
        data(:, 1), data(:, 2), data(:, 3), ...
        'VariableNames', {'Robot_1', 'Robot_2', 'Robot_3'}, ...
        'RowNames', { ...
            'Average Speed (m/s)', ...
            'Average Rotational Speed (rad/s)', ...
            'Position Error (m)', ...
            'Total Distance Traveled (m)', ...
            'Time to Goal (s)' ...
        });

    % Display the simulation result table
    disp('======================= Simulation Results =======================');
    disp(resultTable);
    disp('=================================================================');

    %% Display Total Collision Count separately
    disp(['Total collision count: ', num2str(collisionCount)]);

    %% Display collision times in a table format (if available)
    if ~isempty(collisionTimes)
        collisionTable = table((1:length(collisionTimes))', collisionTimes', ...
            'VariableNames', {'Collision_Number', 'Time_Seconds'});
        disp('======================= Collision Times =====================');
        disp(collisionTable);
        disp('==============================================================');
    else
        disp('No collisions occurred during the simulation.');
    end
end
