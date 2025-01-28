function [v_safe, w_safe] = velocity_obstacle(robot_pos, v_robot, w_robot, goal_pos, obs_pos, v_obs, w_obs, radius, max_velocity, speed_dif, k_size)
    % multi_agent_velocity_optimal_v2 Computes optimal and safe linear and angular velocities for the robot
    % to avoid collisions with both dynamic and stationary obstacles.

    N_points = 100;  % Number of velocity samples
    distance_threshold = 3;  % Distance threshold to consider VO
    safe_radius_multiplier = 1.5;  % Safety margin multiplier for obstacle radius

    % Initialize safe linear and angular velocities
    v_safe = v_robot;
    w_safe = w_robot;

    % Generate random velocities within a defined range around the robot's current velocity
    [x, y] = generate_velocity_samples(N_points, v_robot, w_robot, speed_dif, k_size, max_velocity);
    sample_velocity = [x; y];  % Sample velocities in global coordinates

    % Initialize the minimum penalty
    min_penalty = inf;

    % Compute the direction towards the goal
    goal_direction = atan2(goal_pos(2) - robot_pos(2), goal_pos(1) - robot_pos(1));

    % Iterate through each obstacle
    for i = 1:size(obs_pos, 1)
        % Current obstacle's position and velocity
        cur_obs_pos = obs_pos(i, :);
        cur_v_obs = v_obs(i);
        cur_w_obs = w_obs(i);

        % Calculate relative position between robot and obstacle
        relative_pos = cur_obs_pos - robot_pos;
        dist = norm(relative_pos);  % Distance between robot and obstacle
        
        % Add a safety margin to the obstacle radius
        safe_radius = radius * safe_radius_multiplier;

        % Skip obstacles that are too far away
        if dist > distance_threshold
            continue;
        end

        % Skip obstacles that are too close
        if dist < safe_radius
            warning('Obstacle too close to the robot!');
            continue;
        end

        % Handle stationary obstacle
        if abs(cur_v_obs) < 1e-3 && abs(cur_w_obs) < 1e-3 && dist < 0.5
            % The obstacle is stationary
            angle_to_obs = atan2(relative_pos(2), relative_pos(1));
            theta = asin(safe_radius / dist);
            left_bound = wrapToPi(angle_to_obs - theta);
            right_bound = wrapToPi(angle_to_obs + theta);
        else
            % Dynamic obstacle: Compute relative velocity
            relative_vel = [v_robot; w_robot] - [cur_v_obs; cur_w_obs];
            angle_to_obs = atan2(relative_pos(2), relative_pos(1));
            theta = asin(safe_radius / dist);
            left_bound = wrapToPi(angle_to_obs - theta);
            right_bound = wrapToPi(angle_to_obs + theta);
        end

        % Check sample velocities for safety
        for j = 1:size(sample_velocity, 2)
            vel_sample = sample_velocity(:, j);
            angle_sample = atan2(vel_sample(2), vel_sample(1));
            angle_sample = wrapToPi(angle_sample);

            % Check if the sampled velocity is within the VO cone
            if is_inside_vo_cone(angle_sample, left_bound, right_bound)
                continue;
            end

            % Calculate distance from the optimal velocity towards the goal
            distance_optimal = norm([v_robot; w_robot] - vel_sample);
            goal_alignment = abs(wrapToPi(angle_sample - goal_direction));

            % Penalize velocities that are too close to the VO bounds or misaligned with the goal
            penalty = distance_optimal + 100 * goal_alignment;

            % Update the safe velocity if this penalty is lower
            if penalty < min_penalty
                min_penalty = penalty;
                v_safe = norm(vel_sample);
                w_safe = atan2(vel_sample(2), vel_sample(1)) - atan2(v_robot, w_robot);
                w_safe = min(max(w_safe, -pi/2), pi/2); % Limit angular velocity change
            end
        end
    end

    % If no safe velocity was found, reduce speed as a fallback
    if min_penalty == inf
        warning('No collision-free velocities found, reducing speed!');
        v_safe = v_robot;  % Stop the robot as a last resort
        w_safe = w_robot; 
    end
end

function [x, y] = generate_velocity_samples(N, v_x, v_y, speed_dif, k_size, max_velocity)
    % generate_velocity_samples Generates random velocity samples around the current velocity.

    theta = 2 * pi * rand(1, N);
    r = (speed_dif * (rand(1, N) .^ k_size));  % Adjust spread using k_size
    x = v_x + r .* cos(theta);
    y = v_y + r .* sin(theta);

    % Cap the velocities to max_velocity
    speeds = sqrt(x.^2 + y.^2);
    scale_factor = min(1, max_velocity ./ speeds);
    x = x .* scale_factor;
    y = y .* scale_factor;
end

function is_inside = is_inside_vo_cone(angle_sample, left_bound, right_bound)
    % is_inside_vo_cone Determines if a sample velocity angle is inside the VO cone.
    if left_bound <= right_bound
        is_inside = (angle_sample >= left_bound) && (angle_sample <= right_bound);
    else
        % Handles wrapping around the pi to -pi boundary
        is_inside = (angle_sample >= left_bound) || (angle_sample <= right_bound);
    end
end
