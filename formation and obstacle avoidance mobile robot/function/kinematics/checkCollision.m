function collisionDetected = checkCollision(robotPos, obstaclesPos, robotRadius)
    % Input:
    % robotPos - [x; y] position of the robot
    % obstaclesPos - Nx2 matrix of [x, y] positions of obstacles
    % robotRadius - radius of the robot
    % Output:
    % collisionDetected - true if collision occurs, false otherwise
    
    collisionDetected = false; % Initialize collision status
    for i = 1:size(obstaclesPos, 1)
        obstaclePos = obstaclesPos(i, :)';
        distance = norm(robotPos - obstaclePos); % Calculate distance
        if distance < (robotRadius) % Check for collision
            collisionDetected = true; % Collision detected
            break; % Exit the loop on collision
        end
    end
end
