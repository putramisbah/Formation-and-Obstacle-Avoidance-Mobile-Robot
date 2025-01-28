function obstaclePositions = lidarToWorld(robotPose, ranges, scanAngles, maxRange)
    % lidarToWorld - Convert LIDAR sensor data to world coordinates
    %
    % Inputs:
    %   robotPose  - Current robot pose [x; y; theta] (world frame)
    %   ranges     - LIDAR sensor ranges (distance measurements)
    %   scanAngles - LIDAR sensor scan angles (relative to robot)
    %   maxRange   - Maximum range of LIDAR sensor
    %
    % Outputs:
    %   obstaclePositions - Positions of detected obstacles in world frame [x, y]
    
    % Filter out invalid ranges (greater than maxRange)
    validIdx = ranges > 0 & ranges < maxRange;
    ranges = ranges(validIdx);
    scanAngles = scanAngles(validIdx);
    
    % Calculate obstacle positions in robot's coordinate frame
    xRobot = ranges .* cos(scanAngles);
    yRobot = ranges .* sin(scanAngles);
    
    % Transform from robot's coordinate frame to world coordinate frame
    theta = robotPose(3);
    rotationMatrix = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    obstaclePositionsRobot = [xRobot'; yRobot'];
    
    obstaclePositions = rotationMatrix .* obstaclePositionsRobot + robotPose(1:2);
    obstaclePositions = obstaclePositions';
end
