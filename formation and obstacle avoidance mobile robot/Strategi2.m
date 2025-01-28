clear all
close all
clc

%% Create a folder to save figures
    outputFolder = fullfile(pwd, 'figure', '/Strategi2');
    if ~exist(outputFolder, 'dir')
        mkdir(outputFolder);
    end

%% Simulation Parameters
sampleTime = 0.1;
tVec = 0:sampleTime:35;

%% Define Mobile Robot Parameters
L = 0.5;
R = L/2;
dd = DifferentialDrive(R, L);
radius = L/2;  
max_velocity = 0.5;  
max_velocity_follower = 0.65;  
speed_dif = 0.5;  
k_size = 1; 

%% Create Multi-Robot Environment
numRobots = 5;
env = MultiRobotEnv(numRobots);
env.robotRadius = [L/2]; 
env.showTrajectory = [true; true; true; true];
env.hasWaypoints = true;
env.robotColors = [0 0 1; 0 0 1; 0 0 1; 0 1 0; 0 1 0];
d = 1/(sin(pi/4));      % formation offset
theta_formation = pi/4;  % formation degree

%% Collision Detection Parameters
collisionThreshold = L;               
collisionCount = 0;                  
inCollision = false(numRobots, numRobots); 

%% Define Sensor
sensor1 = MultiRobotLidarSensor;
sensor1.robotIdx = 1;
sensor1.sensorOffset = [0,0];
sensor1.scanAngles = linspace(-pi/4, pi/4, 10);
sensor1.maxRange = 4;
attachLidarSensor(env, sensor1);

sensor2 = MultiRobotLidarSensor;
sensor2.robotIdx = 2;
sensor2 = ObjectDetector;
sensor2.fieldOfView = pi/4;
attachObjectDetector(env,2,sensor2);

sensor3 = MultiRobotLidarSensor;
sensor3.robotIdx = 3;
sensor3 = ObjectDetector;
sensor3.fieldOfView = pi/4;
attachObjectDetector(env,3,sensor3);

%% Define Initial Positions and Static Obstacles
% initial position 
pose1 = [0; 5; 0];
pose2 = [-1; 4; 0];
pose3 = [-1; 6; 0];

pose4 = [8; 2; 3*pi/4];
pose5 = [9; 2; 3*pi/4];

objects = [3, 3, 2;
           8, 7, 2];
env.objectColors = [0 0 0;1 0 0;0 0 1];
env.objectMarkers = 'so^';
env.Poses = [pose1 pose2 pose3 pose4 pose5];

%% Path Planning for Robots
% Load map and inflate it by a safety distance
load Map.mat

% Create a Probabilistic Road Map (PRM)
planner = mobileRobotPRM(map);
planner.NumNodes = 75;
planner.ConnectionDistance = 10;

% Find a path from the start point to a specified goal point
startPoint = [1, 5];
goalPoint  = [15, 5];
waypoints1 = findpath(planner,startPoint,goalPoint);
waypoints2 = [8 2; 2 8];
waypoints3 = [9 2; 3 8];
show(planner);

%% Pengendali Pure Pursuit
controller = controllerPurePursuit;
controller.Waypoints = waypoints1;
controller.LookaheadDistance = 1.25;
controller.DesiredLinearVelocity = 0.5;
controller.MaxAngularVelocity = 1.0;

controller1 = controllerPurePursuit;
controller1.Waypoints = waypoints2;
controller1.LookaheadDistance = 0.35;
controller1.DesiredLinearVelocity = 0.5;
controller1.MaxAngularVelocity = 1.0;

controller2 = controllerPurePursuit;
controller2.Waypoints = waypoints3;
controller2.LookaheadDistance = 0.35;
controller2.DesiredLinearVelocity = 0.5;
controller2.MaxAngularVelocity = 1.0; 

%% Preallocate arrays for plotting
v_safe1_array = zeros(1, numel(tVec));
w_safe1_array = zeros(1, numel(tVec));
v_safe2_array = zeros(1, numel(tVec));
w_safe2_array = zeros(1, numel(tVec));
v_safe3_array = zeros(1, numel(tVec));
w_safe3_array = zeros(1, numel(tVec));
w_safe3_array = zeros(1, numel(tVec));

errorPositionFollower2 = zeros(1, numel(tVec));
errorPositionFollower3 = zeros(1, numel(tVec));

xTargetPosition2_array = zeros(1, numel(tVec));
yTargetPosition2_array = zeros(1, numel(tVec));
xTargetPosition3_array = zeros(1, numel(tVec));
yTargetPosition3_array = zeros(1, numel(tVec));

xcurPose2_array = zeros(1, numel(tVec));
ycurPose2_array = zeros(1, numel(tVec));
xcurPose3_array = zeros(1, numel(tVec));
ycurPose3_array = zeros(1, numel(tVec));

collisionTimes = [];     % Array to store collision times

%% looping simulatioan
r = rateControl(1/sampleTime);
for idx = 2:numel(tVec)
    curPose1 = pose1(:, idx-1);
    curPose2 = pose2(:, idx-1);
    curPose3 = pose3(:, idx-1);
    curPose4 = pose4(:, idx-1);
    curPose5 = pose5(:, idx-1);

    ranges1 = sensor1();
    ranges2 = sensor2(curPose2,objects);
    ranges3 = sensor3(curPose3,objects);

    % Detection collision
    for i = 1:numRobots
        for j = i+1:numRobots
            curPose_i = eval(['pose', num2str(i), '(:, idx-1)']);
            curPose_j = eval(['pose', num2str(j), '(:, idx-1)']);
            distance = norm(curPose_i(1:2) - curPose_j(1:2));
            
            if distance < collisionThreshold
                if ~inCollision(i, j)
                    collisionCount = collisionCount + 1;  
                    collisionTimes = [collisionTimes, tVec(idx)];  
                    disp(['Tabrakan terdeteksi antara Robot ', num2str(i), ...
                          ' dan Robot ', num2str(j), ' pada waktu t = ', num2str(tVec(idx)), ' s']);
                    inCollision(i, j) = true;  
                end
            else
                inCollision(i, j) = false;  
            end
        end
    end


    % Control Leader, Follower and Human (robot 4 & 5)
    [vRef1, wRef1, lookAheadPt] = controller(curPose1);
    [vRef2, wRef2, vRef3, wRef3, targetPosition2, targetPosition3] = VFormationControl(curPose1, curPose2, curPose3, d, theta_formation, max_velocity_follower);
    [vRef4, wRef4, lookAheadPt2] = controller1(curPose4);
    [vRef5, wRef5, lookAheadPt3] = controller2(curPose5);

    % Obstacle Pos
    obstaclesPos = [curPose4(1:2)'; curPose5(1:2)'; objects(1, 1:2); objects(2, 1:2)];
    obstaclesPos1 = [curPose4(1:2)'; curPose5(1:2)'];
    v_obs = [vRef4; vRef5; 0; 0];
    w_obs = [wRef4; wRef5; 0; 0];

    % Small formation for obstacle avoidance
    for i = 1:size(obstaclesPos1, 1)
        distance = norm(curPose1(1:2)' - obstaclesPos1(i, :));
        if distance < 5
            d = 0.5; % Perubahan nilai d jika dekat dengan obstacle
        else
            d = 1/(sin(pi/4));
        end
    end


    % Obstacle Avoidance Control
    [v_safe1, w_safe1] = velocity_obstacle(curPose1(1:2)', vRef1, wRef1, lookAheadPt, obstaclesPos, v_obs, w_obs, radius, max_velocity, speed_dif, k_size);
    [v_safe2, w_safe2] = velocity_obstacle(curPose2(1:2)', vRef2, wRef2, targetPosition2, obstaclesPos, v_obs, w_obs, radius, max_velocity_follower, speed_dif, k_size);
    [v_safe3, w_safe3] = velocity_obstacle(curPose3(1:2)', vRef3, wRef3, targetPosition3, obstaclesPos, v_obs, w_obs, radius, max_velocity_follower, speed_dif, k_size);

    % Store values for plotting
    v_safe1_array(idx) = v_safe1;
    w_safe1_array(idx) = w_safe1;
    v_safe2_array(idx) = v_safe2;
    w_safe2_array(idx) = w_safe2;
    v_safe3_array(idx) = v_safe3;
    w_safe3_array(idx) = w_safe3;

    xTargetPosition2_array(idx) = targetPosition2(1);
    yTargetPosition2_array(idx) = targetPosition2(2);
    xTargetPosition3_array(idx) = targetPosition3(1);
    yTargetPosition3_array(idx) = targetPosition3(2);

    xcurPose2_array(idx) = curPose2(1);
    ycurPose2_array(idx) = curPose2(2);
    xcurPose3_array(idx) = curPose3(1);
    ycurPose3_array(idx) = curPose3(2);

    errorPositionFollower2(idx) = norm(targetPosition2 - curPose2(1:2)');
    errorPositionFollower3(idx) = norm(targetPosition3 - curPose3(1:2)');

    positionDifferenceFollower2 = sqrt((xTargetPosition2_array - xcurPose2_array).^2 + (yTargetPosition2_array - ycurPose2_array).^2);


    % Stop Human
     if norm(curPose4(1:2) - waypoints2(end,:)') < 0.1
        disp('human 1 reached its goal');
        vRef4 = 0; 
        wRef4 = 0;
     end
     if norm(curPose5(1:2) - waypoints3(end,:)') < 0.1
         disp('human 2 reached its goal');
        vRef5 = 0; 
        wRef5 = 0;
     end

    % Control velocities
    velB1 = [v_safe1; 0; w_safe1];
    velB2 = [vRef2; 0; wRef2];
    velB3 = [vRef3; 0; wRef3];
    velB4 = [vRef4; 0; wRef4];
    velB5 = [vRef5; 0; wRef5];

    % Convert from body to world frame
    vel1 = bodyToWorld(velB1, curPose1);
    vel2 = bodyToWorld(velB2, curPose2);
    vel3 = bodyToWorld(velB3, curPose3);
    vel4 = bodyToWorld(velB4, curPose4);
    vel5 = bodyToWorld(velB5, curPose5);

    % Perform discrete time integration
    pose1(:, idx) = pose1(:, idx-1) + vel1 * sampleTime;
    pose2(:, idx) = pose2(:, idx-1) + vel2 * sampleTime;
    pose3(:, idx) = pose3(:, idx-1) + vel3 * sampleTime;
    pose4(:, idx) = pose4(:, idx-1) + vel4 * sampleTime;
    pose5(:, idx) = pose5(:, idx-1) + vel5 * sampleTime;

    % Update visualization of the environment and add legends
    env(1, curPose1, waypoints1, ranges1,objects);      % Robot 1
    env(2, curPose2, waypoints1, ranges2,objects);      % Robot 2
    env(3, curPose3, waypoints1, ranges3, objects);     % Robot 3
    env(4, curPose4, waypoints2, [], objects);          % Human 1
    env(5, curPose5, waypoints3, [], objects);          % Human 2
    xlim([-1 16]);
    ylim([-1 10]);
    xlabel('X Position [m]');
    ylabel('Y Position [m]');
    grid on;

    % Add legend
    hold on;
    h1 = plot(NaN, NaN, 'o', 'Color', [0 0 1], 'MarkerFaceColor', [0 0 1]); % Robot 1-3
    h2 = plot(NaN, NaN, 'o', 'Color', [0 1 0], 'MarkerFaceColor', [0 1 0]); % Human 1-2
    h3 = plot(NaN, NaN, 'o', 'Color', [1 0 0], 'MarkerFaceColor', [1 0 0]); % Static Obstacle 1-2
    legend([h1 h2 h3], {'Robots', 'Humans', 'Static Obstacles'}, 'Location', 'southeast');
    hold off;

    % Capture environment at specific times
    if abs(tVec(idx) - 10) < sampleTime/2
        captureFrames{1} = getframe(gcf);
        savefig(gcf,fullfile(outputFolder, 'Strategi2_10s.fig'));
    elseif abs(tVec(idx) - 20) < sampleTime/2
        captureFrames{2} = getframe(gcf);
        savefig(gcf,fullfile(outputFolder, 'Strategi2_20s.fig'));
    elseif abs(tVec(idx) - 30) < sampleTime/2  % Final frame capture at t=end
        captureFrames{3} = getframe(gcf);
        savefig(gcf,fullfile(outputFolder, 'Strategi2_end.fig'));
    end

    % Stop robot
    if norm(pose1(1:2, idx) - waypoints1(end,:)') < 0.05
        disp('Robot 1 reached its goal');
        break;
    end


    waitfor(r);
end

% plot
plotAndAnalyzeRobotData(tVec, v_safe1_array, w_safe1_array, v_safe2_array, w_safe2_array, ...
                                 v_safe3_array, w_safe3_array, errorPositionFollower2, errorPositionFollower3, ...
                                 xTargetPosition2_array, yTargetPosition2_array, xcurPose2_array, ycurPose2_array, ...
                                 xTargetPosition3_array, yTargetPosition3_array, xcurPose3_array, ycurPose3_array, ...
                                 pose1, waypoints1, collisionCount, collisionTimes,outputFolder);
