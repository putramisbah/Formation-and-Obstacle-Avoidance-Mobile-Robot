function [vRef1, wRef1, vRef2, wRef2, targetPosition1, targetPosition2] = VFormationControl(leaderPose, followerPose1, followerPose2, d, phi, maxSpeed)
% VFormationControl menghitung input kontrol untuk dua pengikut
% agar tetap dalam formasi "V" relatif terhadap pemimpin.
%
% Input:
%   - leaderPose: Posisi dan orientasi saat ini dari pemimpin [x; y; theta]
%   - followerPose1: Posisi dan orientasi saat ini dari pengikut kiri [x; y; theta]
%   - followerPose2: Posisi dan orientasi saat ini dari pengikut kanan [x; y; theta]
%   - d: Jarak dari pemimpin ke pengikut
%   - phi: Sudut formasi "V" dalam radian
%   - maxSpeed: Kecepatan maksimum yang diizinkan untuk pengikut
%
% Output:
%   - vRef1: Kecepatan linear yang diinginkan untuk pengikut kiri
%   - wRef1: Kecepatan angular yang diinginkan untuk pengikut kiri
%   - vRef2: Kecepatan linear yang diinginkan untuk pengikut kanan
%   - wRef2: Kecepatan angular yang diinginkan untuk pengikut kanan

    % Ekstraksi orientasi pemimpin
    theta = leaderPose(3);
    
    % Posisi target untuk pengikut kiri
    targetPosition1 = [leaderPose(1) - d * cos(theta + phi), ...
                       leaderPose(2) - d * sin(theta + phi)];
    
    % Posisi target untuk pengikut kanan
    targetPosition2 = [leaderPose(1) - d * cos(theta - phi), ...
                       leaderPose(2) - d * sin(theta - phi)];
    
    % Hitung kontrol untuk pengikut kiri
    [vRef1, wRef1, angleToTarget1] = CalculateControlInput(followerPose1, targetPosition1, maxSpeed);
    
    % Hitung kontrol untuk pengikut kanan
    [vRef2, wRef2, angleToTarget2] = CalculateControlInput(followerPose2, targetPosition2, maxSpeed);

end

function [vRef, wRef, angleToTarget] = CalculateControlInput(followerPose, targetPosition, maxSpeed)
    % Jarak dan arah ke target
    dx = targetPosition(1) - followerPose(1);
    dy = targetPosition(2) - followerPose(2);
    distanceToTarget = sqrt(dx^2 + dy^2);
    angleToTarget = atan2(dy, dx);
    
    % Sudut error
    alpha = angleToTarget - followerPose(3);
    
    % Kecepatan linear diatur berdasarkan jarak ke target, dibatasi oleh maxSpeed
    vRef = min(distanceToTarget, maxSpeed);
    
    % Kecepatan angular berdasarkan sudut error dan kecepatan linear
    L = 1; % Jarak look-ahead (dapat diatur sesuai kebutuhan)
    wRef = (2 * vRef * sin(alpha)) / L;
end