clear all;
[comPort, robot, id, info, modelNumber, hardwareVersion, firmwareVersion] = controllerSetup;
robot.DeviceWakeup();

request = zeros(3,1);
totalTimeElapsed = 0;
tic;
numSamples = 3000; % Adjust as needed
positions = zeros(3, numSamples);
speeds = zeros(3, numSamples);
timeStamps = zeros(1, numSamples);

figure;
hold on;
grid on;
xlabel('X Position (m)');
ylabel('Y Position (m)');
zlabel('Z Position (m)');
title('Happily Inverse 3 Alpha - 3D Position Plot');
view(3);

count = 0;
while true
    % Read position and velocity
    [pos, vel] = robot.EndEffectorForce(request);
    
    % Store values
    count = count + 1;
    positions(:, count) = pos;
    speeds(:, count) = vel;
    timeStamps(count) = totalTimeElapsed;
    
    % Plot live 3D position
    plot3(pos(1), pos(2), pos(3), 'k.');
    drawnow;
    
    % Print data to console
    fprintf("Time: %.2f s | Position: [%.3f, %.3f, %.3f] | Speed: [%.3f, %.3f, %.3f]\n", ...
        totalTimeElapsed, pos(1), pos(2), pos(3), vel(1), vel(2), vel(3));

    % Wait for sampling period
    while toc < 0.005 
        % Do nothing, just wait
    end
    totalTimeElapsed = totalTimeElapsed + toc;
    tic;

    % Stop after 15 seconds
    if (totalTimeElapsed > 15.0)
        break;
    end
end

% Final plot of position over time
figure;
subplot(2,1,1);
plot(timeStamps(1:count), positions(1,1:count), 'r', timeStamps(1:count), positions(2,1:count), 'g', timeStamps(1:count), positions(3,1:count), 'b');
xlabel('Time (s)');
ylabel('Position (m)');
legend('X', 'Y', 'Z');
title('End-Effector Position Over Time');
grid on;

subplot(2,1,2);
plot(timeStamps(1:count), speeds(1,1:count), 'r', timeStamps(1:count), speeds(2,1:count), 'g', timeStamps(1:count), speeds(3,1:count), 'b');
xlabel('Time (s)');
ylabel('Speed (m/s)');
legend('X Speed', 'Y Speed', 'Z Speed');
title('End-Effector Speed Over Time');
grid on;

%% Controller Setup Block
function [comPort,robot,id,info,modelNumber,hardwareVersion,firmwareVersion] = controllerSetup
    clear all;
    clc;
    addpath('C:\Users\patri\Downloads\SYSC 4206 Proj\dependencies'); % Add path for dependencies
    comPort = 'COM5';
    robot = HardwareAPI(comPort, true, true);
    id = robot.deviceId;
    info = robot.packageInfo();
    modelNumber = robot.deviceModelNumber;
    hardwareVersion = robot.deviceHardwareVersion;
    firmwareVersion = robot.deviceFirmwareVersion;
end
